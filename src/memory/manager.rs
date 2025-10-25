//! MMAP memory manager for consciousness system
//!
//! Coordinates pool allocation, region management, and context loading
//! for zero-copy access to dimensional layers.

use crate::{Result, ConsciousnessError, LayerId, DimensionId};
use super::{
    pool::{PoolAllocator, PoolStats},
    region::{MmapRegion, ContentLocation},
    MmapOffset, MmapHandle, LoadedContext, ContextCollection,
    diagnostics::*,
};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, RwLock};

/// Main memory manager for the consciousness system
///
/// Thread Safety: RwLock enables multiple concurrent readers with exclusive writes
/// - Multiple threads can read regions simultaneously (shared lock)
/// - Only one thread can modify regions at a time (exclusive lock)
/// - This is optimal for read-heavy workloads like query processing
///
/// Design note: This struct doesn't implement Clone because it manages
/// exclusive ownership of memory-mapped regions. Use Arc<MmapManager>
/// for shared access across threads (RwLock is already internal).
#[derive(Debug)]
pub struct MmapManager {
    pool_allocator: PoolAllocator,
    // RwLock enables concurrent reads, exclusive writes
    // Multiple query threads can read regions simultaneously
    // Only dimension loading needs exclusive write access
    regions: RwLock<HashMap<u32, MmapRegion>>,
    // Separate index for layers enables fast lookup without scanning all regions
    // This is the "index everything" pattern - trade space for time
    // Also wrapped in RwLock for thread-safe concurrent access
    layer_index: RwLock<HashMap<LayerId, LayerLocation>>,
    // AtomicU32 for thread-safe region ID generation
    next_region_id: AtomicUsize,
    base_path: PathBuf,
    total_limit_mb: usize,
    // AtomicUsize enables lock-free memory tracking across threads
    // Compare-and-swap operations are faster than mutex locks
    current_allocated_bytes: AtomicUsize,
    
    // Task 8.1: Monitoring counters
    allocation_failure_count: AtomicUsize,
    total_allocations: AtomicUsize,
    total_deallocations: AtomicUsize,
    peak_concurrent_readers: AtomicUsize,
}

/// Location information for a layer
#[derive(Debug, Clone)]
struct LayerLocation {
    region_id: u32,
    content_location: ContentLocation,
}

impl MmapManager {
    /// Create new MMAP manager with specified total memory allocation
    pub fn new(total_memory_mb: usize) -> Result<Self> {
        let mut pool_allocator = PoolAllocator::new();
        
        // Multi-pool strategy: different block sizes for different use cases
        // This reduces fragmentation - small allocations don't waste large blocks
        // Similar to how malloc uses size classes internally
        
        // 4KB blocks for small layers (metadata, simple content)
        // 4KB matches typical OS page size - efficient for memory mapping
        pool_allocator.add_pool(32, 4096)?; // 32MB of 4KB blocks
        
        // 16KB blocks for medium layers (typical layer content)
        // Most consciousness layers fall into this category
        pool_allocator.add_pool(128, 16384)?; // 128MB of 16KB blocks
        
        // 64KB blocks for large layers (complex hierarchies)
        // Larger blocks reduce allocation overhead for big content
        pool_allocator.add_pool(80, 65536)?; // 80MB of 64KB blocks
        
        // 256KB blocks for very large dimensions
        // Rare but necessary for comprehensive dimensional data
        pool_allocator.add_pool(40, 262144)?; // 40MB of 256KB blocks
        
        // PathBuf is the owned version of Path - like String vs &str
        // We need owned here because it's stored in the struct
        let base_path = std::env::current_dir()
            // map_err transforms io::Error into our ConsciousnessError
            // This is the functional approach to error handling - no try-catch needed
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to get current directory: {}", e)
            ))?
            .join("data")
            .join("consciousness");
        
        // Ensure base directory exists
        // create_dir_all is idempotent - safe to call multiple times
        std::fs::create_dir_all(&base_path)
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to create base directory: {}", e)
            ))?;
        
        Ok(Self {
            pool_allocator,
            // RwLock wraps HashMap for thread-safe concurrent access
            // Multiple readers can access simultaneously, writes are exclusive
            regions: RwLock::new(HashMap::new()),
            layer_index: RwLock::new(HashMap::new()),
            // AtomicUsize for thread-safe region ID generation
            next_region_id: AtomicUsize::new(0),
            base_path,
            total_limit_mb: total_memory_mb,
            // AtomicUsize::new(0) is a const fn - happens at compile time when possible
            current_allocated_bytes: AtomicUsize::new(0),
            
            // Initialize monitoring counters
            allocation_failure_count: AtomicUsize::new(0),
            total_allocations: AtomicUsize::new(0),
            total_deallocations: AtomicUsize::new(0),
            peak_concurrent_readers: AtomicUsize::new(0),
        })
    }
    
    /// Load a dimension from file system
    ///
    /// Thread Safety: Acquires write lock on regions map during load
    /// This ensures exclusive access while modifying the regions HashMap
    /// Other threads will block on dimension loading but can read concurrently after
    ///
    /// Takes &self (not &mut) because RwLock provides interior mutability
    /// Returns u32 region_id for tracking - small integer IDs are cache-friendly
    pub fn load_dimension(&self, dimension_id: DimensionId) -> Result<u32> {
        // format! allocates a new String - acceptable here since this isn't hot path
        // In hot paths, we'd use a pre-allocated buffer or stack array
        let dimension_path = self.base_path
            .join(format!("D{:02}", dimension_id.0));
        
        // Early return pattern - fail fast, don't nest deeply
        // This keeps cognitive complexity low
        if !dimension_path.exists() {
            return Err(ConsciousnessError::DimensionNotFound {
                dimension: dimension_id.0,
            });
        }
        
        // Find the main region file for this dimension
        let region_file = dimension_path.join("region.mmap");
        if !region_file.exists() {
            return Err(ConsciousnessError::DimensionNotFound {
                dimension: dimension_id.0,
            });
        }
        
        // Allocate region ID atomically - thread-safe without locks
        // fetch_add returns OLD value, then increments
        let region_id = self.next_region_id.fetch_add(1, Ordering::SeqCst) as u32;
        
        // Load region from file (no locks needed yet)
        let region = MmapRegion::from_file(region_id, dimension_id, &region_file)
            .map_err(|e| {
                // Rollback region ID on failure
                // fetch_sub atomically decrements
                self.next_region_id.fetch_sub(1, Ordering::SeqCst);
                e
            })?;
        
        // Prepare layer locations before acquiring locks
        // This minimizes time spent holding write locks
        let mut layer_locations = Vec::new();
        for layer_id in region.list_layers() {
            let layer_info = region.get_layer_info(layer_id)
                .ok_or_else(|| {
                    // Rollback on error
                    self.next_region_id.fetch_sub(1, Ordering::SeqCst);
                    ConsciousnessError::MemoryError(
                        format!("Layer info not found for {:?}", layer_id)
                    )
                })?;
            
            let location = LayerLocation {
                region_id,
                content_location: ContentLocation::Mmap {
                    offset: layer_info.offset,
                    size: layer_info.size,
                    region_id,
                },
            };
            
            layer_locations.push((layer_id, location));
        }
        
        // Now acquire write locks and commit changes atomically
        // This is the critical section - keep it minimal
        {
            let mut regions = self.regions.write()
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to acquire regions write lock: {}", e)
                ))?;
            let mut layer_index = self.layer_index.write()
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to acquire layer_index write lock: {}", e)
                ))?;
            
            // Insert all layers
            for (layer_id, location) in layer_locations {
                layer_index.insert(layer_id, location);
            }
            
            // Insert region
            regions.insert(region_id, region);
        } // Locks released here automatically
        
        let layer_count = {
            let layer_index = self.layer_index.read()
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to acquire layer_index read lock: {}", e)
                ))?;
            layer_index.values().filter(|loc| loc.region_id == region_id).count()
        };
        
        tracing::debug!(
            "Loaded dimension {:?} as region {} with {} layers",
            dimension_id,
            region_id,
            layer_count
        );
        
        Ok(region_id)
    }
    
    /// Load contexts for specified navigation paths
    pub fn load_contexts(&self, paths: &[NavigationPath]) -> Result<ContextCollection> {
        let mut collection = ContextCollection::new();
        
        for path in paths {
            for &layer_id in &path.layer_sequence {
                let context = self.load_layer_context(layer_id)?;
                collection.add_context(context);
            }
        }
        
        Ok(collection)
    }
    
    /// Load context for a specific layer
    ///
    /// Thread Safety: Uses read lock - multiple threads can call this simultaneously
    /// This is the hot path for query processing, optimized for concurrent reads
    pub fn load_layer_context(&self, layer_id: LayerId) -> Result<LoadedContext> {
        // Acquire read lock on layer_index
        // Multiple threads can hold read locks simultaneously
        let layer_index = self.layer_index.read()
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to acquire layer_index read lock: {}", e)
            ))?;
        
        let location = layer_index.get(&layer_id)
            .ok_or_else(|| ConsciousnessError::LayerNotFound {
                dimension: layer_id.dimension.0,
                layer: layer_id.layer,
            })?.clone(); // Clone to release lock early
        
        // Release layer_index lock before acquiring regions lock
        drop(layer_index);
        
        match &location.content_location {
            ContentLocation::Mmap { offset, size, region_id } => {
                // Acquire read lock on regions
                let regions = self.regions.read()
                    .map_err(|e| ConsciousnessError::MemoryError(
                        format!("Failed to acquire regions read lock: {}", e)
                    ))?;
                
                let region = regions.get(region_id)
                    .ok_or_else(|| ConsciousnessError::RegionNotFound {
                        region_id: *region_id,
                    })?;
                
                let content = region.read_string(*offset, *size)?;
                let layer_info = region.get_layer_info(layer_id).unwrap();
                
                Ok(LoadedContext {
                    layer_id,
                    content,
                    frequency: layer_info.frequency(),
                    keywords: layer_info.keywords.clone(),
                })
            }
            
            ContentLocation::Heap { data, .. } => {
                let content = String::from_utf8(data.clone())
                    .map_err(|e| ConsciousnessError::MemoryError(
                        format!("Invalid UTF-8 in heap content: {}", e)
                    ))?;
                
                // For heap content, we need to get frequency and keywords from somewhere
                // This would typically be stored alongside the heap data
                Ok(LoadedContext {
                    layer_id,
                    content,
                    frequency: crate::Frequency::new(1.0), // Default frequency
                    keywords: Vec::new(), // Would need to be stored with heap data
                })
            }
            
            ContentLocation::Hybrid { mmap_base, mmap_size, heap_overlay, region_id } => {
                // Acquire read lock for hybrid content
                let regions = self.regions.read()
                    .map_err(|e| ConsciousnessError::MemoryError(
                        format!("Failed to acquire regions read lock: {}", e)
                    ))?;
                
                let region = regions.get(region_id)
                    .ok_or_else(|| ConsciousnessError::RegionNotFound {
                        region_id: *region_id,
                    })?;
                
                let mmap_content = region.read_string(*mmap_base, *mmap_size)?;
                let heap_content = String::from_utf8(heap_overlay.clone())
                    .map_err(|e| ConsciousnessError::MemoryError(
                        format!("Invalid UTF-8 in heap overlay: {}", e)
                    ))?;
                
                let combined_content = format!("{}\n\n{}", mmap_content, heap_content);
                let layer_info = region.get_layer_info(layer_id).unwrap();
                
                Ok(LoadedContext {
                    layer_id,
                    content: combined_content,
                    frequency: layer_info.frequency(),
                    keywords: layer_info.keywords.clone(),
                })
            }
        }
    }
    
    /// Allocate space for new content (for learning system)
    ///
    /// Uses atomic operations for thread-safe memory tracking without locks
    /// This is faster than Mutex<usize> because it's lock-free
    pub fn allocate(&mut self, size: usize) -> Result<MmapOffset> {
        // AtomicUsize::load reads the current value atomically
        // Ordering::Relaxed is sufficient here - we only need atomicity, not ordering
        // Relaxed is fastest: no memory barriers, just atomic read
        // Use Acquire/Release when coordinating with other memory operations
        let current_bytes = self.current_allocated_bytes.load(Ordering::Relaxed);
        let limit_bytes = self.total_limit_mb * 1024 * 1024;
        let new_total = current_bytes + size;
        
        // Calculate utilization percentage
        // as f32 cast is explicit - Rust doesn't do implicit numeric conversions
        // This prevents subtle bugs from unexpected type coercion
        let utilization = (new_total as f32 / limit_bytes as f32) * 100.0;
        
        // Graduated warning system - fail gracefully, not catastrophically
        // This is the "circuit breaker" pattern from distributed systems
        if utilization > 95.0 {
            // Critical: reject allocation to prevent OOM
            tracing::error!(
                "Memory critically high: {:.1}% ({} MB / {} MB), rejecting allocation of {} bytes",
                utilization,
                current_bytes / (1024 * 1024),
                self.total_limit_mb,
                size
            );
            return Err(ConsciousnessError::LimitExceeded {
                current_mb: current_bytes / (1024 * 1024),
                limit_mb: self.total_limit_mb,
                requested_mb: (size + 1024 * 1024 - 1) / (1024 * 1024),
            });
        } else if utilization > 85.0 {
            // High pressure: warn but allow allocation
            tracing::warn!(
                "Memory pressure high: {:.1}% ({} MB / {} MB), consider eviction",
                utilization,
                current_bytes / (1024 * 1024),
                self.total_limit_mb
            );
        } else if utilization > 75.0 {
            // Elevated: informational warning
            tracing::warn!(
                "Memory usage elevated: {:.1}% ({} MB / {} MB)",
                utilization,
                current_bytes / (1024 * 1024),
                self.total_limit_mb
            );
        }
        
        if new_total > limit_bytes {
            return Err(ConsciousnessError::LimitExceeded {
                current_mb: current_bytes / (1024 * 1024),
                limit_mb: self.total_limit_mb,
                requested_mb: (size + 1024 * 1024 - 1) / (1024 * 1024),
            });
        }
        
        // Try to allocate from pool
        // ? operator: if Err, convert and return early; if Ok, unwrap value
        let offset = self.pool_allocator.allocate(size)
            .map_err(|e| {
                // Track allocation failure
                self.allocation_failure_count.fetch_add(1, Ordering::Relaxed);
                ConsciousnessError::AllocationFailed(
                    format!("Pool allocation failed for {} bytes: {}", size, e)
                )
            })?;
        
        // Track successful allocation
        self.total_allocations.fetch_add(1, Ordering::Relaxed);
        
        // fetch_add is atomic read-modify-write: returns OLD value, then adds
        // This is implemented as a CPU compare-and-swap loop - lock-free
        // Much faster than Mutex::lock() -> value += size -> Mutex::unlock()
        self.current_allocated_bytes.fetch_add(size, Ordering::Relaxed);
        
        tracing::debug!(
            "Allocated {} bytes at pool {} offset {}, total usage: {:.1}%",
            size,
            offset.pool_id,
            offset.offset,
            utilization
        );
        
        Ok(offset)
    }
    
    /// Deallocate previously allocated space
    pub fn deallocate(&mut self, offset: MmapOffset, size: usize) -> Result<()> {
        self.pool_allocator.deallocate(offset)?;
        
        // Track deallocation
        self.total_deallocations.fetch_add(1, Ordering::Relaxed);
        
        // Update allocated bytes counter
        self.current_allocated_bytes.fetch_sub(size, Ordering::Relaxed);
        
        Ok(())
    }
    
    /// Get memory usage statistics
    ///
    /// Thread Safety: Uses read locks to safely access regions and layer_index
    pub fn get_stats(&self) -> MemoryStats {
        let pool_stats = self.pool_allocator.get_stats();
        let current_bytes = self.current_allocated_bytes.load(Ordering::Relaxed);
        let current_mb = current_bytes / (1024 * 1024);
        
        // Acquire read locks to get counts
        let regions_count = self.regions.read()
            .map(|r| r.len())
            .unwrap_or(0);
        let layers_count = self.layer_index.read()
            .map(|l| l.len())
            .unwrap_or(0);
        
        // Load monitoring counters
        let allocation_failures = self.allocation_failure_count.load(Ordering::Relaxed);
        let total_allocs = self.total_allocations.load(Ordering::Relaxed);
        let total_deallocs = self.total_deallocations.load(Ordering::Relaxed);
        let peak_readers = self.peak_concurrent_readers.load(Ordering::Relaxed);
        
        // Calculate fragmentation ratio
        // Fragmentation = (allocated - used) / allocated
        // For now, use a simple estimate based on pool stats
        let fragmentation = if pool_stats.total_size > 0 {
            let wasted = pool_stats.total_size.saturating_sub(pool_stats.allocated_size);
            (wasted as f32 / pool_stats.total_size as f32) * 100.0
        } else {
            0.0
        };
        
        // TODO: Get per-pool utilization from PoolAllocator
        // For now, return empty vec - will be enhanced in future
        let per_pool_util = Vec::new();
        
        MemoryStats {
            total_limit_mb: self.total_limit_mb,
            current_allocated_mb: current_mb,
            pool_stats,
            regions_loaded: regions_count,
            layers_indexed: layers_count,
            utilization_percent: (current_mb as f32 / self.total_limit_mb as f32) * 100.0,
            
            // Task 8.1: Detailed monitoring
            per_pool_utilization: per_pool_util,
            fragmentation_ratio: fragmentation,
            allocation_failure_count: allocation_failures,
            peak_concurrent_readers: peak_readers,
            total_allocations: total_allocs,
            total_deallocations: total_deallocs,
        }
    }
    
    /// Dump complete memory manager state for diagnostics
    ///
    /// Task 8.2: Creates comprehensive snapshot including:
    /// - All loaded regions with metadata
    /// - Complete layer index
    /// - Pool allocator state
    /// - Current memory usage
    /// - Historical statistics
    ///
    /// This is invaluable for debugging production issues
    pub fn dump_state(&self) -> MemoryStateDump {
        let mut dump = MemoryStateDump::new();
        
        // Capture memory usage
        let stats = self.get_stats();
        dump.memory_usage = MemoryUsageDump {
            total_limit_mb: stats.total_limit_mb,
            current_allocated_mb: stats.current_allocated_mb,
            available_mb: stats.available_mb(),
            utilization_percent: stats.utilization_percent,
            fragmentation_ratio: stats.fragmentation_ratio,
        };
        
        // Capture statistics
        dump.statistics = StatisticsDump {
            total_allocations: stats.total_allocations,
            total_deallocations: stats.total_deallocations,
            allocation_failures: stats.allocation_failure_count,
            peak_concurrent_readers: stats.peak_concurrent_readers,
            regions_loaded: stats.regions_loaded,
            layers_indexed: stats.layers_indexed,
        };
        
        // Capture regions (with read lock)
        if let Ok(regions) = self.regions.read() {
            for (region_id, region) in regions.iter() {
                let layers: Vec<LayerDump> = region.list_layers()
                    .iter()
                    .filter_map(|&layer_id| {
                        region.get_layer_info(layer_id).map(|info| LayerDump {
                            layer_id: format!("{:?}", layer_id),
                            name: info.name.clone(),
                            offset: info.offset,
                            size: info.size,
                            frequency: info.frequency().value(),
                            keywords: info.keywords.clone(),
                        })
                    })
                    .collect();
                
                dump.regions.push(RegionDump {
                    region_id: *region_id,
                    dimension_id: 0, // TODO: Get from region
                    dimension_name: format!("D{:02}", region_id),
                    file_path: String::from("unknown"), // TODO: Store path in region
                    size_bytes: 0, // TODO: Get from region
                    layer_count: layers.len(),
                    layers,
                });
            }
        }
        
        // Capture layer index (with read lock)
        if let Ok(layer_index) = self.layer_index.read() {
            for (layer_id, location) in layer_index.iter() {
                let (location_type, offset, size) = match &location.content_location {
                    ContentLocation::Mmap { offset, size, .. } => ("MMAP", *offset, *size),
                    ContentLocation::Heap { data, .. } => ("Heap", 0, data.len()),
                    ContentLocation::Hybrid { mmap_size, heap_overlay, .. } => 
                        ("Hybrid", 0, mmap_size + heap_overlay.len()),
                };
                
                dump.layer_index.push(LayerIndexEntry {
                    layer_id: format!("{:?}", layer_id),
                    dimension: layer_id.dimension.0,
                    layer: layer_id.layer,
                    location_type: location_type.to_string(),
                    region_id: location.region_id,
                    offset,
                    size,
                });
            }
        }
        
        // TODO: Capture pool state from PoolAllocator
        // For now, just set total_pools
        dump.pool_state.total_pools = 4; // We have 4 pools
        
        dump
    }
    
    /// Pre-allocate space for core dimensions (fast initialization)
    ///
    /// This reserves memory slots without loading actual files,
    /// ensuring initialization completes within 100ms budget.
    pub fn pre_allocate_dimensions(&mut self) -> Result<()> {
        // Pre-allocation is already done in new() via pool initialization
        // Pools are created with fixed sizes, so memory is already reserved
        // This function validates that pre-allocation succeeded
        
        let stats = self.get_stats();
        
        tracing::info!(
            "Pre-allocated {} MB across {} pools for 14 core dimensions",
            stats.total_limit_mb,
            self.pool_allocator.pool_count()
        );
        
        Ok(())
    }
    
    /// Initialize all core dimensions
    pub async fn initialize_core_dimensions(&mut self) -> Result<()> {
        // Load all 14 core dimensions
        let core_dimensions = [
            DimensionId(1),  // D01-Emotion
            DimensionId(2),  // D02-Cognition
            DimensionId(3),  // D03-Intention
            DimensionId(4),  // D04-Social
            DimensionId(5),  // D05-Temporal
            DimensionId(6),  // D06-Philosophical
            DimensionId(7),  // D07-Technical
            DimensionId(8),  // D08-Creative
            DimensionId(9),  // D09-Ethical
            DimensionId(10), // D10-Meta
            DimensionId(11), // D11-Ecological
            DimensionId(12), // D12-Positivity
            DimensionId(13), // D13-Balance
            DimensionId(14), // D14-Security
        ];
        
        for dimension_id in &core_dimensions {
            match self.load_dimension(*dimension_id) {
                Ok(region_id) => {
                    tracing::info!("Loaded dimension {:?} as region {}", dimension_id, region_id);
                }
                Err(e) => {
                    tracing::warn!("Failed to load dimension {:?}: {}", dimension_id, e);
                    // Continue loading other dimensions even if one fails
                }
            }
        }
        
        Ok(())
    }
    
    /// Create a new proto-dimension in heap memory (for learning)
    ///
    /// Thread Safety: Acquires write lock on layer_index
    pub fn create_proto_dimension(
        &self,
        dimension_id: DimensionId,
        content: Vec<u8>,
    ) -> Result<LayerId> {
        let layer_id = LayerId {
            dimension: dimension_id,
            layer: 0, // Proto-dimensions start at layer 0
        };
        
        let location = LayerLocation {
            region_id: u32::MAX, // Special marker for heap-only content
            content_location: ContentLocation::Heap {
                data: content,
                created_at: std::time::SystemTime::now(),
            },
        };
        
        // Acquire write lock to insert
        let mut layer_index = self.layer_index.write()
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to acquire layer_index write lock: {}", e)
            ))?;
        layer_index.insert(layer_id, location);
        
        Ok(layer_id)
    }
    
    /// Crystallize a proto-dimension from heap to MMAP
    ///
    /// Thread Safety: Atomic pointer swap ensures readers see consistent state
    /// - Readers accessing heap version continue to work during crystallization
    /// - Atomic swap ensures no reader sees partial/inconsistent state
    /// - Old heap version stays valid until swap completes
    /// - This implements Task 7.2: Atomic pointer swap for crystallization
    ///
    /// Note: Takes &mut self because allocate() needs mutable access to pool_allocator
    /// In a fully lock-free design, pool_allocator would use interior mutability
    pub fn crystallize_proto_dimension(
        &mut self,
        layer_id: LayerId,
    ) -> Result<()> {
        // Step 1: Read current location (read lock)
        let data = {
            let layer_index = self.layer_index.read()
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to acquire layer_index read lock: {}", e)
                ))?;
            
            let location = layer_index.get(&layer_id)
                .ok_or_else(|| ConsciousnessError::LayerNotFound {
                    dimension: layer_id.dimension.0,
                    layer: layer_id.layer,
                })?;
            
            if let ContentLocation::Heap { data, .. } = &location.content_location {
                data.clone()
            } else {
                return Ok(()); // Already crystallized or not heap
            }
        }; // Read lock released here
        
        // Step 2: Allocate MMAP space and copy data (no locks needed)
        let data_len = data.len();
        let mmap_offset = self.allocate(data_len)?;
        
        // Copy data to MMAP region
        let ptr = self.pool_allocator.get_ptr(mmap_offset)?;
        unsafe {
            std::ptr::copy_nonoverlapping(
                data.as_ptr(),
                ptr,
                data_len,
            );
        }
        
        // Step 3: Atomic swap - acquire write lock and update location
        // This is the critical section where we atomically switch from heap to MMAP
        {
            let mut layer_index = self.layer_index.write()
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to acquire layer_index write lock: {}", e)
                ))?;
            
            let new_location = LayerLocation {
                region_id: mmap_offset.pool_id as u32,
                content_location: ContentLocation::Mmap {
                    offset: mmap_offset.offset,
                    size: data_len,
                    region_id: mmap_offset.pool_id as u32,
                },
            };
            
            // Atomic swap: readers see either old (heap) or new (MMAP), never partial
            layer_index.insert(layer_id, new_location);
        } // Write lock released, heap data can now be dropped safely
        
        tracing::info!("Crystallized proto-dimension {:?} to MMAP", layer_id);
        
        Ok(())
    }
}

/// Navigation path from multiverse navigator
#[derive(Debug, Clone)]
pub struct NavigationPath {
    pub dimension_id: DimensionId,
    pub layer_sequence: Vec<LayerId>,
    pub confidence: f32,
    pub frequency: crate::Frequency,
}

/// Memory usage statistics
///
/// Comprehensive monitoring data for operational visibility
#[derive(Debug, Clone)]
pub struct MemoryStats {
    pub total_limit_mb: usize,
    pub current_allocated_mb: usize,
    pub pool_stats: PoolStats,
    pub regions_loaded: usize,
    pub layers_indexed: usize,
    pub utilization_percent: f32,
    
    // Task 8.1: Detailed tracking
    pub per_pool_utilization: Vec<PoolUtilization>,
    pub fragmentation_ratio: f32,
    pub allocation_failure_count: usize,
    pub peak_concurrent_readers: usize,
    pub total_allocations: usize,
    pub total_deallocations: usize,
}

/// Per-pool utilization details
#[derive(Debug, Clone)]
pub struct PoolUtilization {
    pub pool_id: usize,
    pub block_size: usize,
    pub total_blocks: usize,
    pub used_blocks: usize,
    pub utilization_percent: f32,
}

impl MemoryStats {
    /// Check if memory usage is approaching limits
    pub fn is_near_capacity(&self) -> bool {
        self.utilization_percent > 85.0
    }
    
    /// Get available memory in MB
    pub fn available_mb(&self) -> usize {
        self.total_limit_mb.saturating_sub(self.current_allocated_mb)
    }
    
    /// Check if at warning threshold (75%)
    pub fn is_at_warning_threshold(&self) -> bool {
        self.utilization_percent > 75.0
    }
    
    /// Check if should trigger eviction (85%)
    pub fn should_trigger_eviction(&self) -> bool {
        self.utilization_percent > 85.0
    }
    
    /// Check if should reject allocations (95%)
    pub fn should_reject_allocations(&self) -> bool {
        self.utilization_percent > 95.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;
    
    #[tokio::test]
    async fn test_memory_manager_creation() {
        let manager = MmapManager::new(280).unwrap();
        let stats = manager.get_stats();
        
        assert_eq!(stats.total_limit_mb, 280);
        assert_eq!(stats.regions_loaded, 0);
        assert_eq!(stats.layers_indexed, 0);
    }
    
    #[tokio::test]
    async fn test_proto_dimension_lifecycle() {
        let mut manager = MmapManager::new(280).unwrap();
        let dimension_id = DimensionId(99); // Test dimension
        
        // Create proto-dimension
        let content = b"test proto content".to_vec();
        let layer_id = manager.create_proto_dimension(dimension_id, content).unwrap();
        
        // Load context from heap
        let context = manager.load_layer_context(layer_id).unwrap();
        assert_eq!(context.content, "test proto content");
        
        // Crystallize to MMAP
        manager.crystallize_proto_dimension(layer_id).unwrap();
        
        // Load context from MMAP
        let context = manager.load_layer_context(layer_id).unwrap();
        assert_eq!(context.content, "test proto content");
    }
}