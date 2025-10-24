//! MMAP memory manager for consciousness system
//!
//! Coordinates pool allocation, region management, and context loading
//! for zero-copy access to dimensional layers.

use crate::{Result, ConsciousnessError, LayerId, DimensionId};
use super::{
    pool::{PoolAllocator, PoolStats},
    region::{MmapRegion, ContentLocation},
    MmapOffset, MmapHandle, LoadedContext, ContextCollection,
};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Main memory manager for the consciousness system
#[derive(Debug)]
pub struct MmapManager {
    pool_allocator: PoolAllocator,
    regions: HashMap<u32, MmapRegion>,
    layer_index: HashMap<LayerId, LayerLocation>,
    next_region_id: u32,
    base_path: PathBuf,
    total_allocated_mb: usize,
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
        
        // Initialize standard pools with different block sizes
        // 4KB blocks for small layers (metadata, simple content)
        pool_allocator.add_pool(32, 4096)?; // 32MB of 4KB blocks
        
        // 16KB blocks for medium layers (typical layer content)
        pool_allocator.add_pool(128, 16384)?; // 128MB of 16KB blocks
        
        // 64KB blocks for large layers (complex hierarchies)
        pool_allocator.add_pool(80, 65536)?; // 80MB of 64KB blocks
        
        // 256KB blocks for very large dimensions
        pool_allocator.add_pool(40, 262144)?; // 40MB of 256KB blocks
        
        let base_path = std::env::current_dir()
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to get current directory: {}", e)
            ))?
            .join("data")
            .join("consciousness");
        
        // Ensure base directory exists
        std::fs::create_dir_all(&base_path)
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to create base directory: {}", e)
            ))?;
        
        Ok(Self {
            pool_allocator,
            regions: HashMap::new(),
            layer_index: HashMap::new(),
            next_region_id: 0,
            base_path,
            total_allocated_mb: total_memory_mb,
        })
    }
    
    /// Load a dimension from file system
    pub fn load_dimension(&mut self, dimension_id: DimensionId) -> Result<u32> {
        let dimension_path = self.base_path
            .join(format!("D{:02}", dimension_id.0));
        
        if !dimension_path.exists() {
            return Err(ConsciousnessError::MemoryError(
                format!("Dimension directory not found: {:?}", dimension_path)
            ));
        }
        
        // Find the main region file for this dimension
        let region_file = dimension_path.join("region.mmap");
        if !region_file.exists() {
            return Err(ConsciousnessError::MemoryError(
                format!("Region file not found: {:?}", region_file)
            ));
        }
        
        let region_id = self.next_region_id;
        self.next_region_id += 1;
        
        // Load the MMAP region
        let region = MmapRegion::from_file(region_id, dimension_id, &region_file)?;
        
        // Index all layers in this region
        for layer_id in region.list_layers() {
            let layer_info = region.get_layer_info(layer_id)
                .ok_or_else(|| ConsciousnessError::MemoryError(
                    format!("Layer info not found for {:?}", layer_id)
                ))?;
            
            let location = LayerLocation {
                region_id,
                content_location: ContentLocation::Mmap {
                    offset: layer_info.offset,
                    size: layer_info.size,
                    region_id,
                },
            };
            
            self.layer_index.insert(layer_id, location);
        }
        
        self.regions.insert(region_id, region);
        
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
    pub fn load_layer_context(&self, layer_id: LayerId) -> Result<LoadedContext> {
        let location = self.layer_index.get(&layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("Layer not found in index: {:?}", layer_id)
            ))?;
        
        match &location.content_location {
            ContentLocation::Mmap { offset, size, region_id } => {
                let region = self.regions.get(region_id)
                    .ok_or_else(|| ConsciousnessError::MemoryError(
                        format!("Region not found: {}", region_id)
                    ))?;
                
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
                let region = self.regions.get(region_id)
                    .ok_or_else(|| ConsciousnessError::MemoryError(
                        format!("Region not found: {}", region_id)
                    ))?;
                
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
    pub fn allocate(&mut self, size: usize) -> Result<MmapOffset> {
        self.pool_allocator.allocate(size)
    }
    
    /// Deallocate previously allocated space
    pub fn deallocate(&mut self, offset: MmapOffset) -> Result<()> {
        self.pool_allocator.deallocate(offset)
    }
    
    /// Get memory usage statistics
    pub fn get_stats(&self) -> MemoryStats {
        let pool_stats = self.pool_allocator.get_stats();
        
        MemoryStats {
            total_allocated_mb: self.total_allocated_mb,
            pool_stats,
            regions_loaded: self.regions.len(),
            layers_indexed: self.layer_index.len(),
            utilization_percent: pool_stats.utilization(),
        }
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
    pub fn create_proto_dimension(
        &mut self,
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
        
        self.layer_index.insert(layer_id, location);
        
        Ok(layer_id)
    }
    
    /// Crystallize a proto-dimension from heap to MMAP
    pub fn crystallize_proto_dimension(
        &mut self,
        layer_id: LayerId,
    ) -> Result<()> {
        let location = self.layer_index.get(&layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("Proto-dimension not found: {:?}", layer_id)
            ))?;
        
        if let ContentLocation::Heap { data, .. } = &location.content_location {
            // Allocate space in reserve pool
            let mmap_offset = self.allocate(data.len())?;
            
            // Copy data to MMAP region
            let ptr = self.pool_allocator.get_ptr(mmap_offset)?;
            unsafe {
                std::ptr::copy_nonoverlapping(
                    data.as_ptr(),
                    ptr,
                    data.len()
                );
            }
            
            // Update location to MMAP
            let new_location = LayerLocation {
                region_id: mmap_offset.pool_id as u32,
                content_location: ContentLocation::Mmap {
                    offset: mmap_offset.offset,
                    size: data.len(),
                    region_id: mmap_offset.pool_id as u32,
                },
            };
            
            self.layer_index.insert(layer_id, new_location);
            
            tracing::info!("Crystallized proto-dimension {:?} to MMAP", layer_id);
        }
        
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
#[derive(Debug)]
pub struct MemoryStats {
    pub total_allocated_mb: usize,
    pub pool_stats: PoolStats,
    pub regions_loaded: usize,
    pub layers_indexed: usize,
    pub utilization_percent: f32,
}

impl MemoryStats {
    /// Check if memory usage is approaching limits
    pub fn is_near_capacity(&self) -> bool {
        self.utilization_percent > 85.0
    }
    
    /// Get available memory in MB
    pub fn available_mb(&self) -> usize {
        let used_mb = (self.pool_stats.allocated_size as f32 / (1024.0 * 1024.0)) as usize;
        self.total_allocated_mb.saturating_sub(used_mb)
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
        
        assert_eq!(stats.total_allocated_mb, 280);
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