//! Main memory manager coordinating pools and regions

use crate::{Result, ConsciousnessError, LayerId, DimensionId, Frequency};
use crate::memory::{
    MmapOffset, PoolAllocator, MmapRegion, LayerMetadata, 
    ContentLocation, ContextCollection, LoadedContext
};
use crate::navigation::NavigationPath;
use std::collections::HashMap;

/// Main memory manager for consciousness system
#[derive(Debug)]
pub struct MmapManager {
    /// Pool allocator for memory blocks
    pool_allocator: PoolAllocator,
    
    /// Regions organized by dimension
    regions: HashMap<DimensionId, MmapRegion>,
    
    /// Layer index for fast lookup
    layer_index: HashMap<LayerId, DimensionId>,
    
    /// Total memory budget in MB
    total_memory_mb: usize,
    
    /// Reserve pool for emergent dimensions
    reserve_pool_size: usize,
}

impl MmapManager {
    /// Create new memory manager with specified memory budget
    pub fn new(total_memory_mb: usize) -> Result<Self> {
        let pool_allocator = PoolAllocator::new()?;
        let regions = HashMap::new();
        let layer_index = HashMap::new();
        
        // Reserve 30% of memory for emergent dimensions
        let reserve_pool_size = (total_memory_mb * 30) / 100;
        
        Ok(Self {
            pool_allocator,
            regions,
            layer_index,
            total_memory_mb,
            reserve_pool_size,
        })
    }
    
    /// Initialize dimension regions with predefined layout
    pub fn initialize_dimensions(&mut self) -> Result<()> {
        let dimension_configs = vec![
            (DimensionId(1), 16),  // D01-Emotion: 16MB
            (DimensionId(2), 16),  // D02-Cognition: 16MB
            (DimensionId(3), 16),  // D03-Intention: 16MB
            (DimensionId(4), 8),   // D04-Social: 8MB
            (DimensionId(5), 8),   // D05-Temporal: 8MB
            (DimensionId(6), 16),  // D06-Philosophical: 16MB
            (DimensionId(7), 12),  // D07-Technical: 12MB
            (DimensionId(8), 8),   // D08-Creative: 8MB
            (DimensionId(9), 12),  // D09-Ethical: 12MB
            (DimensionId(10), 8),  // D10-Meta: 8MB
            (DimensionId(11), 8),  // D11-Ecological: 8MB
            (DimensionId(12), 8),  // D12-Positivity: 8MB
            (DimensionId(13), 8),  // D13-Balance: 8MB
            (DimensionId(14), 4),  // D14-Security: 4MB
        ];
        
        for (dimension_id, size_mb) in dimension_configs {
            let size_bytes = size_mb * 1024 * 1024;
            let base_offset = self.pool_allocator.allocate(size_bytes)?;
            
            let region = MmapRegion::new(
                dimension_id.0,
                base_offset,
                size_bytes,
            );
            
            self.regions.insert(dimension_id, region);
        }
        
        Ok(())
    }
    
    /// Store layer content in appropriate region
    pub fn store_layer(
        &mut self,
        layer_id: LayerId,
        name: String,
        frequency: Frequency,
        keywords: Vec<String>,
        content: &[u8],
    ) -> Result<()> {
        // Allocate memory for content
        let content_offset = self.pool_allocator.allocate(content.len())?;
        
        // Write content to allocated memory
        let memory_slice = self.pool_allocator.get_bytes_mut(content_offset)?;
        memory_slice[..content.len()].copy_from_slice(content);
        
        // Create layer metadata
        let mut metadata = LayerMetadata::new(layer_id, name, frequency, keywords);
        metadata.content_location = crate::memory::region::ContentLocationMeta::Mmap {
            offset_bytes: content_offset.offset,
            size: content.len(),
        };
        
        // Add to appropriate region
        let region = self.regions.get_mut(&layer_id.dimension)
            .ok_or_else(|| ConsciousnessError::MemoryError("Dimension region not found".to_string()))?;
        
        region.add_layer(metadata)?;
        
        // Update layer index
        self.layer_index.insert(layer_id, layer_id.dimension);
        
        Ok(())
    }
    
    /// Load contexts for navigation paths
    pub fn load_contexts(&mut self, paths: &[NavigationPath]) -> Result<ContextCollection> {
        let mut collection = ContextCollection::new();
        
        for path in paths {
            for &layer_id in &path.layer_sequence {
                let context = self.load_layer_context(layer_id)?;
                collection.add_context(context);
            }
        }
        
        Ok(collection)
    }
    
    /// Load single layer context
    pub fn load_layer_context(&mut self, layer_id: LayerId) -> Result<LoadedContext> {
        // Find region containing this layer
        let dimension_id = self.layer_index.get(&layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Layer not found in index".to_string()))?;
        
        let region = self.regions.get_mut(dimension_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Region not found".to_string()))?;
        
        let metadata = region.get_layer(layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Layer not found in region".to_string()))?;
        
        // Load content based on location
        let content = match &metadata.content_location {
            crate::memory::region::ContentLocationMeta::Mmap { offset_bytes, size } => {
                let offset = MmapOffset {
                    pool_id: 0, // TODO: proper pool ID mapping
                    offset: *offset_bytes,
                };
                let bytes = self.pool_allocator.get_bytes(offset)?;
                String::from_utf8_lossy(&bytes[..*size]).to_string()
            },
            crate::memory::region::ContentLocationMeta::Heap { .. } => {
                // TODO: implement heap content loading
                "Heap content not yet implemented".to_string()
            },
            crate::memory::region::ContentLocationMeta::Hybrid { .. } => {
                // TODO: implement hybrid content loading
                "Hybrid content not yet implemented".to_string()
            },
        };
        
        Ok(LoadedContext {
            layer_id,
            content,
            frequency: metadata.frequency(),
            keywords: metadata.keywords.clone(),
        })
    }
    
    /// Add synesthetic association to layer
    pub fn add_synesthetic_association(
        &mut self,
        layer_id: LayerId,
        keyword: String,
        associations: Vec<String>,
    ) -> Result<()> {
        let dimension_id = self.layer_index.get(&layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Layer not found".to_string()))?;
        
        let region = self.regions.get_mut(dimension_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Region not found".to_string()))?;
        
        let metadata = region.get_layer(layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Layer not found".to_string()))?;
        
        metadata.add_synesthetic_association(keyword, associations);
        
        Ok(())
    }
    
    /// Find layers matching keywords across all dimensions
    pub fn find_matching_layers(&mut self, keywords: &[String]) -> Vec<(LayerId, f32)> {
        let mut all_matches = Vec::new();
        
        for region in self.regions.values_mut() {
            let matches = region.find_matching_layers(keywords);
            all_matches.extend(matches);
        }
        
        // Sort by score (highest first)
        all_matches.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        
        all_matches
    }
    
    /// Crystallize proto-dimension from heap to mmap
    pub fn crystallize_proto_dimension(
        &mut self,
        dimension_id: DimensionId,
        layers: Vec<(LayerId, String, Frequency, Vec<String>, Vec<u8>)>,
    ) -> Result<()> {
        // Calculate total size needed
        let total_size: usize = layers.iter().map(|(_, _, _, _, content)| content.len()).sum();
        
        // Allocate from reserve pool
        if total_size > self.reserve_pool_size {
            return Err(ConsciousnessError::MemoryError(
                "Proto-dimension too large for reserve pool".to_string()
            ));
        }
        
        // Create new region for this dimension
        let base_offset = self.pool_allocator.allocate(total_size)?;
        let mut region = MmapRegion::new(dimension_id.0, base_offset, total_size);
        
        // Store all layers in the new region
        for (layer_id, name, frequency, keywords, content) in layers {
            let content_offset = self.pool_allocator.allocate(content.len())?;
            let memory_slice = self.pool_allocator.get_bytes_mut(content_offset)?;
            memory_slice[..content.len()].copy_from_slice(&content);
            
            let mut metadata = LayerMetadata::new(layer_id, name, frequency, keywords);
            metadata.content_location = crate::memory::region::ContentLocationMeta::Mmap {
                offset_bytes: content_offset.offset,
                size: content.len(),
            };
            
            region.add_layer(metadata)?;
            self.layer_index.insert(layer_id, dimension_id);
        }
        
        self.regions.insert(dimension_id, region);
        self.reserve_pool_size -= total_size;
        
        Ok(())
    }
    
    /// Get memory usage statistics
    pub fn get_memory_stats(&self) -> MemoryStats {
        let pool_stats = self.pool_allocator.get_stats();
        let total_pool_memory = self.pool_allocator.total_memory_usage();
        
        let region_stats: Vec<_> = self.regions.values()
            .map(|region| region.stats())
            .collect();
        
        let total_layers: usize = region_stats.iter()
            .map(|stats| stats.layer_count)
            .sum();
        
        let total_accesses: u64 = region_stats.iter()
            .map(|stats| stats.total_accesses)
            .sum();
        
        MemoryStats {
            total_memory_mb: self.total_memory_mb,
            pool_memory_bytes: total_pool_memory,
            reserve_pool_size: self.reserve_pool_size,
            total_dimensions: self.regions.len(),
            total_layers,
            total_accesses,
            pool_stats,
            region_stats,
        }
    }
}

/// Overall memory usage statistics
#[derive(Debug, Clone)]
pub struct MemoryStats {
    pub total_memory_mb: usize,
    pub pool_memory_bytes: usize,
    pub reserve_pool_size: usize,
    pub total_dimensions: usize,
    pub total_layers: usize,
    pub total_accesses: u64,
    pub pool_stats: Vec<crate::memory::pool::PoolStats>,
    pub region_stats: Vec<crate::memory::region::RegionStats>,
}