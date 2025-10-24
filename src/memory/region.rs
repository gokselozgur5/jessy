//! Memory region management for consciousness layers

use crate::{Result, ConsciousnessError, LayerId, Frequency};
use crate::memory::MmapOffset;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Location of layer content in memory
#[derive(Debug, Clone)]
pub enum ContentLocation {
    /// Content stored in memory-mapped region
    Mmap { offset: MmapOffset, size: usize },
    
    /// Content stored in heap (temporary, learning)
    Heap { data: Vec<u8> },
    
    /// Hybrid: base content in mmap + overlay in heap
    Hybrid { 
        mmap_base: MmapOffset, 
        mmap_size: usize,
        heap_overlay: Vec<u8> 
    },
}

impl ContentLocation {
    /// Get the total size of content
    pub fn size(&self) -> usize {
        match self {
            ContentLocation::Mmap { size, .. } => *size,
            ContentLocation::Heap { data } => data.len(),
            ContentLocation::Hybrid { mmap_size, heap_overlay, .. } => {
                mmap_size + heap_overlay.len()
            }
        }
    }
    
    /// Check if content is stored in mmap (fast access)
    pub fn is_mmap(&self) -> bool {
        matches!(self, ContentLocation::Mmap { .. } | ContentLocation::Hybrid { .. })
    }
    
    /// Check if content has heap overlay (learning data)
    pub fn has_heap_overlay(&self) -> bool {
        matches!(self, ContentLocation::Heap { .. } | ContentLocation::Hybrid { .. })
    }
}

/// Memory-mapped region for consciousness layers
#[derive(Debug)]
pub struct MmapRegion {
    /// Region identifier
    pub region_id: u8,
    
    /// Base memory offset
    pub base_offset: MmapOffset,
    
    /// Total size of region
    pub total_size: usize,
    
    /// Layers stored in this region
    pub layers: HashMap<LayerId, LayerMetadata>,
    
    /// Free space tracking
    pub free_space: usize,
}

/// Metadata for a layer stored in memory
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerMetadata {
    /// Layer identifier
    pub layer_id: LayerId,
    
    /// Layer name
    pub name: String,
    
    /// Operating frequency
    pub frequency: f32,
    
    /// Content location
    pub content_location: ContentLocationMeta,
    
    /// Keywords for matching
    pub keywords: Vec<String>,
    
    /// Synesthetic associations
    pub synesthetic_map: HashMap<String, Vec<String>>,
    
    /// Parent layer (if any)
    pub parent: Option<LayerId>,
    
    /// Child layers
    pub children: Vec<LayerId>,
    
    /// Last access timestamp
    pub last_accessed: u64,
    
    /// Access count for usage statistics
    pub access_count: u64,
}

/// Serializable version of ContentLocation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ContentLocationMeta {
    Mmap { offset_bytes: usize, size: usize },
    Heap { size: usize },
    Hybrid { mmap_offset_bytes: usize, mmap_size: usize, heap_size: usize },
}

impl LayerMetadata {
    /// Create new layer metadata
    pub fn new(
        layer_id: LayerId,
        name: String,
        frequency: Frequency,
        keywords: Vec<String>,
    ) -> Self {
        Self {
            layer_id,
            name,
            frequency: frequency.hz(),
            content_location: ContentLocationMeta::Heap { size: 0 },
            keywords,
            synesthetic_map: HashMap::new(),
            parent: None,
            children: Vec::new(),
            last_accessed: 0,
            access_count: 0,
        }
    }
    
    /// Update access statistics
    pub fn record_access(&mut self) {
        self.access_count += 1;
        self.last_accessed = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
    }
    
    /// Add synesthetic association
    pub fn add_synesthetic_association(&mut self, keyword: String, associations: Vec<String>) {
        self.synesthetic_map.insert(keyword, associations);
    }
    
    /// Get frequency as Frequency type
    pub fn frequency(&self) -> Frequency {
        Frequency::new(self.frequency)
    }
    
    /// Check if layer matches keyword (literal or synesthetic)
    pub fn matches_keyword(&self, query_keyword: &str) -> f32 {
        let query_lower = query_keyword.to_lowercase();
        
        // Direct keyword match
        for keyword in &self.keywords {
            if keyword.to_lowercase().contains(&query_lower) {
                return 1.0; // Perfect match
            }
        }
        
        // Synesthetic association match
        let mut best_score = 0.0;
        for (keyword, associations) in &self.synesthetic_map {
            if keyword.to_lowercase().contains(&query_lower) {
                best_score = best_score.max(0.8); // Strong synesthetic match
            }
            
            for association in associations {
                if association.to_lowercase().contains(&query_lower) {
                    best_score = best_score.max(0.6); // Moderate synesthetic match
                }
            }
        }
        
        best_score
    }
}

impl MmapRegion {
    /// Create new memory region
    pub fn new(region_id: u8, base_offset: MmapOffset, total_size: usize) -> Self {
        Self {
            region_id,
            base_offset,
            total_size,
            layers: HashMap::new(),
            free_space: total_size,
        }
    }
    
    /// Add layer to region
    pub fn add_layer(&mut self, metadata: LayerMetadata) -> Result<()> {
        let layer_size = match &metadata.content_location {
            ContentLocationMeta::Mmap { size, .. } => *size,
            ContentLocationMeta::Heap { size } => *size,
            ContentLocationMeta::Hybrid { mmap_size, heap_size, .. } => mmap_size + heap_size,
        };
        
        if layer_size > self.free_space {
            return Err(ConsciousnessError::MemoryError(
                "Insufficient space in region".to_string()
            ));
        }
        
        self.free_space -= layer_size;
        self.layers.insert(metadata.layer_id, metadata);
        
        Ok(())
    }
    
    /// Remove layer from region
    pub fn remove_layer(&mut self, layer_id: LayerId) -> Result<LayerMetadata> {
        let metadata = self.layers.remove(&layer_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Layer not found".to_string()))?;
        
        let layer_size = match &metadata.content_location {
            ContentLocationMeta::Mmap { size, .. } => *size,
            ContentLocationMeta::Heap { size } => *size,
            ContentLocationMeta::Hybrid { mmap_size, heap_size, .. } => mmap_size + heap_size,
        };
        
        self.free_space += layer_size;
        
        Ok(metadata)
    }
    
    /// Get layer metadata
    pub fn get_layer(&mut self, layer_id: LayerId) -> Option<&mut LayerMetadata> {
        if let Some(metadata) = self.layers.get_mut(&layer_id) {
            metadata.record_access();
            Some(metadata)
        } else {
            None
        }
    }
    
    /// Find layers matching keywords
    pub fn find_matching_layers(&mut self, keywords: &[String]) -> Vec<(LayerId, f32)> {
        let mut matches = Vec::new();
        
        for (layer_id, metadata) in &mut self.layers {
            let mut total_score = 0.0;
            
            for keyword in keywords {
                let score = metadata.matches_keyword(keyword);
                total_score += score;
            }
            
            if total_score > 0.0 {
                metadata.record_access();
                matches.push(*layer_id, total_score / keywords.len() as f32);
            }
        }
        
        // Sort by score (highest first)
        matches.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        
        matches
    }
    
    /// Get region statistics
    pub fn stats(&self) -> RegionStats {
        let total_layers = self.layers.len();
        let used_space = self.total_size - self.free_space;
        let utilization = used_space as f32 / self.total_size as f32;
        
        let total_accesses: u64 = self.layers.values()
            .map(|m| m.access_count)
            .sum();
        
        RegionStats {
            region_id: self.region_id,
            total_size: self.total_size,
            used_space,
            free_space: self.free_space,
            utilization,
            layer_count: total_layers,
            total_accesses,
        }
    }
}

/// Region usage statistics
#[derive(Debug, Clone)]
pub struct RegionStats {
    pub region_id: u8,
    pub total_size: usize,
    pub used_space: usize,
    pub free_space: usize,
    pub utilization: f32,
    pub layer_count: usize,
    pub total_accesses: u64,
}