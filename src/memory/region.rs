//! Memory region management for consciousness layers
//!
//! Handles individual MMAP regions with content location tracking
//! and hybrid storage for static + dynamic content.

use crate::{Result, ConsciousnessError, LayerId, Frequency};
use memmap2::{Mmap, MmapOptions};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

/// Location of layer content in memory
#[derive(Debug, Clone)]
pub enum ContentLocation {
    /// Content stored in memory-mapped file
    Mmap { 
        offset: usize, 
        size: usize,
        region_id: u32,
    },
    /// Content stored in heap memory (temporary)
    Heap { 
        data: Vec<u8>,
        created_at: std::time::SystemTime,
    },
    /// Hybrid: base content in MMAP + overlay in heap
    Hybrid { 
        mmap_base: usize, 
        mmap_size: usize,
        heap_overlay: Vec<u8>,
        region_id: u32,
    },
}

impl ContentLocation {
    /// Get the total size of content
    pub fn size(&self) -> usize {
        match self {
            ContentLocation::Mmap { size, .. } => *size,
            ContentLocation::Heap { data, .. } => data.len(),
            ContentLocation::Hybrid { mmap_size, heap_overlay, .. } => {
                mmap_size + heap_overlay.len()
            }
        }
    }
    
    /// Check if content is in permanent storage (MMAP)
    pub fn is_permanent(&self) -> bool {
        matches!(self, ContentLocation::Mmap { .. } | ContentLocation::Hybrid { .. })
    }
    
    /// Check if content is temporary (heap only)
    pub fn is_temporary(&self) -> bool {
        matches!(self, ContentLocation::Heap { .. })
    }
}

/// Memory-mapped region for consciousness layers
#[derive(Debug)]
pub struct MmapRegion {
    pub region_id: u32,
    pub dimension_id: crate::DimensionId,
    pub file_path: std::path::PathBuf,
    pub mmap: Mmap,
    pub metadata: RegionMetadata,
}

impl MmapRegion {
    /// Create new MMAP region from file
    pub fn from_file<P: AsRef<Path>>(
        region_id: u32,
        dimension_id: crate::DimensionId,
        file_path: P,
    ) -> Result<Self> {
        let file_path = file_path.as_ref().to_path_buf();
        
        let file = std::fs::File::open(&file_path)
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to open file {:?}: {}", file_path, e)
            ))?;
        
        let mmap = unsafe {
            MmapOptions::new()
                .map(&file)
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to mmap file {:?}: {}", file_path, e)
                ))?
        };
        
        // Parse metadata from file header or separate metadata file
        let metadata = Self::parse_metadata(&mmap)?;
        
        Ok(Self {
            region_id,
            dimension_id,
            file_path,
            mmap,
            metadata,
        })
    }
    
    /// Read content at specific offset and size
    pub fn read_content(&self, offset: usize, size: usize) -> Result<&[u8]> {
        if offset + size > self.mmap.len() {
            return Err(ConsciousnessError::MemoryError(
                format!("Read beyond region bounds: {}+{} > {}", 
                    offset, size, self.mmap.len())
            ));
        }
        
        Ok(&self.mmap[offset..offset + size])
    }
    
    /// Read content as UTF-8 string
    pub fn read_string(&self, offset: usize, size: usize) -> Result<String> {
        let bytes = self.read_content(offset, size)?;
        String::from_utf8(bytes.to_vec())
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Invalid UTF-8 content: {}", e)
            ))
    }
    
    /// Get layer information by layer ID
    pub fn get_layer_info(&self, layer_id: LayerId) -> Option<&LayerInfo> {
        self.metadata.layers.get(&layer_id)
    }
    
    /// List all layers in this region
    pub fn list_layers(&self) -> Vec<LayerId> {
        self.metadata.layers.keys().copied().collect()
    }
    
    /// Parse metadata from MMAP content
    fn parse_metadata(mmap: &Mmap) -> Result<RegionMetadata> {
        // Look for metadata header (first 1KB)
        if mmap.len() < 1024 {
            return Err(ConsciousnessError::MemoryError(
                "Region too small for metadata".to_string()
            ));
        }
        
        let header_bytes = &mmap[0..1024];
        
        // Try to find JSON metadata marker
        if let Some(json_start) = header_bytes.windows(4).position(|w| w == b"JSON") {
            let json_bytes = &header_bytes[json_start + 4..];
            if let Some(json_end) = json_bytes.iter().position(|&b| b == 0) {
                let json_str = std::str::from_utf8(&json_bytes[..json_end])
                    .map_err(|e| ConsciousnessError::MemoryError(
                        format!("Invalid metadata UTF-8: {}", e)
                    ))?;
                
                return serde_json::from_str(json_str)
                    .map_err(|e| ConsciousnessError::MemoryError(
                        format!("Invalid metadata JSON: {}", e)
                    ));
            }
        }
        
        // Fallback: create default metadata
        Ok(RegionMetadata::default())
    }
}

/// Metadata for an MMAP region
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RegionMetadata {
    pub version: u32,
    pub created_at: u64, // Unix timestamp
    pub dimension_name: String,
    pub total_size: usize,
    pub content_offset: usize, // Where actual content starts (after metadata)
    pub layers: HashMap<LayerId, LayerInfo>,
}

impl Default for RegionMetadata {
    fn default() -> Self {
        Self {
            version: 1,
            created_at: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            dimension_name: "unknown".to_string(),
            total_size: 0,
            content_offset: 1024, // Skip first 1KB for metadata
            layers: HashMap::new(),
        }
    }
}

/// Information about a layer within a region
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerInfo {
    pub layer_id: LayerId,
    pub name: String,
    pub frequency: f32,
    pub depth: u8,
    pub offset: usize,
    pub size: usize,
    pub keywords: Vec<String>,
    pub parent: Option<LayerId>,
    pub children: Vec<LayerId>,
    pub last_accessed: u64,
    pub access_count: u64,
}

impl LayerInfo {
    /// Create new layer info
    pub fn new(
        layer_id: LayerId,
        name: String,
        frequency: Frequency,
        depth: u8,
        offset: usize,
        size: usize,
    ) -> Self {
        Self {
            layer_id,
            name,
            frequency: frequency.hz(),
            depth,
            offset,
            size,
            keywords: Vec::new(),
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
    
    /// Get frequency as Frequency type
    pub fn frequency(&self) -> Frequency {
        Frequency::new(self.frequency)
    }
}

/// Builder for creating MMAP regions
pub struct RegionBuilder {
    dimension_id: crate::DimensionId,
    dimension_name: String,
    layers: Vec<(LayerInfo, Vec<u8>)>,
    total_size: usize,
}

impl RegionBuilder {
    /// Create new region builder
    pub fn new(dimension_id: crate::DimensionId, dimension_name: String) -> Self {
        Self {
            dimension_id,
            dimension_name,
            layers: Vec::new(),
            total_size: 1024, // Reserve space for metadata
        }
    }
    
    /// Add a layer to the region
    pub fn add_layer(
        &mut self,
        layer_id: LayerId,
        name: String,
        frequency: Frequency,
        depth: u8,
        content: Vec<u8>,
        keywords: Vec<String>,
    ) -> &mut Self {
        let offset = self.total_size;
        let size = content.len();
        
        let mut layer_info = LayerInfo::new(layer_id, name, frequency, depth, offset, size);
        layer_info.keywords = keywords;
        
        self.layers.push((layer_info, content));
        self.total_size += size;
        
        self
    }
    
    /// Build the region and write to file
    pub fn build_to_file<P: AsRef<Path>>(
        self,
        file_path: P,
        region_id: u32,
    ) -> Result<MmapRegion> {
        let file_path = file_path.as_ref();
        
        // Create metadata
        let mut metadata = RegionMetadata {
            version: 1,
            created_at: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            dimension_name: self.dimension_name.clone(),
            total_size: self.total_size,
            content_offset: 1024,
            layers: HashMap::new(),
        };
        
        // Add layer info to metadata
        for (layer_info, _) in &self.layers {
            metadata.layers.insert(layer_info.layer_id, layer_info.clone());
        }
        
        // Create file and write content
        let mut file_content = Vec::with_capacity(self.total_size);
        
        // Write metadata header (1KB)
        let metadata_json = serde_json::to_string(&metadata)
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to serialize metadata: {}", e)
            ))?;
        
        file_content.extend_from_slice(b"JSON");
        file_content.extend_from_slice(metadata_json.as_bytes());
        
        // Pad to 1KB
        while file_content.len() < 1024 {
            file_content.push(0);
        }
        
        // Write layer content
        for (_, content) in &self.layers {
            file_content.extend_from_slice(content);
        }
        
        // Write to file
        std::fs::write(file_path, &file_content)
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to write region file: {}", e)
            ))?;
        
        // Create MMAP region
        MmapRegion::from_file(region_id, self.dimension_id, file_path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;
    
    #[test]
    fn test_region_builder() {
        let temp_file = NamedTempFile::new().unwrap();
        let dimension_id = crate::DimensionId(1);
        let layer_id = LayerId { dimension: dimension_id, layer: 0 };
        
        let mut builder = RegionBuilder::new(dimension_id, "test_dimension".to_string());
        builder.add_layer(
            layer_id,
            "test_layer".to_string(),
            Frequency::new(1.0),
            0,
            b"test content".to_vec(),
            vec!["test".to_string(), "layer".to_string()],
        );
        
        let region = builder.build_to_file(temp_file.path(), 0).unwrap();
        
        assert_eq!(region.dimension_id, dimension_id);
        assert_eq!(region.metadata.dimension_name, "test_dimension");
        assert_eq!(region.metadata.layers.len(), 1);
        
        let layer_info = region.get_layer_info(layer_id).unwrap();
        assert_eq!(layer_info.name, "test_layer");
        assert_eq!(layer_info.frequency, 1.0);
        assert_eq!(layer_info.keywords, vec!["test", "layer"]);
        
        let content = region.read_string(layer_info.offset, layer_info.size).unwrap();
        assert_eq!(content, "test content");
    }
}