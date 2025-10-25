//! Memory region management for consciousness layers
//!
//! Handles individual MMAP regions with content location tracking
//! and hybrid storage for static + dynamic content.
//!
//! # Design Pattern: Hybrid Storage
//!
//! Static content (crystallized knowledge) → MMAP (fast, persistent)
//! Dynamic content (learning) → Heap (flexible, temporary)
//! Combined → Hybrid (base in MMAP, updates in heap overlay)

use crate::{Result, ConsciousnessError, LayerId, Frequency};
use memmap2::{Mmap, MmapOptions};
use serde::{Deserialize, Serialize};  // For JSON metadata serialization
use std::collections::HashMap;
use std::path::Path;

/// Location of layer content in memory
///
/// This is an enum with data - each variant carries different information
/// This is more type-safe than a struct with optional fields
/// Impossible to have "region_id when in Heap mode" bugs
///
/// Clone is derived because we need to duplicate location info
/// Debug is derived for debugging output
#[derive(Debug, Clone)]
pub enum ContentLocation {
    /// Content stored in memory-mapped file (zero-copy, persistent)
    ///
    /// This is the fast path - no allocation, just pointer arithmetic
    Mmap { 
        offset: usize,  // Byte offset into MMAP region
        size: usize,    // Size in bytes
        region_id: u32, // Which region contains this content
    },
    /// Content stored in heap memory (temporary, for learning)
    ///
    /// Used for proto-dimensions before crystallization
    /// Vec<u8> is heap-allocated, growable byte array
    Heap { 
        data: Vec<u8>,  // Owned data on heap
        created_at: std::time::SystemTime,  // For aging/eviction
    },
    /// Hybrid: base content in MMAP + overlay in heap
    ///
    /// This enables incremental updates without rewriting MMAP files
    /// Read base from MMAP, apply overlay from heap
    Hybrid { 
        mmap_base: usize,       // Offset of base content in MMAP
        mmap_size: usize,       // Size of base content
        heap_overlay: Vec<u8>,  // Additional/updated content
        region_id: u32,
    },
}

impl ContentLocation {
    /// Get the total size of content
    ///
    /// match is exhaustive - compiler ensures we handle all variants
    /// If we add a new variant, this will fail to compile until we handle it
    pub fn size(&self) -> usize {
        match self {
            // .. syntax ignores other fields - we only need size
            // *size dereferences the &usize to usize
            ContentLocation::Mmap { size, .. } => *size,
            // Vec::len() is O(1) - just returns stored length
            ContentLocation::Heap { data, .. } => data.len(),
            // Hybrid size is sum of base + overlay
            ContentLocation::Hybrid { mmap_size, heap_overlay, .. } => {
                mmap_size + heap_overlay.len()
            }
        }
    }
    
    /// Check if content is in permanent storage (MMAP)
    ///
    /// matches! macro is like match but returns bool
    /// More concise than match { ... => true, _ => false }
    pub fn is_permanent(&self) -> bool {
        matches!(self, ContentLocation::Mmap { .. } | ContentLocation::Hybrid { .. })
    }
    
    /// Check if content is temporary (heap only)
    pub fn is_temporary(&self) -> bool {
        matches!(self, ContentLocation::Heap { .. })
    }
}

/// Memory-mapped region for consciousness layers
///
/// Represents one dimension's data mapped into memory
/// Mmap (not MmapMut) because regions are read-only after creation
/// This enables lock-free concurrent reads - multiple threads can read safely
#[derive(Debug)]
pub struct MmapRegion {
    pub region_id: u32,
    pub dimension_id: crate::DimensionId,
    pub file_path: std::path::PathBuf,  // Keep path for debugging/logging
    pub mmap: Mmap,  // Immutable MMAP - read-only, thread-safe
    pub metadata: RegionMetadata,  // Parsed from file header
}

impl MmapRegion {
    /// Create new MMAP region from file
    ///
    /// Generic over P: AsRef<Path> - accepts &str, String, Path, PathBuf
    /// This is Rust's way of "function overloading" - one function, many types
    pub fn from_file<P: AsRef<Path>>(
        region_id: u32,
        dimension_id: crate::DimensionId,
        file_path: P,
    ) -> Result<Self> {
        // as_ref() converts P to &Path, then to_path_buf() makes it owned
        // We need owned PathBuf because we're storing it in the struct
        let file_path = file_path.as_ref().to_path_buf();
        
        // Open file - this gets a file descriptor from the OS
        let file = std::fs::File::open(&file_path)
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to open file {:?}: {}", file_path, e)
            ))?;
        
        // unsafe block: MMAP is inherently unsafe
        // The file could be modified by another process while we're reading
        // We accept this risk for performance - it's the MMAP trade-off
        let mmap = unsafe {
            MmapOptions::new()
                .map(&file)  // Map file into virtual memory
                .map_err(|e| ConsciousnessError::MemoryError(
                    format!("Failed to mmap file {:?}: {}", file_path, e)
                ))?
        };
        
        // Parse metadata from file header
        // This reads the first 1KB to get layer information
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
    ///
    /// Returns &[u8] (borrowed slice) - zero-copy, no allocation
    /// The slice borrows from self.mmap, so it can't outlive this region
    /// This is Rust's borrow checker preventing use-after-free bugs
    pub fn read_content(&self, offset: usize, size: usize) -> Result<&[u8]> {
        // Bounds check prevents buffer overruns
        // This is why safe Rust can't have buffer overflow vulnerabilities
        if offset + size > self.mmap.len() {
            return Err(ConsciousnessError::OutOfBounds {
                offset,
                size,
                region_size: self.mmap.len(),
            });
        }
        
        // Slice creation is zero-cost - just creates a fat pointer (ptr + len)
        // No copying, no allocation - this is why MMAP is fast
        Ok(&self.mmap[offset..offset + size])
    }
    
    /// Read content as UTF-8 string
    ///
    /// Note: This DOES allocate (String) because we need owned data
    /// If you just need to read, use read_content() and from_utf8() for &str
    pub fn read_string(&self, offset: usize, size: usize) -> Result<String> {
        let bytes = self.read_content(offset, size)?;
        // to_vec() copies bytes to heap - necessary because String needs owned data
        // from_utf8 validates UTF-8 and creates String
        // This is the one place we copy - can't avoid it for String ownership
        String::from_utf8(bytes.to_vec())
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Invalid UTF-8 content: {}", e)
            ))
    }
    
    /// Get layer information by layer ID
    pub fn get_layer_info(&self, layer_id: LayerId) -> Option<&LayerInfo> {
        self.metadata.layers.iter().find(|info| info.layer_id == layer_id)
    }
    
    /// List all layers in this region
    pub fn list_layers(&self) -> Vec<LayerId> {
        self.metadata.layers.iter().map(|info| info.layer_id).collect()
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
    // Use Vec instead of HashMap for JSON serialization compatibility
    // LayerId as HashMap key doesn't serialize well to JSON (keys must be strings)
    pub layers: Vec<LayerInfo>,
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
            layers: Vec::new(),
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
            layers: Vec::new(),
        };
        
        // Add layer info to metadata
        for (layer_info, _) in &self.layers {
            metadata.layers.push(layer_info.clone());
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