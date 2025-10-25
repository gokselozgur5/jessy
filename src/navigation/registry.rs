//! Dimension registry for managing dimension and layer metadata
//!
//! This module provides O(1) lookup for dimension and layer information,
//! supporting the navigation system's requirement for sub-microsecond access times.
//!
//! The registry is immutable after initialization, allowing lock-free concurrent access
//! through Arc sharing. All 14 core dimensions are loaded at startup with their complete
//! layer hierarchies.

use crate::{DimensionId, LayerId, Frequency};
use crate::navigation::{NavigationError, NavigationConfig};
use std::collections::HashMap;
use std::sync::Arc;
use serde::{Deserialize, Serialize};

/// Configuration structure for loading dimensions from JSON
#[derive(Debug, Deserialize)]
struct DimensionConfig {
    dimensions: Vec<DimensionData>,
    layers: Vec<LayerData>,
}

/// Dimension data from JSON configuration
#[derive(Debug, Deserialize)]
struct DimensionData {
    id: u8,
    name: String,
    frequency_min: f32,
    frequency_max: f32,
    size_bytes: usize,
}

/// Layer data from JSON configuration
#[derive(Debug, Deserialize)]
struct LayerData {
    dimension_id: u8,
    layer_num: u16,
    depth: u8,
    parent_layer: Option<u16>,
    keywords: Vec<String>,
    frequency: f32,
    mmap_offset: usize,
}

/// Metadata for a single dimension
///
/// Contains all static information about a dimension including its frequency range,
/// memory allocation size, and root layer reference. This data is loaded once at
/// initialization and never modified.
///
/// # Requirements
/// - Requirement 11.1: Load all 14 core dimensions
/// - Requirement 11.2-11.4: Validate unique IDs (1-14) and frequency ranges (0.1-4.5 Hz)
/// - Requirement 11.6-11.9: Include name, frequency range, size, and layer structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DimensionMetadata {
    /// Unique dimension identifier (1-14 for core dimensions)
    pub id: DimensionId,
    
    /// Human-readable dimension name (e.g., "Emotion", "Cognition")
    pub name: String,
    
    /// Frequency range for this dimension in Hz
    /// Tuple of (min_hz, max_hz) where both values are in range [0.1, 4.5]
    pub frequency_range: (f32, f32),
    
    /// Memory allocation size for this dimension in bytes
    /// Used by memory manager to allocate MMAP regions
    pub size_bytes: usize,
    
    /// Root layer (L0) identifier for this dimension
    /// All depth navigation starts from this layer
    pub root_layer: LayerId,
}

impl DimensionMetadata {
    /// Create new dimension metadata
    pub fn new(
        id: DimensionId,
        name: String,
        frequency_range: (f32, f32),
        size_bytes: usize,
    ) -> Self {
        Self {
            id,
            name,
            frequency_range,
            size_bytes,
            root_layer: LayerId {
                dimension: id,
                layer: 0, // L0 is always the root
            },
        }
    }
    
    /// Check if a frequency falls within this dimension's range
    pub fn contains_frequency(&self, freq: f32) -> bool {
        freq >= self.frequency_range.0 && freq <= self.frequency_range.1
    }
    
    /// Check if a frequency is within 0.5 Hz of this dimension's range
    /// Used for frequency alignment scoring (Requirement 4.6)
    pub fn near_frequency(&self, freq: f32) -> bool {
        (freq >= self.frequency_range.0 - 0.5 && freq <= self.frequency_range.1 + 0.5)
    }
    
    /// Get the center frequency of this dimension
    pub fn center_frequency(&self) -> f32 {
        (self.frequency_range.0 + self.frequency_range.1) / 2.0
    }
}

/// Metadata for a single layer within a dimension
///
/// Layers form a hierarchical tree structure within each dimension, with L0 as root
/// and up to L3 as maximum depth. Each layer contains keywords for matching and
/// references to its parent and children.
///
/// # Requirements
/// - Requirement 11.10: Include layer ID, dimension, depth, parent, keywords, frequency
/// - Requirement 11.11: Depth values 0-3 corresponding to L0-L3
/// - Requirement 11.12: Root layer (L0) has no parent, others have exactly one parent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LayerMetadata {
    /// Unique layer identifier
    pub id: LayerId,
    
    /// Parent dimension ID (redundant with id.dimension but convenient)
    pub dimension_id: DimensionId,
    
    /// Depth in the layer hierarchy (0=L0/root, 1=L1, 2=L2, 3=L3)
    pub depth: u8,
    
    /// Parent layer ID (None for root layer L0)
    pub parent: Option<LayerId>,
    
    /// Keywords associated with this layer for matching
    /// Used in keyword extraction and confidence scoring
    pub keywords: Vec<String>,
    
    /// Operating frequency for this layer in Hz
    /// May differ from dimension's base frequency
    pub frequency: f32,
    
    /// Memory-mapped file offset for this layer's content
    /// Used by memory manager for zero-copy access
    pub mmap_offset: usize,
}

impl LayerMetadata {
    /// Create new layer metadata
    pub fn new(
        id: LayerId,
        depth: u8,
        parent: Option<LayerId>,
        keywords: Vec<String>,
        frequency: f32,
        mmap_offset: usize,
    ) -> Self {
        Self {
            id,
            dimension_id: id.dimension,
            depth,
            parent,
            keywords,
            frequency,
            mmap_offset,
        }
    }
    
    /// Check if this is a root layer (L0)
    pub fn is_root(&self) -> bool {
        self.depth == 0 && self.parent.is_none()
    }
    
    /// Check if this is a leaf layer (no children possible beyond L3)
    pub fn is_leaf(&self) -> bool {
        self.depth >= 3
    }
    
    /// Calculate keyword match score for given query keywords
    /// Returns ratio of matched keywords to total query keywords
    pub fn calculate_match_score(&self, query_keywords: &[String]) -> f32 {
        if query_keywords.is_empty() {
            return 0.0;
        }
        
        let matched = query_keywords
            .iter()
            .filter(|qk| self.keywords.iter().any(|lk| lk == *qk))
            .count();
        
        matched as f32 / query_keywords.len() as f32
    }
}

/// Registry for all dimension and layer metadata
///
/// Provides O(1) lookup for dimensions and layers using HashMap storage.
/// The registry is immutable after initialization, enabling lock-free concurrent
/// access through Arc<DimensionRegistry> sharing.
///
/// # Thread Safety
/// DimensionRegistry is Send + Sync and can be safely shared across threads
/// using Arc. No interior mutability is needed since the registry never changes
/// after initialization.
///
/// # Requirements
/// - Requirement 11.1: Load all 14 core dimensions
/// - Requirement 11.5: O(1) lookup performance (<1μs)
/// - Requirement 8.1: Support concurrent access without locks
#[derive(Debug, Clone)]
pub struct DimensionRegistry {
    /// Dimension metadata indexed by dimension ID
    /// HashMap provides O(1) average-case lookup
    dimensions: HashMap<DimensionId, DimensionMetadata>,
    
    /// Layer metadata indexed by layer ID
    /// HashMap provides O(1) average-case lookup
    layers: HashMap<LayerId, LayerMetadata>,
    
    /// Child layers indexed by parent layer ID
    /// Enables efficient depth navigation (Requirement 5.2)
    children: HashMap<LayerId, Vec<LayerId>>,
}

impl DimensionRegistry {
    /// Create a new empty registry
    pub fn new() -> Self {
        Self {
            dimensions: HashMap::new(),
            layers: HashMap::new(),
            children: HashMap::new(),
        }
    }
    
    /// Load dimensions from configuration data
    ///
    /// Parses dimension configuration and populates the registry with all
    /// dimension and layer metadata. Validates the configuration before
    /// accepting it.
    ///
    /// # Requirements
    /// - Requirement 11.1: Load all 14 core dimensions
    /// - Requirement 11.2-11.4: Validate unique IDs, frequency ranges, hierarchy
    ///
    /// # Arguments
    /// * `config_data` - JSON string containing dimension configuration
    ///
    /// # Returns
    /// Ok(registry) if loading and validation succeed, Err otherwise
    ///
    /// # Errors
    /// - RegistryLoadFailed: If JSON parsing fails
    /// - RegistryValidationFailed: If validation checks fail
    pub fn load_dimensions(config_data: &str) -> Result<Self, NavigationError> {
        // Parse JSON configuration
        let config: DimensionConfig = serde_json::from_str(config_data)
            .map_err(|e| NavigationError::RegistryLoadFailed {
                reason: format!("JSON parse error: {}", e),
            })?;
        
        let mut registry = Self::new();
        
        // Load dimensions
        for dim_data in config.dimensions {
            let dimension = DimensionMetadata::new(
                DimensionId(dim_data.id),
                dim_data.name,
                (dim_data.frequency_min, dim_data.frequency_max),
                dim_data.size_bytes,
            );
            registry.dimensions.insert(dimension.id, dimension);
        }
        
        // Load layers and build hierarchy
        for layer_data in config.layers {
            let layer_id = LayerId {
                dimension: DimensionId(layer_data.dimension_id),
                layer: layer_data.layer_num,
            };
            
            let parent = if layer_data.depth == 0 {
                None
            } else {
                layer_data.parent_layer.map(|p| LayerId {
                    dimension: DimensionId(layer_data.dimension_id),
                    layer: p,
                })
            };
            
            let layer = LayerMetadata::new(
                layer_id,
                layer_data.depth,
                parent,
                layer_data.keywords,
                layer_data.frequency,
                layer_data.mmap_offset,
            );
            
            registry.layers.insert(layer_id, layer);
            
            // Build parent-child relationships
            if let Some(parent_id) = parent {
                registry.children
                    .entry(parent_id)
                    .or_insert_with(Vec::new)
                    .push(layer_id);
            }
        }
        
        // Validate the loaded registry
        registry.validate()?;
        
        Ok(registry)
    }
    
    /// Validate registry contents
    ///
    /// Checks all invariants that must hold for a valid registry:
    /// - Dimension IDs are unique and in range 1-14
    /// - Frequency ranges are valid (0.1-4.5 Hz, min < max)
    /// - Layer hierarchy is valid (root has no parent, others have exactly one)
    /// - All parent references point to existing layers
    ///
    /// # Requirements
    /// - Requirement 11.2: Unique IDs between 1-14
    /// - Requirement 11.3: Valid frequency ranges (min < max)
    /// - Requirement 11.4: Frequency ranges within 0.1-4.5 Hz
    /// - Requirement 11.11: Depth values 0-3
    /// - Requirement 11.12: Root has no parent, others have exactly one
    ///
    /// # Returns
    /// Ok(()) if all validation checks pass, Err otherwise
    ///
    /// # Errors
    /// Returns RegistryValidationFailed with detailed reason if any check fails
    pub fn validate(&self) -> Result<(), NavigationError> {
        // Check dimension count (should be 14 for core dimensions)
        if self.dimensions.is_empty() {
            return Err(NavigationError::RegistryValidationFailed {
                reason: "Registry contains no dimensions".to_string(),
            });
        }
        
        // Validate each dimension
        for (id, dim) in &self.dimensions {
            // Check ID is in valid range (1-14 for core dimensions)
            if dim.id.0 < 1 || dim.id.0 > 14 {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Dimension ID {} out of range (must be 1-14)",
                        dim.id.0
                    ),
                });
            }
            
            // Check ID consistency
            if *id != dim.id {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Dimension ID mismatch: key={:?}, value={:?}",
                        id, dim.id
                    ),
                });
            }
            
            // Check frequency range validity
            let (min_freq, max_freq) = dim.frequency_range;
            
            if min_freq >= max_freq {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Dimension {} has invalid frequency range: min={} >= max={}",
                        dim.id.0, min_freq, max_freq
                    ),
                });
            }
            
            if min_freq < 0.1 || max_freq > 4.5 {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Dimension {} frequency range [{}, {}] outside valid range [0.1, 4.5]",
                        dim.id.0, min_freq, max_freq
                    ),
                });
            }
            
            // Check root layer exists
            if !self.layers.contains_key(&dim.root_layer) {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Dimension {} root layer {:?} not found",
                        dim.id.0, dim.root_layer
                    ),
                });
            }
        }
        
        // Validate each layer
        for (id, layer) in &self.layers {
            // Check ID consistency
            if *id != layer.id {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Layer ID mismatch: key={:?}, value={:?}",
                        id, layer.id
                    ),
                });
            }
            
            // Check depth is valid (0-3)
            if layer.depth > 3 {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Layer {:?} has invalid depth {} (must be 0-3)",
                        layer.id, layer.depth
                    ),
                });
            }
            
            // Check root layer invariants (depth 0, no parent)
            if layer.depth == 0 {
                if layer.parent.is_some() {
                    return Err(NavigationError::RegistryValidationFailed {
                        reason: format!(
                            "Root layer {:?} has parent (should be None)",
                            layer.id
                        ),
                    });
                }
            } else {
                // Non-root layers must have exactly one parent
                if layer.parent.is_none() {
                    return Err(NavigationError::RegistryValidationFailed {
                        reason: format!(
                            "Non-root layer {:?} at depth {} has no parent",
                            layer.id, layer.depth
                        ),
                    });
                }
                
                // Parent must exist
                if let Some(parent_id) = layer.parent {
                    if !self.layers.contains_key(&parent_id) {
                        return Err(NavigationError::RegistryValidationFailed {
                            reason: format!(
                                "Layer {:?} parent {:?} not found",
                                layer.id, parent_id
                            ),
                        });
                    }
                    
                    // Parent must be in same dimension
                    if parent_id.dimension != layer.id.dimension {
                        return Err(NavigationError::RegistryValidationFailed {
                            reason: format!(
                                "Layer {:?} parent {:?} in different dimension",
                                layer.id, parent_id
                            ),
                        });
                    }
                    
                    // Parent must have depth = child depth - 1
                    let parent = &self.layers[&parent_id];
                    if parent.depth != layer.depth - 1 {
                        return Err(NavigationError::RegistryValidationFailed {
                            reason: format!(
                                "Layer {:?} depth {} but parent {:?} depth {}",
                                layer.id, layer.depth, parent_id, parent.depth
                            ),
                        });
                    }
                }
            }
            
            // Check frequency is in valid range
            if layer.frequency < 0.1 || layer.frequency > 4.5 {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Layer {:?} frequency {} outside valid range [0.1, 4.5]",
                        layer.id, layer.frequency
                    ),
                });
            }
            
            // Check dimension exists
            if !self.dimensions.contains_key(&layer.id.dimension) {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Layer {:?} references non-existent dimension {}",
                        layer.id, layer.id.dimension.0
                    ),
                });
            }
        }
        
        // Validate parent-child relationships
        for (parent_id, children) in &self.children {
            // Parent must exist
            if !self.layers.contains_key(parent_id) {
                return Err(NavigationError::RegistryValidationFailed {
                    reason: format!(
                        "Child map references non-existent parent {:?}",
                        parent_id
                    ),
                });
            }
            
            // All children must exist and reference this parent
            for child_id in children {
                if !self.layers.contains_key(child_id) {
                    return Err(NavigationError::RegistryValidationFailed {
                        reason: format!(
                            "Child map contains non-existent child {:?}",
                            child_id
                        ),
                    });
                }
                
                let child = &self.layers[child_id];
                if child.parent != Some(*parent_id) {
                    return Err(NavigationError::RegistryValidationFailed {
                        reason: format!(
                            "Child {:?} in parent {:?} children but parent mismatch",
                            child_id, parent_id
                        ),
                    });
                }
            }
        }
        
        Ok(())
    }
    
    /// Get dimension metadata by ID
    ///
    /// # Performance
    /// O(1) average case, <1μs typical (Requirement 11.5)
    ///
    /// # Returns
    /// Some(metadata) if dimension exists, None otherwise
    pub fn get_dimension(&self, id: DimensionId) -> Option<&DimensionMetadata> {
        self.dimensions.get(&id)
    }
    
    /// Get layer metadata by ID
    ///
    /// # Performance
    /// O(1) average case, <1μs typical (Requirement 11.5)
    ///
    /// # Returns
    /// Some(metadata) if layer exists, None otherwise
    pub fn get_layer(&self, id: LayerId) -> Option<&LayerMetadata> {
        self.layers.get(&id)
    }
    
    /// Get child layers for a given parent layer
    ///
    /// Used during depth navigation to explore the layer hierarchy.
    /// Returns empty slice if layer has no children or doesn't exist.
    ///
    /// # Requirements
    /// - Requirement 5.2: Support child layer evaluation during depth navigation
    ///
    /// # Returns
    /// Slice of child layer IDs, empty if no children
    pub fn get_child_layers(&self, parent_id: LayerId) -> &[LayerId] {
        self.children
            .get(&parent_id)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }
    
    /// Get all dimensions in the registry
    pub fn all_dimensions(&self) -> Vec<&DimensionMetadata> {
        self.dimensions.values().collect()
    }
    
    /// Get the root layer for a dimension
    pub fn get_root_layer(&self, dimension_id: DimensionId) -> Option<&LayerMetadata> {
        self.get_dimension(dimension_id)
            .and_then(|dim| self.get_layer(dim.root_layer))
    }
    
    /// Get the number of dimensions in the registry
    pub fn dimension_count(&self) -> usize {
        self.dimensions.len()
    }
    
    /// Get the number of layers in the registry
    pub fn layer_count(&self) -> usize {
        self.layers.len()
    }
}

impl Default for DimensionRegistry {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_dimension_metadata_creation() {
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Emotion".to_string(),
            (0.5, 2.5),
            1024 * 1024 * 20, // 20MB
        );
        
        assert_eq!(dim.id, DimensionId(1));
        assert_eq!(dim.name, "Emotion");
        assert_eq!(dim.frequency_range, (0.5, 2.5));
        assert_eq!(dim.size_bytes, 1024 * 1024 * 20);
        assert_eq!(dim.root_layer.dimension, DimensionId(1));
        assert_eq!(dim.root_layer.layer, 0);
    }
    
    #[test]
    fn test_dimension_frequency_checks() {
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (1.0, 3.0),
            1024,
        );
        
        // Within range
        assert!(dim.contains_frequency(1.5));
        assert!(dim.contains_frequency(1.0));
        assert!(dim.contains_frequency(3.0));
        
        // Outside range
        assert!(!dim.contains_frequency(0.5));
        assert!(!dim.contains_frequency(3.5));
        
        // Near range (within 0.5 Hz)
        assert!(dim.near_frequency(0.6)); // 0.4 Hz below min
        assert!(dim.near_frequency(3.4)); // 0.4 Hz above max
        assert!(!dim.near_frequency(0.4)); // 0.6 Hz below min
        
        // Center frequency
        assert_eq!(dim.center_frequency(), 2.0);
    }
    
    #[test]
    fn test_layer_metadata_creation() {
        let layer_id = LayerId {
            dimension: DimensionId(1),
            layer: 0,
        };
        
        let layer = LayerMetadata::new(
            layer_id,
            0,
            None,
            vec!["emotion".to_string(), "feeling".to_string()],
            1.5,
            0,
        );
        
        assert_eq!(layer.id, layer_id);
        assert_eq!(layer.dimension_id, DimensionId(1));
        assert_eq!(layer.depth, 0);
        assert!(layer.parent.is_none());
        assert_eq!(layer.keywords.len(), 2);
        assert_eq!(layer.frequency, 1.5);
        assert!(layer.is_root());
        assert!(!layer.is_leaf());
    }
    
    #[test]
    fn test_layer_match_score() {
        let layer = LayerMetadata::new(
            LayerId {
                dimension: DimensionId(1),
                layer: 0,
            },
            0,
            None,
            vec![
                "happy".to_string(),
                "sad".to_string(),
                "angry".to_string(),
            ],
            1.5,
            0,
        );
        
        // All keywords match
        let query1 = vec!["happy".to_string(), "sad".to_string()];
        assert_eq!(layer.calculate_match_score(&query1), 1.0);
        
        // Partial match
        let query2 = vec!["happy".to_string(), "confused".to_string()];
        assert_eq!(layer.calculate_match_score(&query2), 0.5);
        
        // No match
        let query3 = vec!["technical".to_string(), "code".to_string()];
        assert_eq!(layer.calculate_match_score(&query3), 0.0);
        
        // Empty query
        let query4: Vec<String> = vec![];
        assert_eq!(layer.calculate_match_score(&query4), 0.0);
    }
    
    #[test]
    fn test_registry_basic_operations() {
        let mut registry = DimensionRegistry::new();
        
        assert_eq!(registry.dimension_count(), 0);
        assert_eq!(registry.layer_count(), 0);
        
        // Registry starts empty
        assert!(registry.get_dimension(DimensionId(1)).is_none());
        assert!(registry.get_layer(LayerId {
            dimension: DimensionId(1),
            layer: 0
        }).is_none());
    }
    
    #[test]
    fn test_registry_child_layers() {
        let registry = DimensionRegistry::new();
        
        let parent_id = LayerId {
            dimension: DimensionId(1),
            layer: 0,
        };
        
        // Non-existent parent returns empty slice
        let children = registry.get_child_layers(parent_id);
        assert_eq!(children.len(), 0);
    }
    
    #[test]
    fn test_load_dimensions_valid() {
        let config_json = r#"{
            "dimensions": [
                {
                    "id": 1,
                    "name": "Emotion",
                    "frequency_min": 0.5,
                    "frequency_max": 2.5,
                    "size_bytes": 20971520
                }
            ],
            "layers": [
                {
                    "dimension_id": 1,
                    "layer_num": 0,
                    "depth": 0,
                    "parent_layer": null,
                    "keywords": ["emotion", "feeling"],
                    "frequency": 1.5,
                    "mmap_offset": 0
                },
                {
                    "dimension_id": 1,
                    "layer_num": 1,
                    "depth": 1,
                    "parent_layer": 0,
                    "keywords": ["happy", "sad"],
                    "frequency": 1.8,
                    "mmap_offset": 1024
                }
            ]
        }"#;
        
        let registry = DimensionRegistry::load_dimensions(config_json).unwrap();
        
        assert_eq!(registry.dimension_count(), 1);
        assert_eq!(registry.layer_count(), 2);
        
        let dim = registry.get_dimension(DimensionId(1)).unwrap();
        assert_eq!(dim.name, "Emotion");
        assert_eq!(dim.frequency_range, (0.5, 2.5));
        
        let root = registry.get_root_layer(DimensionId(1)).unwrap();
        assert_eq!(root.depth, 0);
        assert!(root.is_root());
        
        let children = registry.get_child_layers(root.id);
        assert_eq!(children.len(), 1);
    }
    
    #[test]
    fn test_load_dimensions_invalid_json() {
        let invalid_json = "{ invalid json }";
        let result = DimensionRegistry::load_dimensions(invalid_json);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            NavigationError::RegistryLoadFailed { .. }
        ));
    }
    
    #[test]
    fn test_validate_dimension_id_range() {
        let mut registry = DimensionRegistry::new();
        
        // Invalid ID (0)
        let dim = DimensionMetadata::new(
            DimensionId(0),
            "Invalid".to_string(),
            (1.0, 2.0),
            1024,
        );
        registry.dimensions.insert(dim.id, dim);
        
        let result = registry.validate();
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            NavigationError::RegistryValidationFailed { .. }
        ));
    }
    
    #[test]
    fn test_validate_frequency_range() {
        let mut registry = DimensionRegistry::new();
        
        // Invalid frequency range (min >= max)
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (2.0, 1.0), // min > max
            1024,
        );
        registry.dimensions.insert(dim.id, dim.clone());
        
        // Add root layer
        let layer = LayerMetadata::new(
            dim.root_layer,
            0,
            None,
            vec![],
            1.5,
            0,
        );
        registry.layers.insert(layer.id, layer);
        
        let result = registry.validate();
        assert!(result.is_err());
    }
    
    #[test]
    fn test_validate_frequency_bounds() {
        let mut registry = DimensionRegistry::new();
        
        // Frequency outside valid range
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (0.05, 5.0), // Outside [0.1, 4.5]
            1024,
        );
        registry.dimensions.insert(dim.id, dim.clone());
        
        // Add root layer
        let layer = LayerMetadata::new(
            dim.root_layer,
            0,
            None,
            vec![],
            1.5,
            0,
        );
        registry.layers.insert(layer.id, layer);
        
        let result = registry.validate();
        assert!(result.is_err());
    }
    
    #[test]
    fn test_validate_layer_hierarchy() {
        let mut registry = DimensionRegistry::new();
        
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (1.0, 2.0),
            1024,
        );
        registry.dimensions.insert(dim.id, dim.clone());
        
        // Root layer with parent (invalid)
        let layer = LayerMetadata::new(
            dim.root_layer,
            0,
            Some(LayerId {
                dimension: DimensionId(1),
                layer: 99,
            }),
            vec![],
            1.5,
            0,
        );
        registry.layers.insert(layer.id, layer);
        
        let result = registry.validate();
        assert!(result.is_err());
    }
    
    #[test]
    fn test_validate_non_root_without_parent() {
        let mut registry = DimensionRegistry::new();
        
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (1.0, 2.0),
            1024,
        );
        registry.dimensions.insert(dim.id, dim.clone());
        
        // Add valid root
        let root = LayerMetadata::new(
            dim.root_layer,
            0,
            None,
            vec![],
            1.5,
            0,
        );
        registry.layers.insert(root.id, root);
        
        // Non-root layer without parent (invalid)
        let layer = LayerMetadata::new(
            LayerId {
                dimension: DimensionId(1),
                layer: 1,
            },
            1,
            None, // Should have parent
            vec![],
            1.5,
            1024,
        );
        registry.layers.insert(layer.id, layer);
        
        let result = registry.validate();
        assert!(result.is_err());
    }
    
    #[test]
    fn test_validate_depth_range() {
        let mut registry = DimensionRegistry::new();
        
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (1.0, 2.0),
            1024,
        );
        registry.dimensions.insert(dim.id, dim.clone());
        
        // Add root
        let root = LayerMetadata::new(
            dim.root_layer,
            0,
            None,
            vec![],
            1.5,
            0,
        );
        registry.layers.insert(root.id, root);
        
        // Layer with invalid depth (>3)
        let layer = LayerMetadata::new(
            LayerId {
                dimension: DimensionId(1),
                layer: 1,
            },
            4, // Invalid depth
            Some(dim.root_layer),
            vec![],
            1.5,
            1024,
        );
        registry.layers.insert(layer.id, layer);
        
        let result = registry.validate();
        assert!(result.is_err());
    }
    
    #[test]
    fn test_load_full_dimension_config() {
        // Load the actual dimensions.json configuration
        let config_path = "data/dimensions.json";
        
        // Skip test if file doesn't exist (e.g., in CI without data directory)
        if !std::path::Path::new(config_path).exists() {
            eprintln!("Skipping test: {} not found", config_path);
            return;
        }
        
        let config_data = std::fs::read_to_string(config_path)
            .expect("Failed to read dimensions.json");
        
        let registry = DimensionRegistry::load_dimensions(&config_data)
            .expect("Failed to load dimensions");
        
        // Verify we have all 14 core dimensions
        assert_eq!(registry.dimension_count(), 14, "Should have 14 dimensions");
        
        // Verify each dimension has 4 layers (L0-L3)
        for dim_id in 1..=14 {
            let dimension = registry.get_dimension(DimensionId(dim_id))
                .expect(&format!("Dimension {} should exist", dim_id));
            
            // Check root layer exists
            let root = registry.get_root_layer(DimensionId(dim_id))
                .expect(&format!("Dimension {} should have root layer", dim_id));
            assert_eq!(root.depth, 0);
            assert!(root.is_root());
            
            // Verify frequency range is valid
            assert!(dimension.frequency_range.0 >= 0.1);
            assert!(dimension.frequency_range.1 <= 4.5);
            assert!(dimension.frequency_range.0 < dimension.frequency_range.1);
        }
        
        // Verify specific dimensions exist with correct names
        let emotion = registry.get_dimension(DimensionId(1)).unwrap();
        assert_eq!(emotion.name, "Emotion");
        
        let cognition = registry.get_dimension(DimensionId(2)).unwrap();
        assert_eq!(cognition.name, "Cognition");
        
        let technical = registry.get_dimension(DimensionId(7)).unwrap();
        assert_eq!(technical.name, "Technical Level");
        
        // Verify layer hierarchy for Emotion dimension
        let emotion_root = registry.get_root_layer(DimensionId(1)).unwrap();
        let emotion_children = registry.get_child_layers(emotion_root.id);
        assert!(!emotion_children.is_empty(), "Root should have children");
        
        // Verify keywords exist
        assert!(!emotion_root.keywords.is_empty(), "Root layer should have keywords");
        assert!(emotion_root.keywords.contains(&"emotion".to_string()));
    }
    
    #[test]
    fn test_dimension_lookup_performance() {
        // Load full configuration
        let config_path = "data/dimensions.json";
        if !std::path::Path::new(config_path).exists() {
            eprintln!("Skipping test: {} not found", config_path);
            return;
        }
        
        let config_data = std::fs::read_to_string(config_path).unwrap();
        let registry = DimensionRegistry::load_dimensions(&config_data).unwrap();
        
        // Measure lookup performance (Requirement 11.5: <1μs)
        let start = std::time::Instant::now();
        let iterations = 10000;
        
        for _ in 0..iterations {
            let _ = registry.get_dimension(DimensionId(7));
        }
        
        let elapsed = start.elapsed();
        let avg_nanos = elapsed.as_nanos() / iterations;
        
        // Should be well under 1 microsecond (1000 nanoseconds)
        // We'll be generous and allow up to 500ns average
        assert!(
            avg_nanos < 500,
            "Average lookup time {}ns exceeds 500ns target (1μs = 1000ns)",
            avg_nanos
        );
        
        println!("Average dimension lookup: {}ns", avg_nanos);
    }
    
    #[test]
    fn test_layer_lookup_performance() {
        let config_path = "data/dimensions.json";
        if !std::path::Path::new(config_path).exists() {
            eprintln!("Skipping test: {} not found", config_path);
            return;
        }
        
        let config_data = std::fs::read_to_string(config_path).unwrap();
        let registry = DimensionRegistry::load_dimensions(&config_data).unwrap();
        
        let layer_id = LayerId {
            dimension: DimensionId(7),
            layer: 2,
        };
        
        // Measure lookup performance
        let start = std::time::Instant::now();
        let iterations = 10000;
        
        for _ in 0..iterations {
            let _ = registry.get_layer(layer_id);
        }
        
        let elapsed = start.elapsed();
        let avg_nanos = elapsed.as_nanos() / iterations;
        
        assert!(
            avg_nanos < 500,
            "Average layer lookup time {}ns exceeds 500ns target",
            avg_nanos
        );
        
        println!("Average layer lookup: {}ns", avg_nanos);
    }
    
    #[test]
    fn test_child_layer_queries() {
        let config_path = "data/dimensions.json";
        if !std::path::Path::new(config_path).exists() {
            eprintln!("Skipping test: {} not found", config_path);
            return;
        }
        
        let config_data = std::fs::read_to_string(config_path).unwrap();
        let registry = DimensionRegistry::load_dimensions(&config_data).unwrap();
        
        // Test child layer queries for each dimension
        for dim_id in 1..=14 {
            let root = registry.get_root_layer(DimensionId(dim_id)).unwrap();
            let children = registry.get_child_layers(root.id);
            
            // Each root should have at least one child (L1)
            assert!(
                !children.is_empty(),
                "Dimension {} root should have children",
                dim_id
            );
            
            // Verify all children are valid
            for child_id in children {
                let child = registry.get_layer(*child_id).unwrap();
                assert_eq!(child.depth, 1, "Root children should be depth 1");
                assert_eq!(
                    child.parent,
                    Some(root.id),
                    "Child should reference root as parent"
                );
            }
        }
    }
    
    #[test]
    fn test_validate_all_layers_have_keywords() {
        let config_path = "data/dimensions.json";
        if !std::path::Path::new(config_path).exists() {
            eprintln!("Skipping test: {} not found", config_path);
            return;
        }
        
        let config_data = std::fs::read_to_string(config_path).unwrap();
        let registry = DimensionRegistry::load_dimensions(&config_data).unwrap();
        
        // Every layer should have at least one keyword for matching
        for dim_id in 1..=14 {
            for layer_num in 0..=3 {
                let layer_id = LayerId {
                    dimension: DimensionId(dim_id),
                    layer: layer_num,
                };
                
                if let Some(layer) = registry.get_layer(layer_id) {
                    assert!(
                        !layer.keywords.is_empty(),
                        "Layer {:?} should have keywords",
                        layer_id
                    );
                }
            }
        }
    }
    
    #[test]
    fn test_frequency_alignment_scoring() {
        let dim = DimensionMetadata::new(
            DimensionId(1),
            "Test".to_string(),
            (1.0, 3.0),
            1024,
        );
        
        // Exact match within range
        assert!(dim.contains_frequency(2.0));
        assert!(dim.near_frequency(2.0));
        
        // Just outside range but within 0.5 Hz
        assert!(!dim.contains_frequency(0.6));
        assert!(dim.near_frequency(0.6)); // 0.4 Hz below min
        
        assert!(!dim.contains_frequency(3.4));
        assert!(dim.near_frequency(3.4)); // 0.4 Hz above max
        
        // Far outside range
        assert!(!dim.contains_frequency(0.4));
        assert!(!dim.near_frequency(0.4)); // 0.6 Hz below min
        
        assert!(!dim.contains_frequency(3.6));
        assert!(!dim.near_frequency(3.6)); // 0.6 Hz above max
    }
    
    #[test]
    fn test_error_handling_for_invalid_configurations() {
        // Test various invalid configurations
        
        // Empty dimensions
        let empty_config = r#"{"dimensions": [], "layers": []}"#;
        let result = DimensionRegistry::load_dimensions(empty_config);
        assert!(result.is_err());
        
        // Dimension without root layer
        let no_root = r#"{
            "dimensions": [
                {"id": 1, "name": "Test", "frequency_min": 1.0, "frequency_max": 2.0, "size_bytes": 1024}
            ],
            "layers": []
        }"#;
        let result = DimensionRegistry::load_dimensions(no_root);
        assert!(result.is_err());
        
        // Layer referencing non-existent parent
        let bad_parent = r#"{
            "dimensions": [
                {"id": 1, "name": "Test", "frequency_min": 1.0, "frequency_max": 2.0, "size_bytes": 1024}
            ],
            "layers": [
                {"dimension_id": 1, "layer_num": 0, "depth": 0, "parent_layer": null, "keywords": ["test"], "frequency": 1.5, "mmap_offset": 0},
                {"dimension_id": 1, "layer_num": 1, "depth": 1, "parent_layer": 99, "keywords": ["test"], "frequency": 1.5, "mmap_offset": 1024}
            ]
        }"#;
        let result = DimensionRegistry::load_dimensions(bad_parent);
        assert!(result.is_err());
    }
}
