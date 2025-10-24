//! Layer hierarchy and management

use crate::{LayerId, Frequency, Result};
use std::collections::HashMap;

/// Layer information
#[derive(Debug, Clone)]
pub struct LayerInfo {
    pub id: LayerId,
    pub name: String,
    pub frequency: Frequency,
    pub depth: u8,
    pub parent: Option<LayerId>,
    pub children: Vec<LayerId>,
}

/// Layer hierarchy manager
#[derive(Debug)]
pub struct LayerHierarchy {
    layers: HashMap<LayerId, LayerInfo>,
}

impl LayerHierarchy {
    /// Create new layer hierarchy
    pub fn new() -> Self {
        Self {
            layers: HashMap::new(),
        }
    }
    
    /// Add layer to hierarchy
    pub fn add_layer(&mut self, layer: LayerInfo) {
        self.layers.insert(layer.id, layer);
    }
    
    /// Get layer by ID
    pub fn get_layer(&self, id: LayerId) -> Option<&LayerInfo> {
        self.layers.get(&id)
    }
}

impl Default for LayerHierarchy {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;
    
    #[test]
    fn test_layer_hierarchy() {
        let mut hierarchy = LayerHierarchy::new();
        
        let layer = LayerInfo {
            id: LayerId { dimension: DimensionId(1), layer: 0 },
            name: "Root".to_string(),
            frequency: Frequency::new(1.0),
            depth: 0,
            parent: None,
            children: vec![],
        };
        
        hierarchy.add_layer(layer);
        assert!(hierarchy.get_layer(LayerId { dimension: DimensionId(1), layer: 0 }).is_some());
    }
}
