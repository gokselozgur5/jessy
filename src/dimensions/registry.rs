//! Dimension registry for lookup and management

use crate::{DimensionId, Result};
use super::{DimensionInfo, CoreDimension};
use std::collections::HashMap;

/// Registry for dimension lookup
#[derive(Debug)]
pub struct DimensionRegistry {
    dimensions: HashMap<DimensionId, DimensionInfo>,
}

impl DimensionRegistry {
    /// Create new dimension registry
    pub fn new() -> Self {
        let mut registry = Self {
            dimensions: HashMap::new(),
        };
        
        // Register core dimensions
        registry.register_core_dimensions();
        
        registry
    }
    
    /// Register all core dimensions
    fn register_core_dimensions(&mut self) {
        for core_dim in CoreDimension::all() {
            let info = DimensionInfo::new(
                core_dim.id(),
                core_dim.name().to_string(),
                core_dim.frequency_range(),
                16, // Default size
            );
            self.dimensions.insert(core_dim.id(), info);
        }
    }
    
    /// Get dimension info by ID
    pub fn get(&self, id: DimensionId) -> Option<&DimensionInfo> {
        self.dimensions.get(&id)
    }
    
    /// Register custom dimension
    pub fn register(&mut self, info: DimensionInfo) {
        self.dimensions.insert(info.id, info);
    }
}

impl Default for DimensionRegistry {
    fn default() -> Self {
        Self::new()
    }
}

/// Dimension activation result
pub use crate::dimensions::DimensionActivation;

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_registry_creation() {
        let registry = DimensionRegistry::new();
        
        // Should have all 14 core dimensions
        assert_eq!(registry.dimensions.len(), 14);
    }
    
    #[test]
    fn test_dimension_lookup() {
        let registry = DimensionRegistry::new();
        
        let emotion = registry.get(DimensionId(1));
        assert!(emotion.is_some());
        assert_eq!(emotion.unwrap().name, "Emotion");
    }
}
