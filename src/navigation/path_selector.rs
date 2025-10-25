//! Path selection and confidence scoring

use crate::{DimensionId, LayerId, Result};
use super::{NavigationPath, NavigationConfig};

/// Path selector for choosing optimal navigation paths
pub struct PathSelector {
    config: NavigationConfig,
}

impl PathSelector {
    /// Create new path selector
    pub fn new() -> Self {
        Self {
            config: NavigationConfig::default(),
        }
    }
    
    /// Create path selector with custom configuration
    pub fn with_config(config: NavigationConfig) -> Self {
        Self { config }
    }
    
    /// Select top paths from candidates
    pub fn select_top_paths(&self, mut paths: Vec<NavigationPath>) -> Vec<NavigationPath> {
        // Sort by confidence
        paths.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());
        
        // Take top paths up to max_dimensions
        paths.into_iter()
            .filter(|p| p.is_viable(self.config.confidence_threshold))
            .take(self.config.max_dimensions)
            .collect()
    }
    
    /// Navigate to appropriate depth in dimension
    pub fn navigate_depth(
        &self,
        dimension_id: DimensionId,
        keywords: &[String],
        max_depth: usize,
    ) -> Vec<LayerId> {
        // Simple depth navigation for now
        let mut layers = Vec::new();
        
        for depth in 0..max_depth.min(self.config.max_depth) {
            layers.push(LayerId {
                dimension: dimension_id,
                layer: depth as u16,
            });
        }
        
        layers
    }
    
    /// Check if complexity threshold is exceeded
    pub fn check_complexity(&self, paths: &[NavigationPath]) -> ComplexityCheck {
        let strong_paths = paths.iter()
            .filter(|p| p.confidence > 0.7)
            .count();
        
        ComplexityCheck {
            should_return_to_source: strong_paths > self.config.complexity_threshold,
            complexity_score: strong_paths as f32,
        }
    }
}

impl Default for PathSelector {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of complexity check
#[derive(Debug)]
pub struct ComplexityCheck {
    pub should_return_to_source: bool,
    pub complexity_score: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Frequency;
    
    #[test]
    fn test_path_selection() {
        let selector = PathSelector::new();
        
        let mut path1 = NavigationPath::new(DimensionId(1), Frequency::new(1.0));
        path1.confidence = 0.9;
        path1.add_layer(LayerId { dimension: DimensionId(1), layer: 0 }, 0.0);
        
        let mut path2 = NavigationPath::new(DimensionId(2), Frequency::new(1.5));
        path2.confidence = 0.5;
        path2.add_layer(LayerId { dimension: DimensionId(2), layer: 0 }, 0.0);
        
        let mut path3 = NavigationPath::new(DimensionId(3), Frequency::new(2.0));
        path3.confidence = 0.2; // Below threshold
        path3.add_layer(LayerId { dimension: DimensionId(3), layer: 0 }, 0.0);
        
        let paths = vec![path1, path2, path3];
        let selected = selector.select_top_paths(paths);
        
        assert_eq!(selected.len(), 2); // Only viable paths (above 0.3 threshold)
        assert_eq!(selected[0].dimension_id, DimensionId(1)); // Highest confidence first
    }
    
    #[test]
    fn test_complexity_check() {
        let selector = PathSelector::new();
        
        let paths: Vec<NavigationPath> = (0..7)
            .map(|i| {
                let mut path = NavigationPath::new(DimensionId(i), Frequency::new(1.0));
                path.confidence = 0.8; // Strong confidence
                path
            })
            .collect();
        
        let check = selector.check_complexity(&paths);
        assert!(check.should_return_to_source); // 7 > 6 threshold
    }
}
