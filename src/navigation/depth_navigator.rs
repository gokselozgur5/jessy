//! Depth navigation for layer traversal
//!
//! This module implements the DepthNavigator component responsible for:
//! - Navigating through layer hierarchies within dimensions
//! - Calculating layer match scores based on query keywords
//! - Building optimal layer sequences from root to deepest relevant layer
//!
//! The DepthNavigator uses a greedy algorithm:
//! 1. Start at root layer (L0)
//! 2. Calculate match scores for all child layers
//! 3. Select child with highest score above threshold (0.1)
//! 4. Repeat until no viable children or max depth (4) reached
//!
//! Requirements covered: 5.1-5.7

use crate::{DimensionId, LayerId};
use super::{NavigationError, DimensionRegistry};
use std::sync::Arc;

/// Depth navigator for layer traversal
///
/// The DepthNavigator traverses layer hierarchies within dimensions to find
/// the most relevant layers for a given query. It implements the depth
/// navigation algorithm defined in Requirements 5.1-5.7.
///
/// # Layer Match Scoring
///
/// Match score is calculated as:
/// - Ratio of layer keywords found in query keywords
/// - Value between 0.0 and 1.0
/// - Threshold: 0.1 (layers below this are not traversed)
///
/// # Navigation Strategy
///
/// 1. Start at root layer (depth 0)
/// 2. Get all child layers
/// 3. Calculate match score for each child
/// 4. Select child with highest score >= 0.1
/// 5. If tie, use lexicographic order (layer ID)
/// 6. Repeat until no viable children or depth 4
///
/// # Examples
///
/// ```rust,ignore
/// use jessy::navigation::{DepthNavigator, DimensionRegistry};
/// use std::sync::Arc;
///
/// let registry = Arc::new(DimensionRegistry::new());
/// let navigator = DepthNavigator::new(registry);
/// let layers = navigator.navigate_depth(dimension_id, &query_keywords)?;
/// ```
pub struct DepthNavigator {
    /// Reference to dimension registry for layer metadata
    registry: Arc<DimensionRegistry>,
}

impl DepthNavigator {
    /// Create new depth navigator
    ///
    /// # Arguments
    ///
    /// * `registry` - Shared reference to dimension registry
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let registry = Arc::new(DimensionRegistry::new());
    /// let navigator = DepthNavigator::new(registry);
    /// ```
    pub fn new(registry: Arc<DimensionRegistry>) -> Self {
        Self { registry }
    }
    
    /// Get reference to the registry
    pub fn registry(&self) -> &Arc<DimensionRegistry> {
        &self.registry
    }
    
    // ============================================================================
    // Layer Match Scoring (to be implemented in tasks 6.2-6.3)
    // ============================================================================
    
    /// Calculate layer match score
    ///
    /// Computes how well a layer's keywords match the query keywords.
    /// Returns a value between 0.0 and 1.0.
    ///
    /// Requirement 5.2: Match score = (layer keywords in query) / (total layer keywords)
    ///
    /// # Arguments
    ///
    /// * `layer_keywords` - Keywords associated with the layer
    /// * `query_keywords` - Keywords from the user query
    ///
    /// # Returns
    ///
    /// Match score between 0.0 and 1.0
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let score = navigator.calculate_layer_match(&layer_keywords, &query_keywords);
    /// assert!(score >= 0.0 && score <= 1.0);
    /// ```
    pub fn calculate_layer_match(
        &self,
        layer_keywords: &[String],
        query_keywords: &[String],
    ) -> f32 {
        if layer_keywords.is_empty() {
            return 0.0;
        }
        
        let matched = layer_keywords
            .iter()
            .filter(|kw| query_keywords.contains(kw))
            .count();
        
        matched as f32 / layer_keywords.len() as f32
    }
    
    // ============================================================================
    // Depth Navigation (to be implemented in tasks 6.4-6.5)
    // ============================================================================
    
    /// Navigate through layer hierarchy
    ///
    /// Traverses from root layer to deepest relevant layer based on query keywords.
    ///
    /// Requirements 5.1, 5.3-5.7:
    /// - Start at root layer (L0)
    /// - Select best child above 0.1 threshold
    /// - Use lexicographic tiebreaker
    /// - Max depth of 4 layers
    /// - Return complete layer sequence
    ///
    /// # Arguments
    ///
    /// * `dimension_id` - Dimension to navigate
    /// * `query_keywords` - Keywords from the user query
    ///
    /// # Returns
    ///
    /// Vector of LayerIds representing the navigation path from root to leaf
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Dimension not found
    /// - Root layer not found
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let layers = navigator.navigate_depth(DimensionId(1), &keywords)?;
    /// assert!(!layers.is_empty()); // At least root layer
    /// assert!(layers.len() <= 4);  // Max depth
    /// ```
    pub fn navigate_depth(
        &self,
        dimension_id: DimensionId,
        query_keywords: &[String],
    ) -> Result<Vec<LayerId>, NavigationError> {
        // TODO: Implement in task 6.5
        
        // Get dimension metadata
        let dimension = self.registry
            .get_dimension(dimension_id)
            .ok_or_else(|| NavigationError::DimensionNotFound { 
                dimension_id: dimension_id.0 
            })?;
        
        // Start with root layer
        let mut path = vec![dimension.root_layer];
        let mut current_layer = dimension.root_layer;
        
        // Navigate up to max depth of 4
        for _ in 0..3 {  // Already have root, so 3 more levels
            // Get child layers
            let children = self.registry.get_child_layers(current_layer);
            
            if children.is_empty() {
                break;  // No more children
            }
            
            // Calculate match scores for all children
            let mut best_score = 0.0f32;
            let mut best_layer: Option<LayerId> = None;
            
            for child_id in children {
                let layer = self.registry
                    .get_layer(*child_id)
                    .ok_or_else(|| NavigationError::LayerNotFound { 
                        dimension_id: child_id.dimension.0,
                        layer_id: child_id.layer,
                    })?;
                
                let score = self.calculate_layer_match(&layer.keywords, query_keywords);
                
                // Select if score is above threshold and better than current best
                if score >= 0.1 {
                    if score > best_score {
                        best_score = score;
                        best_layer = Some(*child_id);
                    } else if score == best_score {
                        // Lexicographic tiebreaker: lower layer ID wins
                        if let Some(current_best) = best_layer {
                            if child_id.layer < current_best.layer {
                                best_layer = Some(*child_id);
                            }
                        }
                    }
                }
            }
            
            // If no viable child found, stop navigation
            if let Some(next_layer) = best_layer {
                path.push(next_layer);
                current_layer = next_layer;
            } else {
                break;
            }
        }
        
        Ok(path)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Frequency;
    
    fn create_test_registry() -> Arc<DimensionRegistry> {
        // Load real registry from dimensions.json
        let config_data = std::fs::read_to_string("data/dimensions.json")
            .expect("Failed to read dimensions.json");
        let registry = DimensionRegistry::load_dimensions(&config_data)
            .expect("Failed to load test registry");
        Arc::new(registry)
    }
    
    #[test]
    fn test_depth_navigator_creation() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry.clone());
        
        assert!(Arc::ptr_eq(navigator.registry(), &registry));
    }
    
    #[test]
    fn test_calculate_layer_match_perfect() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        let layer_keywords = vec!["empathy".to_string(), "compassion".to_string()];
        let query_keywords = vec!["empathy".to_string(), "compassion".to_string(), "love".to_string()];
        
        let score = navigator.calculate_layer_match(&layer_keywords, &query_keywords);
        assert_eq!(score, 1.0); // All layer keywords in query
    }
    
    #[test]
    fn test_calculate_layer_match_partial() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        let layer_keywords = vec!["empathy".to_string(), "compassion".to_string()];
        let query_keywords = vec!["empathy".to_string(), "love".to_string()];
        
        let score = navigator.calculate_layer_match(&layer_keywords, &query_keywords);
        assert_eq!(score, 0.5); // 1 out of 2 keywords matched
    }
    
    #[test]
    fn test_calculate_layer_match_no_match() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        let layer_keywords = vec!["empathy".to_string(), "compassion".to_string()];
        let query_keywords = vec!["anger".to_string(), "frustration".to_string()];
        
        let score = navigator.calculate_layer_match(&layer_keywords, &query_keywords);
        assert_eq!(score, 0.0); // No keywords matched
    }
    
    #[test]
    fn test_calculate_layer_match_empty_layer() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        let layer_keywords: Vec<String> = vec![];
        let query_keywords = vec!["empathy".to_string()];
        
        let score = navigator.calculate_layer_match(&layer_keywords, &query_keywords);
        assert_eq!(score, 0.0); // Empty layer keywords returns 0.0
    }
    
    #[test]
    fn test_navigate_depth_root_only() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        // Query with no matching child keywords
        let query_keywords = vec!["random".to_string(), "words".to_string()];
        
        let layers = navigator.navigate_depth(DimensionId(1), &query_keywords).unwrap();
        
        // Should only have root layer
        assert_eq!(layers.len(), 1);
        assert_eq!(layers[0].dimension.0, 1);
    }
    
    #[test]
    fn test_navigate_depth_with_children() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        // Query matching emotional keywords from D01
        let query_keywords = vec![
            "empathy".to_string(),
            "compassion".to_string(),
            "listening".to_string(),
        ];
        
        let layers = navigator.navigate_depth(DimensionId(1), &query_keywords).unwrap();
        
        // Should navigate through hierarchy
        assert!(layers.len() >= 1);
        assert_eq!(layers[0].dimension.0, 1); // Root of D01
    }
    
    #[test]
    fn test_navigate_depth_max_depth() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        // Query matching multiple layers
        let query_keywords = vec![
            "empathy".to_string(),
            "compassion".to_string(),
            "understanding".to_string(),
            "presence".to_string(),
        ];
        
        let layers = navigator.navigate_depth(DimensionId(1), &query_keywords).unwrap();
        
        // Should not exceed max depth of 4
        assert!(layers.len() <= 4);
        assert!(layers.len() >= 1); // At least root
    }
    
    // Integration tests with real dimension registry
    
    #[test]
    fn test_integration_real_registry() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        // Test with all 14 dimensions
        for dim_id in 1..=14 {
            let result = navigator.navigate_depth(DimensionId(dim_id), &vec!["test".to_string()]);
            assert!(result.is_ok(), "Failed to navigate dimension {}", dim_id);
            
            let layers = result.unwrap();
            assert!(!layers.is_empty(), "Dimension {} has no layers", dim_id);
            assert!(layers.len() <= 4, "Dimension {} exceeds max depth", dim_id);
        }
    }
    
    #[test]
    fn test_integration_various_query_types() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        // Emotional query
        let emotional_keywords = vec!["empathy".to_string(), "compassion".to_string(), "love".to_string()];
        let layers = navigator.navigate_depth(DimensionId(1), &emotional_keywords).unwrap();
        assert!(!layers.is_empty());
        
        // Technical query
        let technical_keywords = vec!["algorithm".to_string(), "code".to_string(), "system".to_string()];
        let layers = navigator.navigate_depth(DimensionId(7), &technical_keywords).unwrap();
        assert!(!layers.is_empty());
        
        // Philosophical query
        let philosophical_keywords = vec!["meaning".to_string(), "existence".to_string(), "truth".to_string()];
        let layers = navigator.navigate_depth(DimensionId(6), &philosophical_keywords).unwrap();
        assert!(!layers.is_empty());
    }
    
    #[test]
    fn test_integration_edge_cases() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        // Empty keywords
        let layers = navigator.navigate_depth(DimensionId(1), &vec![]).unwrap();
        assert_eq!(layers.len(), 1); // Only root
        
        // Very long keyword list
        let many_keywords: Vec<String> = (0..100).map(|i| format!("keyword{}", i)).collect();
        let layers = navigator.navigate_depth(DimensionId(1), &many_keywords).unwrap();
        assert!(!layers.is_empty());
        
        // Invalid dimension
        let result = navigator.navigate_depth(DimensionId(99), &vec!["test".to_string()]);
        assert!(result.is_err());
    }
    
    #[test]
    fn test_integration_performance() {
        let registry = create_test_registry();
        let navigator = DepthNavigator::new(registry);
        
        let keywords = vec!["test".to_string(), "query".to_string()];
        
        // Should complete quickly even with multiple navigations
        let start = std::time::Instant::now();
        for _ in 0..100 {
            let _ = navigator.navigate_depth(DimensionId(1), &keywords);
        }
        let duration = start.elapsed();
        
        // 100 navigations should complete in under 100ms
        assert!(duration.as_millis() < 100, "Navigation too slow: {:?}", duration);
    }
}
