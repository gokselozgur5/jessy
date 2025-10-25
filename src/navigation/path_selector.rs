//! Path selection and confidence scoring
//!
//! This module implements the PathSelector component responsible for:
//! - Calculating confidence scores for dimension activations
//! - Ranking and selecting optimal navigation paths
//! - Managing complexity through return-to-source protocol
//!
//! The PathSelector uses a weighted scoring algorithm combining:
//! - Keyword match score (50% weight): Ratio of matched to total keywords
//! - Synesthetic score (30% weight): Sum of learned association strengths
//! - Frequency alignment (20% weight): How well query frequency matches dimension
//!
//! Requirements covered: 4.1-4.10

use crate::{DimensionId, LayerId, Result};
use super::{NavigationPath, NavigationConfig, NavigationError};

/// Path selector for choosing optimal navigation paths
///
/// The PathSelector evaluates dimension activations and selects the most relevant
/// paths based on confidence scoring. It implements the path selection algorithm
/// defined in Requirements 4.1-4.10.
///
/// # Confidence Scoring
///
/// Confidence is calculated as a weighted average:
/// - 50% keyword match score (matched keywords / total query keywords)
/// - 30% synesthetic score (sum of association strengths / keyword count)
/// - 20% frequency alignment (1.0 if in range, 0.5 if within 0.5Hz, 0.0 otherwise)
///
/// # Path Selection
///
/// Paths are:
/// 1. Filtered by confidence threshold (>= 0.3)
/// 2. Ranked by confidence (descending)
/// 3. Limited to max 8 dimensions
///
/// # Complexity Management
///
/// If more than 6 dimensions are activated, return-to-source is triggered,
/// reducing selection to the top 3 highest confidence dimensions.
///
/// # Examples
///
/// ```rust,ignore
/// use jessy::navigation::{PathSelector, NavigationConfig};
///
/// let selector = PathSelector::new();
/// let selected = selector.select_paths(activations, query_frequency)?;
/// ```
pub struct PathSelector {
    /// Configuration parameters for path selection
    config: NavigationConfig,
}

impl PathSelector {
    /// Create new path selector with default configuration
    ///
    /// Uses default values from NavigationConfig:
    /// - Confidence threshold: 0.3
    /// - Max dimensions: 8
    /// - Complexity threshold: 6
    pub fn new() -> Self {
        Self {
            config: NavigationConfig::default(),
        }
    }
    
    /// Create path selector with custom configuration
    ///
    /// # Arguments
    ///
    /// * `config` - Custom navigation configuration
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let mut config = NavigationConfig::default();
    /// config.confidence_threshold = 0.4;
    /// let selector = PathSelector::with_config(config);
    /// ```
    pub fn with_config(config: NavigationConfig) -> Self {
        Self { config }
    }
    
    /// Get reference to the configuration
    pub fn config(&self) -> &NavigationConfig {
        &self.config
    }
    
    // ============================================================================
    // Confidence Scoring Methods (to be implemented in tasks 5.2-5.7)
    // ============================================================================
    
    /// Calculate keyword match score
    ///
    /// Computes the ratio of matched keywords to total query keywords.
    /// Returns a value between 0.0 and 1.0.
    ///
    /// Requirement 4.4: Keyword match score = matched / total
    ///
    /// # Arguments
    ///
    /// * `matched_count` - Number of keywords that matched
    /// * `total_count` - Total number of query keywords
    ///
    /// # Returns
    ///
    /// Score between 0.0 and 1.0, or 0.0 if total_count is 0
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let score = selector.calculate_keyword_match_score(3, 10);
    /// assert_eq!(score, 0.3);
    /// ```
    pub fn calculate_keyword_match_score(&self, matched_count: usize, total_count: usize) -> f32 {
        // TODO: Implement in task 5.3
        if total_count == 0 {
            return 0.0;
        }
        matched_count as f32 / total_count as f32
    }
    
    /// Calculate frequency alignment score
    ///
    /// Determines how well the query frequency aligns with a dimension's frequency range.
    ///
    /// Requirement 4.5-4.7:
    /// - Returns 1.0 if query frequency is within dimension range
    /// - Returns 0.5 if within 0.5 Hz of range boundaries
    /// - Returns 0.0 otherwise
    ///
    /// # Arguments
    ///
    /// * `query_frequency` - Estimated frequency of the query (Hz)
    /// * `dimension_min_freq` - Minimum frequency of dimension range (Hz)
    /// * `dimension_max_freq` - Maximum frequency of dimension range (Hz)
    ///
    /// # Returns
    ///
    /// Alignment score: 1.0, 0.5, or 0.0
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// // Query frequency within range
    /// let score = selector.calculate_frequency_alignment(2.0, 1.5, 2.5);
    /// assert_eq!(score, 1.0);
    ///
    /// // Query frequency near range
    /// let score = selector.calculate_frequency_alignment(1.0, 1.3, 2.0);
    /// assert_eq!(score, 0.5);
    /// ```
    pub fn calculate_frequency_alignment(
        &self,
        query_frequency: f32,
        dimension_min_freq: f32,
        dimension_max_freq: f32,
    ) -> f32 {
        // TODO: Implement in task 5.5
        if query_frequency >= dimension_min_freq && query_frequency <= dimension_max_freq {
            1.0
        } else if (query_frequency - dimension_max_freq).abs() <= 0.5
            || (dimension_min_freq - query_frequency).abs() <= 0.5
        {
            0.5
        } else {
            0.0
        }
    }
    
    /// Calculate overall confidence score
    ///
    /// Combines keyword match, synesthetic, and frequency alignment scores
    /// using weighted average.
    ///
    /// Requirement 4.7-4.8:
    /// - Weighted average: 50% keyword + 30% synesthetic + 20% frequency
    /// - Clamped to [0.0, 1.0] range
    ///
    /// # Arguments
    ///
    /// * `keyword_score` - Keyword match score (0.0-1.0)
    /// * `synesthetic_score` - Synesthetic association score (0.0-1.0)
    /// * `frequency_score` - Frequency alignment score (0.0-1.0)
    ///
    /// # Returns
    ///
    /// Overall confidence score between 0.0 and 1.0
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let confidence = selector.calculate_confidence(0.8, 0.6, 1.0);
    /// // 0.5 * 0.8 + 0.3 * 0.6 + 0.2 * 1.0 = 0.78
    /// assert_eq!(confidence, 0.78);
    /// ```
    pub fn calculate_confidence(
        &self,
        keyword_score: f32,
        synesthetic_score: f32,
        frequency_score: f32,
    ) -> f32 {
        // TODO: Implement in task 5.7
        let confidence = self.config.keyword_match_weight * keyword_score
            + self.config.synesthetic_weight * synesthetic_score
            + self.config.frequency_alignment_weight * frequency_score;
        
        confidence.clamp(0.0, 1.0)
    }
    
    // ============================================================================
    // Path Selection Methods (to be implemented in tasks 5.8-5.11)
    // ============================================================================
    
    /// Rank paths by confidence with tiebreakers
    ///
    /// Sorts paths in descending order by:
    /// 1. Confidence score (primary)
    /// 2. Keyword match count (tiebreaker)
    /// 3. Dimension ID (second tiebreaker, lower is better)
    ///
    /// Requirements 4.1, 4.9-4.10
    ///
    /// # Arguments
    ///
    /// * `paths` - Mutable vector of navigation paths to rank
    pub fn rank_paths(&self, paths: &mut Vec<NavigationPath>) {
        // TODO: Implement in task 5.9
        paths.sort_by(|a, b| {
            // Primary: confidence descending
            match b.confidence.partial_cmp(&a.confidence) {
                Some(std::cmp::Ordering::Equal) => {
                    // Tiebreaker 1: keyword count descending
                    match b.keywords_matched.len().cmp(&a.keywords_matched.len()) {
                        std::cmp::Ordering::Equal => {
                            // Tiebreaker 2: dimension ID ascending
                            a.dimension_id.0.cmp(&b.dimension_id.0)
                        }
                        other => other,
                    }
                }
                Some(other) => other,
                None => std::cmp::Ordering::Equal,
            }
        });
    }
    
    /// Select optimal navigation paths from activations
    ///
    /// Filters, ranks, and limits paths according to configuration.
    ///
    /// Requirements 4.1-4.3:
    /// - Filters by confidence threshold (>= 0.3)
    /// - Ranks by confidence (with tiebreakers)
    /// - Limits to max 8 dimensions
    ///
    /// # Arguments
    ///
    /// * `paths` - Vector of candidate navigation paths
    ///
    /// # Returns
    ///
    /// Vector of selected paths, sorted by confidence
    ///
    /// # Examples
    ///
    /// ```rust,ignore
    /// let selected = selector.select_paths(candidate_paths);
    /// assert!(selected.len() <= 8);
    /// assert!(selected.iter().all(|p| p.confidence >= 0.3));
    /// ```
    pub fn select_paths(&self, mut paths: Vec<NavigationPath>) -> Vec<NavigationPath> {
        // TODO: Implement in task 5.11
        
        // Filter by confidence threshold
        paths.retain(|p| p.confidence >= self.config.confidence_threshold);
        
        // Rank paths
        self.rank_paths(&mut paths);
        
        // Limit to max dimensions
        paths.truncate(self.config.max_dimensions);
        
        paths
    }
    
    // ============================================================================
    // Complexity Management
    // ============================================================================
    
    /// Check if complexity threshold is exceeded
    ///
    /// Determines if return-to-source protocol should be triggered.
    ///
    /// Requirement 6.1-6.2: Trigger if > 6 dimensions activated
    ///
    /// # Arguments
    ///
    /// * `paths` - Current navigation paths
    ///
    /// # Returns
    ///
    /// ComplexityCheck indicating if return-to-source should trigger
    pub fn check_complexity(&self, paths: &[NavigationPath]) -> ComplexityCheck {
        let dimension_count = paths.len();
        
        ComplexityCheck {
            should_return_to_source: dimension_count > self.config.complexity_threshold,
            complexity_score: dimension_count as f32,
            original_count: dimension_count,
        }
    }
    
    /// Apply return-to-source simplification
    ///
    /// Reduces activated dimensions to top 3 highest confidence.
    ///
    /// Requirement 6.3: Reduce to top 3 by confidence
    ///
    /// # Arguments
    ///
    /// * `paths` - Mutable vector of navigation paths
    ///
    /// # Returns
    ///
    /// Number of paths removed
    pub fn apply_return_to_source(&self, paths: &mut Vec<NavigationPath>) -> usize {
        let original_count = paths.len();
        
        if original_count <= 3 {
            return 0;
        }
        
        // Ensure paths are ranked
        self.rank_paths(paths);
        
        // Keep only top 3
        paths.truncate(3);
        
        original_count - paths.len()
    }
}

impl Default for PathSelector {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of complexity check
///
/// Indicates whether return-to-source protocol should be triggered
/// and provides metrics about the complexity.
#[derive(Debug, Clone)]
pub struct ComplexityCheck {
    /// Whether return-to-source should be triggered
    pub should_return_to_source: bool,
    
    /// Complexity score (currently just dimension count)
    pub complexity_score: f32,
    
    /// Original number of dimensions before simplification
    pub original_count: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Frequency;
    
    #[test]
    fn test_path_selector_creation() {
        let selector = PathSelector::new();
        assert_eq!(selector.config().confidence_threshold, 0.3);
        assert_eq!(selector.config().max_dimensions, 8);
        assert_eq!(selector.config().complexity_threshold, 6);
    }
    
    #[test]
    fn test_path_selector_with_config() {
        let mut config = NavigationConfig::default();
        config.confidence_threshold = 0.5;
        config.max_dimensions = 5;
        
        let selector = PathSelector::with_config(config);
        assert_eq!(selector.config().confidence_threshold, 0.5);
        assert_eq!(selector.config().max_dimensions, 5);
    }
    
    #[test]
    fn test_keyword_match_score_basic() {
        let selector = PathSelector::new();
        
        // Perfect match
        assert_eq!(selector.calculate_keyword_match_score(5, 5), 1.0);
        
        // Partial match
        assert_eq!(selector.calculate_keyword_match_score(3, 10), 0.3);
        
        // No match
        assert_eq!(selector.calculate_keyword_match_score(0, 5), 0.0);
        
        // Edge case: zero total
        assert_eq!(selector.calculate_keyword_match_score(0, 0), 0.0);
    }
    
    #[test]
    fn test_frequency_alignment_in_range() {
        let selector = PathSelector::new();
        
        // Query frequency within range
        let score = selector.calculate_frequency_alignment(2.0, 1.5, 2.5);
        assert_eq!(score, 1.0);
        
        // Query at lower boundary
        let score = selector.calculate_frequency_alignment(1.5, 1.5, 2.5);
        assert_eq!(score, 1.0);
        
        // Query at upper boundary
        let score = selector.calculate_frequency_alignment(2.5, 1.5, 2.5);
        assert_eq!(score, 1.0);
    }
    
    #[test]
    fn test_frequency_alignment_near_range() {
        let selector = PathSelector::new();
        
        // Within 0.5 Hz below range
        let score = selector.calculate_frequency_alignment(1.0, 1.3, 2.0);
        assert_eq!(score, 0.5);
        
        // Within 0.5 Hz above range
        let score = selector.calculate_frequency_alignment(2.5, 1.5, 2.0);
        assert_eq!(score, 0.5);
    }
    
    #[test]
    fn test_frequency_alignment_far_outside() {
        let selector = PathSelector::new();
        
        // Far below range
        let score = selector.calculate_frequency_alignment(1.0, 2.0, 3.0);
        assert_eq!(score, 0.0);
        
        // Far above range
        let score = selector.calculate_frequency_alignment(4.0, 1.0, 2.0);
        assert_eq!(score, 0.0);
    }
    
    #[test]
    fn test_confidence_calculation() {
        let selector = PathSelector::new();
        
        // Test weighted average: 0.5 * 0.8 + 0.3 * 0.6 + 0.2 * 1.0
        let confidence = selector.calculate_confidence(0.8, 0.6, 1.0);
        assert!((confidence - 0.78).abs() < 0.001);
        
        // Test clamping (shouldn't exceed 1.0)
        let confidence = selector.calculate_confidence(1.0, 1.0, 1.0);
        assert_eq!(confidence, 1.0);
        
        // Test zero scores
        let confidence = selector.calculate_confidence(0.0, 0.0, 0.0);
        assert_eq!(confidence, 0.0);
    }
    
    #[test]
    fn test_path_ranking() {
        let selector = PathSelector::new();
        
        let mut path1 = NavigationPath::new(DimensionId(1), Frequency::new(1.0));
        path1.confidence = 0.5;
        path1.add_keyword("test".to_string(), 0.0);
        
        let mut path2 = NavigationPath::new(DimensionId(2), Frequency::new(1.5));
        path2.confidence = 0.9;
        path2.add_keyword("test".to_string(), 0.0);
        
        let mut path3 = NavigationPath::new(DimensionId(3), Frequency::new(2.0));
        path3.confidence = 0.5;
        path3.add_keyword("test".to_string(), 0.0);
        path3.add_keyword("another".to_string(), 0.0);
        
        let mut paths = vec![path1, path2, path3];
        selector.rank_paths(&mut paths);
        
        // Should be sorted by confidence descending
        assert_eq!(paths[0].dimension_id, DimensionId(2)); // confidence 0.9
        // Tiebreaker: path3 has more keywords than path1
        assert_eq!(paths[1].dimension_id, DimensionId(3)); // confidence 0.5, 2 keywords
        assert_eq!(paths[2].dimension_id, DimensionId(1)); // confidence 0.5, 1 keyword
    }
    
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
        let selected = selector.select_paths(paths);
        
        assert_eq!(selected.len(), 2); // Only viable paths (above 0.3 threshold)
        assert_eq!(selected[0].dimension_id, DimensionId(1)); // Highest confidence first
        assert_eq!(selected[1].dimension_id, DimensionId(2));
    }
    
    #[test]
    fn test_path_selection_max_limit() {
        let selector = PathSelector::new();
        
        // Create 10 paths, all above threshold
        let paths: Vec<NavigationPath> = (0..10)
            .map(|i| {
                let mut path = NavigationPath::new(DimensionId(i), Frequency::new(1.0));
                path.confidence = 0.5 + (i as f32 * 0.05);
                path.add_layer(LayerId { dimension: DimensionId(i), layer: 0 }, 0.0);
                path
            })
            .collect();
        
        let selected = selector.select_paths(paths);
        
        // Should be limited to max_dimensions (8)
        assert_eq!(selected.len(), 8);
        
        // Should be sorted by confidence (highest first)
        assert!(selected[0].confidence >= selected[1].confidence);
    }
    
    #[test]
    fn test_complexity_check() {
        let selector = PathSelector::new();
        
        // Below threshold
        let paths: Vec<NavigationPath> = (0..5)
            .map(|i| NavigationPath::new(DimensionId(i), Frequency::new(1.0)))
            .collect();
        
        let check = selector.check_complexity(&paths);
        assert!(!check.should_return_to_source);
        assert_eq!(check.original_count, 5);
        
        // At threshold
        let paths: Vec<NavigationPath> = (0..6)
            .map(|i| NavigationPath::new(DimensionId(i), Frequency::new(1.0)))
            .collect();
        
        let check = selector.check_complexity(&paths);
        assert!(!check.should_return_to_source); // 6 is not > 6
        
        // Above threshold
        let paths: Vec<NavigationPath> = (0..7)
            .map(|i| NavigationPath::new(DimensionId(i), Frequency::new(1.0)))
            .collect();
        
        let check = selector.check_complexity(&paths);
        assert!(check.should_return_to_source); // 7 > 6
        assert_eq!(check.original_count, 7);
    }
    
    #[test]
    fn test_return_to_source() {
        let selector = PathSelector::new();
        
        // Create 7 paths with varying confidence
        let mut paths: Vec<NavigationPath> = (0..7)
            .map(|i| {
                let mut path = NavigationPath::new(DimensionId(i), Frequency::new(1.0));
                path.confidence = 0.5 + (i as f32 * 0.1);
                path
            })
            .collect();
        
        let removed = selector.apply_return_to_source(&mut paths);
        
        assert_eq!(removed, 4); // 7 - 3 = 4
        assert_eq!(paths.len(), 3);
        
        // Should keep top 3 by confidence
        assert!(paths[0].confidence >= paths[1].confidence);
        assert!(paths[1].confidence >= paths[2].confidence);
    }
    
    #[test]
    fn test_return_to_source_already_simple() {
        let selector = PathSelector::new();
        
        let mut paths: Vec<NavigationPath> = (0..2)
            .map(|i| NavigationPath::new(DimensionId(i), Frequency::new(1.0)))
            .collect();
        
        let removed = selector.apply_return_to_source(&mut paths);
        
        assert_eq!(removed, 0); // No paths removed
        assert_eq!(paths.len(), 2); // Still 2 paths
    }
}
