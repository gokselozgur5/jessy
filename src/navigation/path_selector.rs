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
    
    // ============================================================================
    // Task 5.2: Tests for keyword match score (RED phase)
    // Requirements: 4.4
    // ============================================================================
    
    #[test]
    fn test_keyword_match_score_perfect_match() {
        let selector = PathSelector::new();
        
        // Requirement 4.4: Perfect match should return 1.0
        // When all keywords match, score should be 1.0
        assert_eq!(
            selector.calculate_keyword_match_score(5, 5),
            1.0,
            "Perfect match (5/5) should return 1.0"
        );
    }
    
    #[test]
    fn test_keyword_match_score_partial_match() {
        let selector = PathSelector::new();
        
        // Requirement 4.4: Partial match should return ratio
        // 3 matched out of 10 total should return 0.3
        assert_eq!(
            selector.calculate_keyword_match_score(3, 10),
            0.3,
            "Partial match (3/10) should return 0.3"
        );
        
        // Additional partial match cases
        assert_eq!(
            selector.calculate_keyword_match_score(1, 2),
            0.5,
            "Partial match (1/2) should return 0.5"
        );
        
        assert_eq!(
            selector.calculate_keyword_match_score(7, 20),
            0.35,
            "Partial match (7/20) should return 0.35"
        );
    }
    
    #[test]
    fn test_keyword_match_score_no_match() {
        let selector = PathSelector::new();
        
        // Requirement 4.4: No match should return 0.0
        // When no keywords match, score should be 0.0
        assert_eq!(
            selector.calculate_keyword_match_score(0, 5),
            0.0,
            "No match (0/5) should return 0.0"
        );
        
        assert_eq!(
            selector.calculate_keyword_match_score(0, 100),
            0.0,
            "No match (0/100) should return 0.0"
        );
    }
    
    #[test]
    fn test_keyword_match_score_edge_case_zero_total() {
        let selector = PathSelector::new();
        
        // Requirement 4.4: Edge case - zero total keywords should return 0.0
        // This prevents division by zero and represents "no keywords to match"
        assert_eq!(
            selector.calculate_keyword_match_score(0, 0),
            0.0,
            "Zero total keywords (0/0) should return 0.0"
        );
    }
    
    #[test]
    fn test_keyword_match_score_range_validation() {
        let selector = PathSelector::new();
        
        // Verify score is always in [0.0, 1.0] range
        let test_cases = vec![
            (0, 1),
            (1, 1),
            (5, 10),
            (10, 10),
            (0, 0),
            (1, 100),
            (99, 100),
        ];
        
        for (matched, total) in test_cases {
            let score = selector.calculate_keyword_match_score(matched, total);
            assert!(
                score >= 0.0 && score <= 1.0,
                "Score for {}/{} should be in [0.0, 1.0], got {}",
                matched,
                total,
                score
            );
        }
    }
    
    // ============================================================================
    // Task 5.4: Tests for frequency alignment (RED phase)
    // Requirements: 4.5-4.7, 15.1-15.8
    // ============================================================================
    
    #[test]
    fn test_frequency_alignment_in_range() {
        let selector = PathSelector::new();
        
        // Requirement 4.6: Query frequency within dimension range should return 1.0
        // Test case from task: query=2.0Hz, dim=(1.5-2.5Hz) → 1.0
        let score = selector.calculate_frequency_alignment(2.0, 1.5, 2.5);
        assert_eq!(
            score, 1.0,
            "Query frequency 2.0Hz within range [1.5-2.5Hz] should return 1.0"
        );
        
        // Additional in-range tests
        // Query at lower boundary
        let score = selector.calculate_frequency_alignment(1.5, 1.5, 2.5);
        assert_eq!(
            score, 1.0,
            "Query frequency at lower boundary should return 1.0"
        );
        
        // Query at upper boundary
        let score = selector.calculate_frequency_alignment(2.5, 1.5, 2.5);
        assert_eq!(
            score, 1.0,
            "Query frequency at upper boundary should return 1.0"
        );
        
        // Query in middle of range
        let score = selector.calculate_frequency_alignment(3.0, 2.0, 4.0);
        assert_eq!(
            score, 1.0,
            "Query frequency in middle of range should return 1.0"
        );
        
        // Test with various frequency ranges (Requirements 15.1-15.8)
        // Low urgency base: 0.5 Hz
        let score = selector.calculate_frequency_alignment(0.5, 0.1, 1.0);
        assert_eq!(
            score, 1.0,
            "Low urgency frequency 0.5Hz within range should return 1.0"
        );
        
        // Medium urgency base: 2.0 Hz
        let score = selector.calculate_frequency_alignment(2.0, 1.5, 2.5);
        assert_eq!(
            score, 1.0,
            "Medium urgency frequency 2.0Hz within range should return 1.0"
        );
        
        // High urgency base: 3.5 Hz
        let score = selector.calculate_frequency_alignment(3.5, 3.0, 4.0);
        assert_eq!(
            score, 1.0,
            "High urgency frequency 3.5Hz within range should return 1.0"
        );
    }
    
    #[test]
    fn test_frequency_alignment_near_range() {
        let selector = PathSelector::new();
        
        // Requirement 4.6: Query frequency within 0.5 Hz of range should return 0.5
        // Test case from task: query=1.0Hz, dim=(1.3-2.0Hz) → 0.5 (within 0.5Hz)
        let score = selector.calculate_frequency_alignment(1.0, 1.3, 2.0);
        assert_eq!(
            score, 0.5,
            "Query frequency 1.0Hz within 0.5Hz of range [1.3-2.0Hz] should return 0.5"
        );
        
        // Within 0.5 Hz below lower boundary
        let score = selector.calculate_frequency_alignment(1.0, 1.3, 2.0);
        assert_eq!(
            score, 0.5,
            "Query 0.3Hz below lower boundary should return 0.5"
        );
        
        // Exactly 0.5 Hz below lower boundary
        let score = selector.calculate_frequency_alignment(1.0, 1.5, 2.5);
        assert_eq!(
            score, 0.5,
            "Query exactly 0.5Hz below lower boundary should return 0.5"
        );
        
        // Within 0.5 Hz above upper boundary
        let score = selector.calculate_frequency_alignment(2.5, 1.5, 2.0);
        assert_eq!(
            score, 0.5,
            "Query 0.5Hz above upper boundary should return 0.5"
        );
        
        // Exactly 0.5 Hz above upper boundary
        let score = selector.calculate_frequency_alignment(3.0, 1.5, 2.5);
        assert_eq!(
            score, 0.5,
            "Query exactly 0.5Hz above upper boundary should return 0.5"
        );
        
        // Near range from below (0.2 Hz away)
        let score = selector.calculate_frequency_alignment(1.8, 2.0, 3.0);
        assert_eq!(
            score, 0.5,
            "Query 0.2Hz below range should return 0.5"
        );
        
        // Near range from above (0.4 Hz away)
        let score = selector.calculate_frequency_alignment(3.4, 2.0, 3.0);
        assert_eq!(
            score, 0.5,
            "Query 0.4Hz above range should return 0.5"
        );
    }
    
    #[test]
    fn test_frequency_alignment_far_outside() {
        let selector = PathSelector::new();
        
        // Requirement 4.6: Query frequency more than 0.5 Hz outside range should return 0.0
        // Test case from task: query=1.0Hz, dim=(2.0-3.0Hz) → 0.0
        let score = selector.calculate_frequency_alignment(1.0, 2.0, 3.0);
        assert_eq!(
            score, 0.0,
            "Query frequency 1.0Hz far from range [2.0-3.0Hz] should return 0.0"
        );
        
        // Far below range (more than 0.5 Hz)
        let score = selector.calculate_frequency_alignment(0.5, 2.0, 3.0);
        assert_eq!(
            score, 0.0,
            "Query 1.5Hz below lower boundary should return 0.0"
        );
        
        // Far above range (more than 0.5 Hz)
        let score = selector.calculate_frequency_alignment(4.0, 1.0, 2.0);
        assert_eq!(
            score, 0.0,
            "Query 2.0Hz above upper boundary should return 0.0"
        );
        
        // Very far below
        let score = selector.calculate_frequency_alignment(0.1, 3.0, 4.5);
        assert_eq!(
            score, 0.0,
            "Query 2.9Hz below range should return 0.0"
        );
        
        // Very far above
        let score = selector.calculate_frequency_alignment(4.5, 0.1, 1.0);
        assert_eq!(
            score, 0.0,
            "Query 3.5Hz above range should return 0.0"
        );
    }
    
    #[test]
    fn test_frequency_alignment_boundary_cases() {
        let selector = PathSelector::new();
        
        // Requirement 4.5-4.7: Test boundary conditions
        
        // Exactly at 0.5 Hz threshold below
        let score = selector.calculate_frequency_alignment(1.0, 1.5, 2.5);
        assert_eq!(
            score, 0.5,
            "Query exactly 0.5Hz below should return 0.5"
        );
        
        // Just over 0.5 Hz threshold below (0.51 Hz away)
        let score = selector.calculate_frequency_alignment(0.99, 1.5, 2.5);
        assert_eq!(
            score, 0.0,
            "Query 0.51Hz below should return 0.0"
        );
        
        // Exactly at 0.5 Hz threshold above
        let score = selector.calculate_frequency_alignment(3.0, 1.5, 2.5);
        assert_eq!(
            score, 0.5,
            "Query exactly 0.5Hz above should return 0.5"
        );
        
        // Just over 0.5 Hz threshold above (0.51 Hz away)
        let score = selector.calculate_frequency_alignment(3.01, 1.5, 2.5);
        assert_eq!(
            score, 0.0,
            "Query 0.51Hz above should return 0.0"
        );
        
        // Minimum frequency (0.1 Hz) - Requirement 15.7
        let score = selector.calculate_frequency_alignment(0.1, 0.1, 0.5);
        assert_eq!(
            score, 1.0,
            "Minimum frequency 0.1Hz at lower boundary should return 1.0"
        );
        
        // Maximum frequency (4.5 Hz) - Requirement 15.7
        let score = selector.calculate_frequency_alignment(4.5, 4.0, 4.5);
        assert_eq!(
            score, 1.0,
            "Maximum frequency 4.5Hz at upper boundary should return 1.0"
        );
        
        // Single point range (min == max)
        let score = selector.calculate_frequency_alignment(2.0, 2.0, 2.0);
        assert_eq!(
            score, 1.0,
            "Query matching single-point range should return 1.0"
        );
        
        // Near single point range
        let score = selector.calculate_frequency_alignment(2.3, 2.0, 2.0);
        assert_eq!(
            score, 0.5,
            "Query 0.3Hz from single-point range should return 0.5"
        );
        
        // Far from single point range
        let score = selector.calculate_frequency_alignment(3.0, 2.0, 2.0);
        assert_eq!(
            score, 0.0,
            "Query 1.0Hz from single-point range should return 0.0"
        );
    }
    
    #[test]
    fn test_frequency_alignment_with_adjustments() {
        let selector = PathSelector::new();
        
        // Test frequency alignment with typical query frequency adjustments
        // Requirements 15.1-15.8
        
        // Philosophical adjustment: base 2.0 - 0.5 = 1.5 Hz
        let philosophical_freq = 1.5;
        let score = selector.calculate_frequency_alignment(philosophical_freq, 1.0, 2.0);
        assert_eq!(
            score, 1.0,
            "Philosophical frequency 1.5Hz should align with [1.0-2.0Hz]"
        );
        
        // Technical adjustment: base 2.0 + 0.5 = 2.5 Hz
        let technical_freq = 2.5;
        let score = selector.calculate_frequency_alignment(technical_freq, 2.0, 3.0);
        assert_eq!(
            score, 1.0,
            "Technical frequency 2.5Hz should align with [2.0-3.0Hz]"
        );
        
        // High-intensity emotional: base 2.0 + 1.0 = 3.0 Hz
        let emotional_freq = 3.0;
        let score = selector.calculate_frequency_alignment(emotional_freq, 2.5, 3.5);
        assert_eq!(
            score, 1.0,
            "High-intensity emotional frequency 3.0Hz should align with [2.5-3.5Hz]"
        );
        
        // Clamped to minimum: 0.1 Hz
        let clamped_min = 0.1;
        let score = selector.calculate_frequency_alignment(clamped_min, 0.1, 0.5);
        assert_eq!(
            score, 1.0,
            "Clamped minimum frequency 0.1Hz should align with [0.1-0.5Hz]"
        );
        
        // Clamped to maximum: 4.5 Hz
        let clamped_max = 4.5;
        let score = selector.calculate_frequency_alignment(clamped_max, 4.0, 4.5);
        assert_eq!(
            score, 1.0,
            "Clamped maximum frequency 4.5Hz should align with [4.0-4.5Hz]"
        );
    }
    
    #[test]
    fn test_frequency_alignment_return_values() {
        let selector = PathSelector::new();
        
        // Requirement 4.6: Verify only valid return values (1.0, 0.5, 0.0)
        
        let test_cases = vec![
            // (query_freq, min_freq, max_freq, expected_score)
            (2.0, 1.5, 2.5, 1.0),   // In range
            (1.5, 1.5, 2.5, 1.0),   // At lower boundary
            (2.5, 1.5, 2.5, 1.0),   // At upper boundary
            (1.0, 1.3, 2.0, 0.5),   // Near range (below)
            (2.5, 1.5, 2.0, 0.5),   // Near range (above)
            (1.0, 1.5, 2.5, 0.5),   // Exactly 0.5 Hz below
            (3.0, 1.5, 2.5, 0.5),   // Exactly 0.5 Hz above
            (1.0, 2.0, 3.0, 0.0),   // Far outside (below)
            (4.0, 1.0, 2.0, 0.0),   // Far outside (above)
            (0.5, 2.0, 3.0, 0.0),   // Very far below
            (4.5, 0.1, 1.0, 0.0),   // Very far above
        ];
        
        for (query_freq, min_freq, max_freq, expected) in test_cases {
            let score = selector.calculate_frequency_alignment(query_freq, min_freq, max_freq);
            assert_eq!(
                score, expected,
                "Frequency alignment for query={}, range=[{}-{}] should return {}",
                query_freq, min_freq, max_freq, expected
            );
            
            // Verify score is one of the three valid values
            assert!(
                score == 0.0 || score == 0.5 || score == 1.0,
                "Frequency alignment score must be 0.0, 0.5, or 1.0, got {}",
                score
            );
        }
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
