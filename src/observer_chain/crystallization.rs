//! Crystallization logic and criteria
//!
//! Determines when an observation has "crystallized" - when the chain
//! has gathered enough information to provide a confident response.

use super::Observation;

/// Why crystallization occurred
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrystallizationReason {
    /// Confidence exceeded 0.95 threshold
    HighConfidence,
    /// Last 2 observations were consistent
    Consistency,
    /// Complexity reduced below 6 layers (Return to Source)
    LowComplexity,
    /// Pattern matched in cache
    PatternMatch,
    /// Reached maximum stages (forced)
    MaxStagesReached,
}

impl CrystallizationReason {
    /// Check if crystallization was natural (not forced)
    pub fn is_natural(&self) -> bool {
        !matches!(self, Self::MaxStagesReached)
    }
}

/// Crystallization check result
#[derive(Debug, Clone)]
pub struct CrystallizationCheck {
    /// Whether crystallization should occur
    pub is_crystallized: bool,
    /// Why crystallization occurred (if it did)
    pub reason: Option<CrystallizationReason>,
    /// Confidence score from current observation
    pub confidence: f32,
    /// Whether last 2 observations are consistent
    pub consistency: bool,
    /// Number of active cognitive layers
    pub complexity: usize,
    /// Whether pattern matched in cache
    pub pattern_match: bool,
}

impl CrystallizationCheck {
    /// Create a new crystallization check
    pub fn new(
        current: &Observation,
        previous: Option<&Observation>,
        pattern_match: bool,
    ) -> Self {
        let confidence = current.confidence;
        let complexity = current.layer_count();
        let consistency = Self::check_consistency(current, previous);

        // Determine if crystallized and why
        let (is_crystallized, reason) = if confidence > 0.95 {
            (true, Some(CrystallizationReason::HighConfidence))
        } else if consistency {
            (true, Some(CrystallizationReason::Consistency))
        } else if complexity < 6 && confidence > 0.85 {
            // Return to Source: Simple query with decent confidence
            (true, Some(CrystallizationReason::LowComplexity))
        } else if pattern_match {
            (true, Some(CrystallizationReason::PatternMatch))
        } else {
            (false, None)
        };

        Self {
            is_crystallized,
            reason,
            confidence,
            consistency,
            complexity,
            pattern_match,
        }
    }

    /// Force crystallization (used at max stages)
    pub fn force() -> Self {
        Self {
            is_crystallized: true,
            reason: Some(CrystallizationReason::MaxStagesReached),
            confidence: 0.0,
            consistency: false,
            complexity: 0,
            pattern_match: false,
        }
    }

    /// Check consistency between current and previous observation
    ///
    /// Observations are consistent if:
    /// - They identify similar cognitive layers (>70% overlap)
    /// - Both have confidence >0.75
    fn check_consistency(current: &Observation, previous: Option<&Observation>) -> bool {
        let Some(prev) = previous else {
            return false;
        };

        // Both need decent confidence
        if current.confidence < 0.75 || prev.confidence < 0.75 {
            return false;
        }

        // Check layer overlap
        let current_set: std::collections::HashSet<_> =
            current.cognitive_layers.iter().collect();
        let prev_set: std::collections::HashSet<_> = prev.cognitive_layers.iter().collect();

        let overlap = current_set.intersection(&prev_set).count();
        let total = current_set.union(&prev_set).count();

        if total == 0 {
            return false;
        }

        let overlap_ratio = overlap as f32 / total as f32;
        overlap_ratio > 0.7
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;

    fn make_observation(
        stage: usize,
        layers: Vec<u8>,
        confidence: f32,
    ) -> Observation {
        Observation::new(
            stage,
            layers.into_iter().map(DimensionId).collect(),
            "test content".to_string(),
            confidence,
        )
    }

    #[test]
    fn test_high_confidence_crystallization() {
        let obs = make_observation(1, vec![1, 2], 0.97);
        let check = CrystallizationCheck::new(&obs, None, false);

        assert!(check.is_crystallized);
        assert_eq!(check.reason, Some(CrystallizationReason::HighConfidence));
    }

    #[test]
    fn test_consistency_crystallization() {
        let obs1 = make_observation(1, vec![1, 2, 7], 0.8);
        let obs2 = make_observation(2, vec![1, 2, 7], 0.82);

        let check = CrystallizationCheck::new(&obs2, Some(&obs1), false);

        assert!(check.is_crystallized);
        assert_eq!(check.reason, Some(CrystallizationReason::Consistency));
    }

    #[test]
    fn test_low_complexity_crystallization() {
        let obs = make_observation(1, vec![1, 2], 0.88); // 2 layers, 0.88 confidence

        let check = CrystallizationCheck::new(&obs, None, false);

        assert!(check.is_crystallized);
        assert_eq!(check.reason, Some(CrystallizationReason::LowComplexity));
    }

    #[test]
    fn test_pattern_match_crystallization() {
        let obs = make_observation(1, vec![1, 2, 3], 0.7);

        let check = CrystallizationCheck::new(&obs, None, true);

        assert!(check.is_crystallized);
        assert_eq!(check.reason, Some(CrystallizationReason::PatternMatch));
    }

    #[test]
    fn test_no_crystallization() {
        let obs = make_observation(1, vec![1, 2, 3, 4, 5, 6], 0.7); // Complex, low confidence

        let check = CrystallizationCheck::new(&obs, None, false);

        assert!(!check.is_crystallized);
        assert_eq!(check.reason, None);
    }

    #[test]
    fn test_consistency_requires_overlap() {
        let obs1 = make_observation(1, vec![1, 2, 3], 0.8);
        let obs2 = make_observation(2, vec![7, 8, 9], 0.8); // Different layers

        let check = CrystallizationCheck::new(&obs2, Some(&obs1), false);

        assert!(!check.consistency); // No overlap
    }

    #[test]
    fn test_consistency_requires_decent_confidence() {
        let obs1 = make_observation(1, vec![1, 2, 3], 0.6); // Low confidence
        let obs2 = make_observation(2, vec![1, 2, 3], 0.8);

        let check = CrystallizationCheck::new(&obs2, Some(&obs1), false);

        assert!(!check.consistency); // obs1 confidence too low
    }

    #[test]
    fn test_forced_crystallization() {
        let check = CrystallizationCheck::force();

        assert!(check.is_crystallized);
        assert_eq!(check.reason, Some(CrystallizationReason::MaxStagesReached));
        assert!(!check.reason.unwrap().is_natural());
    }

    #[test]
    fn test_crystallization_reason_natural() {
        assert!(CrystallizationReason::HighConfidence.is_natural());
        assert!(CrystallizationReason::Consistency.is_natural());
        assert!(CrystallizationReason::LowComplexity.is_natural());
        assert!(CrystallizationReason::PatternMatch.is_natural());
        assert!(!CrystallizationReason::MaxStagesReached.is_natural());
    }
}
