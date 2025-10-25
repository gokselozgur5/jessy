//! Simple interference calculator for consciousness orchestrator
//!
//! This module provides a minimal interference calculation for MVP.
//! It creates basic InterferenceResult from loaded contexts without
//! complex harmonic analysis.
//!
//! # Design Decision (ADR-001)
//!
//! We use simple frequency averaging instead of full harmonic analysis for MVP:
//! - Iteration system requires InterferenceResult
//! - Full interference engine not yet implemented
//! - Simple calculation sufficient for MVP
//! - Can be enhanced in Phase 2 without API changes

use crate::interference::{
    FrequencyState, InterferencePattern, InterferenceResult,
};
use crate::memory::ContextCollection;
use crate::Frequency;

/// Create simple interference result from context collection
///
/// This function performs minimal interference calculation:
/// 1. Collect frequencies from all loaded contexts
/// 2. Calculate average as dominant frequency
/// 3. Create basic InterferencePattern
/// 4. Suggest return-to-source if >6 dimensions
///
/// # Arguments
///
/// * `contexts` - Collection of loaded dimensional contexts
///
/// # Returns
///
/// Returns `InterferenceResult` with:
/// - Pattern containing all frequency states
/// - Dominant frequency (simple average)
/// - Complexity score (number of frequencies)
/// - Return-to-source suggestion if >6 dimensions
///
/// # Example
///
/// ```no_run
/// use jessy::consciousness::interference_simple::create_simple_interference;
/// use jessy::memory::ContextCollection;
///
/// let contexts = ContextCollection::new();
/// let result = create_simple_interference(&contexts);
/// ```
pub fn create_simple_interference(contexts: &ContextCollection) -> InterferenceResult {
    let mut pattern = InterferencePattern::new();
    
    // Collect frequency states from all contexts
    for context in &contexts.contexts {
        let freq_state = FrequencyState::new(
            context.frequency,
            context.layer_id.dimension,
            1.0, // Full confidence for loaded contexts
        );
        pattern.add_frequency(freq_state);
    }
    
    // Calculate simple dominant frequency (average)
    let dominant_freq = if pattern.frequencies.is_empty() {
        Frequency::new(1.0) // Default to balanced frequency
    } else {
        let sum: f32 = pattern.frequencies.iter()
            .map(|f| f.frequency.hz())
            .sum();
        let avg = sum / pattern.frequencies.len() as f32;
        Frequency::new(avg)
    };
    
    pattern.dominant_frequency = dominant_freq;
    pattern.complexity_score = pattern.frequencies.len() as f32;
    
    // Determine if return-to-source is needed (>6 dimensions)
    let return_to_source = pattern.frequencies.len() > 6;
    
    InterferenceResult {
        pattern,
        modulation_suggestions: vec![], // No modulation in simple version
        balance_activation_needed: false, // No balance detection in simple version
        return_to_source_suggested: return_to_source,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{DimensionId, LayerId};
    use crate::memory::LoadedContext;
    
    #[test]
    fn test_empty_contexts() {
        let contexts = ContextCollection::new();
        let result = create_simple_interference(&contexts);
        
        assert_eq!(result.pattern.frequencies.len(), 0);
        assert_eq!(result.pattern.dominant_frequency.hz(), 1.0);
        assert_eq!(result.pattern.complexity_score, 0.0);
        assert!(!result.return_to_source_suggested);
    }
    
    #[test]
    fn test_single_context() {
        let mut contexts = ContextCollection::new();
        contexts.add_context(LoadedContext {
            layer_id: LayerId {
                dimension: DimensionId(1),
                layer: 0,
            },
            content: "Test content".to_string(),
            frequency: Frequency::new(1.5),
            keywords: vec!["test".to_string()],
        });
        
        let result = create_simple_interference(&contexts);
        
        assert_eq!(result.pattern.frequencies.len(), 1);
        assert_eq!(result.pattern.dominant_frequency.hz(), 1.5);
        assert_eq!(result.pattern.complexity_score, 1.0);
        assert!(!result.return_to_source_suggested);
    }
    
    #[test]
    fn test_multiple_contexts_average_frequency() {
        let mut contexts = ContextCollection::new();
        
        // Add contexts with different frequencies
        contexts.add_context(LoadedContext {
            layer_id: LayerId {
                dimension: DimensionId(1),
                layer: 0,
            },
            content: "Content 1".to_string(),
            frequency: Frequency::new(1.0),
            keywords: vec![],
        });
        
        contexts.add_context(LoadedContext {
            layer_id: LayerId {
                dimension: DimensionId(2),
                layer: 0,
            },
            content: "Content 2".to_string(),
            frequency: Frequency::new(2.0),
            keywords: vec![],
        });
        
        contexts.add_context(LoadedContext {
            layer_id: LayerId {
                dimension: DimensionId(3),
                layer: 0,
            },
            content: "Content 3".to_string(),
            frequency: Frequency::new(3.0),
            keywords: vec![],
        });
        
        let result = create_simple_interference(&contexts);
        
        assert_eq!(result.pattern.frequencies.len(), 3);
        // Average of 1.0, 2.0, 3.0 = 2.0
        assert_eq!(result.pattern.dominant_frequency.hz(), 2.0);
        assert_eq!(result.pattern.complexity_score, 3.0);
        assert!(!result.return_to_source_suggested);
    }
    
    #[test]
    fn test_return_to_source_triggered() {
        let mut contexts = ContextCollection::new();
        
        // Add 7 contexts to trigger return-to-source (>6)
        for i in 1..=7 {
            contexts.add_context(LoadedContext {
                layer_id: LayerId {
                    dimension: DimensionId(i),
                    layer: 0,
                },
                content: format!("Content {}", i),
                frequency: Frequency::new(1.5),
                keywords: vec![],
            });
        }
        
        let result = create_simple_interference(&contexts);
        
        assert_eq!(result.pattern.frequencies.len(), 7);
        assert_eq!(result.pattern.complexity_score, 7.0);
        assert!(result.return_to_source_suggested);
    }
    
    #[test]
    fn test_return_to_source_not_triggered_at_boundary() {
        let mut contexts = ContextCollection::new();
        
        // Add exactly 6 contexts (boundary case)
        for i in 1..=6 {
            contexts.add_context(LoadedContext {
                layer_id: LayerId {
                    dimension: DimensionId(i),
                    layer: 0,
                },
                content: format!("Content {}", i),
                frequency: Frequency::new(1.5),
                keywords: vec![],
            });
        }
        
        let result = create_simple_interference(&contexts);
        
        assert_eq!(result.pattern.frequencies.len(), 6);
        assert_eq!(result.pattern.complexity_score, 6.0);
        assert!(!result.return_to_source_suggested);
    }
    
    #[test]
    fn test_frequency_states_have_full_confidence() {
        let mut contexts = ContextCollection::new();
        contexts.add_context(LoadedContext {
            layer_id: LayerId {
                dimension: DimensionId(1),
                layer: 0,
            },
            content: "Test".to_string(),
            frequency: Frequency::new(2.5),
            keywords: vec![],
        });
        
        let result = create_simple_interference(&contexts);
        
        assert_eq!(result.pattern.frequencies.len(), 1);
        assert_eq!(result.pattern.frequencies[0].confidence, 1.0);
        assert_eq!(result.pattern.frequencies[0].frequency.hz(), 2.5);
    }
    
    #[test]
    fn test_no_modulation_suggestions() {
        let mut contexts = ContextCollection::new();
        contexts.add_context(LoadedContext {
            layer_id: LayerId {
                dimension: DimensionId(1),
                layer: 0,
            },
            content: "Test".to_string(),
            frequency: Frequency::new(1.5),
            keywords: vec![],
        });
        
        let result = create_simple_interference(&contexts);
        
        assert!(result.modulation_suggestions.is_empty());
        assert!(!result.balance_activation_needed);
    }
}
