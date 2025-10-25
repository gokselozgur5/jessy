//! Integration tests for interference engine with real dimensional frequencies

#[cfg(test)]
mod tests {
    use super::super::*;
    use crate::{Frequency, DimensionId};
    
    fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
        FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
    }
    
    #[test]
    fn test_integration_with_dimensional_frequencies() {
        // Given: Engine and realistic dimensional frequencies
        // D01 Technical (0.8 Hz), D02 Emotional (1.2 Hz), D03 Philosophical (0.6 Hz)
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(0.8, 1),  // D01 Technical
            create_freq_state(1.2, 2),  // D02 Emotional
            create_freq_state(0.6, 3),  // D03 Philosophical
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should produce valid pattern
        assert_eq!(result.pattern.frequencies.len(), 3);
        assert!(result.pattern.dominant_frequency.hz() > 0.0);
        assert!(result.pattern.amplitude > 0.0);
        
        // Should detect some constructive pairs (frequencies are close)
        assert!(!result.pattern.constructive_pairs.is_empty());
        
        // Should not need balance (all frequencies in normal range)
        assert!(!result.balance_activation_needed);
    }
    
    #[test]
    fn test_integration_with_extreme_activation() {
        // Given: Engine with extreme frequency activation
        // Simulating high stress or intense focus
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(4.2, 1),  // Extreme activation
            create_freq_state(1.0, 2),  // Normal
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should generate modulation suggestions for extreme frequency
        assert!(!result.modulation_suggestions.is_empty());
        
        // Should have at least one critical or high priority suggestion
        let has_high_priority = result.modulation_suggestions.iter()
            .any(|s| matches!(s.priority, ModulationPriority::Critical | ModulationPriority::High));
        assert!(has_high_priority);
        
        // Should have suggestion for the extreme frequency
        let has_extreme_suggestion = result.modulation_suggestions.iter()
            .any(|s| s.reason == ModulationReason::TooExtreme);
        assert!(has_extreme_suggestion);
    }
    
    #[test]
    fn test_integration_with_harmonic_resonance() {
        // Given: Engine with harmonically related frequencies
        // Simulating coherent thought patterns
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),   // Base
            create_freq_state(2.0, 2),   // Octave
            create_freq_state(1.5, 3),   // Perfect 5th
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should detect multiple harmonics
        assert!(result.pattern.harmonics.len() >= 2);
        
        // Should have consonant harmonics
        let consonant_count = result.pattern.consonant_harmonics().len();
        assert!(consonant_count >= 2);
        
        // Should be stable (low dissonance)
        assert!(result.pattern.is_stable());
    }
    
    #[test]
    fn test_integration_with_dissonant_conflict() {
        // Given: Engine with conflicting frequencies
        // Simulating cognitive dissonance or internal conflict
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(3.7, 2),  // Far from first, no harmonic
            create_freq_state(2.3, 3),  // Creates more dissonance
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should detect destructive pairs (far apart frequencies)
        assert!(!result.pattern.destructive_pairs.is_empty());
        
        // Should generate modulation suggestions
        assert!(!result.modulation_suggestions.is_empty());
    }
    
    #[test]
    fn test_integration_with_many_dimensions() {
        // Given: Engine with many active dimensions
        // Simulating complex multi-dimensional thinking
        let engine = InterferenceEngine::new();
        let frequencies: Vec<_> = (1..=8)
            .map(|i| create_freq_state(i as f32 * 0.4, i))
            .collect();
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should suggest return to source (>6 frequencies)
        assert!(result.return_to_source_suggested);
        
        // Should have high complexity score
        assert!(result.pattern.complexity_score > 2.0);
        
        // Should detect some pairs
        assert!(result.pattern.constructive_pairs.len() + 
                result.pattern.destructive_pairs.len() > 0);
    }
    
    #[test]
    fn test_integration_end_to_end_pipeline() {
        // Given: Engine and realistic mixed scenario
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(0.8, 1),   // D01 Technical (normal)
            create_freq_state(1.2, 2),   // D02 Emotional (normal)
            create_freq_state(2.4, 3),   // D03 Philosophical (elevated)
            create_freq_state(1.6, 4),   // D04 Creative (normal)
            create_freq_state(3.8, 5),   // D05 Analytical (extreme)
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Complete analysis should be present
        assert_eq!(result.pattern.frequencies.len(), 5);
        
        // Should have calculated dominant frequency
        assert!(result.pattern.dominant_frequency.hz() > 0.0);
        assert!(result.pattern.dominant_frequency.hz() < 5.0);
        
        // Should have detected some pairs
        assert!(!result.pattern.constructive_pairs.is_empty() || 
                !result.pattern.destructive_pairs.is_empty());
        
        // Should have complexity score
        assert!(result.pattern.complexity_score > 0.0);
        
        // Should have amplitude
        assert!(result.pattern.amplitude > 0.0);
        
        // May or may not need balance (depends on dominant frequency)
        // But should have modulation suggestions for extreme frequency
        assert!(!result.modulation_suggestions.is_empty());
    }
    
    #[test]
    fn test_integration_with_balance_dimension() {
        // Given: Engine with balance dimension active
        // D13 Balance operates at 1.2 Hz
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(4.0, 1),   // Extreme frequency
            create_freq_state(1.2, 13),  // D13 Balance
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Balance dimension should moderate the extreme
        // Dominant frequency should be pulled toward balance
        assert!(result.pattern.dominant_frequency.hz() < 4.0);
        
        // Should generate modulation for extreme frequency
        let extreme_suggestions = result.modulation_suggestions.iter()
            .filter(|s| s.reason == ModulationReason::TooExtreme)
            .count();
        assert!(extreme_suggestions > 0);
    }
    
    #[test]
    fn test_integration_empty_input() {
        // Given: Engine with no frequencies
        let engine = InterferenceEngine::new();
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should return valid empty result
        assert_eq!(result.pattern.frequencies.len(), 0);
        assert!(!result.balance_activation_needed);
        assert!(!result.return_to_source_suggested);
        assert!(result.modulation_suggestions.is_empty());
    }
}
