//! Modulation suggestion generation
//!
//! This module generates specific modulation suggestions for frequencies that
//! need adjustment, including priority assignment and reason determination.

use super::{
    ModulationSuggestion, ModulationReason, ModulationPriority,
    FrequencyState, InterferencePattern, HarmonicRelationship, HarmonicType,
};
use crate::Frequency;

/// Generate modulation suggestions for an interference pattern
///
/// Analyzes the pattern and creates suggestions for frequencies that need
/// adjustment, with appropriate priorities and reasons.
pub fn generate_modulation_suggestions(
    pattern: &InterferencePattern,
) -> Vec<ModulationSuggestion> {
    let mut suggestions = Vec::new();
    
    // Check for extreme frequencies
    for (idx, freq_state) in pattern.frequencies.iter().enumerate() {
        if freq_state.frequency.is_extreme() {
            let target = super::balance::calculate_balance_target(freq_state.frequency.hz());
            let strength = super::balance::calculate_modulation_strength(freq_state.frequency.hz());
            let priority = determine_priority_for_extreme(freq_state.frequency.hz());
            
            suggestions.push(ModulationSuggestion::new(
                freq_state.dimension_id,
                freq_state.frequency,
                Frequency::new(target),
                ModulationReason::TooExtreme,
                priority,
                strength,
            ));
        }
    }
    
    // Check for dissonant frequencies
    for &(idx1, idx2) in &pattern.dissonances {
        // Suggest modulation for the higher frequency (easier to bring down)
        let (higher_idx, lower_idx) = if pattern.frequencies[idx1].frequency.hz() > 
                                          pattern.frequencies[idx2].frequency.hz() {
            (idx1, idx2)
        } else {
            (idx2, idx1)
        };
        
        let higher_freq = &pattern.frequencies[higher_idx];
        let lower_freq = &pattern.frequencies[lower_idx];
        
        // Suggest bringing higher frequency closer to lower
        let target = find_harmonic_target(higher_freq.frequency.hz(), lower_freq.frequency.hz());
        let strength = 0.6; // Moderate strength for dissonance resolution
        
        suggestions.push(ModulationSuggestion::new(
            higher_freq.dimension_id,
            higher_freq.frequency,
            Frequency::new(target),
            ModulationReason::Dissonant,
            ModulationPriority::Medium,
            strength,
        ));
    }
    
    // Check for unbalanced pattern
    if pattern.balance_needed {
        // Suggest modulation for frequencies far from balance center
        for freq_state in &pattern.frequencies {
            let distance_from_center = (freq_state.frequency.hz() - 1.2).abs();
            if distance_from_center > 1.5 {
                let target = super::balance::calculate_balance_target(freq_state.frequency.hz());
                let strength = (distance_from_center / 3.0).min(1.0);
                
                suggestions.push(ModulationSuggestion::new(
                    freq_state.dimension_id,
                    freq_state.frequency,
                    Frequency::new(target),
                    ModulationReason::Unbalanced,
                    ModulationPriority::Medium,
                    strength,
                ));
            }
        }
    }
    
    // Check for constructive harmonics that should be reinforced
    for harmonic in &pattern.harmonics {
        if harmonic.should_reinforce() {
            // Suggest slight boost for both frequencies in the harmonic
            let freq1 = &pattern.frequencies[harmonic.freq1_index];
            let freq2 = &pattern.frequencies[harmonic.freq2_index];
            
            // Boost by 5% to reinforce harmonic
            let target1 = freq1.frequency.hz() * 1.05;
            let target2 = freq2.frequency.hz() * 1.05;
            
            suggestions.push(ModulationSuggestion::new(
                freq1.dimension_id,
                freq1.frequency,
                Frequency::new(target1),
                ModulationReason::Constructive,
                ModulationPriority::Low,
                0.3, // Low strength for reinforcement
            ));
            
            suggestions.push(ModulationSuggestion::new(
                freq2.dimension_id,
                freq2.frequency,
                Frequency::new(target2),
                ModulationReason::Constructive,
                ModulationPriority::Low,
                0.3,
            ));
        }
    }
    
    // Deduplicate suggestions for same dimension (keep highest priority)
    deduplicate_suggestions(suggestions)
}

/// Determine priority for extreme frequency modulation
fn determine_priority_for_extreme(frequency: f32) -> ModulationPriority {
    if frequency > 4.0 {
        ModulationPriority::Critical
    } else if frequency > 3.5 {
        ModulationPriority::High
    } else {
        ModulationPriority::Medium
    }
}

/// Find harmonic target frequency for dissonance resolution
///
/// Attempts to find a nearby harmonic relationship with the reference frequency.
fn find_harmonic_target(current: f32, reference: f32) -> f32 {
    // Try to find nearest harmonic ratio
    let ratios = [2.0, 1.5, 4.0/3.0, 1.25]; // Octave, 5th, 4th, 3rd
    
    let mut best_target = current;
    let mut best_distance = f32::INFINITY;
    
    for &ratio in &ratios {
        let target = reference * ratio;
        let distance = (current - target).abs();
        
        if distance < best_distance && distance < current * 0.3 {
            best_distance = distance;
            best_target = target;
        }
    }
    
    // If no good harmonic found, just move 20% toward reference
    if best_distance == f32::INFINITY {
        current + (reference - current) * 0.2
    } else {
        best_target
    }
}

/// Deduplicate suggestions for the same dimension
///
/// Keeps the suggestion with highest priority for each dimension.
fn deduplicate_suggestions(suggestions: Vec<ModulationSuggestion>) -> Vec<ModulationSuggestion> {
    use std::collections::HashMap;
    
    let mut best_suggestions: HashMap<crate::DimensionId, ModulationSuggestion> = HashMap::new();
    
    for suggestion in suggestions {
        let should_replace = match best_suggestions.get(&suggestion.dimension_id) {
            None => true,
            Some(existing) => priority_value(&suggestion.priority) > priority_value(&existing.priority),
        };
        
        if should_replace {
            best_suggestions.insert(suggestion.dimension_id, suggestion);
        }
    }
    
    best_suggestions.into_values().collect()
}

/// Convert priority to numeric value for comparison
fn priority_value(priority: &ModulationPriority) -> u8 {
    match priority {
        ModulationPriority::Critical => 4,
        ModulationPriority::High => 3,
        ModulationPriority::Medium => 2,
        ModulationPriority::Low => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;
    
    fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
        FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
    }
    
    #[test]
    fn test_determine_priority_critical() {
        // Given: Very extreme frequency
        let priority = determine_priority_for_extreme(4.5);
        
        // Then: Should be critical
        assert_eq!(priority, ModulationPriority::Critical);
    }
    
    #[test]
    fn test_determine_priority_high() {
        // Given: Extreme frequency
        let priority = determine_priority_for_extreme(3.7);
        
        // Then: Should be high
        assert_eq!(priority, ModulationPriority::High);
    }
    
    #[test]
    fn test_determine_priority_medium() {
        // Given: Moderately extreme frequency
        let priority = determine_priority_for_extreme(3.2);
        
        // Then: Should be medium
        assert_eq!(priority, ModulationPriority::Medium);
    }
    
    #[test]
    fn test_find_harmonic_target_octave() {
        // Given: Current frequency near octave of reference
        let target = find_harmonic_target(2.1, 1.0);
        
        // Then: Should suggest octave (2.0)
        assert!((target - 2.0).abs() < 0.1);
    }
    
    #[test]
    fn test_find_harmonic_target_no_harmonic() {
        // Given: Current frequency with no nearby harmonic
        let target = find_harmonic_target(3.0, 1.0);
        
        // Then: Should move toward reference
        assert!(target < 3.0);
        assert!(target > 1.0);
    }
    
    #[test]
    fn test_generate_suggestions_extreme_frequency() {
        // Given: Pattern with extreme frequency
        let mut pattern = InterferencePattern::new();
        pattern.add_frequency(create_freq_state(4.5, 1));
        
        // When: Generating suggestions
        let suggestions = generate_modulation_suggestions(&pattern);
        
        // Then: Should suggest modulation for extreme frequency
        assert!(!suggestions.is_empty());
        assert_eq!(suggestions[0].reason, ModulationReason::TooExtreme);
        assert_eq!(suggestions[0].priority, ModulationPriority::Critical);
    }
    
    #[test]
    fn test_generate_suggestions_dissonant() {
        // Given: Pattern with dissonant frequencies
        let mut pattern = InterferencePattern::new();
        pattern.add_frequency(create_freq_state(1.0, 1));
        pattern.add_frequency(create_freq_state(3.5, 2));
        pattern.dissonances = vec![(0, 1)];
        
        // When: Generating suggestions
        let suggestions = generate_modulation_suggestions(&pattern);
        
        // Then: Should suggest modulation for dissonance
        let dissonant_suggestions: Vec<_> = suggestions.iter()
            .filter(|s| s.reason == ModulationReason::Dissonant)
            .collect();
        assert!(!dissonant_suggestions.is_empty());
    }
    
    #[test]
    fn test_generate_suggestions_unbalanced() {
        // Given: Pattern needing balance
        let mut pattern = InterferencePattern::new();
        pattern.add_frequency(create_freq_state(3.0, 1));
        pattern.balance_needed = true;
        
        // When: Generating suggestions
        let suggestions = generate_modulation_suggestions(&pattern);
        
        // Then: Should suggest modulation for balance
        let unbalanced_suggestions: Vec<_> = suggestions.iter()
            .filter(|s| s.reason == ModulationReason::Unbalanced)
            .collect();
        assert!(!unbalanced_suggestions.is_empty());
    }
    
    #[test]
    fn test_generate_suggestions_constructive() {
        // Given: Pattern with strong harmonic
        let mut pattern = InterferencePattern::new();
        pattern.add_frequency(create_freq_state(1.0, 1));
        pattern.add_frequency(create_freq_state(2.0, 2));
        
        let harmonic = HarmonicRelationship::new(
            0,
            1,
            HarmonicType::Octave,
            0.9, // Strong harmonic
        );
        pattern.add_harmonic(harmonic);
        
        // When: Generating suggestions
        let suggestions = generate_modulation_suggestions(&pattern);
        
        // Then: Should suggest reinforcement
        let constructive_suggestions: Vec<_> = suggestions.iter()
            .filter(|s| s.reason == ModulationReason::Constructive)
            .collect();
        assert!(!constructive_suggestions.is_empty());
    }
    
    #[test]
    fn test_deduplicate_suggestions() {
        // Given: Multiple suggestions for same dimension
        let suggestions = vec![
            ModulationSuggestion::new(
                DimensionId(1),
                Frequency::new(3.0),
                Frequency::new(2.0),
                ModulationReason::Unbalanced,
                ModulationPriority::Medium,
                0.5,
            ),
            ModulationSuggestion::new(
                DimensionId(1),
                Frequency::new(3.0),
                Frequency::new(1.5),
                ModulationReason::TooExtreme,
                ModulationPriority::High,
                0.8,
            ),
        ];
        
        // When: Deduplicating
        let deduped = deduplicate_suggestions(suggestions);
        
        // Then: Should keep only highest priority
        assert_eq!(deduped.len(), 1);
        assert_eq!(deduped[0].priority, ModulationPriority::High);
    }
    
    #[test]
    fn test_generate_suggestions_empty_pattern() {
        // Given: Empty pattern
        let pattern = InterferencePattern::new();
        
        // When: Generating suggestions
        let suggestions = generate_modulation_suggestions(&pattern);
        
        // Then: Should return empty list
        assert!(suggestions.is_empty());
    }
}
