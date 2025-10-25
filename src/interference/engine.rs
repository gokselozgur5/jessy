//! Interference Engine - Main orchestrator for frequency interference calculation
//!
//! This module provides the main InterferenceEngine that coordinates all aspects
//! of interference calculation: pair detection, harmonic analysis, amplitude
//! modulation, balance detection, and modulation suggestions.

use crate::{Result, Frequency};
use super::{
    InterferenceResult, InterferencePattern, InterferenceConfig,
    FrequencyState, HarmonicAnalyzer,
};

/// Main interference calculation engine
///
/// Orchestrates the full interference calculation pipeline:
/// 1. Collect frequency states
/// 2. Detect constructive/destructive pairs
/// 3. Analyze harmonic relationships
/// 4. Calculate interference effects
/// 5. Determine dominant frequency
/// 6. Detect balance needs
/// 7. Generate modulation suggestions
#[derive(Debug)]
pub struct InterferenceEngine {
    config: InterferenceConfig,
    harmonic_analyzer: HarmonicAnalyzer,
}

impl InterferenceEngine {
    /// Create new interference engine with default configuration
    pub fn new() -> Self {
        Self {
            config: InterferenceConfig::default(),
            harmonic_analyzer: HarmonicAnalyzer::new(),
        }
    }
    
    /// Create interference engine with custom configuration
    pub fn with_config(config: InterferenceConfig) -> Self {
        Self {
            config,
            harmonic_analyzer: HarmonicAnalyzer::new(),
        }
    }
    
    /// Calculate interference pattern from frequency states
    ///
    /// This is the main entry point for interference calculation.
    /// Performs the complete analysis pipeline and returns structured results.
    ///
    /// # Performance
    /// Target: <10ms for up to 14 frequencies
    /// - Frequency collection: <1ms
    /// - Pair detection: <5ms (O(n²))
    /// - Harmonic analysis: <5ms (O(n²))
    /// - Dominant calculation: <1ms
    /// - Balance detection: <1ms
    pub fn calculate(&self, frequencies: &[FrequencyState]) -> Result<InterferenceResult> {
        // Step 1: Create pattern with frequency states
        let mut pattern = InterferencePattern::new();
        for freq_state in frequencies {
            pattern.add_frequency(freq_state.clone());
        }
        
        // Step 2: Detect constructive and destructive pairs
        let constructive_pairs = super::pair_detection::detect_constructive_pairs(
            &pattern.frequencies,
            self.config.constructive_threshold,
        );
        let destructive_pairs = super::pair_detection::detect_destructive_pairs(
            &pattern.frequencies,
            self.config.destructive_threshold,
        );
        
        pattern.constructive_pairs = constructive_pairs.clone();
        pattern.destructive_pairs = destructive_pairs.clone();
        
        // Step 3: Analyze harmonic relationships
        let harmonics = self.harmonic_analyzer.analyze_all(&pattern.frequencies);
        pattern.harmonics = harmonics;
        
        // Identify dissonances (pairs that are neither constructive nor harmonic)
        pattern.dissonances = self.identify_dissonances(&pattern);
        
        // Step 4: Calculate interference effects on amplitudes
        super::calculation::calculate_interference_effects(
            &mut pattern.frequencies,
            &constructive_pairs,
            &destructive_pairs,
            &self.config,
        );
        
        // Step 5: Calculate dominant frequency
        let dominant_hz = super::calculation::calculate_dominant_frequency(&pattern.frequencies);
        pattern.dominant_frequency = Frequency::new(dominant_hz);
        
        // Step 6: Calculate overall amplitude
        pattern.amplitude = super::calculation::calculate_overall_amplitude(&pattern.frequencies);
        
        // Step 7: Calculate complexity score
        let frequency_spread = super::balance::calculate_frequency_spread(&pattern.frequencies);
        pattern.complexity_score = super::balance::calculate_complexity_score(
            pattern.frequencies.len(),
            pattern.dissonances.len(),
            frequency_spread,
        );
        
        // Step 8: Detect balance needs
        pattern.balance_needed = super::balance::detect_balance_needs(&pattern);
        
        // Step 9: Generate modulation suggestions
        let modulation_suggestions = super::modulation::generate_modulation_suggestions(&pattern);
        
        // Step 10: Determine if return-to-source is needed
        let return_to_source_suggested = self.should_return_to_source(&pattern);
        
        // Capture values before moving pattern
        let balance_activation_needed = pattern.balance_needed;
        
        // Return complete result
        Ok(InterferenceResult {
            pattern,
            modulation_suggestions,
            balance_activation_needed,
            return_to_source_suggested,
        })
    }
    
    /// Identify dissonant frequency pairs
    ///
    /// Dissonances are pairs that are destructive but not harmonic.
    fn identify_dissonances(&self, pattern: &InterferencePattern) -> Vec<(usize, usize)> {
        let mut dissonances = Vec::new();
        
        for &(i, j) in &pattern.destructive_pairs {
            // Check if this pair has a harmonic relationship
            let has_harmonic = pattern.harmonics.iter().any(|h| {
                (h.freq1_index == i && h.freq2_index == j) ||
                (h.freq1_index == j && h.freq2_index == i)
            });
            
            // If destructive but not harmonic, it's dissonant
            if !has_harmonic {
                dissonances.push((i, j));
            }
        }
        
        dissonances
    }
    
    /// Determine if return-to-source protocol should be triggered
    ///
    /// Returns true when:
    /// - Frequency count exceeds 6 (too many dimensions active)
    /// - Complexity score exceeds 5.0 (system too complex)
    fn should_return_to_source(&self, pattern: &InterferencePattern) -> bool {
        pattern.frequencies.len() > 6 || pattern.complexity_score > 5.0
    }
}

impl Default for InterferenceEngine {
    fn default() -> Self {
        Self::new()
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
    fn test_engine_initialization() {
        // Given/When: Creating engine
        let engine = InterferenceEngine::new();
        
        // Then: Should initialize successfully
        assert_eq!(engine.config.constructive_threshold, 0.2);
        assert_eq!(engine.config.destructive_threshold, 2.0);
    }
    
    #[test]
    fn test_engine_with_custom_config() {
        // Given: Custom configuration
        let mut config = InterferenceConfig::default();
        config.constructive_threshold = 0.3;
        
        // When: Creating engine with config
        let engine = InterferenceEngine::with_config(config);
        
        // Then: Should use custom config
        assert_eq!(engine.config.constructive_threshold, 0.3);
    }
    
    #[test]
    fn test_calculate_single_frequency() {
        // Given: Engine and single frequency
        let engine = InterferenceEngine::new();
        let frequencies = vec![create_freq_state(1.5, 1)];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies);
        
        // Then: Should succeed
        assert!(result.is_ok());
        let result = result.unwrap();
        assert_eq!(result.pattern.frequencies.len(), 1);
        assert!((result.pattern.dominant_frequency.hz() - 1.5).abs() < 0.01);
    }
    
    #[test]
    fn test_calculate_constructive_pair() {
        // Given: Engine and two close frequencies
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(1.1, 2),
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should detect constructive pair
        assert_eq!(result.pattern.constructive_pairs.len(), 1);
        assert_eq!(result.pattern.constructive_pairs[0], (0, 1));
    }
    
    #[test]
    fn test_calculate_destructive_pair() {
        // Given: Engine and two far frequencies
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(3.5, 2),
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should detect destructive pair
        assert_eq!(result.pattern.destructive_pairs.len(), 1);
        assert_eq!(result.pattern.destructive_pairs[0], (0, 1));
    }
    
    #[test]
    fn test_calculate_harmonic_relationship() {
        // Given: Engine and two frequencies in octave relationship
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(2.0, 2),
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should detect harmonic
        assert!(!result.pattern.harmonics.is_empty());
    }
    
    #[test]
    fn test_calculate_dissonance() {
        // Given: Engine and two frequencies that are destructive but not harmonic
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(3.7, 2), // Far apart, no harmonic relationship
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should detect dissonance
        assert!(!result.pattern.dissonances.is_empty());
    }
    
    #[test]
    fn test_calculate_balance_needed() {
        // Given: Engine and extreme frequency
        let engine = InterferenceEngine::new();
        let frequencies = vec![create_freq_state(4.5, 1)];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should flag balance needed
        assert!(result.balance_activation_needed);
    }
    
    #[test]
    fn test_calculate_modulation_suggestions() {
        // Given: Engine and extreme frequency
        let engine = InterferenceEngine::new();
        let frequencies = vec![create_freq_state(4.5, 1)];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should generate modulation suggestions
        assert!(!result.modulation_suggestions.is_empty());
    }
    
    #[test]
    fn test_calculate_return_to_source() {
        // Given: Engine and many frequencies
        let engine = InterferenceEngine::new();
        let frequencies: Vec<_> = (1..=8)
            .map(|i| create_freq_state(i as f32, i))
            .collect();
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should suggest return to source
        assert!(result.return_to_source_suggested);
    }
    
    #[test]
    fn test_calculate_complexity_score() {
        // Given: Engine and multiple frequencies
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(2.0, 2),
            create_freq_state(3.5, 3),
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should calculate complexity score
        assert!(result.pattern.complexity_score > 0.0);
    }
    
    #[test]
    fn test_calculate_empty_frequencies() {
        // Given: Engine and empty frequency list
        let engine = InterferenceEngine::new();
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should return valid result
        assert_eq!(result.pattern.frequencies.len(), 0);
        assert!(!result.balance_activation_needed);
        assert!(!result.return_to_source_suggested);
    }
    
    #[test]
    fn test_full_pipeline_integration() {
        // Given: Engine and realistic frequency set
        let engine = InterferenceEngine::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),   // Base
            create_freq_state(1.5, 2),   // Perfect 5th with base
            create_freq_state(2.0, 3),   // Octave with base
            create_freq_state(4.2, 4),   // Dissonant, very extreme
        ];
        
        // When: Calculating interference
        let result = engine.calculate(&frequencies).unwrap();
        
        // Then: Should have complete analysis
        assert_eq!(result.pattern.frequencies.len(), 4);
        assert!(!result.pattern.harmonics.is_empty()); // Should find harmonics
        assert!(!result.pattern.dissonances.is_empty()); // Should find dissonance
        // Balance may or may not be needed depending on dominant frequency calculation
        assert!(!result.modulation_suggestions.is_empty()); // Should suggest modulation for extreme freq
        assert!(result.pattern.complexity_score > 0.0);
    }
}

