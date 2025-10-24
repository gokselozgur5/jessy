//! Interference engine for frequency calculation

use crate::{Frequency, Result};
use super::{InterferenceConfig, InterferenceResult, InterferencePattern, FrequencyState};

/// Main interference engine
pub struct InterferenceEngine {
    config: InterferenceConfig,
}

impl InterferenceEngine {
    /// Create new interference engine
    pub fn new() -> Self {
        Self {
            config: InterferenceConfig::default(),
        }
    }
    
    /// Create engine with custom configuration
    pub fn with_config(config: InterferenceConfig) -> Self {
        Self { config }
    }
    
    /// Calculate interference patterns from frequencies
    pub fn calculate(&self, frequencies: &[Frequency]) -> Result<InterferenceResult> {
        let mut pattern = InterferencePattern::new();
        
        // Convert frequencies to states
        for (i, freq) in frequencies.iter().enumerate() {
            let state = FrequencyState::new(
                *freq,
                crate::DimensionId(i as u8),
                1.0, // Default confidence
            );
            pattern.add_frequency(state);
        }
        
        // Calculate dominant frequency (simple average for now)
        if !frequencies.is_empty() {
            let avg = frequencies.iter().map(|f| f.hz()).sum::<f32>() / frequencies.len() as f32;
            pattern.dominant_frequency = Frequency::new(avg);
            pattern.amplitude = 1.0;
        }
        
        // Check if balance is needed
        pattern.balance_needed = pattern.needs_balance();
        
        Ok(InterferenceResult {
            pattern,
            modulation_suggestions: vec![],
            balance_activation_needed: false,
            return_to_source_suggested: false,
        })
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
    
    #[test]
    fn test_interference_calculation() {
        let engine = InterferenceEngine::new();
        
        let frequencies = vec![
            Frequency::new(1.0),
            Frequency::new(1.5),
            Frequency::new(2.0),
        ];
        
        let result = engine.calculate(&frequencies).unwrap();
        
        assert_eq!(result.pattern.frequency_count(), 3);
        assert!(result.pattern.dominant_frequency.hz() > 0.0);
    }
    
    #[test]
    fn test_extreme_frequency_detection() {
        let engine = InterferenceEngine::new();
        
        let frequencies = vec![
            Frequency::new(4.0), // Extreme
        ];
        
        let result = engine.calculate(&frequencies).unwrap();
        assert!(result.pattern.needs_balance());
    }
}
