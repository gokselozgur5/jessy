//! Frequency interference engine for consciousness system
//!
//! This module calculates interference patterns between multiple dimensional frequencies,
//! determines dominant frequencies, and detects harmonic/dissonance relationships.

pub mod engine;
pub mod patterns;
pub mod harmonics;

pub use engine::InterferenceEngine;
pub use harmonics::HarmonicAnalyzer;

use crate::{Frequency, Result};
use std::collections::HashMap;

/// Configuration for interference calculations
#[derive(Debug, Clone)]
pub struct InterferenceConfig {
    /// Threshold for constructive interference
    pub constructive_threshold: f32,
    
    /// Threshold for destructive interference  
    pub destructive_threshold: f32,
    
    /// Weight for harmonic relationships
    pub harmonic_weight: f32,
    
    /// Weight for dissonance relationships
    pub dissonance_weight: f32,
    
    /// Balance modulation strength
    pub balance_modulation: f32,
}

impl Default for InterferenceConfig {
    fn default() -> Self {
        Self {
            constructive_threshold: 0.2,   // Frequencies within 0.2 Hz reinforce
            destructive_threshold: 2.0,    // Frequencies >2.0 Hz apart may cancel
            harmonic_weight: 1.5,          // Boost harmonic relationships
            dissonance_weight: 0.5,        // Reduce dissonant relationships
            balance_modulation: 0.8,       // Strength of balance dimension influence
        }
    }
}

/// State of a frequency in the interference calculation
#[derive(Debug, Clone)]
pub struct FrequencyState {
    pub frequency: Frequency,
    pub amplitude: f32,
    pub phase: f32,
    pub dimension_id: crate::DimensionId,
    pub confidence: f32,
}

impl FrequencyState {
    /// Create new frequency state
    pub fn new(
        frequency: Frequency,
        dimension_id: crate::DimensionId,
        confidence: f32,
    ) -> Self {
        Self {
            frequency,
            amplitude: confidence, // Use confidence as amplitude
            phase: 0.0,           // Default phase
            dimension_id,
            confidence,
        }
    }
    
    /// Calculate phase difference with another frequency
    pub fn phase_difference(&self, other: &FrequencyState) -> f32 {
        (self.frequency.hz() - other.frequency.hz()).abs()
    }
    
    /// Check if this frequency is in constructive interference with another
    pub fn is_constructive_with(&self, other: &FrequencyState, threshold: f32) -> bool {
        self.phase_difference(other) < threshold
    }
    
    /// Check if this frequency is in destructive interference with another
    pub fn is_destructive_with(&self, other: &FrequencyState, threshold: f32) -> bool {
        self.phase_difference(other) > threshold
    }
}

/// Pattern of interference between frequencies
#[derive(Debug, Clone)]
pub struct InterferencePattern {
    pub frequencies: Vec<FrequencyState>,
    pub dominant_frequency: Frequency,
    pub amplitude: f32,
    pub harmonics: Vec<Frequency>,
    pub dissonances: Vec<(usize, usize)>, // Indices of dissonant frequency pairs
    pub constructive_pairs: Vec<(usize, usize)>,
    pub destructive_pairs: Vec<(usize, usize)>,
    pub balance_needed: bool,
    pub complexity_score: f32,
}

impl InterferencePattern {
    /// Create new interference pattern
    pub fn new() -> Self {
        Self {
            frequencies: Vec::new(),
            dominant_frequency: Frequency::new(1.0),
            amplitude: 0.0,
            harmonics: Vec::new(),
            dissonances: Vec::new(),
            constructive_pairs: Vec::new(),
            destructive_pairs: Vec::new(),
            balance_needed: false,
            complexity_score: 0.0,
        }
    }
    
    /// Add frequency to the pattern
    pub fn add_frequency(&mut self, freq_state: FrequencyState) {
        self.frequencies.push(freq_state);
    }
    
    /// Calculate if balance dimension should be activated
    pub fn needs_balance(&self) -> bool {
        self.dominant_frequency.is_extreme() || 
        self.dissonances.len() > 2 ||
        self.complexity_score > 3.0
    }
    
    /// Get the number of active frequencies
    pub fn frequency_count(&self) -> usize {
        self.frequencies.len()
    }
    
    /// Check if pattern is stable (low dissonance)
    pub fn is_stable(&self) -> bool {
        self.dissonances.len() <= 1 && self.amplitude > 0.5
    }
    
    /// Get frequencies that need modulation
    pub fn frequencies_needing_modulation(&self) -> Vec<&FrequencyState> {
        self.frequencies.iter()
            .filter(|f| f.frequency.is_extreme())
            .collect()
    }
}

impl Default for InterferencePattern {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of interference calculation
#[derive(Debug)]
pub struct InterferenceResult {
    pub pattern: InterferencePattern,
    pub modulation_suggestions: Vec<ModulationSuggestion>,
    pub balance_activation_needed: bool,
    pub return_to_source_suggested: bool,
}

/// Suggestion for frequency modulation
#[derive(Debug, Clone)]
pub struct ModulationSuggestion {
    pub dimension_id: crate::DimensionId,
    pub current_frequency: Frequency,
    pub suggested_frequency: Frequency,
    pub reason: ModulationReason,
    pub priority: ModulationPriority,
}

/// Reason for frequency modulation
#[derive(Debug, Clone, PartialEq)]
pub enum ModulationReason {
    TooExtreme,         // Frequency >3.5 Hz needs dampening
    Dissonant,          // Conflicts with other frequencies
    Unbalanced,         // Needs centering toward equilibrium
    Constructive,       // Boost for harmonic reinforcement
}

/// Priority level for modulation
#[derive(Debug, Clone, PartialEq)]
pub enum ModulationPriority {
    Critical,   // Must be modulated for system stability
    High,       // Should be modulated for optimal performance
    Medium,     // Could be modulated for improvement
    Low,        // Optional modulation
}

/// Harmonic relationship between frequencies
#[derive(Debug, Clone)]
pub struct HarmonicRelationship {
    pub freq1_index: usize,
    pub freq2_index: usize,
    pub relationship_type: HarmonicType,
    pub strength: f32,
}

/// Type of harmonic relationship
#[derive(Debug, Clone, PartialEq)]
pub enum HarmonicType {
    Fundamental,    // Base frequency
    Octave,         // 2:1 ratio
    Perfect5th,     // 3:2 ratio
    Perfect4th,     // 4:3 ratio
    MajorThird,     // 5:4 ratio
    Subharmonic,    // 1:2 ratio (below fundamental)
    Dissonant,      // No clear harmonic relationship
}

impl HarmonicRelationship {
    /// Create new harmonic relationship
    pub fn new(
        freq1_index: usize,
        freq2_index: usize,
        relationship_type: HarmonicType,
        strength: f32,
    ) -> Self {
        Self {
            freq1_index,
            freq2_index,
            relationship_type,
            strength,
        }
    }
    
    /// Check if this is a consonant (pleasant) relationship
    pub fn is_consonant(&self) -> bool {
        matches!(
            self.relationship_type,
            HarmonicType::Octave | 
            HarmonicType::Perfect5th | 
            HarmonicType::Perfect4th |
            HarmonicType::MajorThird
        )
    }
    
    /// Check if this relationship should be reinforced
    pub fn should_reinforce(&self) -> bool {
        self.is_consonant() && self.strength > 0.6
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;
    
    #[test]
    fn test_frequency_state() {
        let freq1 = FrequencyState::new(
            Frequency::new(1.0),
            DimensionId(1),
            0.8,
        );
        
        let freq2 = FrequencyState::new(
            Frequency::new(1.1),
            DimensionId(2),
            0.7,
        );
        
        assert!(freq1.is_constructive_with(&freq2, 0.2));
        assert!(!freq1.is_destructive_with(&freq2, 2.0));
        assert_eq!(freq1.phase_difference(&freq2), 0.1);
    }
    
    #[test]
    fn test_interference_pattern() {
        let mut pattern = InterferencePattern::new();
        
        let freq_state = FrequencyState::new(
            Frequency::new(4.0), // Extreme frequency
            DimensionId(1),
            0.9,
        );
        
        pattern.add_frequency(freq_state);
        pattern.dominant_frequency = Frequency::new(4.0);
        
        assert!(pattern.needs_balance());
        assert_eq!(pattern.frequency_count(), 1);
        assert_eq!(pattern.frequencies_needing_modulation().len(), 1);
    }
    
    #[test]
    fn test_harmonic_relationship() {
        let harmonic = HarmonicRelationship::new(
            0,
            1,
            HarmonicType::Perfect5th,
            0.8,
        );
        
        assert!(harmonic.is_consonant());
        assert!(harmonic.should_reinforce());
    }
    
    #[test]
    fn test_interference_config() {
        let config = InterferenceConfig::default();
        assert_eq!(config.constructive_threshold, 0.2);
        assert_eq!(config.destructive_threshold, 2.0);
        assert!(config.harmonic_weight > 1.0);
        assert!(config.dissonance_weight < 1.0);
    }
}