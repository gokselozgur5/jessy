//! Harmonic analysis for frequency relationships

use super::{HarmonicRelationship, HarmonicType, FrequencyState};

/// Harmonic analyzer for frequency relationships
pub struct HarmonicAnalyzer {
    tolerance: f32,
}

impl HarmonicAnalyzer {
    /// Create new harmonic analyzer
    pub fn new() -> Self {
        Self {
            tolerance: 0.1, // 10% tolerance for harmonic detection
        }
    }
    
    /// Analyze harmonic relationships between frequencies
    pub fn analyze(&self, frequencies: &[FrequencyState]) -> Vec<HarmonicRelationship> {
        let mut relationships = Vec::new();
        
        for i in 0..frequencies.len() {
            for j in (i + 1)..frequencies.len() {
                if let Some(relationship) = self.detect_relationship(&frequencies[i], &frequencies[j], i, j) {
                    relationships.push(relationship);
                }
            }
        }
        
        relationships
    }
    
    /// Detect harmonic relationship between two frequencies
    fn detect_relationship(
        &self,
        freq1: &FrequencyState,
        freq2: &FrequencyState,
        idx1: usize,
        idx2: usize,
    ) -> Option<HarmonicRelationship> {
        let ratio = freq2.frequency.hz() / freq1.frequency.hz();
        
        let (relationship_type, strength) = if self.is_near(ratio, 2.0) {
            (HarmonicType::Octave, 0.9)
        } else if self.is_near(ratio, 1.5) {
            (HarmonicType::Perfect5th, 0.8)
        } else if self.is_near(ratio, 1.333) {
            (HarmonicType::Perfect4th, 0.8)
        } else if self.is_near(ratio, 1.25) {
            (HarmonicType::MajorThird, 0.7)
        } else if self.is_near(ratio, 0.5) {
            (HarmonicType::Subharmonic, 0.7)
        } else {
            (HarmonicType::Dissonant, 0.3)
        };
        
        Some(HarmonicRelationship::new(
            idx1,
            idx2,
            relationship_type,
            strength,
        ))
    }
    
    /// Check if value is near target within tolerance
    fn is_near(&self, value: f32, target: f32) -> bool {
        (value - target).abs() / target < self.tolerance
    }
}

impl Default for HarmonicAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frequency, DimensionId};
    
    #[test]
    fn test_harmonic_analyzer() {
        let analyzer = HarmonicAnalyzer::new();
        
        let frequencies = vec![
            FrequencyState::new(Frequency::new(1.0), DimensionId(1), 0.8),
            FrequencyState::new(Frequency::new(2.0), DimensionId(2), 0.8), // Octave
        ];
        
        let relationships = analyzer.analyze(&frequencies);
        
        assert_eq!(relationships.len(), 1);
        assert_eq!(relationships[0].relationship_type, HarmonicType::Octave);
        assert!(relationships[0].is_consonant());
    }
    
    #[test]
    fn test_dissonant_detection() {
        let analyzer = HarmonicAnalyzer::new();
        
        let frequencies = vec![
            FrequencyState::new(Frequency::new(1.0), DimensionId(1), 0.8),
            FrequencyState::new(Frequency::new(1.7), DimensionId(2), 0.8), // Dissonant
        ];
        
        let relationships = analyzer.analyze(&frequencies);
        
        assert_eq!(relationships.len(), 1);
        assert_eq!(relationships[0].relationship_type, HarmonicType::Dissonant);
        assert!(!relationships[0].is_consonant());
    }
}
