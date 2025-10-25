//! Harmonic relationship detection and analysis
//!
//! This module detects harmonic relationships between frequencies using ratio matching.
//! Supported harmonics: Octave (2:1), Perfect 5th (3:2), Perfect 4th (4:3), Major 3rd (5:4).

use super::{HarmonicRelationship, HarmonicType, FrequencyState};

/// Analyzer for detecting harmonic relationships between frequencies
///
/// Uses ratio matching with configurable tolerance to identify musical/mathematical
/// relationships between frequency pairs.
#[derive(Debug, Clone)]
pub struct HarmonicAnalyzer {
    /// Tolerance for ratio matching (default: 0.05 = 5%)
    tolerance: f32,
}

impl HarmonicAnalyzer {
    /// Create new harmonic analyzer with default tolerance
    pub fn new() -> Self {
        Self {
            tolerance: 0.05,
        }
    }
    
    /// Create analyzer with custom tolerance
    pub fn with_tolerance(tolerance: f32) -> Self {
        Self { tolerance }
    }
    
    /// Analyze all frequency pairs and detect harmonic relationships
    ///
    /// Returns a vector of harmonic relationships found between frequency pairs.
    /// Uses O(n²) algorithm to check all pairs.
    pub fn analyze_all(&self, frequencies: &[FrequencyState]) -> Vec<HarmonicRelationship> {
        let mut harmonics = Vec::new();
        
        for i in 0..frequencies.len() {
            for j in (i + 1)..frequencies.len() {
                if let Some(harmonic) = self.analyze_pair(
                    frequencies[i].frequency.hz(),
                    frequencies[j].frequency.hz(),
                    i,
                    j,
                ) {
                    harmonics.push(harmonic);
                }
            }
        }
        
        harmonics
    }
    
    /// Analyze a single frequency pair for harmonic relationship
    ///
    /// Returns Some(HarmonicRelationship) if a harmonic is detected, None otherwise.
    pub fn analyze_pair(
        &self,
        freq1: f32,
        freq2: f32,
        index1: usize,
        index2: usize,
    ) -> Option<HarmonicRelationship> {
        let (lower, higher, lower_idx, higher_idx) = if freq1 < freq2 {
            (freq1, freq2, index1, index2)
        } else {
            (freq2, freq1, index2, index1)
        };
        
        // Avoid division by zero
        if lower < 0.001 {
            return None;
        }
        
        let ratio = higher / lower;
        
        // Check each harmonic type
        if let Some((harmonic_type, strength)) = self.detect_harmonic_type(ratio) {
            Some(HarmonicRelationship::new(
                lower_idx,
                higher_idx,
                harmonic_type,
                strength,
            ))
        } else {
            None
        }
    }
    
    /// Detect the type of harmonic relationship from a frequency ratio
    ///
    /// Returns (HarmonicType, strength) if a harmonic is detected.
    /// Strength is 1.0 for perfect match, decreasing linearly to 0.0 at tolerance edge.
    fn detect_harmonic_type(&self, ratio: f32) -> Option<(HarmonicType, f32)> {
        // Check octave (2:1)
        if let Some(strength) = self.calculate_strength(ratio, 2.0) {
            return Some((HarmonicType::Octave, strength));
        }
        
        // Check perfect 5th (3:2 = 1.5)
        if let Some(strength) = self.calculate_strength(ratio, 1.5) {
            return Some((HarmonicType::Perfect5th, strength));
        }
        
        // Check perfect 4th (4:3 ≈ 1.333)
        if let Some(strength) = self.calculate_strength(ratio, 4.0 / 3.0) {
            return Some((HarmonicType::Perfect4th, strength));
        }
        
        // Check major 3rd (5:4 = 1.25)
        if let Some(strength) = self.calculate_strength(ratio, 1.25) {
            return Some((HarmonicType::MajorThird, strength));
        }
        
        None
    }
    
    /// Calculate strength of harmonic relationship
    ///
    /// Returns Some(strength) if ratio is within tolerance of target, None otherwise.
    /// Strength is 1.0 for perfect match, decreasing linearly to 0.0 at tolerance edge.
    fn calculate_strength(&self, ratio: f32, target: f32) -> Option<f32> {
        let diff = (ratio - target).abs();
        let max_diff = target * self.tolerance;
        
        if diff > max_diff {
            None
        } else {
            // Linear falloff: 1.0 at perfect match, 0.0 at tolerance edge
            Some(1.0 - (diff / max_diff))
        }
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
    
    fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
        FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
    }
    
    #[test]
    fn test_detect_octave() {
        // Given: Analyzer and two frequencies in octave relationship (2:1)
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing pair
        let result = analyzer.analyze_pair(1.0, 2.0, 0, 1);
        
        // Then: Should detect octave with high strength
        assert!(result.is_some());
        let harmonic = result.unwrap();
        assert_eq!(harmonic.relationship_type, HarmonicType::Octave);
        assert!(harmonic.strength > 0.99); // Nearly perfect
    }
    
    #[test]
    fn test_detect_perfect_5th() {
        // Given: Analyzer and two frequencies in perfect 5th relationship (3:2)
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing pair
        let result = analyzer.analyze_pair(2.0, 3.0, 0, 1);
        
        // Then: Should detect perfect 5th
        assert!(result.is_some());
        let harmonic = result.unwrap();
        assert_eq!(harmonic.relationship_type, HarmonicType::Perfect5th);
        assert!(harmonic.strength > 0.99);
    }
    
    #[test]
    fn test_detect_perfect_4th() {
        // Given: Analyzer and two frequencies in perfect 4th relationship (4:3)
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing pair
        let result = analyzer.analyze_pair(3.0, 4.0, 0, 1);
        
        // Then: Should detect perfect 4th
        assert!(result.is_some());
        let harmonic = result.unwrap();
        assert_eq!(harmonic.relationship_type, HarmonicType::Perfect4th);
        assert!(harmonic.strength > 0.99);
    }
    
    #[test]
    fn test_detect_major_3rd() {
        // Given: Analyzer and two frequencies in major 3rd relationship (5:4)
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing pair
        let result = analyzer.analyze_pair(4.0, 5.0, 0, 1);
        
        // Then: Should detect major 3rd
        assert!(result.is_some());
        let harmonic = result.unwrap();
        assert_eq!(harmonic.relationship_type, HarmonicType::MajorThird);
        assert!(harmonic.strength > 0.99);
    }
    
    #[test]
    fn test_strength_calculation_perfect_match() {
        // Given: Analyzer with default tolerance
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Calculating strength for perfect octave
        let strength = analyzer.calculate_strength(2.0, 2.0);
        
        // Then: Should return 1.0 (perfect match)
        assert!(strength.is_some());
        assert!((strength.unwrap() - 1.0).abs() < 0.001);
    }
    
    #[test]
    fn test_strength_calculation_near_match() {
        // Given: Analyzer with default tolerance (0.05)
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Calculating strength for near-octave (2.05 vs 2.0)
        let strength = analyzer.calculate_strength(2.05, 2.0);
        
        // Then: Should return value between 0 and 1
        assert!(strength.is_some());
        let s = strength.unwrap();
        assert!(s > 0.0 && s < 1.0);
    }
    
    #[test]
    fn test_strength_calculation_beyond_tolerance() {
        // Given: Analyzer with default tolerance (0.05 = 5%)
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Calculating strength for ratio beyond tolerance
        // 2.0 * 0.05 = 0.1, so 2.15 is beyond tolerance
        let strength = analyzer.calculate_strength(2.15, 2.0);
        
        // Then: Should return None
        assert!(strength.is_none());
    }
    
    #[test]
    fn test_analyze_pair_order_independence() {
        // Given: Analyzer
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing same pair in different orders
        let result1 = analyzer.analyze_pair(1.0, 2.0, 0, 1);
        let result2 = analyzer.analyze_pair(2.0, 1.0, 1, 0);
        
        // Then: Should detect same harmonic type
        assert!(result1.is_some());
        assert!(result2.is_some());
        assert_eq!(
            result1.unwrap().relationship_type,
            result2.unwrap().relationship_type
        );
    }
    
    #[test]
    fn test_analyze_pair_no_harmonic() {
        // Given: Analyzer and two frequencies with no harmonic relationship
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing pair
        let result = analyzer.analyze_pair(1.0, 1.7, 0, 1);
        
        // Then: Should return None
        assert!(result.is_none());
    }
    
    #[test]
    fn test_analyze_all_multiple_harmonics() {
        // Given: Multiple frequencies with harmonic relationships
        let analyzer = HarmonicAnalyzer::new();
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(2.0, 2),  // Octave with 1.0
            create_freq_state(1.5, 3),  // Perfect 5th with 1.0, Perfect 4th with 2.0
        ];
        
        // When: Analyzing all pairs
        let harmonics = analyzer.analyze_all(&frequencies);
        
        // Then: Should find three harmonics (1.0-2.0 octave, 1.0-1.5 5th, 1.5-2.0 4th)
        assert_eq!(harmonics.len(), 3);
        
        // Check that we found the expected harmonics
        let has_octave = harmonics.iter().any(|h| h.relationship_type == HarmonicType::Octave);
        let has_5th = harmonics.iter().any(|h| h.relationship_type == HarmonicType::Perfect5th);
        let has_4th = harmonics.iter().any(|h| h.relationship_type == HarmonicType::Perfect4th);
        assert!(has_octave);
        assert!(has_5th);
        assert!(has_4th);
    }
    
    #[test]
    fn test_analyze_all_empty() {
        // Given: Empty frequency list
        let analyzer = HarmonicAnalyzer::new();
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Analyzing all pairs
        let harmonics = analyzer.analyze_all(&frequencies);
        
        // Then: Should return empty vector
        assert_eq!(harmonics.len(), 0);
    }
    
    #[test]
    fn test_custom_tolerance() {
        // Given: Analyzer with tight tolerance
        let analyzer = HarmonicAnalyzer::with_tolerance(0.01);
        
        // When: Analyzing slightly off octave (2.05 vs 2.0)
        let result = analyzer.analyze_pair(1.0, 2.05, 0, 1);
        
        // Then: Should NOT detect harmonic (beyond 1% tolerance)
        assert!(result.is_none());
    }
    
    #[test]
    fn test_zero_frequency_handling() {
        // Given: Analyzer and zero frequency
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing pair with zero
        let result = analyzer.analyze_pair(0.0, 2.0, 0, 1);
        
        // Then: Should return None (avoid division by zero)
        assert!(result.is_none());
    }
}
