//! Balance modulation detection and target calculation
//!
//! This module detects when frequencies need balance modulation and calculates
//! target frequencies to bring the system back to equilibrium.

use super::{InterferencePattern, FrequencyState};

/// Balance center frequency (D13 Balance dimension operates here)
const BALANCE_CENTER: f32 = 1.2;

/// Pull strength for balance modulation (0.0-1.0)
const PULL_STRENGTH: f32 = 0.8;

/// Detect if balance modulation is needed
///
/// Balance is needed when:
/// - Dominant frequency exceeds 3.5 Hz (too extreme)
/// - Dissonance count exceeds 2 (too much conflict)
/// - Complexity score exceeds 3.0 (too complex)
pub fn detect_balance_needs(pattern: &InterferencePattern) -> bool {
    pattern.dominant_frequency.hz() > 3.5 ||
    pattern.dissonances.len() > 2 ||
    pattern.complexity_score > 3.0
}

/// Calculate target frequency for balance modulation
///
/// Pulls extreme frequencies toward the balance center (1.2 Hz) with
/// configurable pull strength (default 0.8).
///
/// # Formula
/// target = current + (balance_center - current) * pull_strength
pub fn calculate_balance_target(current_frequency: f32) -> f32 {
    current_frequency + (BALANCE_CENTER - current_frequency) * PULL_STRENGTH
}

/// Calculate modulation strength based on how extreme the frequency is
///
/// Returns value between 0.0 and 1.0:
/// - 0.0 for frequencies near balance center
/// - 1.0 for very extreme frequencies (>4.0 Hz)
pub fn calculate_modulation_strength(current_frequency: f32) -> f32 {
    let distance_from_center = (current_frequency - BALANCE_CENTER).abs();
    
    // Linear scaling: 0.0 at center, 1.0 at 4.0 Hz from center
    (distance_from_center / 4.0).min(1.0)
}

/// Calculate complexity score for interference pattern
///
/// Complexity increases with:
/// - Number of active frequencies
/// - Number of dissonances
/// - Spread of frequencies
pub fn calculate_complexity_score(
    frequency_count: usize,
    dissonance_count: usize,
    frequency_spread: f32,
) -> f32 {
    let count_factor = frequency_count as f32 * 0.3;
    let dissonance_factor = dissonance_count as f32 * 0.5;
    let spread_factor = frequency_spread * 0.2;
    
    count_factor + dissonance_factor + spread_factor
}

/// Calculate frequency spread (max - min)
pub fn calculate_frequency_spread(frequencies: &[FrequencyState]) -> f32 {
    if frequencies.is_empty() {
        return 0.0;
    }
    
    let min = frequencies.iter()
        .map(|f| f.frequency.hz())
        .fold(f32::INFINITY, f32::min);
    
    let max = frequencies.iter()
        .map(|f| f.frequency.hz())
        .fold(f32::NEG_INFINITY, f32::max);
    
    max - min
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frequency, DimensionId};
    
    fn create_pattern_with_dominant(hz: f32) -> InterferencePattern {
        let mut pattern = InterferencePattern::new();
        pattern.dominant_frequency = Frequency::new(hz);
        pattern
    }
    
    fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
        FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
    }
    
    #[test]
    fn test_detect_balance_needs_extreme_frequency() {
        // Given: Pattern with extreme dominant frequency
        let pattern = create_pattern_with_dominant(4.0);
        
        // When: Detecting balance needs
        let needs_balance = detect_balance_needs(&pattern);
        
        // Then: Should need balance
        assert!(needs_balance);
    }
    
    #[test]
    fn test_detect_balance_needs_high_dissonance() {
        // Given: Pattern with high dissonance count
        let mut pattern = create_pattern_with_dominant(2.0);
        pattern.dissonances = vec![(0, 1), (0, 2), (1, 2)]; // 3 dissonances
        
        // When: Detecting balance needs
        let needs_balance = detect_balance_needs(&pattern);
        
        // Then: Should need balance
        assert!(needs_balance);
    }
    
    #[test]
    fn test_detect_balance_needs_high_complexity() {
        // Given: Pattern with high complexity score
        let mut pattern = create_pattern_with_dominant(2.0);
        pattern.complexity_score = 3.5;
        
        // When: Detecting balance needs
        let needs_balance = detect_balance_needs(&pattern);
        
        // Then: Should need balance
        assert!(needs_balance);
    }
    
    #[test]
    fn test_detect_balance_needs_normal() {
        // Given: Pattern with normal values
        let mut pattern = create_pattern_with_dominant(2.0);
        pattern.dissonances = vec![(0, 1)]; // 1 dissonance
        pattern.complexity_score = 2.0;
        
        // When: Detecting balance needs
        let needs_balance = detect_balance_needs(&pattern);
        
        // Then: Should NOT need balance
        assert!(!needs_balance);
    }
    
    #[test]
    fn test_calculate_balance_target_high_frequency() {
        // Given: High frequency (4.0 Hz)
        let current = 4.0;
        
        // When: Calculating balance target
        let target = calculate_balance_target(current);
        
        // Then: Should pull toward balance center
        // 4.0 + (1.2 - 4.0) * 0.8 = 4.0 - 2.24 = 1.76
        assert!((target - 1.76).abs() < 0.01);
        assert!(target < current);
        assert!(target > BALANCE_CENTER);
    }
    
    #[test]
    fn test_calculate_balance_target_low_frequency() {
        // Given: Low frequency (0.5 Hz)
        let current = 0.5;
        
        // When: Calculating balance target
        let target = calculate_balance_target(current);
        
        // Then: Should pull toward balance center
        // 0.5 + (1.2 - 0.5) * 0.8 = 0.5 + 0.56 = 1.06
        assert!((target - 1.06).abs() < 0.01);
        assert!(target > current);
        assert!(target < BALANCE_CENTER);
    }
    
    #[test]
    fn test_calculate_balance_target_at_center() {
        // Given: Frequency at balance center
        let current = BALANCE_CENTER;
        
        // When: Calculating balance target
        let target = calculate_balance_target(current);
        
        // Then: Should remain at center
        assert!((target - BALANCE_CENTER).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_modulation_strength_at_center() {
        // Given: Frequency at balance center
        let strength = calculate_modulation_strength(BALANCE_CENTER);
        
        // Then: Should have zero strength
        assert!((strength - 0.0).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_modulation_strength_extreme() {
        // Given: Very extreme frequency
        let strength = calculate_modulation_strength(5.2); // 4.0 Hz from center
        
        // Then: Should have maximum strength
        assert!((strength - 1.0).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_modulation_strength_moderate() {
        // Given: Moderately extreme frequency
        let strength = calculate_modulation_strength(3.2); // 2.0 Hz from center
        
        // Then: Should have moderate strength (0.5)
        assert!((strength - 0.5).abs() < 0.01);
    }
    
    #[test]
    fn test_calculate_complexity_score_simple() {
        // Given: Simple pattern (few frequencies, no dissonance)
        let score = calculate_complexity_score(2, 0, 1.0);
        
        // Then: Should have low complexity
        // 2 * 0.3 + 0 * 0.5 + 1.0 * 0.2 = 0.6 + 0 + 0.2 = 0.8
        assert!((score - 0.8).abs() < 0.01);
    }
    
    #[test]
    fn test_calculate_complexity_score_complex() {
        // Given: Complex pattern (many frequencies, high dissonance, wide spread)
        let score = calculate_complexity_score(10, 5, 3.0);
        
        // Then: Should have high complexity
        // 10 * 0.3 + 5 * 0.5 + 3.0 * 0.2 = 3.0 + 2.5 + 0.6 = 6.1
        assert!((score - 6.1).abs() < 0.01);
    }
    
    #[test]
    fn test_calculate_frequency_spread() {
        // Given: Multiple frequencies
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(2.5, 2),
            create_freq_state(3.5, 3),
        ];
        
        // When: Calculating spread
        let spread = calculate_frequency_spread(&frequencies);
        
        // Then: Should return max - min (3.5 - 1.0 = 2.5)
        assert!((spread - 2.5).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_frequency_spread_single() {
        // Given: Single frequency
        let frequencies = vec![create_freq_state(2.0, 1)];
        
        // When: Calculating spread
        let spread = calculate_frequency_spread(&frequencies);
        
        // Then: Should return 0
        assert!((spread - 0.0).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_frequency_spread_empty() {
        // Given: Empty frequency list
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Calculating spread
        let spread = calculate_frequency_spread(&frequencies);
        
        // Then: Should return 0
        assert!((spread - 0.0).abs() < 0.001);
    }
}
