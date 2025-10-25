//! Interference calculation for amplitude modulation and dominant frequency
//!
//! This module calculates the effects of constructive and destructive interference
//! on frequency amplitudes and determines the dominant frequency.

use super::{FrequencyState, InterferenceConfig};

/// Calculate interference effects on amplitudes
///
/// Applies amplitude boost for constructive interference and reduction for
/// destructive interference based on frequency differences.
///
/// # Algorithm
/// - Constructive: frequencies within 0.2 Hz get amplitude boost (up to 1.5x)
/// - Destructive: frequencies >2.0 Hz apart get amplitude reduction (down to 0.7x)
pub fn calculate_interference_effects(
    frequencies: &mut [FrequencyState],
    constructive_pairs: &[(usize, usize)],
    destructive_pairs: &[(usize, usize)],
    config: &InterferenceConfig,
) {
    // Apply constructive interference boost
    for &(i, j) in constructive_pairs {
        let diff = (frequencies[i].frequency.hz() - frequencies[j].frequency.hz()).abs();
        let boost = calculate_constructive_boost(diff, config.constructive_threshold);
        
        frequencies[i].amplitude *= boost;
        frequencies[j].amplitude *= boost;
    }
    
    // Apply destructive interference reduction
    for &(i, j) in destructive_pairs {
        let diff = (frequencies[i].frequency.hz() - frequencies[j].frequency.hz()).abs();
        let reduction = calculate_destructive_reduction(diff, config.destructive_threshold);
        
        frequencies[i].amplitude *= reduction;
        frequencies[j].amplitude *= reduction;
    }
}

/// Calculate amplitude boost for constructive interference
///
/// Closer frequencies = stronger boost (up to 1.5x at 0 Hz difference)
fn calculate_constructive_boost(diff: f32, threshold: f32) -> f32 {
    if diff >= threshold {
        return 1.0; // No boost
    }
    
    // Linear boost: 1.5x at diff=0, 1.0x at diff=threshold
    1.0 + (threshold - diff) / threshold * 0.5
}

/// Calculate amplitude reduction for destructive interference
///
/// Further apart = more cancellation (up to 0.7x at 4.0 Hz difference)
fn calculate_destructive_reduction(diff: f32, threshold: f32) -> f32 {
    if diff <= threshold {
        return 1.0; // No reduction
    }
    
    // Linear reduction: 1.0x at threshold, 0.7x at threshold+2.0
    let excess = (diff - threshold).min(2.0);
    1.0 - (excess / 2.0) * 0.3
}

/// Calculate dominant frequency using weighted average
///
/// Weights each frequency by its amplitude (after interference effects).
/// Returns the weighted average frequency.
pub fn calculate_dominant_frequency(frequencies: &[FrequencyState]) -> f32 {
    if frequencies.is_empty() {
        return 1.0; // Default to 1.0 Hz
    }
    
    let mut total_weight = 0.0;
    let mut weighted_sum = 0.0;
    
    for freq_state in frequencies {
        let weight = freq_state.amplitude;
        weighted_sum += freq_state.frequency.hz() * weight;
        total_weight += weight;
    }
    
    if total_weight < 0.001 {
        // Avoid division by zero
        return frequencies[0].frequency.hz();
    }
    
    weighted_sum / total_weight
}

/// Calculate overall amplitude from frequency states
///
/// Uses RMS (root mean square) of individual amplitudes.
pub fn calculate_overall_amplitude(frequencies: &[FrequencyState]) -> f32 {
    if frequencies.is_empty() {
        return 0.0;
    }
    
    let sum_squares: f32 = frequencies.iter()
        .map(|f| f.amplitude * f.amplitude)
        .sum();
    
    (sum_squares / frequencies.len() as f32).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frequency, DimensionId};
    
    fn create_freq_state(hz: f32, amplitude: f32, id: u8) -> FrequencyState {
        let mut state = FrequencyState::new(Frequency::new(hz), DimensionId(id), amplitude);
        state.amplitude = amplitude;
        state
    }
    
    #[test]
    fn test_constructive_boost_zero_diff() {
        // Given: Zero frequency difference
        let boost = calculate_constructive_boost(0.0, 0.2);
        
        // Then: Should get maximum boost (1.5x)
        assert!((boost - 1.5).abs() < 0.001);
    }
    
    #[test]
    fn test_constructive_boost_at_threshold() {
        // Given: Difference at threshold
        let boost = calculate_constructive_boost(0.2, 0.2);
        
        // Then: Should get no boost (1.0x)
        assert!((boost - 1.0).abs() < 0.001);
    }
    
    #[test]
    fn test_constructive_boost_mid_range() {
        // Given: Difference at half threshold
        let boost = calculate_constructive_boost(0.1, 0.2);
        
        // Then: Should get mid-range boost (1.25x)
        assert!((boost - 1.25).abs() < 0.001);
    }
    
    #[test]
    fn test_destructive_reduction_at_threshold() {
        // Given: Difference at threshold
        let reduction = calculate_destructive_reduction(2.0, 2.0);
        
        // Then: Should get no reduction (1.0x)
        assert!((reduction - 1.0).abs() < 0.001);
    }
    
    #[test]
    fn test_destructive_reduction_max() {
        // Given: Difference at threshold + 2.0
        let reduction = calculate_destructive_reduction(4.0, 2.0);
        
        // Then: Should get maximum reduction (0.7x)
        assert!((reduction - 0.7).abs() < 0.001);
    }
    
    #[test]
    fn test_destructive_reduction_mid_range() {
        // Given: Difference at threshold + 1.0
        let reduction = calculate_destructive_reduction(3.0, 2.0);
        
        // Then: Should get mid-range reduction (0.85x)
        assert!((reduction - 0.85).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_interference_effects_constructive() {
        // Given: Two frequencies with constructive interference
        let config = InterferenceConfig::default();
        let mut frequencies = vec![
            create_freq_state(1.0, 1.0, 1),
            create_freq_state(1.1, 1.0, 2),
        ];
        let constructive_pairs = vec![(0, 1)];
        let destructive_pairs = vec![];
        
        // When: Calculating interference effects
        calculate_interference_effects(
            &mut frequencies,
            &constructive_pairs,
            &destructive_pairs,
            &config,
        );
        
        // Then: Both amplitudes should be boosted
        assert!(frequencies[0].amplitude > 1.0);
        assert!(frequencies[1].amplitude > 1.0);
    }
    
    #[test]
    fn test_calculate_interference_effects_destructive() {
        // Given: Two frequencies with destructive interference
        let config = InterferenceConfig::default();
        let mut frequencies = vec![
            create_freq_state(1.0, 1.0, 1),
            create_freq_state(3.5, 1.0, 2),
        ];
        let constructive_pairs = vec![];
        let destructive_pairs = vec![(0, 1)];
        
        // When: Calculating interference effects
        calculate_interference_effects(
            &mut frequencies,
            &constructive_pairs,
            &destructive_pairs,
            &config,
        );
        
        // Then: Both amplitudes should be reduced
        assert!(frequencies[0].amplitude < 1.0);
        assert!(frequencies[1].amplitude < 1.0);
    }
    
    #[test]
    fn test_calculate_dominant_frequency_single() {
        // Given: Single frequency
        let frequencies = vec![create_freq_state(2.5, 1.0, 1)];
        
        // When: Calculating dominant frequency
        let dominant = calculate_dominant_frequency(&frequencies);
        
        // Then: Should return that frequency
        assert!((dominant - 2.5).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_dominant_frequency_equal_weights() {
        // Given: Two frequencies with equal amplitude
        let frequencies = vec![
            create_freq_state(1.0, 1.0, 1),
            create_freq_state(3.0, 1.0, 2),
        ];
        
        // When: Calculating dominant frequency
        let dominant = calculate_dominant_frequency(&frequencies);
        
        // Then: Should return average (2.0)
        assert!((dominant - 2.0).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_dominant_frequency_weighted() {
        // Given: Two frequencies with different amplitudes
        let frequencies = vec![
            create_freq_state(1.0, 0.8, 1),
            create_freq_state(3.0, 0.2, 2),
        ];
        
        // When: Calculating dominant frequency
        let dominant = calculate_dominant_frequency(&frequencies);
        
        // Then: Should be weighted toward higher amplitude
        // (1.0 * 0.8 + 3.0 * 0.2) / (0.8 + 0.2) = 1.4
        assert!((dominant - 1.4).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_dominant_frequency_empty() {
        // Given: Empty frequency list
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Calculating dominant frequency
        let dominant = calculate_dominant_frequency(&frequencies);
        
        // Then: Should return default (1.0)
        assert!((dominant - 1.0).abs() < 0.001);
    }
    
    #[test]
    fn test_calculate_overall_amplitude() {
        // Given: Multiple frequencies with different amplitudes
        let frequencies = vec![
            create_freq_state(1.0, 0.8, 1),
            create_freq_state(2.0, 0.6, 2),
        ];
        
        // When: Calculating overall amplitude
        let amplitude = calculate_overall_amplitude(&frequencies);
        
        // Then: Should return RMS
        // sqrt((0.8^2 + 0.6^2) / 2) = sqrt(0.5) ≈ 0.707
        assert!((amplitude - 0.707).abs() < 0.01);
    }
    
    #[test]
    fn test_calculate_overall_amplitude_empty() {
        // Given: Empty frequency list
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Calculating overall amplitude
        let amplitude = calculate_overall_amplitude(&frequencies);
        
        // Then: Should return 0.0
        assert!((amplitude - 0.0).abs() < 0.001);
    }
}
