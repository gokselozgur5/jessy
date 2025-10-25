//! Pair detection for constructive and destructive interference
//!
//! This module identifies frequency pairs that exhibit constructive or destructive
//! interference based on their frequency differences.

use super::{FrequencyState, InterferenceConfig};

/// Detect constructive interference pairs (frequencies within threshold)
///
/// Constructive interference occurs when frequencies are close enough to reinforce
/// each other. The default threshold is 0.2 Hz.
///
/// # Algorithm
/// Uses O(n²) nested loop to check all pairs. For n ≤ 14 dimensions, this is
/// acceptable (max 196 comparisons).
pub fn detect_constructive_pairs(
    frequencies: &[FrequencyState],
    threshold: f32,
) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    
    for i in 0..frequencies.len() {
        for j in (i + 1)..frequencies.len() {
            let diff = (frequencies[i].frequency.hz() - frequencies[j].frequency.hz()).abs();
            if diff < threshold {
                pairs.push((i, j));
            }
        }
    }
    
    pairs
}

/// Detect destructive interference pairs (frequencies beyond threshold)
///
/// Destructive interference occurs when frequencies are far enough apart to
/// potentially cancel each other. The default threshold is 2.0 Hz.
///
/// # Algorithm
/// Uses O(n²) nested loop to check all pairs. For n ≤ 14 dimensions, this is
/// acceptable (max 196 comparisons).
pub fn detect_destructive_pairs(
    frequencies: &[FrequencyState],
    threshold: f32,
) -> Vec<(usize, usize)> {
    let mut pairs = Vec::new();
    
    for i in 0..frequencies.len() {
        for j in (i + 1)..frequencies.len() {
            let diff = (frequencies[i].frequency.hz() - frequencies[j].frequency.hz()).abs();
            if diff > threshold {
                pairs.push((i, j));
            }
        }
    }
    
    pairs
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frequency, DimensionId};
    
    fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
        FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
    }
    
    #[test]
    fn test_detect_constructive_pairs_within_threshold() {
        // Given: Two frequencies within 0.2 Hz
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(1.1, 2),
        ];
        
        // When: Detecting constructive pairs
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should find one pair
        assert_eq!(pairs.len(), 1);
        assert_eq!(pairs[0], (0, 1));
    }
    
    #[test]
    fn test_detect_constructive_pairs_exactly_at_threshold() {
        // Given: Two frequencies exactly at threshold
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(1.2, 2),
        ];
        
        // When: Detecting constructive pairs with 0.2 threshold
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should NOT find pair (< not <=)
        assert_eq!(pairs.len(), 0);
    }
    
    #[test]
    fn test_detect_constructive_pairs_beyond_threshold() {
        // Given: Two frequencies beyond threshold
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(1.5, 2),
        ];
        
        // When: Detecting constructive pairs
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should find no pairs
        assert_eq!(pairs.len(), 0);
    }
    
    #[test]
    fn test_detect_constructive_pairs_multiple() {
        // Given: Multiple frequencies with some close together
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(1.05, 2),  // Close to 1.0
            create_freq_state(2.0, 3),
            create_freq_state(2.1, 4),   // Close to 2.0
        ];
        
        // When: Detecting constructive pairs
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should find two pairs
        assert_eq!(pairs.len(), 2);
        assert!(pairs.contains(&(0, 1)));
        assert!(pairs.contains(&(2, 3)));
    }
    
    #[test]
    fn test_detect_constructive_pairs_same_frequency() {
        // Given: Two identical frequencies
        let frequencies = vec![
            create_freq_state(1.5, 1),
            create_freq_state(1.5, 2),
        ];
        
        // When: Detecting constructive pairs
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should find one pair (0 < 0.2)
        assert_eq!(pairs.len(), 1);
        assert_eq!(pairs[0], (0, 1));
    }
    
    #[test]
    fn test_detect_constructive_pairs_empty() {
        // Given: Empty frequency list
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Detecting constructive pairs
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should find no pairs
        assert_eq!(pairs.len(), 0);
    }
    
    #[test]
    fn test_detect_constructive_pairs_single() {
        // Given: Single frequency
        let frequencies = vec![create_freq_state(1.0, 1)];
        
        // When: Detecting constructive pairs
        let pairs = detect_constructive_pairs(&frequencies, 0.2);
        
        // Then: Should find no pairs
        assert_eq!(pairs.len(), 0);
    }
    
    #[test]
    fn test_detect_destructive_pairs_beyond_threshold() {
        // Given: Two frequencies beyond 2.0 Hz apart
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(3.5, 2),
        ];
        
        // When: Detecting destructive pairs
        let pairs = detect_destructive_pairs(&frequencies, 2.0);
        
        // Then: Should find one pair
        assert_eq!(pairs.len(), 1);
        assert_eq!(pairs[0], (0, 1));
    }
    
    #[test]
    fn test_detect_destructive_pairs_exactly_at_threshold() {
        // Given: Two frequencies exactly at threshold
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(3.0, 2),
        ];
        
        // When: Detecting destructive pairs with 2.0 threshold
        let pairs = detect_destructive_pairs(&frequencies, 2.0);
        
        // Then: Should NOT find pair (> not >=)
        assert_eq!(pairs.len(), 0);
    }
    
    #[test]
    fn test_detect_destructive_pairs_within_threshold() {
        // Given: Two frequencies within threshold
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(2.5, 2),
        ];
        
        // When: Detecting destructive pairs
        let pairs = detect_destructive_pairs(&frequencies, 2.0);
        
        // Then: Should find no pairs
        assert_eq!(pairs.len(), 0);
    }
    
    #[test]
    fn test_detect_destructive_pairs_mixed_scenario() {
        // Given: Mix of close and far frequencies
        let frequencies = vec![
            create_freq_state(1.0, 1),
            create_freq_state(1.1, 2),   // Close to 1.0
            create_freq_state(4.0, 3),   // Far from 1.0 and 1.1
        ];
        
        // When: Detecting destructive pairs
        let pairs = detect_destructive_pairs(&frequencies, 2.0);
        
        // Then: Should find two pairs (1.0-4.0 and 1.1-4.0)
        assert_eq!(pairs.len(), 2);
        assert!(pairs.contains(&(0, 2)));
        assert!(pairs.contains(&(1, 2)));
    }
    
    #[test]
    fn test_detect_destructive_pairs_empty() {
        // Given: Empty frequency list
        let frequencies: Vec<FrequencyState> = vec![];
        
        // When: Detecting destructive pairs
        let pairs = detect_destructive_pairs(&frequencies, 2.0);
        
        // Then: Should find no pairs
        assert_eq!(pairs.len(), 0);
    }
}
