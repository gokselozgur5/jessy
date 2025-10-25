//! Performance tests for interference engine
//!
//! These tests validate that the interference engine meets performance targets.

#[cfg(test)]
mod tests {
    use super::super::*;
    use crate::{Frequency, DimensionId};
    use std::time::Instant;
    
    fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
        FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
    }
    
    #[test]
    fn test_performance_1_frequency() {
        // Given: Engine and 1 frequency
        let engine = InterferenceEngine::new();
        let frequencies = vec![create_freq_state(1.5, 1)];
        
        // When: Calculating interference (measure time)
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = engine.calculate(&frequencies).unwrap();
        }
        let duration = start.elapsed();
        
        // Then: Should complete in <1ms per calculation
        let avg_micros = duration.as_micros() / 1000;
        println!("1 frequency: {} μs per calculation", avg_micros);
        assert!(avg_micros < 1000, "Should be <1ms, was {} μs", avg_micros);
    }
    
    #[test]
    fn test_performance_7_frequencies() {
        // Given: Engine and 7 frequencies (typical case)
        let engine = InterferenceEngine::new();
        let frequencies: Vec<_> = (1..=7)
            .map(|i| create_freq_state(i as f32 * 0.5, i))
            .collect();
        
        // When: Calculating interference (measure time)
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = engine.calculate(&frequencies).unwrap();
        }
        let duration = start.elapsed();
        
        // Then: Should complete in <10ms per calculation
        let avg_micros = duration.as_micros() / 1000;
        println!("7 frequencies: {} μs per calculation", avg_micros);
        assert!(avg_micros < 10000, "Should be <10ms, was {} μs", avg_micros);
    }
    
    #[test]
    fn test_performance_14_frequencies() {
        // Given: Engine and 14 frequencies (maximum case)
        let engine = InterferenceEngine::new();
        let frequencies: Vec<_> = (1..=14)
            .map(|i| create_freq_state(i as f32 * 0.3, i))
            .collect();
        
        // When: Calculating interference (measure time)
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = engine.calculate(&frequencies).unwrap();
        }
        let duration = start.elapsed();
        
        // Then: Should complete in <20ms per calculation
        let avg_micros = duration.as_micros() / 1000;
        println!("14 frequencies: {} μs per calculation", avg_micros);
        assert!(avg_micros < 20000, "Should be <20ms, was {} μs", avg_micros);
    }
    
    #[test]
    fn test_performance_pair_detection() {
        // Given: 14 frequencies
        let frequencies: Vec<_> = (1..=14)
            .map(|i| create_freq_state(i as f32 * 0.3, i))
            .collect();
        
        // When: Detecting pairs (measure time)
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = pair_detection::detect_constructive_pairs(&frequencies, 0.2);
            let _ = pair_detection::detect_destructive_pairs(&frequencies, 2.0);
        }
        let duration = start.elapsed();
        
        // Then: Should complete in <5ms per calculation
        let avg_micros = duration.as_micros() / 1000;
        println!("Pair detection (14 freqs): {} μs per calculation", avg_micros);
        assert!(avg_micros < 5000, "Should be <5ms, was {} μs", avg_micros);
    }
    
    #[test]
    fn test_performance_harmonic_analysis() {
        // Given: 14 frequencies and analyzer
        let frequencies: Vec<_> = (1..=14)
            .map(|i| create_freq_state(i as f32 * 0.3, i))
            .collect();
        let analyzer = HarmonicAnalyzer::new();
        
        // When: Analyzing harmonics (measure time)
        let start = Instant::now();
        for _ in 0..1000 {
            let _ = analyzer.analyze_all(&frequencies);
        }
        let duration = start.elapsed();
        
        // Then: Should complete in <5ms per calculation
        let avg_micros = duration.as_micros() / 1000;
        println!("Harmonic analysis (14 freqs): {} μs per calculation", avg_micros);
        assert!(avg_micros < 5000, "Should be <5ms, was {} μs", avg_micros);
    }
}
