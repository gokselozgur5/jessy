//! Interference pattern analysis

// Re-export from mod.rs to avoid duplication
pub use super::{InterferencePattern, FrequencyState};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Frequency, DimensionId};
    
    #[test]
    fn test_pattern_creation() {
        let mut pattern = InterferencePattern::new();
        
        let state = FrequencyState::new(
            Frequency::new(1.5),
            DimensionId(1),
            0.8,
        );
        
        pattern.add_frequency(state);
        
        assert_eq!(pattern.frequency_count(), 1);
    }
}
