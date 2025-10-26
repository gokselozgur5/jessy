//! Pattern detection types

use crate::{DimensionId, Frequency};

/// Unique pattern identifier
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PatternId(pub u64);

/// Detected pattern from observations
#[derive(Debug, Clone)]
pub struct DetectedPattern {
    /// Unique pattern identifier
    pub pattern_id: PatternId,
    
    /// Common keywords in pattern
    pub keywords: Vec<String>,
    
    /// Frequency range of observations
    pub frequency_range: (f32, f32),
    
    /// Number of observations supporting this pattern
    pub observation_count: usize,
    
    /// Confidence score (0.0-1.0)
    pub confidence: f32,
    
    /// Suggested dimension ID if pattern should become dimension
    pub suggested_dimension: Option<DimensionId>,
}
