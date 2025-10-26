//! Observation recording and management

use crate::{DimensionId, Frequency};
use std::time::SystemTime;

/// Observation from a query interaction
#[derive(Debug, Clone)]
pub struct Observation {
    /// Original query text
    pub query: String,
    
    /// Dimensions activated during navigation
    pub activated_dimensions: Vec<DimensionId>,
    
    /// Keywords extracted from query
    pub keywords: Vec<String>,
    
    /// Estimated frequency from query analysis
    pub frequency: Frequency,
    
    /// Timestamp of observation
    pub timestamp: SystemTime,
}

impl Observation {
    /// Create new observation
    pub fn new(
        query: String,
        activated_dimensions: Vec<DimensionId>,
        keywords: Vec<String>,
        frequency: Frequency,
    ) -> Self {
        Self {
            query,
            activated_dimensions,
            keywords,
            frequency,
            timestamp: SystemTime::now(),
        }
    }
}
