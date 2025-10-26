//! Proto-dimension types and management

use crate::DimensionId;
use std::time::SystemTime;

/// Proto-dimension stored in heap memory
#[derive(Debug, Clone)]
pub struct ProtoDimension {
    /// Dimension identifier (>100 for learned dimensions)
    pub dimension_id: DimensionId,
    
    /// Content stored in heap
    pub content: Vec<u8>,
    
    /// Confidence score
    pub confidence: f32,
    
    /// Creation timestamp
    pub created_at: SystemTime,
    
    /// Last access timestamp
    pub last_accessed: SystemTime,
    
    /// Size in bytes
    pub size_bytes: usize,
}

impl ProtoDimension {
    /// Create new proto-dimension
    pub fn new(dimension_id: DimensionId, content: Vec<u8>, confidence: f32) -> Self {
        let size_bytes = content.len();
        let now = SystemTime::now();
        
        Self {
            dimension_id,
            content,
            confidence,
            created_at: now,
            last_accessed: now,
            size_bytes,
        }
    }
}
