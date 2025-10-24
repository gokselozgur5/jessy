//! Dimension type definitions and behavior

use crate::{DimensionId, Frequency, Result};

/// Dimension information and metadata
#[derive(Debug, Clone)]
pub struct DimensionInfo {
    pub id: DimensionId,
    pub name: String,
    pub frequency_range: (f32, f32),
    pub size_mb: usize,
}

impl DimensionInfo {
    /// Create new dimension info
    pub fn new(id: DimensionId, name: String, frequency_range: (f32, f32), size_mb: usize) -> Self {
        Self {
            id,
            name,
            frequency_range,
            size_mb,
        }
    }
}

/// Dimension trait for common behavior
pub trait Dimension {
    fn id(&self) -> DimensionId;
    fn name(&self) -> &str;
    fn frequency_range(&self) -> (f32, f32);
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_dimension_info_creation() {
        let info = DimensionInfo::new(
            DimensionId(1),
            "Emotion".to_string(),
            (0.2, 4.5),
            16,
        );
        
        assert_eq!(info.id, DimensionId(1));
        assert_eq!(info.name, "Emotion");
    }
}
