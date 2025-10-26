//! Proto-dimension management

use super::{ProtoDimension, DetectedPattern, LearningConfig, LearningError};
use crate::{DimensionId, Result};
use std::collections::HashMap;

/// Manages proto-dimensions in heap memory
pub struct ProtoDimensionManager {
    proto_dimensions: HashMap<DimensionId, ProtoDimension>,
    next_dimension_id: u32,
    config: LearningConfig,
}

impl ProtoDimensionManager {
    /// Create new proto-dimension manager
    pub fn new(config: LearningConfig) -> Self {
        Self {
            proto_dimensions: HashMap::new(),
            next_dimension_id: 101, // Start from 101 for learned dimensions
            config,
        }
    }
    
    /// Create proto-dimension from detected pattern
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Pattern confidence too low
    /// - Size would exceed limit (16MB)
    /// - Max proto-dimensions reached (10)
    pub fn create_proto_dimension(
        &mut self,
        pattern: &DetectedPattern,
    ) -> Result<DimensionId> {
        // Check confidence threshold
        if pattern.confidence < self.config.confidence_threshold {
            return Err(LearningError::ConfidenceTooLow(pattern.confidence).into());
        }
        
        // Check count limit
        if self.proto_dimensions.len() >= self.config.max_proto_dimensions {
            return Err(LearningError::MaxProtoDimensionsReached(
                self.config.max_proto_dimensions
            ).into());
        }
        
        // Create content from pattern
        let content = self.create_content_from_pattern(pattern)?;
        
        // Check size limit
        if content.len() > self.config.max_proto_dimension_size {
            return Err(LearningError::SizeExceeded {
                size: content.len(),
                limit: self.config.max_proto_dimension_size,
            }.into());
        }
        
        // Assign dimension ID
        let dimension_id = DimensionId(self.next_dimension_id);
        self.next_dimension_id += 1;
        
        // Create proto-dimension
        let proto = ProtoDimension::new(dimension_id, content, pattern.confidence);
        
        // Store in heap
        self.proto_dimensions.insert(dimension_id, proto);
        
        Ok(dimension_id)
    }
    
    /// Get proto-dimension by ID
    pub fn get(&self, dimension_id: DimensionId) -> Option<&ProtoDimension> {
        self.proto_dimensions.get(&dimension_id)
    }
    
    /// Get mutable proto-dimension by ID
    pub fn get_mut(&mut self, dimension_id: DimensionId) -> Option<&mut ProtoDimension> {
        self.proto_dimensions.get_mut(&dimension_id)
    }
    
    /// Check if proto-dimension exists
    pub fn has_proto_dimension(&self, dimension_id: DimensionId) -> bool {
        self.proto_dimensions.contains_key(&dimension_id)
    }
    
    /// Remove proto-dimension (after crystallization)
    pub fn remove(&mut self, dimension_id: DimensionId) -> Option<ProtoDimension> {
        self.proto_dimensions.remove(&dimension_id)
    }
    
    /// Get count of proto-dimensions
    pub fn count(&self) -> usize {
        self.proto_dimensions.len()
    }
    
    /// Get total memory usage of proto-dimensions
    pub fn memory_usage(&self) -> usize {
        self.proto_dimensions
            .values()
            .map(|proto| proto.size_bytes)
            .sum()
    }
    
    /// Create content from pattern
    fn create_content_from_pattern(&self, pattern: &DetectedPattern) -> Result<Vec<u8>> {
        // Simple implementation: serialize pattern data
        // In production, this would create proper dimensional content
        let content = format!(
            "Pattern: {}\nKeywords: {}\nFrequency Range: {:.2}-{:.2} Hz\nObservations: {}\nConfidence: {:.2}",
            pattern.pattern_id.0,
            pattern.keywords.join(", "),
            pattern.frequency_range.0,
            pattern.frequency_range.1,
            pattern.observation_count,
            pattern.confidence
        );
        
        Ok(content.into_bytes())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::learning::PatternId;
    
    fn create_test_pattern(confidence: f32) -> DetectedPattern {
        DetectedPattern {
            pattern_id: PatternId(1),
            keywords: vec!["test".to_string(), "pattern".to_string()],
            frequency_range: (1.0, 2.0),
            observation_count: 50,
            confidence,
            suggested_dimension: None,
        }
    }
    
    #[test]
    fn test_proto_dimension_manager_creation() {
        let config = LearningConfig::default();
        let manager = ProtoDimensionManager::new(config);
        assert_eq!(manager.count(), 0);
        assert_eq!(manager.next_dimension_id, 101);
    }
    
    #[test]
    fn test_create_proto_dimension_success() {
        let config = LearningConfig::default();
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.90);
        let result = manager.create_proto_dimension(&pattern);
        
        assert!(result.is_ok());
        let dimension_id = result.unwrap();
        assert_eq!(dimension_id.0, 101);
        assert_eq!(manager.count(), 1);
        assert!(manager.has_proto_dimension(dimension_id));
    }
    
    #[test]
    fn test_create_proto_dimension_low_confidence() {
        let config = LearningConfig {
            confidence_threshold: 0.85,
            ..Default::default()
        };
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.80); // Below threshold
        let result = manager.create_proto_dimension(&pattern);
        
        assert!(result.is_err());
        assert_eq!(manager.count(), 0);
    }
    
    #[test]
    fn test_create_proto_dimension_max_count() {
        let config = LearningConfig {
            max_proto_dimensions: 2,
            ..Default::default()
        };
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.90);
        
        // Create 2 proto-dimensions (max)
        manager.create_proto_dimension(&pattern).unwrap();
        manager.create_proto_dimension(&pattern).unwrap();
        
        // Try to create 3rd (should fail)
        let result = manager.create_proto_dimension(&pattern);
        assert!(result.is_err());
        assert_eq!(manager.count(), 2);
    }
    
    #[test]
    fn test_get_proto_dimension() {
        let config = LearningConfig::default();
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.90);
        let dimension_id = manager.create_proto_dimension(&pattern).unwrap();
        
        let proto = manager.get(dimension_id);
        assert!(proto.is_some());
        assert_eq!(proto.unwrap().dimension_id, dimension_id);
        assert_eq!(proto.unwrap().confidence, 0.90);
    }
    
    #[test]
    fn test_remove_proto_dimension() {
        let config = LearningConfig::default();
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.90);
        let dimension_id = manager.create_proto_dimension(&pattern).unwrap();
        
        assert_eq!(manager.count(), 1);
        
        let removed = manager.remove(dimension_id);
        assert!(removed.is_some());
        assert_eq!(manager.count(), 0);
        assert!(!manager.has_proto_dimension(dimension_id));
    }
    
    #[test]
    fn test_memory_usage() {
        let config = LearningConfig::default();
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.90);
        manager.create_proto_dimension(&pattern).unwrap();
        
        let usage = manager.memory_usage();
        assert!(usage > 0);
        assert!(usage < 1024); // Should be small for test pattern
    }
    
    #[test]
    fn test_multiple_proto_dimensions() {
        let config = LearningConfig::default();
        let mut manager = ProtoDimensionManager::new(config);
        
        let pattern = create_test_pattern(0.90);
        
        let id1 = manager.create_proto_dimension(&pattern).unwrap();
        let id2 = manager.create_proto_dimension(&pattern).unwrap();
        let id3 = manager.create_proto_dimension(&pattern).unwrap();
        
        assert_eq!(id1.0, 101);
        assert_eq!(id2.0, 102);
        assert_eq!(id3.0, 103);
        assert_eq!(manager.count(), 3);
    }
}
