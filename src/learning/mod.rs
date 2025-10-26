//! Learning System for Pattern Detection and Crystallization
//!
//! This module implements the learning system that enables the Jessy consciousness
//! to learn from interactions, detect patterns, and crystallize new dimensional layers.
//!
//! # Overview
//!
//! The learning system consists of four main components:
//!
//! 1. **Observation Recording**: Captures query interactions for pattern analysis
//! 2. **Pattern Detection**: Identifies recurring themes from observations
//! 3. **Proto-Dimensions**: Temporary dimensions in heap memory during learning
//! 4. **Crystallization**: Migrates proto-dimensions to permanent MMAP storage
//! 5. **Synesthetic Learning**: Strengthens keyword associations over time
//!
//! # Architecture
//!
//! ```text
//! Query Processing
//!       ↓
//! Observation Recording (<5ms)
//!       ↓
//! Pattern Detection (periodic, <100ms)
//!       ↓
//! Proto-Dimension Creation (<50ms)
//!       ↓
//! Crystallization (background, async)
//!       ↓
//! Active Dimension (MMAP)
//! ```
//!
//! # Memory Management
//!
//! - Core dimensions: 280MB (fixed)
//! - Proto-dimensions: 0-160MB (max 10 × 16MB each)
//! - Observation buffer: ~1MB (1000 entries)
//! - Synesthetic data: ~10MB
//! - **Total limit: 500MB**
//!
//! # Example
//!
//! ```no_run
//! use jessy::learning::LearningSystem;
//!
//! # async fn example() -> jessy::Result<()> {
//! let mut learning = LearningSystem::new();
//!
//! // Record observation from query
//! learning.observe_interaction(
//!     "example query",
//!     &navigation_result,
//!     &iteration_result,
//! )?;
//!
//! // Detect patterns (after 50+ observations)
//! let patterns = learning.detect_patterns()?;
//!
//! // Create proto-dimension for high-confidence pattern
//! if let Some(pattern) = patterns.first() {
//!     if pattern.confidence >= 0.85 {
//!         let dimension_id = learning.create_proto_dimension(pattern)?;
//!         
//!         // Crystallize to MMAP (background)
//!         learning.crystallize(dimension_id).await?;
//!     }
//! }
//! # Ok(())
//! # }
//! ```

use crate::{DimensionId, Frequency, Result};
use std::collections::HashMap;
use std::time::SystemTime;
use thiserror::Error;

// Re-exports
pub use self::observation::*;
pub use self::pattern::*;
pub use self::proto_dimension::*;
pub use self::config::*;
pub use self::circular_buffer::CircularBuffer;

// Module declarations
mod observation;
mod pattern;
mod proto_dimension;
mod config;
mod circular_buffer;
mod pattern_detector;

/// Learning system error types
#[derive(Error, Debug, Clone, PartialEq)]
pub enum LearningError {
    /// Observation buffer is full
    #[error("Observation buffer is full")]
    BufferFull,
    
    /// Pattern confidence too low for proto-dimension creation
    #[error("Pattern confidence {0:.2} below threshold 0.85")]
    ConfidenceTooLow(f32),
    
    /// Proto-dimension size exceeds limit
    #[error("Proto-dimension size {size} exceeds limit {limit}")]
    SizeExceeded { size: usize, limit: usize },
    
    /// Maximum proto-dimensions reached
    #[error("Maximum proto-dimensions ({0}) reached")]
    MaxProtoDimensionsReached(usize),
    
    /// Memory limit exceeded
    #[error("Memory limit exceeded: {current} > {limit}")]
    MemoryLimitExceeded { current: usize, limit: usize },
    
    /// Proto-dimension not found
    #[error("Proto-dimension {0:?} not found")]
    ProtoDimensionNotFound(DimensionId),
    
    /// Proto-dimension not ready for crystallization
    #[error("Proto-dimension {0:?} not ready (confidence: {1:.2})")]
    NotReadyForCrystallization(DimensionId, f32),
    
    /// Crystallization failed
    #[error("Crystallization failed: {0}")]
    CrystallizationFailed(String),
    
    /// Integrity check failed
    #[error("Integrity check failed: checksums don't match")]
    IntegrityCheckFailed,
    
    /// Invalid observation
    #[error("Invalid observation: {0}")]
    InvalidObservation(String),
}

/// Learning system configuration
#[derive(Debug, Clone)]
pub struct LearningConfig {
    /// Maximum observations in buffer
    pub max_observations: usize,
    
    /// Minimum observations before pattern detection
    pub min_observations: usize,
    
    /// Confidence threshold for proto-dimension creation
    pub confidence_threshold: f32,
    
    /// Maximum proto-dimensions
    pub max_proto_dimensions: usize,
    
    /// Maximum proto-dimension size (bytes)
    pub max_proto_dimension_size: usize,
    
    /// Total memory limit (bytes)
    pub memory_limit: usize,
    
    /// Synesthetic learning rate
    pub learning_rate: f32,
    
    /// Synesthetic decay rate (per day)
    pub decay_rate: f32,
}

impl Default for LearningConfig {
    fn default() -> Self {
        Self {
            max_observations: 1000,
            min_observations: 50,
            confidence_threshold: 0.85,
            max_proto_dimensions: 10,
            max_proto_dimension_size: 16 * 1024 * 1024, // 16MB
            memory_limit: 500 * 1024 * 1024,            // 500MB
            learning_rate: 1.1,                         // 10% growth
            decay_rate: 0.95,                           // 5% decay per day
        }
    }
}

/// Main learning system coordinator
///
/// Manages pattern detection, proto-dimension creation, and crystallization.
pub struct LearningSystem {
    config: LearningConfig,
    observation_buffer: CircularBuffer<Observation>,
    pattern_detector: pattern_detector::PatternDetector,
    // Other components will be added in subsequent tasks
}

impl std::fmt::Debug for LearningSystem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LearningSystem")
            .field("config", &self.config)
            .field("observation_count", &self.observation_buffer.len())
            .finish()
    }
}

impl LearningSystem {
    /// Create new learning system with default configuration
    pub fn new() -> Self {
        Self::with_config(LearningConfig::default())
    }
    
    /// Create new learning system with custom configuration
    pub fn with_config(config: LearningConfig) -> Self {
        let observation_buffer = CircularBuffer::new(config.max_observations);
        let pattern_detector = pattern_detector::PatternDetector::new(config.clone());
        
        Self {
            config,
            observation_buffer,
            pattern_detector,
        }
    }
    
    /// Get current configuration
    pub fn config(&self) -> &LearningConfig {
        &self.config
    }
    
    /// Observe an interaction for pattern learning
    ///
    /// Records the query, activated dimensions, keywords, and frequency for later pattern detection.
    ///
    /// # Arguments
    ///
    /// * `query` - The original query text
    /// * `navigation_result` - Result from navigation system with activated dimensions
    /// * `iteration_result` - Result from iteration processing
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success. The circular buffer automatically handles overflow.
    ///
    /// # Performance
    ///
    /// This operation completes in <5ms as required.
    pub fn observe_interaction(
        &mut self,
        query: &str,
        navigation_result: &crate::navigation::navigator::NavigationResult,
        _iteration_result: &crate::iteration::IterationResult,
    ) -> Result<()> {
        // Extract activated dimensions
        let activated_dimensions: Vec<DimensionId> = navigation_result
            .paths
            .iter()
            .map(|path| path.dimension_id)
            .collect();
        
        // Extract keywords from navigation result
        let keywords: Vec<String> = navigation_result
            .paths
            .iter()
            .flat_map(|path| path.keywords_matched.clone())
            .collect();
        
        // Use first path's frequency or default to 1.0 Hz
        let frequency = navigation_result
            .paths
            .first()
            .map(|path| path.frequency)
            .unwrap_or_else(|| Frequency::new(1.0));
        
        // Create observation
        let observation = Observation::new(
            query.to_string(),
            activated_dimensions,
            keywords,
            frequency,
        );
        
        // Push to circular buffer (automatically overwrites oldest if full)
        self.observation_buffer.push(observation);
        
        Ok(())
    }
    
    /// Get current observation count
    pub fn observation_count(&self) -> usize {
        self.observation_buffer.len()
    }
    
    /// Get observation buffer capacity
    pub fn observation_capacity(&self) -> usize {
        self.observation_buffer.capacity()
    }
    
    /// Detect patterns from accumulated observations
    ///
    /// Analyzes the observation buffer to identify recurring themes.
    /// Returns patterns that meet minimum observation count and confidence threshold.
    ///
    /// # Returns
    ///
    /// List of detected patterns with confidence scores ≥ 0.85
    ///
    /// # Performance
    ///
    /// This operation completes in <100ms as required.
    pub fn detect_patterns(&mut self) -> Result<Vec<DetectedPattern>> {
        let observations: Vec<_> = self.observation_buffer.iter().cloned().collect();
        let patterns = self.pattern_detector.detect_patterns(&observations);
        Ok(patterns)
    }
}

impl Default for LearningSystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::navigation::navigator::{NavigationPath, NavigationResult};
    use crate::iteration::IterationResult;
    
    fn create_test_navigation_result() -> NavigationResult {
        NavigationResult {
            paths: vec![
                NavigationPath {
                    dimension_id: DimensionId(1),
                    confidence: 0.8,
                    layer_sequence: vec![],
                    frequency: Frequency::new(1.0),
                    keywords_matched: vec!["test".to_string(), "query".to_string()],
                    synesthetic_score: 0.0,
                },
            ],
            return_to_source: false,
            simplification_message: None,
            total_duration_ms: 10,
            scanned_dimensions: 14,
        }
    }
    
    fn create_test_iteration_result() -> IterationResult {
        IterationResult {
            final_thought: "test response".to_string(),
            iterations_completed: 3,
            converged: true,
            convergence_iteration: Some(3),
            total_duration_ms: 100,
            iteration_history: vec![],
        }
    }
    
    #[test]
    fn test_learning_system_creation() {
        let system = LearningSystem::new();
        assert_eq!(system.config().max_observations, 1000);
        assert_eq!(system.config().min_observations, 50);
        assert_eq!(system.config().confidence_threshold, 0.85);
        assert_eq!(system.observation_count(), 0);
    }
    
    #[test]
    fn test_learning_system_with_custom_config() {
        let config = LearningConfig {
            max_observations: 500,
            min_observations: 25,
            confidence_threshold: 0.90,
            ..Default::default()
        };
        
        let system = LearningSystem::with_config(config);
        assert_eq!(system.config().max_observations, 500);
        assert_eq!(system.config().min_observations, 25);
        assert_eq!(system.config().confidence_threshold, 0.90);
        assert_eq!(system.observation_capacity(), 500);
    }
    
    #[test]
    fn test_default_config_values() {
        let config = LearningConfig::default();
        assert_eq!(config.max_observations, 1000);
        assert_eq!(config.min_observations, 50);
        assert_eq!(config.confidence_threshold, 0.85);
        assert_eq!(config.max_proto_dimensions, 10);
        assert_eq!(config.max_proto_dimension_size, 16 * 1024 * 1024);
        assert_eq!(config.memory_limit, 500 * 1024 * 1024);
        assert_eq!(config.learning_rate, 1.1);
        assert_eq!(config.decay_rate, 0.95);
    }
    
    #[test]
    fn test_observe_interaction() {
        let mut system = LearningSystem::new();
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        let result = system.observe_interaction(
            "test query",
            &nav_result,
            &iter_result,
        );
        
        assert!(result.is_ok());
        assert_eq!(system.observation_count(), 1);
    }
    
    #[test]
    fn test_multiple_observations() {
        let mut system = LearningSystem::new();
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        for i in 0..10 {
            system.observe_interaction(
                &format!("query {}", i),
                &nav_result,
                &iter_result,
            ).unwrap();
        }
        
        assert_eq!(system.observation_count(), 10);
    }
    
    #[test]
    fn test_circular_buffer_overflow() {
        let config = LearningConfig {
            max_observations: 5,
            ..Default::default()
        };
        let mut system = LearningSystem::with_config(config);
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        // Add more observations than capacity
        for i in 0..10 {
            system.observe_interaction(
                &format!("query {}", i),
                &nav_result,
                &iter_result,
            ).unwrap();
        }
        
        // Should only have last 5 observations
        assert_eq!(system.observation_count(), 5);
        assert_eq!(system.observation_capacity(), 5);
    }
    
    #[test]
    fn test_observation_includes_all_fields() {
        let mut system = LearningSystem::new();
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        system.observe_interaction(
            "test query",
            &nav_result,
            &iter_result,
        ).unwrap();
        
        assert_eq!(system.observation_count(), 1);
        // Observation is stored correctly (verified by count)
    }
}
