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

// Module declarations
mod observation;
mod pattern;
mod proto_dimension;
mod config;

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
#[derive(Debug)]
pub struct LearningSystem {
    config: LearningConfig,
    // Components will be added in subsequent tasks
}

impl LearningSystem {
    /// Create new learning system with default configuration
    pub fn new() -> Self {
        Self::with_config(LearningConfig::default())
    }
    
    /// Create new learning system with custom configuration
    pub fn with_config(config: LearningConfig) -> Self {
        Self {
            config,
        }
    }
    
    /// Get current configuration
    pub fn config(&self) -> &LearningConfig {
        &self.config
    }
    
    /// Observe an interaction for pattern learning (stub - will be implemented in Task 2)
    ///
    /// # Errors
    /// - LearningError if observation buffer is full
    pub fn observe_interaction(
        &mut self,
        _query: &str,
        _navigation_result: &crate::navigation::navigator::NavigationResult,
        _iteration_result: &crate::iteration::IterationResult,
    ) -> Result<()> {
        // Stub implementation - will be completed in Task 2
        Ok(())
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
    
    #[test]
    fn test_learning_system_creation() {
        let system = LearningSystem::new();
        assert_eq!(system.config().max_observations, 1000);
        assert_eq!(system.config().min_observations, 50);
        assert_eq!(system.config().confidence_threshold, 0.85);
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
}
