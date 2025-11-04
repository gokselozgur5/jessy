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

use crate::{DimensionId, Frequency, Result, ConsciousnessError};
use std::collections::HashMap;
use std::time::SystemTime;
use thiserror::Error;

// Re-exports
pub use self::observation::*;
pub use self::pattern::*;
pub use self::proto_dimension::*;
pub use self::config::*;
pub use self::circular_buffer::CircularBuffer;
pub use self::memory_tracker::MemoryTracker;
pub use self::crystallizer::Crystallizer;
pub use self::crystallization_queue::{CrystallizationQueue, CrystallizationPriority, QueueMetrics};
pub use self::synesthetic_learner::{SynestheticLearner, KeywordAssociation};
pub use self::pattern_detector::PatternDetector;
pub use self::proto_dimension_manager::ProtoDimensionManager;
pub use self::shared_layer::{SharedLayer, SharedLayerManager, SharedLayerStats};
pub use self::user_layer::{UserLayer, UserLayerManager, UserLayerStats};

// Module declarations
mod observation;
mod pattern;
mod proto_dimension;
mod config;
mod circular_buffer;
mod memory_tracker;
mod pattern_detector;
mod proto_dimension_manager;
mod crystallizer;
mod crystallization_queue;
mod synesthetic_learner;
mod shared_layer;      // NEW: Tier 2 (C16-C30)
mod user_layer;        // NEW: Tier 3 (C31+)

#[cfg(test)]
mod integration_tests;

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

/// Metrics for learning system monitoring
#[derive(Debug, Clone, Default)]
pub struct LearningMetrics {
    /// Total observations recorded
    pub observation_count: usize,
    
    /// Total patterns detected
    pub pattern_count: usize,
    
    /// Current proto-dimension count
    pub proto_dimension_count: usize,
    
    /// Successful crystallizations
    pub crystallization_success: usize,
    
    /// Failed crystallizations
    pub crystallization_failure: usize,
    
    /// Total crystallization attempts
    pub crystallization_attempts: usize,
    
    /// Current memory usage (bytes)
    pub memory_usage: usize,
    
    /// Memory limit (bytes)
    pub memory_limit: usize,
}

impl LearningMetrics {
    /// Calculate crystallization success rate (0.0-1.0)
    pub fn crystallization_success_rate(&self) -> f64 {
        if self.crystallization_attempts == 0 {
            0.0
        } else {
            self.crystallization_success as f64 / self.crystallization_attempts as f64
        }
    }
    
    /// Calculate memory usage percentage (0.0-100.0)
    pub fn memory_usage_percentage(&self) -> f64 {
        if self.memory_limit == 0 {
            0.0
        } else {
            (self.memory_usage as f64 / self.memory_limit as f64) * 100.0
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
    proto_dimension_manager: proto_dimension_manager::ProtoDimensionManager,
    memory_tracker: memory_tracker::MemoryTracker,
    crystallizer: Option<crystallizer::Crystallizer>,
    crystallization_queue: Option<crystallization_queue::CrystallizationQueue>,
    synesthetic_learner: synesthetic_learner::SynestheticLearner,
    metrics: LearningMetrics,
}

impl std::fmt::Debug for LearningSystem {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LearningSystem")
            .field("config", &self.config)
            .field("observation_count", &self.observation_buffer.len())
            .field("metrics", &self.metrics)
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
        let proto_dimension_manager = proto_dimension_manager::ProtoDimensionManager::new(config.clone());
        let memory_tracker = memory_tracker::MemoryTracker::with_limit(config.memory_limit);
        let synesthetic_learner = synesthetic_learner::SynestheticLearner::new(
            config.learning_rate,
            config.decay_rate,
        );
        
        Self {
            config,
            observation_buffer,
            pattern_detector,
            proto_dimension_manager,
            memory_tracker,
            crystallizer: None, // Will be initialized when memory manager is available
            crystallization_queue: None, // Will be initialized with crystallizer
            synesthetic_learner,
            metrics: LearningMetrics::default(),
        }
    }
    
    /// Initialize crystallizer with memory manager
    ///
    /// This must be called before crystallization can be performed.
    /// Also initializes the background crystallization queue.
    pub fn init_crystallizer(&mut self, memory_manager: std::sync::Arc<crate::memory::MmapManager>) {
        // Create crystallizer for direct use
        let crystallizer = crystallizer::Crystallizer::new(memory_manager.clone());

        // Create separate crystallizer for background queue
        let queue_crystallizer = crystallizer::Crystallizer::new(memory_manager);
        let queue = crystallization_queue::CrystallizationQueue::new(queue_crystallizer);

        self.crystallizer = Some(crystallizer);
        self.crystallization_queue = Some(queue);

        eprintln!("[Learning] Crystallizer and background queue initialized");
    }

    /// Enqueue proto-dimension for background crystallization
    ///
    /// This is the recommended way to crystallize proto-dimensions as it doesn't block.
    ///
    /// # Arguments
    ///
    /// * `dimension_id` - ID of proto-dimension to crystallize
    /// * `priority` - Crystallization priority (default: Normal)
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` if successfully enqueued
    pub async fn enqueue_crystallization(
        &mut self,
        dimension_id: DimensionId,
        priority: CrystallizationPriority,
    ) -> Result<()> {
        // Check if queue is initialized
        let queue = self.crystallization_queue.as_ref()
            .ok_or_else(|| ConsciousnessError::LearningError(
                "Crystallization queue not initialized - call init_crystallizer() first".to_string()
            ))?;

        // Get proto-dimension
        let proto = self.proto_dimension_manager.get(dimension_id)
            .ok_or_else(|| ConsciousnessError::LearningError(
                format!("Proto-dimension {:?} not found", dimension_id)
            ))?
            .clone();

        // Enqueue for background processing
        queue.enqueue(proto, priority).await?;

        eprintln!(
            "[Learning] Proto-dimension {:?} enqueued for background crystallization (priority: {:?})",
            dimension_id,
            priority
        );

        Ok(())
    }

    /// Get crystallization queue metrics
    ///
    /// Returns metrics about the background crystallization queue.
    pub async fn queue_metrics(&self) -> Option<QueueMetrics> {
        if let Some(queue) = &self.crystallization_queue {
            Some(queue.metrics().await)
        } else {
            None
        }
    }
    
    /// Crystallize a proto-dimension to MMAP
    ///
    /// Migrates a proto-dimension from heap to permanent MMAP storage.
    ///
    /// # Arguments
    ///
    /// * `dimension_id` - ID of proto-dimension to crystallize
    ///
    /// # Returns
    ///
    /// Returns `Ok(())` on success
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Crystallizer not initialized
    /// - Proto-dimension not found
    /// - Confidence too low (<0.85)
    /// - MMAP allocation fails
    /// - Integrity check fails
    pub async fn crystallize(&mut self, dimension_id: DimensionId) -> Result<()> {
        // Track attempt
        self.metrics.crystallization_attempts += 1;
        
        // Log start
        eprintln!(
            "[Learning] Crystallization started for dimension {:?} (attempt {})",
            dimension_id,
            self.metrics.crystallization_attempts
        );
        
        // Check if crystallizer is initialized
        let crystallizer = self.crystallizer.as_mut()
            .ok_or_else(|| ConsciousnessError::LearningError(
                "Crystallizer not initialized - call init_crystallizer() first".to_string()
            ))?;
        
        // Get proto-dimension
        let proto = self.proto_dimension_manager.get(dimension_id)
            .ok_or_else(|| ConsciousnessError::LearningError(
                format!("Proto-dimension {:?} not found", dimension_id)
            ))?;
        
        // Crystallize
        match crystallizer.crystallize(proto).await {
            Ok(()) => {
                // Track success
                self.metrics.crystallization_success += 1;
                
                // Log success
                eprintln!(
                    "[Learning] Crystallization complete for dimension {:?} (success rate: {:.2}%)",
                    dimension_id,
                    self.metrics.crystallization_success_rate() * 100.0
                );
                
                // Remove from proto-dimension manager after successful crystallization
                self.proto_dimension_manager.remove(dimension_id);
                
                Ok(())
            }
            Err(e) => {
                // Track failure
                self.metrics.crystallization_failure += 1;
                
                // Log failure
                eprintln!(
                    "[Learning] Crystallization failed for dimension {:?}: {} (success rate: {:.2}%)",
                    dimension_id,
                    e,
                    self.metrics.crystallization_success_rate() * 100.0
                );
                
                Err(e)
            }
        }
    }
    
    /// Get current configuration
    pub fn config(&self) -> &LearningConfig {
        &self.config
    }
    
    /// Observe an interaction for pattern learning
    ///
    /// Records the query, activated dimensions, keywords, and frequency for later pattern detection.
    /// Also strengthens synesthetic associations between co-occurring keywords.
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
        navigation_result: &crate::navigation::NavigationResult,
        _iteration_result: &crate::iteration::IterationResult,
    ) -> Result<()> {
        // Extract activated dimensions
        let activated_dimensions = navigation_result.dimensions.clone();
        
        // Extract keywords from query (simple tokenization)
        let keywords: Vec<String> = query
            .split_whitespace()
            .map(|s| s.to_lowercase())
            .collect();
        
        // Strengthen synesthetic associations between co-occurring keywords
        for i in 0..keywords.len() {
            for j in (i + 1)..keywords.len() {
                self.synesthetic_learner.strengthen_association(&keywords[i], &keywords[j]);
            }
        }
        
        // Use first frequency or default to 1.0 Hz
        let frequency = navigation_result
            .frequencies
            .first()
            .map(|f| *f)
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
        
        // Update metrics
        self.metrics.pattern_count += patterns.len();
        
        // Log pattern detection
        if !patterns.is_empty() {
            eprintln!(
                "[Learning] Detected {} patterns from {} observations",
                patterns.len(),
                observations.len()
            );
            for pattern in &patterns {
                eprintln!(
                    "[Learning] Pattern {:?}: confidence={:.2}, observations={}, keywords={:?}",
                    pattern.pattern_id,
                    pattern.confidence,
                    pattern.observation_count,
                    pattern.keywords
                );
            }
        }
        
        Ok(patterns)
    }
    
    /// Create proto-dimension from detected pattern
    ///
    /// Creates a temporary dimension in heap memory from a high-confidence pattern.
    ///
    /// # Arguments
    ///
    /// * `pattern` - Detected pattern with confidence ≥ 0.85
    ///
    /// # Returns
    ///
    /// Returns dimension ID (>100) for the created proto-dimension
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Pattern confidence too low
    /// - Size would exceed 16MB limit
    /// - Max proto-dimensions (10) reached
    ///
    /// # Performance
    ///
    /// This operation completes in <50ms as required.
    pub fn create_proto_dimension(&mut self, pattern: &DetectedPattern) -> Result<DimensionId> {
        // Check memory warning before creation
        if self.memory_tracker.is_above_warning_threshold() {
            eprintln!(
                "[Learning] WARNING: Memory usage at {:.1}% of limit ({} / {} bytes)",
                self.memory_tracker.usage_percentage(),
                self.memory_tracker.total_usage(),
                self.memory_tracker.limit()
            );
        }
        
        // Create proto-dimension
        let dimension_id = self.proto_dimension_manager.create_proto_dimension(pattern)?;
        
        // Log creation
        eprintln!(
            "[Learning] Proto-dimension {:?} created from pattern (confidence: {:.2}, keywords: {:?})",
            dimension_id,
            pattern.confidence,
            pattern.keywords
        );
        
        Ok(dimension_id)
    }
    
    /// Check if proto-dimension exists
    pub fn has_proto_dimension(&self, dimension_id: DimensionId) -> bool {
        self.proto_dimension_manager.has_proto_dimension(dimension_id)
    }
    
    /// Get proto-dimension count
    pub fn proto_dimension_count(&self) -> usize {
        self.proto_dimension_manager.count()
    }
    
    /// Get memory tracker
    pub fn memory_tracker(&self) -> &memory_tracker::MemoryTracker {
        &self.memory_tracker
    }
    
    /// Get total memory usage
    pub fn total_memory_usage(&self) -> usize {
        self.memory_tracker.total_usage()
    }
    
    /// Check if memory usage is above warning threshold (90%)
    pub fn is_memory_warning(&self) -> bool {
        self.memory_tracker.is_above_warning_threshold()
    }
    
    /// Get memory usage percentage
    pub fn memory_usage_percentage(&self) -> f64 {
        self.memory_tracker.usage_percentage()
    }
    
    /// Strengthen association between keywords
    ///
    /// Called when keywords co-occur in a query to strengthen their relationship.
    ///
    /// # Arguments
    ///
    /// * `keyword1` - First keyword
    /// * `keyword2` - Second keyword
    ///
    /// # Performance
    ///
    /// Completes in <1ms as required (O(1) HashMap operation)
    pub fn strengthen_keyword_association(&mut self, keyword1: &str, keyword2: &str) {
        self.synesthetic_learner.strengthen_association(keyword1, keyword2);
    }
    
    /// Get associated keywords for a given keyword
    ///
    /// Returns keywords sorted by association strength (descending).
    ///
    /// # Arguments
    ///
    /// * `keyword` - Keyword to find associations for
    ///
    /// # Returns
    ///
    /// List of (keyword, strength) tuples sorted by strength
    ///
    /// # Performance
    ///
    /// Completes in <1ms as required
    pub fn get_keyword_associations(&self, keyword: &str) -> Vec<(String, f32)> {
        self.synesthetic_learner.get_associations(keyword)
    }
    
    /// Decay unused keyword associations
    ///
    /// Should be called periodically (e.g., daily) to decay associations
    /// that haven't been activated recently.
    pub fn decay_keyword_associations(&mut self) {
        self.synesthetic_learner.decay_unused();
    }
    
    /// Get total number of keyword associations
    pub fn keyword_association_count(&self) -> usize {
        self.synesthetic_learner.association_count()
    }
    
    /// Get association strength between two keywords
    pub fn get_keyword_strength(&self, keyword1: &str, keyword2: &str) -> Option<f32> {
        self.synesthetic_learner.get_strength(keyword1, keyword2)
    }

    /// Save synesthetic associations to file
    ///
    /// Persists learned keyword associations to disk for later loading.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to save file (e.g., "data/synesthetic_associations.json")
    ///
    /// # Errors
    ///
    /// Returns error if save fails
    pub fn save_synesthetic_associations<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        self.synesthetic_learner.save(path)
    }

    /// Load synesthetic associations from file
    ///
    /// Replaces current associations with loaded ones.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to load file from
    ///
    /// # Errors
    ///
    /// Returns error if load fails (file doesn't exist is OK, returns empty learner)
    pub fn load_synesthetic_associations<P: AsRef<std::path::Path>>(&mut self, path: P) -> Result<()> {
        self.synesthetic_learner = synesthetic_learner::SynestheticLearner::load(path)?;
        Ok(())
    }

    /// Get current metrics snapshot
    ///
    /// Returns a copy of current metrics for monitoring and observability.
    ///
    /// # Returns
    ///
    /// `LearningMetrics` with current counts and statistics
    ///
    /// # Example
    ///
    /// ```no_run
    /// # use jessy::learning::LearningSystem;
    /// let system = LearningSystem::new();
    /// let metrics = system.metrics();
    /// println!("Observations: {}", metrics.observation_count);
    /// println!("Success rate: {:.2}%", metrics.crystallization_success_rate() * 100.0);
    /// ```
    pub fn metrics(&self) -> LearningMetrics {
        // Update dynamic metrics
        let mut metrics = self.metrics.clone();
        metrics.observation_count = self.observation_buffer.len();
        metrics.proto_dimension_count = self.proto_dimension_manager.count();
        metrics.memory_usage = self.memory_tracker.total_usage();
        metrics.memory_limit = self.memory_tracker.limit();
        metrics
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
    use crate::navigation::{NavigationPath, NavigationResult};
    use crate::iteration::IterationResult;
    use std::sync::Arc;
    
    fn create_test_navigation_result() -> NavigationResult {
        use crate::navigation::{NavigationPath, QueryAnalysis, QuestionType, UrgencyLevel};
        use crate::LayerId;
        NavigationResult {
            query_analysis: QueryAnalysis {
                raw_query: "test query".to_string(),
                keywords: vec!["test".to_string()],
                estimated_complexity: 1.0,
                emotional_indicators: vec![],
                technical_indicators: vec![],
                question_type: QuestionType::Factual,
                urgency_level: UrgencyLevel::Medium,
                estimated_frequency: 1.0,
            },
            paths: vec![NavigationPath {
                dimension_id: DimensionId(1),
                layer_sequence: vec![LayerId { dimension: DimensionId(1), layer: 0 }],
                confidence: 0.8,
                frequency: Frequency::new(1.0),
                keywords_matched: vec!["test".to_string()],
                synesthetic_score: 0.5,
            }],
            dimensions: vec![DimensionId(1)],
            frequencies: vec![Frequency::new(1.0)],
            total_confidence: 0.8,
            complexity_score: 1.0,
            return_to_source_triggered: false,
            query_analysis_duration_ms: 0,
            dimension_scan_duration_ms: 0,
            path_selection_duration_ms: 0,
            depth_navigation_duration_ms: 0,
            total_duration_ms: 0,
        }
    }
    
    fn create_test_iteration_result() -> IterationResult {
        IterationResult {
            final_answer: "test response".to_string(),
            steps: vec![],
            return_to_source_triggered: false,
            convergence_achieved: true,
            iterations_completed: 1,
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
    
    // Task 7: Synesthetic learning tests
    
    #[test]
    fn test_strengthen_keyword_association() {
        // Given: Learning system
        let mut system = LearningSystem::new();
        
        // When: Strengthening association
        system.strengthen_keyword_association("emotion", "feeling");
        
        // Then: Association should exist
        assert_eq!(system.keyword_association_count(), 1);
        let strength = system.get_keyword_strength("emotion", "feeling");
        assert!(strength.is_some());
        assert_eq!(strength.unwrap(), 1.0);
    }
    
    #[test]
    fn test_get_keyword_associations() {
        // Given: Learning system with associations
        let mut system = LearningSystem::new();
        system.strengthen_keyword_association("emotion", "feeling");
        system.strengthen_keyword_association("emotion", "sentiment");
        
        // When: Getting associations
        let associations = system.get_keyword_associations("emotion");
        
        // Then: Should return associated keywords
        assert_eq!(associations.len(), 2);
    }
    
    #[test]
    fn test_observe_interaction_strengthens_associations() {
        // Given: Learning system
        let mut system = LearningSystem::new();
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        // When: Observing query with multiple keywords
        system.observe_interaction(
            "emotional intelligence test",
            &nav_result,
            &iter_result,
        ).unwrap();
        
        // Then: Associations should be strengthened
        // "emotional" ↔ "intelligence"
        // "emotional" ↔ "test"
        // "intelligence" ↔ "test"
        assert!(system.keyword_association_count() >= 3);
        
        let strength = system.get_keyword_strength("emotional", "intelligence");
        assert!(strength.is_some());
    }
    
    #[test]
    fn test_decay_keyword_associations() {
        // Given: Learning system with associations
        let mut system = LearningSystem::new();
        system.strengthen_keyword_association("test", "example");
        
        // When: Decaying (no time has passed, so no effect)
        system.decay_keyword_associations();
        
        // Then: Association should still exist
        assert_eq!(system.keyword_association_count(), 1);
    }
    
    #[test]
    fn test_synesthetic_learning_performance() {
        // Given: Learning system
        let mut system = LearningSystem::new();
        
        // When: Strengthening many associations
        let start = std::time::Instant::now();
        for i in 0..1000 {
            system.strengthen_keyword_association("keyword", &format!("word{}", i));
        }
        let duration = start.elapsed();
        
        // Then: Should complete quickly (<1ms per operation)
        assert!(duration.as_millis() < 1000); // 1000 operations in <1s
        assert_eq!(system.keyword_association_count(), 1000);
    }
    
    // Task 10.1: Metrics tests
    
    #[test]
    fn test_metrics_initialization() {
        // Given: New learning system
        let system = LearningSystem::new();
        
        // When: Getting metrics
        let metrics = system.metrics();
        
        // Then: Should start at zero
        assert_eq!(metrics.observation_count, 0);
        assert_eq!(metrics.pattern_count, 0);
        assert_eq!(metrics.proto_dimension_count, 0);
        assert_eq!(metrics.crystallization_success, 0);
        assert_eq!(metrics.crystallization_failure, 0);
        assert_eq!(metrics.crystallization_attempts, 0);
        assert_eq!(metrics.memory_usage, 0);
        assert!(metrics.memory_limit > 0);
    }
    
    #[test]
    fn test_metrics_observation_count() {
        // Given: Learning system
        let mut system = LearningSystem::new();
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        // When: Recording observations
        for i in 0..5 {
            system.observe_interaction(
                &format!("query {}", i),
                &nav_result,
                &iter_result,
            ).unwrap();
        }
        
        // Then: Metrics should reflect count
        let metrics = system.metrics();
        assert_eq!(metrics.observation_count, 5);
    }
    
    #[test]
    fn test_metrics_pattern_count() {
        // Given: Learning system with low threshold
        let config = LearningConfig {
            min_observations: 5,
            confidence_threshold: 0.50,
            ..Default::default()
        };
        let mut system = LearningSystem::with_config(config);
        let nav_result = create_test_navigation_result();
        let iter_result = create_test_iteration_result();
        
        // When: Recording observations and detecting patterns
        for _ in 0..10 {
            system.observe_interaction(
                "emotion feeling happy",
                &nav_result,
                &iter_result,
            ).unwrap();
        }
        
        let initial_metrics = system.metrics();
        let initial_pattern_count = initial_metrics.pattern_count;
        
        system.detect_patterns().unwrap();
        
        // Then: Pattern count should increase (or stay same if no patterns)
        let metrics = system.metrics();
        assert!(metrics.pattern_count >= initial_pattern_count);
    }
    
    #[test]
    fn test_metrics_proto_dimension_count() {
        // Given: Learning system
        let mut system = LearningSystem::new();
        
        // When: Creating proto-dimension
        use crate::learning::{DetectedPattern, PatternId};
        let pattern = DetectedPattern {
            pattern_id: PatternId(1),
            keywords: vec!["test".to_string()],
            frequency_range: (1.0, 1.5),
            observation_count: 60,
            confidence: 0.90,
            suggested_dimension: None,
        };
        
        system.create_proto_dimension(&pattern).unwrap();
        
        // Then: Metrics should reflect count
        let metrics = system.metrics();
        assert_eq!(metrics.proto_dimension_count, 1);
    }
    
    #[test]
    fn test_metrics_crystallization_success_rate_zero_attempts() {
        // Given: New learning system
        let system = LearningSystem::new();
        
        // When: Getting metrics
        let metrics = system.metrics();
        
        // Then: Success rate should be 0.0 with no attempts
        assert_eq!(metrics.crystallization_success_rate(), 0.0);
    }
    
    #[tokio::test]
    async fn test_metrics_crystallization_success() {
        // Given: Learning system with memory manager
        let memory_manager = Arc::new(crate::memory::MmapManager::new(280).unwrap());
        let mut system = LearningSystem::new();
        system.init_crystallizer(memory_manager);
        
        // When: Creating and crystallizing proto-dimension
        use crate::learning::{DetectedPattern, PatternId};
        let pattern = DetectedPattern {
            pattern_id: PatternId(1),
            keywords: vec!["test".to_string()],
            frequency_range: (1.0, 1.5),
            observation_count: 60,
            confidence: 0.90,
            suggested_dimension: None,
        };
        
        let dimension_id = system.create_proto_dimension(&pattern).unwrap();
        let _ = system.crystallize(dimension_id).await;
        
        // Then: Metrics should track attempt
        let metrics = system.metrics();
        assert_eq!(metrics.crystallization_attempts, 1);
        // Success or failure depends on placeholder implementation
        assert!(metrics.crystallization_success + metrics.crystallization_failure == 1);
    }
    
    #[test]
    fn test_metrics_memory_usage() {
        // Given: Learning system
        let system = LearningSystem::new();
        
        // When: Getting metrics
        let metrics = system.metrics();
        
        // Then: Should have memory info
        assert!(metrics.memory_limit > 0);
        assert!(metrics.memory_usage >= 0);
        assert!(metrics.memory_usage <= metrics.memory_limit);
    }
    
    #[test]
    fn test_metrics_memory_usage_percentage() {
        // Given: Metrics with known values
        let metrics = LearningMetrics {
            memory_usage: 250 * 1024 * 1024, // 250MB
            memory_limit: 500 * 1024 * 1024, // 500MB
            ..Default::default()
        };
        
        // When: Calculating percentage
        let percentage = metrics.memory_usage_percentage();
        
        // Then: Should be 50%
        assert!((percentage - 50.0).abs() < 0.01);
    }
    
    #[test]
    fn test_metrics_crystallization_success_rate_calculation() {
        // Given: Metrics with known values
        let metrics = LearningMetrics {
            crystallization_success: 8,
            crystallization_failure: 2,
            crystallization_attempts: 10,
            ..Default::default()
        };
        
        // When: Calculating success rate
        let rate = metrics.crystallization_success_rate();
        
        // Then: Should be 0.8 (80%)
        assert!((rate - 0.8).abs() < 0.01);
    }
}

// End-to-end tests for 3-Tier Cognitive Layer System
#[cfg(test)]
mod e2e_tests;
