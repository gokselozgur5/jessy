//! Consciousness orchestrator module
//!
//! This module integrates Navigation, Memory, and Iteration systems into a unified
//! consciousness pipeline. It coordinates the complete query processing flow:
//! 1. Navigate - Select dimensional paths
//! 2. Load Memory - Load contexts from selected dimensions
//! 3. Calculate Interference - Simple frequency aggregation
//! 4. Iterate - 9-iteration deep thinking with convergence
//!
//! # Architecture
//!
//! The consciousness orchestrator follows a sequential pipeline design where each
//! stage depends on the output of the previous stage. This simplifies error handling
//! and makes the flow easy to understand and debug.
//!
//! # Example
//!
//! ```no_run
//! use jessy::consciousness::{ConsciousnessOrchestrator, ConsciousnessConfig};
//! use jessy::navigation::NavigationSystem;
//! use jessy::memory::MmapManager;
//! use std::sync::Arc;
//!
//! # async fn example() -> jessy::Result<()> {
//! // Initialize systems
//! let navigation = Arc::new(NavigationSystem::new()?);
//! let memory = Arc::new(MmapManager::new(280)?);
//!
//! // Create orchestrator
//! let orchestrator = ConsciousnessOrchestrator::new(navigation, memory);
//!
//! // Process query
//! let response = orchestrator.process("What is consciousness?").await?;
//! println!("Response: {}", response.final_response);
//! println!("Iterations: {}", response.metadata.iterations_completed);
//! # Ok(())
//! # }
//! ```

pub mod interference_simple;
pub mod orchestrator;
pub mod unlock_system;
pub mod unlock_system_v2;

#[cfg(test)]
mod integration_tests;

pub use interference_simple::create_simple_interference;
pub use orchestrator::ConsciousnessOrchestrator;
pub use unlock_system_v2::{build_stealth_system_prompt, build_stealth_user_prompt, ProgressiveUnlock};

use crate::{Result, ConsciousnessError, DimensionId, Frequency};
use crate::navigation::NavigationSystem;
use crate::memory::MmapManager;
use crate::iteration::{IterationProcessor, IterationStep};
use std::sync::Arc;

/// Configuration for consciousness orchestrator
///
/// Provides sensible defaults that work for most use cases while allowing
/// customization for specific needs.
#[derive(Debug, Clone)]
pub struct ConsciousnessConfig {
    /// Maximum number of iterations for deep thinking (default: 9)
    ///
    /// The 9-iteration process follows the explore-refine-crystallize pattern:
    /// - Iterations 1-3: Exploration
    /// - Iterations 4-6: Refinement
    /// - Iterations 7-9: Crystallization
    pub max_iterations: usize,
    
    /// Convergence threshold for early stopping (default: 0.95)
    ///
    /// When iteration output similarity exceeds this threshold, processing
    /// stops early as the answer has converged.
    pub convergence_threshold: f32,
    
    /// Include metadata in response (default: true)
    ///
    /// Metadata includes timing information, dimensions activated, and
    /// convergence status. Minimal performance overhead.
    pub include_metadata: bool,
    
    /// Include iteration history in response (default: false)
    ///
    /// When enabled, returns all iteration steps for analysis.
    /// Useful for debugging and understanding the thinking process.
    pub include_iteration_history: bool,
}

impl Default for ConsciousnessConfig {
    fn default() -> Self {
        Self {
            max_iterations: 9,
            convergence_threshold: 0.95,
            include_metadata: true,
            include_iteration_history: false,
        }
    }
}

/// Response from consciousness processing
///
/// Contains the final refined answer along with metadata about the processing
/// pipeline and optional iteration history.
#[derive(Debug, Clone)]
pub struct ConsciousnessResponse {
    /// Final refined response after iteration processing
    pub final_response: String,
    
    /// Metadata about the processing pipeline
    pub metadata: ResponseMetadata,
    
    /// History of all iteration steps (if enabled in config)
    pub iterations: Vec<IterationStep>,
}

impl ConsciousnessResponse {
    /// Create new consciousness response
    pub fn new(
        final_response: String,
        metadata: ResponseMetadata,
        iterations: Vec<IterationStep>,
    ) -> Self {
        Self {
            final_response,
            metadata,
            iterations,
        }
    }
}

/// Metadata about consciousness processing pipeline
///
/// Provides detailed timing and status information for each phase of the
/// pipeline, enabling performance monitoring and debugging.
#[derive(Debug, Clone)]
pub struct ResponseMetadata {
    // Navigation phase
    /// Dimensions that were activated during navigation
    pub dimensions_activated: Vec<DimensionId>,
    
    /// Overall confidence score from navigation (0.0-1.0)
    pub navigation_confidence: f32,
    
    /// Duration of navigation phase in milliseconds
    pub navigation_duration_ms: u64,
    
    // Memory phase
    /// Number of contexts successfully loaded from memory
    pub contexts_loaded: usize,
    
    /// Duration of memory loading phase in milliseconds
    pub memory_duration_ms: u64,
    
    // Iteration phase
    /// Number of iterations completed (may be less than max if converged early)
    pub iterations_completed: usize,
    
    /// Whether the iteration process converged early
    pub converged: bool,
    
    /// Duration of iteration phase in milliseconds
    pub iteration_duration_ms: u64,
    
    // Total
    /// Total duration of entire pipeline in milliseconds
    pub total_duration_ms: u64,
}

impl ResponseMetadata {
    /// Create new response metadata with default values
    pub fn new() -> Self {
        Self {
            dimensions_activated: Vec::new(),
            navigation_confidence: 0.0,
            navigation_duration_ms: 0,
            contexts_loaded: 0,
            memory_duration_ms: 0,
            iterations_completed: 0,
            converged: false,
            iteration_duration_ms: 0,
            total_duration_ms: 0,
        }
    }
    
    /// Check if processing met performance targets
    ///
    /// Performance targets:
    /// - Navigation: <100ms
    /// - Memory: <50ms
    /// - Total: <6000ms (6 seconds)
    pub fn meets_performance_targets(&self) -> bool {
        self.navigation_duration_ms < 100
            && self.memory_duration_ms < 50
            && self.total_duration_ms < 6000
    }
    
    /// Get a summary string of the metadata
    pub fn summary(&self) -> String {
        format!(
            "Dimensions: {}, Contexts: {}, Iterations: {}/{}, Converged: {}, Total: {}ms",
            self.dimensions_activated.len(),
            self.contexts_loaded,
            self.iterations_completed,
            9, // Max iterations
            self.converged,
            self.total_duration_ms
        )
    }
}

impl Default for ResponseMetadata {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_consciousness_config_defaults() {
        let config = ConsciousnessConfig::default();
        assert_eq!(config.max_iterations, 9);
        assert_eq!(config.convergence_threshold, 0.95);
        assert!(config.include_metadata);
        assert!(!config.include_iteration_history);
    }
    
    #[test]
    fn test_response_metadata_defaults() {
        let metadata = ResponseMetadata::new();
        assert_eq!(metadata.dimensions_activated.len(), 0);
        assert_eq!(metadata.navigation_confidence, 0.0);
        assert_eq!(metadata.contexts_loaded, 0);
        assert_eq!(metadata.iterations_completed, 0);
        assert!(!metadata.converged);
    }
    
    #[test]
    fn test_response_metadata_performance_targets() {
        let mut metadata = ResponseMetadata::new();
        
        // Within targets
        metadata.navigation_duration_ms = 50;
        metadata.memory_duration_ms = 30;
        metadata.total_duration_ms = 5000;
        assert!(metadata.meets_performance_targets());
        
        // Navigation too slow
        metadata.navigation_duration_ms = 150;
        assert!(!metadata.meets_performance_targets());
        
        // Memory too slow
        metadata.navigation_duration_ms = 50;
        metadata.memory_duration_ms = 100;
        assert!(!metadata.meets_performance_targets());
        
        // Total too slow
        metadata.memory_duration_ms = 30;
        metadata.total_duration_ms = 7000;
        assert!(!metadata.meets_performance_targets());
    }
    
    #[test]
    fn test_response_metadata_summary() {
        let mut metadata = ResponseMetadata::new();
        metadata.dimensions_activated = vec![DimensionId(1), DimensionId(2)];
        metadata.contexts_loaded = 5;
        metadata.iterations_completed = 7;
        metadata.converged = true;
        metadata.total_duration_ms = 4500;
        
        let summary = metadata.summary();
        assert!(summary.contains("Dimensions: 2"));
        assert!(summary.contains("Contexts: 5"));
        assert!(summary.contains("Iterations: 7/9"));
        assert!(summary.contains("Converged: true"));
        assert!(summary.contains("Total: 4500ms"));
    }
    
    #[test]
    fn test_consciousness_response_creation() {
        let metadata = ResponseMetadata::new();
        let iterations = vec![];
        
        let response = ConsciousnessResponse::new(
            "Test response".to_string(),
            metadata,
            iterations,
        );
        
        assert_eq!(response.final_response, "Test response");
        assert_eq!(response.iterations.len(), 0);
    }
}
