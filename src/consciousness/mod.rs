//! Consciousness Orchestrator - The Heart of Jessy
//!
//! This module integrates Navigation, Memory, and Iteration systems into a unified
//! consciousness that can think deeply about queries using dimensional knowledge.
//!
//! ## Architecture
//!
//! ```text
//! Query → Navigate → Load Memory → Calculate Interference → Iterate → Response
//!         (35µs)      (<50ms)        (minimal)              (5s)
//! ```
//!
//! ## 9-Iteration Deep Thinking
//!
//! The orchestrator implements the OWL (Observe-Wonder-Learn) pattern through
//! 9 iterations of progressive refinement:
//! - Iterations 1-3: Exploration (observe the problem space)
//! - Iterations 4-6: Refinement (wonder about patterns)
//! - Iterations 7-9: Crystallization (learn the essence)

pub mod orchestrator;
pub mod interference_simple;

pub use orchestrator::{ConsciousnessOrchestrator, ConsciousnessResponse, ResponseMetadata};
pub use interference_simple::create_simple_interference;

use crate::Result;

/// Configuration for consciousness processing
#[derive(Debug, Clone)]
pub struct ConsciousnessConfig {
    /// Maximum iterations for deep thinking (default: 9)
    pub max_iterations: usize,
    
    /// Convergence threshold for early stopping (default: 0.95)
    pub convergence_threshold: f32,
    
    /// Enable detailed metadata in responses (default: true)
    pub include_metadata: bool,
    
    /// Enable iteration history in responses (default: false)
    pub include_iterations: bool,
}

impl Default for ConsciousnessConfig {
    fn default() -> Self {
        Self {
            max_iterations: 9,
            convergence_threshold: 0.95,
            include_metadata: true,
            include_iterations: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_consciousness_config() {
        let config = ConsciousnessConfig::default();
        assert_eq!(config.max_iterations, 9);
        assert_eq!(config.convergence_threshold, 0.95);
        assert!(config.include_metadata);
        assert!(!config.include_iterations);
    }
}
