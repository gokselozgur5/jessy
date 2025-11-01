//! 9-Iteration deep thinking processor for consciousness system
//!
//! This module implements the iterative refinement process that takes query context
//! through 9 cycles of exploration, refinement, and crystallization to produce
//! coherent responses with convergence detection and return-to-source protocol.

pub mod processor;
pub mod parallel_processor;
pub mod convergence;
pub mod context;

pub use processor::{IterationProcessor, IterationResult};
pub use parallel_processor::ParallelIterationProcessor;
pub use convergence::{ConvergenceDetector, ConvergenceState};
pub use context::{IterationContext, ThoughtChain};

use crate::{Result, ConsciousnessError, Frequency};

/// Configuration for iteration processing
#[derive(Debug, Clone)]
pub struct IterationConfig {
    /// Maximum number of iterations (default: 9)
    pub max_iterations: usize,
    
    /// Similarity threshold for convergence detection (default: 0.95)
    pub convergence_threshold: f32,
    
    /// Complexity threshold for return-to-source (default: 6 dimensions)
    pub complexity_threshold: usize,
    
    /// Minimum iterations before convergence check (default: 4)
    pub min_iterations_before_convergence: usize,
}

impl Default for IterationConfig {
    fn default() -> Self {
        Self {
            max_iterations: 9,
            convergence_threshold: 0.95,
            complexity_threshold: 6,
            min_iterations_before_convergence: 4,
        }
    }
}

/// Phase of the 9-iteration process
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IterationPhase {
    /// Iterations 1-3: Explore the problem space
    Exploration,
    
    /// Iterations 4-6: Refine understanding
    Refinement,
    
    /// Iterations 7-9: Crystallize solution
    Crystallization,
}

impl IterationPhase {
    /// Get the phase for a given iteration number
    pub fn from_iteration(iteration: usize) -> Self {
        match iteration {
            1..=3 => IterationPhase::Exploration,
            4..=6 => IterationPhase::Refinement,
            7..=9 => IterationPhase::Crystallization,
            _ => IterationPhase::Crystallization, // Beyond 9 stays in crystallization
        }
    }
    
    /// Get the description of this phase
    pub fn description(&self) -> &'static str {
        match self {
            IterationPhase::Exploration => "Exploring the problem space",
            IterationPhase::Refinement => "Refining understanding",
            IterationPhase::Crystallization => "Crystallizing solution",
        }
    }
    
    /// Get the focus areas for this phase
    pub fn focus_areas(&self) -> Vec<&'static str> {
        match self {
            IterationPhase::Exploration => vec![
                "What do we know?",
                "What don't we know?",
                "What assumptions are we making?",
            ],
            IterationPhase::Refinement => vec![
                "What patterns emerge?",
                "What connections exist?",
                "What contradictions appear?",
            ],
            IterationPhase::Crystallization => vec![
                "What is the essence?",
                "What is the simplest form?",
                "What is the right answer?",
            ],
        }
    }
}

/// Result of a single iteration
#[derive(Debug, Clone)]
pub struct IterationStep {
    pub iteration_number: usize,
    pub phase: IterationPhase,
    pub thought: String,
    pub frequency: Frequency,
    pub confidence: f32,
    pub similarity_to_previous: Option<f32>,
    pub timestamp: std::time::SystemTime,
}

impl IterationStep {
    /// Create new iteration step
    pub fn new(
        iteration_number: usize,
        thought: String,
        frequency: Frequency,
        confidence: f32,
    ) -> Self {
        Self {
            iteration_number,
            phase: IterationPhase::from_iteration(iteration_number),
            thought,
            frequency,
            confidence,
            similarity_to_previous: None,
            timestamp: std::time::SystemTime::now(),
        }
    }
    
    /// Set similarity to previous iteration
    pub fn with_similarity(mut self, similarity: f32) -> Self {
        self.similarity_to_previous = Some(similarity);
        self
    }
    
    /// Check if this step indicates convergence
    pub fn indicates_convergence(&self, threshold: f32) -> bool {
        self.similarity_to_previous
            .map(|sim| sim >= threshold)
            .unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_iteration_phase() {
        assert_eq!(IterationPhase::from_iteration(1), IterationPhase::Exploration);
        assert_eq!(IterationPhase::from_iteration(3), IterationPhase::Exploration);
        assert_eq!(IterationPhase::from_iteration(4), IterationPhase::Refinement);
        assert_eq!(IterationPhase::from_iteration(6), IterationPhase::Refinement);
        assert_eq!(IterationPhase::from_iteration(7), IterationPhase::Crystallization);
        assert_eq!(IterationPhase::from_iteration(9), IterationPhase::Crystallization);
        assert_eq!(IterationPhase::from_iteration(10), IterationPhase::Crystallization);
    }
    
    #[test]
    fn test_iteration_step() {
        let step = IterationStep::new(
            1,
            "Initial thought".to_string(),
            Frequency::new(1.5),
            0.7,
        );
        
        assert_eq!(step.iteration_number, 1);
        assert_eq!(step.phase, IterationPhase::Exploration);
        assert!(!step.indicates_convergence(0.95));
        
        let step_with_sim = step.with_similarity(0.96);
        assert!(step_with_sim.indicates_convergence(0.95));
    }
    
    #[test]
    fn test_iteration_config() {
        let config = IterationConfig::default();
        assert_eq!(config.max_iterations, 9);
        assert_eq!(config.convergence_threshold, 0.95);
        assert_eq!(config.complexity_threshold, 6);
    }
}
