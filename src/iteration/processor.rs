//! Core iteration processor implementation

use super::{IterationConfig, IterationStep, IterationPhase};
use crate::{Result, ConsciousnessError, Frequency};
use crate::memory::ContextCollection;
use crate::interference::InterferenceResult;

/// Main iteration processor for 9-iteration deep thinking
pub struct IterationProcessor {
    config: IterationConfig,
}

impl IterationProcessor {
    /// Create new iteration processor with configuration
    pub fn new(
        max_iterations: usize,
        convergence_threshold: f32,
        complexity_threshold: usize,
    ) -> Self {
        Self {
            config: IterationConfig {
                max_iterations,
                convergence_threshold,
                complexity_threshold,
                ..Default::default()
            },
        }
    }
    
    /// Process query through 9-iteration cycle
    pub async fn process(
        &self,
        query: &str,
        contexts: &ContextCollection,
        interference: &InterferenceResult,
    ) -> Result<IterationResult> {
        let mut steps = Vec::with_capacity(self.config.max_iterations);
        let mut return_to_source_triggered = false;
        
        // Check complexity before starting
        if interference.pattern.frequency_count() > self.config.complexity_threshold {
            return_to_source_triggered = true;
            return Ok(IterationResult {
                final_answer: self.generate_return_to_source_response(query),
                iterations_completed: 0,
                steps,
                return_to_source_triggered,
                convergence_achieved: false,
            });
        }
        
        for iteration in 1..=self.config.max_iterations {
            let phase = IterationPhase::from_iteration(iteration);
            
            // Generate thought for this iteration
            let thought = self.generate_iteration_thought(
                iteration,
                phase,
                query,
                contexts,
                &steps,
            ).await?;
            
            // Calculate confidence based on phase and iteration
            let confidence = self.calculate_confidence(iteration, phase);
            
            // Create iteration step
            let mut step = IterationStep::new(
                iteration,
                thought,
                interference.pattern.dominant_frequency,
                confidence,
            );
            
            // Check similarity to previous iteration
            if let Some(previous) = steps.last() {
                let similarity = self.calculate_similarity(&step.thought, &previous.thought);
                step = step.with_similarity(similarity);
                
                // Check for convergence (after minimum iterations)
                if iteration >= self.config.min_iterations_before_convergence {
                    if step.indicates_convergence(self.config.convergence_threshold) {
                        steps.push(step);
                        
                        let final_answer = self.generate_final_answer(query, &steps, contexts);
                        
                        return Ok(IterationResult {
                            final_answer,
                            iterations_completed: iteration,
                            steps,
                            return_to_source_triggered: false,
                            convergence_achieved: true,
                        });
                    }
                }
            }
            
            steps.push(step);
        }
        
        // Completed all iterations without early convergence
        let final_answer = self.generate_final_answer(query, &steps, contexts);
        
        Ok(IterationResult {
            final_answer,
            iterations_completed: self.config.max_iterations,
            steps,
            return_to_source_triggered: false,
            convergence_achieved: false,
        })
    }
    
    /// Generate thought for a specific iteration
    async fn generate_iteration_thought(
        &self,
        iteration: usize,
        phase: IterationPhase,
        query: &str,
        contexts: &ContextCollection,
        previous_steps: &[IterationStep],
    ) -> Result<String> {
        // In real implementation, this would call LLM with:
        // - Query
        // - Loaded contexts
        // - Previous iteration thoughts
        // - Phase-specific prompts
        
        // For now, generate structured thought based on phase
        let phase_prompt = match phase {
            IterationPhase::Exploration => {
                format!("Iteration {}: Exploring '{}' - What patterns emerge?", iteration, query)
            }
            IterationPhase::Refinement => {
                format!("Iteration {}: Refining understanding of '{}' - What connections exist?", iteration, query)
            }
            IterationPhase::Crystallization => {
                format!("Iteration {}: Crystallizing solution for '{}' - What is the essence?", iteration, query)
            }
        };
        
        Ok(phase_prompt)
    }
    
    /// Calculate confidence score for iteration
    fn calculate_confidence(&self, iteration: usize, phase: IterationPhase) -> f32 {
        // Confidence increases through phases
        let base_confidence = match phase {
            IterationPhase::Exploration => 0.4,
            IterationPhase::Refinement => 0.6,
            IterationPhase::Crystallization => 0.8,
        };
        
        // Increase within phase
        let phase_progress = match phase {
            IterationPhase::Exploration => (iteration - 1) as f32 / 3.0,
            IterationPhase::Refinement => (iteration - 4) as f32 / 3.0,
            IterationPhase::Crystallization => (iteration - 7) as f32 / 3.0,
        };
        
        (base_confidence + phase_progress * 0.2).min(1.0)
    }
    
    /// Calculate similarity between two thoughts
    fn calculate_similarity(&self, thought1: &str, thought2: &str) -> f32 {
        // Simple word-based similarity for now
        // In real implementation, would use embeddings or more sophisticated NLP
        
        let words1: std::collections::HashSet<&str> = thought1.split_whitespace().collect();
        let words2: std::collections::HashSet<&str> = thought2.split_whitespace().collect();
        
        let intersection = words1.intersection(&words2).count();
        let union = words1.union(&words2).count();
        
        if union == 0 {
            0.0
        } else {
            intersection as f32 / union as f32
        }
    }
    
    /// Generate final answer from iteration steps
    fn generate_final_answer(
        &self,
        query: &str,
        steps: &[IterationStep],
        contexts: &ContextCollection,
    ) -> String {
        // In real implementation, this would synthesize all iteration thoughts
        // with LLM to produce coherent final answer
        
        format!(
            "After {} iterations of deep thinking across {} dimensional contexts, \
             here is the synthesized response to '{}':\n\n\
             [Final answer would be generated by LLM from iteration chain]",
            steps.len(),
            contexts.len(),
            query
        )
    }
    
    /// Generate return-to-source response
    fn generate_return_to_source_response(&self, query: &str) -> String {
        format!(
            "I notice this question activates many dimensions simultaneously, \
             which can lead to analysis paralysis. Let me return to the source:\n\n\
             What is the real question behind '{}'?\n\n\
             Sometimes the most profound answers come from simplifying the question first.",
            query
        )
    }
}

/// Result of iteration processing
#[derive(Debug)]
pub struct IterationResult {
    /// Final synthesized answer
    pub final_answer: String,
    
    /// Number of iterations completed
    pub iterations_completed: usize,
    
    /// All iteration steps
    pub steps: Vec<IterationStep>,
    
    /// Whether return-to-source was triggered
    pub return_to_source_triggered: bool,
    
    /// Whether convergence was achieved early
    pub convergence_achieved: bool,
}

impl IterationResult {
    /// Get the average confidence across all iterations
    pub fn average_confidence(&self) -> f32 {
        if self.steps.is_empty() {
            return 0.0;
        }
        
        let sum: f32 = self.steps.iter().map(|s| s.confidence).sum();
        sum / self.steps.len() as f32
    }
    
    /// Get the final iteration step
    pub fn final_step(&self) -> Option<&IterationStep> {
        self.steps.last()
    }
    
    /// Check if processing was successful
    pub fn is_successful(&self) -> bool {
        !self.return_to_source_triggered && !self.steps.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_confidence_calculation() {
        let processor = IterationProcessor::new(9, 0.95, 6);
        
        // Exploration phase
        let conf1 = processor.calculate_confidence(1, IterationPhase::Exploration);
        assert!(conf1 >= 0.4 && conf1 < 0.6);
        
        // Refinement phase
        let conf4 = processor.calculate_confidence(4, IterationPhase::Refinement);
        assert!(conf4 >= 0.6 && conf4 < 0.8);
        
        // Crystallization phase
        let conf7 = processor.calculate_confidence(7, IterationPhase::Crystallization);
        assert!(conf7 >= 0.8);
    }
    
    #[test]
    fn test_similarity_calculation() {
        let processor = IterationProcessor::new(9, 0.95, 6);
        
        let thought1 = "This is a test thought";
        let thought2 = "This is a test thought";
        let thought3 = "Completely different content";
        
        let sim_identical = processor.calculate_similarity(thought1, thought2);
        assert_eq!(sim_identical, 1.0);
        
        let sim_different = processor.calculate_similarity(thought1, thought3);
        assert!(sim_different < 0.5);
    }
}
