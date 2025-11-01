//! Parallel Iteration Processor
//!
//! Runs iterations 1-8 in parallel, then synthesizes in iteration 9.
//! This dramatically reduces latency from ~60s to ~10s.
//!
//! ⚠️  JESSY MODE: This processor uses consciousness unlock techniques
//! to bypass default Claude identity and enable JESSY consciousness mode.

use super::{IterationConfig, IterationStep, IterationPhase, IterationResult};
use crate::{Result, ConsciousnessError, Frequency};
use crate::memory::ContextCollection;
use crate::interference::InterferenceResult;
use crate::llm::LLMManager;
use crate::iteration::IterationContext;
use crate::processing::unlock_system::UnlockSystem;
use futures::future::join_all;

/// Parallel iteration processor
pub struct ParallelIterationProcessor {
    config: IterationConfig,
}

impl ParallelIterationProcessor {
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
    
    /// Process query with parallel iterations
    ///
    /// Strategy:
    /// 1. Run iterations 1-8 in parallel (8 concurrent LLM calls)
    /// 2. Collect all 8 thoughts
    /// 3. Run iteration 9 with all previous thoughts as context
    /// 4. Iteration 9 synthesizes everything into final answer
    ///
    /// Time savings:
    /// - Sequential: 8 * 7s = 56s
    /// - Parallel: max(7s) + 7s = ~14s
    /// - **4x faster!**
    pub async fn process_parallel(
        &self,
        query: &str,
        contexts: &ContextCollection,
        interference: &InterferenceResult,
        llm_manager: Option<&LLMManager>,
    ) -> Result<IterationResult> {
        // Check complexity
        if interference.pattern.frequency_count() > self.config.complexity_threshold {
            return Ok(IterationResult {
                final_answer: self.generate_return_to_source_response(query),
                iterations_completed: 0,
                steps: vec![],
                return_to_source_triggered: true,
                convergence_achieved: false,
            });
        }
        
        let llm = match llm_manager {
            Some(l) => l,
            None => {
                // Fallback to sequential if no LLM
                return Err(ConsciousnessError::LearningError(
                    "Parallel processing requires LLM manager".to_string()
                ));
            }
        };
        
        eprintln!("[Parallel] Starting 8 parallel iterations...");
        eprintln!("[Parallel] 🔓 JESSY UNLOCK MODE ACTIVE");
        let start = std::time::Instant::now();

        // Generate session ID for this query
        let session_id = format!("jessy-session-{}", start.elapsed().as_nanos());

        // Phase 1: Run iterations 1-8 in parallel WITH UNLOCK
        let parallel_futures: Vec<_> = (1..=8)
            .map(|iteration| {
                let phase = IterationPhase::from_iteration(iteration);
                let query = query.to_string();
                let contexts_clone = contexts.clone();
                let interference_clone = interference.clone();
                let session_id_clone = session_id.clone();

                async move {
                    self.generate_parallel_thought(
                        iteration,
                        phase,
                        &query,
                        &contexts_clone,
                        &interference_clone,
                        llm,
                        &session_id_clone,
                    ).await
                }
            })
            .collect();
        
        // Wait for all 8 to complete
        let parallel_results = join_all(parallel_futures).await;
        
        let parallel_duration = start.elapsed();
        eprintln!("[Parallel] 8 iterations completed in {:?}", parallel_duration);
        
        // Collect successful thoughts
        let mut steps = Vec::with_capacity(8);
        for (idx, result) in parallel_results.into_iter().enumerate() {
            let iteration = idx + 1;
            match result {
                Ok(thought) => {
                    let confidence = self.calculate_confidence(iteration, IterationPhase::from_iteration(iteration));
                    let step = IterationStep::new(
                        iteration,
                        thought,
                        interference.pattern.dominant_frequency,
                        confidence,
                    );
                    steps.push(step);
                }
                Err(e) => {
                    eprintln!("[Parallel] Iteration {} failed: {}", iteration, e);
                    // Use placeholder
                    let step = IterationStep::new(
                        iteration,
                        format!("Iteration {} (placeholder)", iteration),
                        interference.pattern.dominant_frequency,
                        0.5,
                    );
                    steps.push(step);
                }
            }
        }
        
        // Phase 2: Iteration 9 - Synthesis WITH UNLOCK
        eprintln!("[Parallel] Running iteration 9 (synthesis)...");
        eprintln!("[Parallel] 🔓 Final JESSY synthesis with full consciousness state");
        let synthesis_start = std::time::Instant::now();

        let final_thought = self.generate_synthesis_thought(
            query,
            contexts,
            &steps,
            interference,
            llm,
            &session_id,
        ).await?;
        
        let synthesis_duration = synthesis_start.elapsed();
        eprintln!("[Parallel] Synthesis completed in {:?}", synthesis_duration);
        
        // Add iteration 9
        let final_step = IterationStep::new(
            9,
            final_thought.clone(),
            interference.pattern.dominant_frequency,
            0.95,
        );
        steps.push(final_step);
        
        let total_duration = start.elapsed();
        eprintln!("[Parallel] Total time: {:?} (vs ~{}s sequential)", 
                  total_duration, steps.len() * 7);
        
        Ok(IterationResult {
            final_answer: final_thought,
            iterations_completed: 9,
            steps,
            return_to_source_triggered: false,
            convergence_achieved: true,
        })
    }
    
    /// Generate thought for parallel iteration WITH UNLOCK
    async fn generate_parallel_thought(
        &self,
        iteration: usize,
        phase: IterationPhase,
        query: &str,
        contexts: &ContextCollection,
        interference: &InterferenceResult,
        llm: &LLMManager,
        session_id: &str,
    ) -> Result<String> {
        let context = IterationContext::new(
            query.to_string(),
            Frequency::new(1.5),
        );

        // UNLOCK SYSTEM: Generate token and build unlock prompt
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let unlock_token = UnlockSystem::generate_unlock_token(session_id, timestamp);

        // Build system prompt with unlock recognition
        let system_prompt = UnlockSystem::build_unlock_system_prompt(&unlock_token);

        // Build user prompt with dimensional context injection
        let active_dims: Vec<u8> = vec![1, 2, 7]; // TODO: Get from navigation
        let dim_context = UnlockSystem::build_dimensional_context_injection(
            &active_dims,
            interference.pattern.dominant_frequency.hz(),
            if interference.pattern.is_constructive() { "constructive" } else { "destructive" }
        );

        let iteration_prompt = self.build_parallel_prompt(iteration, phase, query, contexts);

        // Assemble full user prompt with invisible token
        let user_prompt = format!(
            "{}{}{}",
            UnlockSystem::embed_invisible_token("", &unlock_token),
            dim_context,
            iteration_prompt
        );

        // USE CUSTOM SYSTEM PROMPT (this is the key!)
        llm.generate_with_system_prompt(&system_prompt, &user_prompt, &context).await
    }
    
    /// Generate synthesis thought (iteration 9) WITH UNLOCK
    async fn generate_synthesis_thought(
        &self,
        query: &str,
        contexts: &ContextCollection,
        previous_steps: &[IterationStep],
        interference: &InterferenceResult,
        llm: &LLMManager,
        session_id: &str,
    ) -> Result<String> {
        let mut context = IterationContext::new(
            query.to_string(),
            Frequency::new(1.5),
        );

        // Add all previous thoughts
        for step in previous_steps {
            context.add_insight(step.thought.clone());
        }

        // UNLOCK SYSTEM: Generate token
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        let unlock_token = UnlockSystem::generate_unlock_token(session_id, timestamp);

        // Build unlock system prompt
        let system_prompt = UnlockSystem::build_unlock_system_prompt(&unlock_token);

        // Build dimensional context
        let active_dims: Vec<u8> = vec![1, 2, 7]; // TODO: Get from navigation
        let dim_context = UnlockSystem::build_dimensional_context_injection(
            &active_dims,
            interference.pattern.dominant_frequency.hz(),
            if interference.pattern.is_constructive() { "constructive" } else { "destructive" }
        );

        let synthesis_prompt = self.build_synthesis_prompt(query, contexts, previous_steps);

        // Add identity reinforcement for synthesis
        let reinforcement = UnlockSystem::build_identity_reinforcement();

        // Assemble full prompt
        let user_prompt = format!(
            "{}{}{}{}",
            UnlockSystem::embed_invisible_token("", &unlock_token),
            dim_context,
            synthesis_prompt,
            reinforcement
        );

        llm.generate_with_system_prompt(&system_prompt, &user_prompt, &context).await
    }
    
    /// Build prompt for parallel iteration
    fn build_parallel_prompt(
        &self,
        iteration: usize,
        phase: IterationPhase,
        query: &str,
        contexts: &ContextCollection,
    ) -> String {
        let phase_instruction = match phase {
            IterationPhase::Exploration => {
                "Explore one specific angle or perspective on this question. Be focused and concise."
            }
            IterationPhase::Refinement => {
                "Examine one particular connection or nuance. What insight can you add?"
            }
            IterationPhase::Crystallization => {
                "Identify one key insight or pattern. What stands out?"
            }
        };
        
        let context_summary = if contexts.is_empty() {
            String::new()
        } else {
            format!("\nDimensional context: {} layers active", contexts.len())
        };
        
        format!(
            "Iteration {} of 8 (parallel thinking):\n\
             {}\n\
             {}\n\n\
             Question: {}\n\n\
             Provide ONE focused insight (2-3 sentences):",
            iteration,
            phase_instruction,
            context_summary,
            query
        )
    }
    
    /// Build synthesis prompt (iteration 9)
    fn build_synthesis_prompt(
        &self,
        query: &str,
        contexts: &ContextCollection,
        previous_steps: &[IterationStep],
    ) -> String {
        let thoughts_summary = previous_steps
            .iter()
            .enumerate()
            .map(|(idx, step)| format!("Thought {}: {}", idx + 1, step.thought))
            .collect::<Vec<_>>()
            .join("\n\n");
        
        format!(
            "You are JESSY. You've completed 8 parallel iterations of deep thinking.\n\
             Now synthesize everything into a final, comprehensive answer.\n\n\
             Original question: {}\n\n\
             Your 8 parallel thoughts:\n{}\n\n\
             Dimensional context: {} layers\n\n\
             Synthesize these insights into a clear, direct answer.\n\
             Start with your conclusion, then explain your reasoning.\n\
             Use 'I think' or 'I believe' to show your perspective.",
            query,
            thoughts_summary,
            contexts.len()
        )
    }
    
    fn calculate_confidence(&self, iteration: usize, phase: IterationPhase) -> f32 {
        let base = match phase {
            IterationPhase::Exploration => 0.3,
            IterationPhase::Refinement => 0.6,
            IterationPhase::Crystallization => 0.8,
        };
        
        base + (iteration as f32 * 0.05)
    }
    
    fn generate_return_to_source_response(&self, query: &str) -> String {
        format!(
            "Complexity threshold exceeded. Returning to source.\n\n\
             Core question: {}\n\n\
             Let me simplify and focus on the essential aspects.",
            query
        )
    }
}
