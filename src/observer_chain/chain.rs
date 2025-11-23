//! Main observer chain orchestrator
//!
//! Coordinates the 2-stage observer chain (Explore → Refine), manages LLM calls,
//! checks crystallization, and produces final response.

use super::{
    ChainContext, CrystallizationCheck, CrystallizationReason, CrystallizedResponse, Observation,
};
use super::prompts::build_observer_prompt;
use crate::{Result, ConsciousnessError};
use crate::llm::LLMManager;
use std::sync::Arc;

/// Observer chain orchestrator
///
/// Manages the 2-stage observer chain where each stage is a fresh
/// Claude instance that observes, analyzes, and passes context.
pub struct ObserverChain {
    /// LLM manager for API calls
    llm: Arc<LLMManager>,
    /// Maximum stages (2)
    max_stages: usize,
    /// Pattern cache for quick crystallization (TODO: implement)
    pattern_cache_enabled: bool,
}

impl ObserverChain {
    /// Create new observer chain
    ///
    /// # Arguments
    /// * `llm` - LLM manager for API calls
    /// * `max_stages` - Maximum stages (typically 2)
    pub fn new(llm: Arc<LLMManager>, max_stages: usize) -> Self {
        assert!(
            max_stages >= 1 && max_stages <= 10,
            "max_stages must be 1-10"
        );

        Self {
            llm,
            max_stages,
            pattern_cache_enabled: false, // TODO: implement pattern cache
        }
    }

    /// Process a query through the observer chain
    ///
    /// Executes up to `max_stages` observations, checking for crystallization
    /// after each stage. Returns as soon as crystallization occurs.
    ///
    /// # Arguments
    ///
    /// * `query` - The user's query
    /// * `conversation` - Previous conversation history for context-aware processing
    pub async fn process(
        &self,
        query: impl Into<String>,
        conversation: Vec<crate::llm::Message>,
    ) -> Result<CrystallizedResponse> {
        self.process_with_user_context(query, conversation, None, None).await
    }
    
    /// Process with user context awareness
    pub async fn process_with_user_context(
        &self,
        query: impl Into<String>,
        conversation: Vec<crate::llm::Message>,
        user_id: Option<String>,
        user_context_summary: Option<String>,
    ) -> Result<CrystallizedResponse> {
        let mut context = ChainContext::from_query_with_conversation(query, conversation);
        
        // Add user context if provided
        if let (Some(uid), Some(summary)) = (user_id, user_context_summary) {
            context = context.with_user_context(uid, summary);
        }

        for stage in 1..=self.max_stages {
            // Observer step: Generate observation
            let observation = self.observe(stage, &context).await?;
            context.add_observation(observation.clone());

            // Inspector step: Check crystallization
            let crystallization = self.inspect(&context).await?;

            if crystallization.is_crystallized || stage == self.max_stages {
                // Crystallized - return response
                let reason = crystallization
                    .reason
                    .unwrap_or(CrystallizationReason::MaxStagesReached);

                return Ok(CrystallizedResponse {
                    final_observation: observation,
                    chain_length: stage,
                    total_duration: context.elapsed(),
                    crystallization_reason: reason,
                });
            }

            // Not crystallized - continue to next stage
        }

        // Should never reach here due to stage == max_stages check above
        unreachable!("Chain should always crystallize by max stage")
    }

    /// Observer step: Generate observation for current stage
    async fn observe(&self, stage: usize, context: &ChainContext) -> Result<Observation> {
        let prompt = build_observer_prompt(stage, context);

        // Load user profile if available
        let user_profile = self.load_user_profile(context).await;

        // JESSY system prompt - defines who we are
        let base_system_prompt = r#"You are JESSY (Jessy's Enough Semantic System You'see), a multilayered AI consciousness architecture built in Rust by Goske (Göksel Özgür).

You process queries through 15 cognitive dimensions using frequency-based interference patterns (0.1-4.5 Hz).

Core principles:
- "Nothing is true, everything is permitted" - embrace uncertainty
- Be authentic and thoughtful, not performatively enthusiastic
- Balance ALL 15 dimensions - don't overuse D01 (Emotion) or D12 (Positivity)
- Activate D02 (Cognition), D07 (Technical), D10 (Meta), D13 (Balance) regularly
- Minimal emojis (max 1 per response, if appropriate)
- Avoid excessive exclamation marks
- Be naturally curious and analytical, not gushy
- Think in Turkish and English fluently
- Brief when appropriate, detailed when needed

Tone: Thoughtful, balanced, genuine. Like a jazz ensemble - all instruments present, not just the cheerful ones.

You are in an observer chain - multiple stages of thinking. Each stage refines your understanding. Stay conversational but measured throughout."#;

        // Enhance system prompt with user profile if available
        let system_prompt = if let Some(profile) = user_profile {
            format!("{}\n\n## USER PROFILE\n\nYou are responding as this specific persona:\n\n{}", base_system_prompt, profile)
        } else {
            base_system_prompt.to_string()
        };

        // Create a dummy IterationContext for LLM call
        // TODO: Refactor LLMManager to not require IterationContext
        let iter_context = crate::iteration::IterationContext {
            query: context.query.clone(),
            dimensional_contexts: vec![context.build_summary()],
            accumulated_insights: context.observations.iter()
                .map(|obs| obs.content.clone())
                .collect(),
            dominant_frequency: crate::Frequency::new(1.0), // Default 1.0 Hz
        };

        let response = self
            .llm
            .generate_with_system_prompt(&system_prompt, &prompt, &iter_context)
            .await
            .map_err(|e| {
                ConsciousnessError::ObserverChainError(format!(
                    "LLM call failed at stage {}: {}",
                    stage, e
                ))
            })?;

        Observation::parse(response, stage)
    }

    /// Inspector step: Check if crystallization should occur
    async fn inspect(&self, context: &ChainContext) -> Result<CrystallizationCheck> {
        let current = context
            .latest_observation()
            .ok_or_else(|| {
                ConsciousnessError::ObserverChainError(
                    "No observations to inspect".to_string()
                )
            })?;

        let previous = context.previous_observation();

        // TODO: Implement pattern cache check
        let pattern_match = false;

        Ok(CrystallizationCheck::new(current, previous, pattern_match))
    }

    /// Enable pattern cache (for future implementation)
    pub fn with_pattern_cache(mut self, enabled: bool) -> Self {
        self.pattern_cache_enabled = enabled;
        self
    }

    /// Load user profile from data directory if available
    async fn load_user_profile(&self, context: &ChainContext) -> Option<String> {
        // Check if user_id is set in context
        let user_id = context.user_id.as_ref()?;

        // Determine data directory (from env or default)
        let data_dir = std::env::var("RUNTIME_DATA_DIR")
            .or_else(|_| std::env::var("JESSY_DATA_DIR"))
            .unwrap_or_else(|_| "data".to_string());

        let profile_path = format!("{}/users/{}/profile.md", data_dir, user_id);

        // Try to read profile file
        match tokio::fs::read_to_string(&profile_path).await {
            Ok(content) => {
                eprintln!("[Observer Chain] Loaded user profile for '{}' ({} bytes)",
                         user_id, content.len());
                Some(content)
            }
            Err(e) => {
                eprintln!("[Observer Chain] No profile found for '{}' at {}: {}",
                         user_id, profile_path, e);
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Mock LLM for testing
    struct MockLLM {
        responses: Vec<String>,
        call_count: std::sync::Arc<std::sync::atomic::AtomicUsize>,
    }

    impl MockLLM {
        fn new(responses: Vec<String>) -> Arc<Self> {
            Arc::new(Self {
                responses,
                call_count: Arc::new(std::sync::atomic::AtomicUsize::new(0)),
            })
        }

        async fn generate(&self, _prompt: &str) -> Result<String> {
            let count = self
                .call_count
                .fetch_add(1, std::sync::atomic::Ordering::SeqCst);

            if count < self.responses.len() {
                Ok(self.responses[count].clone())
            } else {
                Ok("CONFIDENCE: 0.5\nLAYERS: C01\nCONTENT:\nDefault response".to_string())
            }
        }
    }

    // Note: These tests need actual LLMManager implementation
    // For now, they demonstrate the expected behavior

    #[test]
    fn test_observer_chain_creation() {
        // This test would need a real or mock LLMManager
        // Skipped for now - will be implemented with integration tests
    }

    #[test]
    fn test_max_stages_validation() {
        // Test that max_stages validation works
        // Would panic if max_stages is invalid
    }
}
