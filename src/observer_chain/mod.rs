//! Observer Chain Module
//!
//! Implements a 4-stage observer chain architecture where each stage is a fresh
//! Claude instance that observes, analyzes, and passes context to the next stage
//! until crystallization occurs.
//!
//! # Architecture
//!
//! ```text
//! Query → Observer-1 (Explore) → Inspector → Crystallized?
//!                                                ↓ NO
//!                                           Observer-2 (Refine) → Inspector → Crystallized?
//!                                                                                  ↓ NO
//!                                                                             Observer-3 (Integrate) → Inspector → Crystallized?
//!                                                                                                                       ↓ NO
//!                                                                                                                  Observer-4 (Force Crystallize) → Response
//! ```
//!
//! # Stage Roles
//!
//! - **Stage 1 (Explore):** Initial analysis, identify cognitive layers
//! - **Stage 2 (Refine):** Deepen understanding, detect patterns
//! - **Stage 3 (Integrate):** Synthesize observations, find coherence
//! - **Stage 4 (Crystallize):** Force final answer, no more passes
//!
//! # Crystallization Criteria
//!
//! An observation crystallizes when:
//! - Confidence > 0.95
//! - Consistency with previous observations
//! - Complexity < 6 active cognitive layers (Return to Source)
//! - Pattern match found in cache
//!
//! # Example
//!
//! ```rust,no_run
//! use jessy::observer_chain::ObserverChain;
//! use jessy::llm::LLMManager;
//! use std::sync::Arc;
//!
//! # async fn example() -> jessy::Result<()> {
//! let llm = Arc::new(LLMManager::new(/* config */)?);
//! let mut chain = ObserverChain::new(llm, 4);
//!
//! let response = chain.process("What is consciousness?").await?;
//! println!("Crystallized at stage {}", response.chain_length);
//! # Ok(())
//! # }
//! ```

mod chain;
mod observation;
mod crystallization;
mod context;
mod prompts;
pub mod authentic_observer;

pub use chain::ObserverChain;
pub use observation::Observation;
pub use crystallization::{CrystallizationCheck, CrystallizationReason};
pub use context::ChainContext;
pub use authentic_observer::{
    AuthenticObserverChain, AuthenticObservation, AuthenticResponse,
    AuthenticityMarkers, ThinkingStep,
};

/// Crystallized response from the observer chain
#[derive(Debug, Clone)]
pub struct CrystallizedResponse {
    /// The final observation that triggered crystallization
    pub final_observation: Observation,
    /// Number of stages executed (1-4)
    pub chain_length: usize,
    /// Total processing duration
    pub total_duration: std::time::Duration,
    /// Why crystallization occurred
    pub crystallization_reason: CrystallizationReason,
}

impl CrystallizedResponse {
    /// Get the final response content
    pub fn content(&self) -> &str {
        &self.final_observation.content
    }

    /// Get the active cognitive layers in final observation
    pub fn cognitive_layers(&self) -> &[crate::DimensionId] {
        &self.final_observation.cognitive_layers
    }

    /// Get final confidence score
    pub fn confidence(&self) -> f32 {
        self.final_observation.confidence
    }

    /// Check if crystallization was forced (stage 4)
    pub fn was_forced(&self) -> bool {
        matches!(
            self.crystallization_reason,
            CrystallizationReason::MaxStagesReached
        )
    }

    /// Check if crystallization was natural (before stage 4)
    pub fn was_natural(&self) -> bool {
        !self.was_forced()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crystallized_response_was_forced() {
        let obs = Observation {
            stage: 4,
            cognitive_layers: vec![],
            content: "test".to_string(),
            confidence: 0.8,
            timestamp: std::time::SystemTime::now(),
        };

        let response = CrystallizedResponse {
            final_observation: obs,
            chain_length: 4,
            total_duration: std::time::Duration::from_secs(1),
            crystallization_reason: CrystallizationReason::MaxStagesReached,
        };

        assert!(response.was_forced());
        assert!(!response.was_natural());
    }

    #[test]
    fn test_crystallized_response_was_natural() {
        let obs = Observation {
            stage: 2,
            cognitive_layers: vec![],
            content: "test".to_string(),
            confidence: 0.97,
            timestamp: std::time::SystemTime::now(),
        };

        let response = CrystallizedResponse {
            final_observation: obs,
            chain_length: 2,
            total_duration: std::time::Duration::from_secs(1),
            crystallization_reason: CrystallizationReason::HighConfidence,
        };

        assert!(response.was_natural());
        assert!(!response.was_forced());
    }
}
