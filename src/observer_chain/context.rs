//! Chain context accumulation
//!
//! Manages context that accumulates across observer stages.
//! Each stage sees the query + all previous observations.

use super::Observation;
use std::time::{Duration, SystemTime};

/// Context accumulated across observer chain stages
#[derive(Debug, Clone)]
pub struct ChainContext {
    /// Original user query
    pub query: String,
    /// All observations so far (ordered by stage)
    pub observations: Vec<Observation>,
    /// When processing started
    pub start_time: SystemTime,
    /// Conversation history (for context-aware deep thinking)
    pub conversation_history: Vec<crate::llm::Message>,
}

impl ChainContext {
    /// Create new context from query
    pub fn from_query(query: impl Into<String>) -> Self {
        Self {
            query: query.into(),
            observations: Vec::new(),
            start_time: SystemTime::now(),
            conversation_history: Vec::new(),
        }
    }

    /// Create new context from query with conversation history
    pub fn from_query_with_conversation(
        query: impl Into<String>,
        conversation: Vec<crate::llm::Message>
    ) -> Self {
        Self {
            query: query.into(),
            observations: Vec::new(),
            start_time: SystemTime::now(),
            conversation_history: conversation,
        }
    }

    /// Add an observation to context
    pub fn add_observation(&mut self, observation: Observation) {
        self.observations.push(observation);
    }

    /// Get the most recent observation
    pub fn latest_observation(&self) -> Option<&Observation> {
        self.observations.last()
    }

    /// Get the previous observation (before latest)
    pub fn previous_observation(&self) -> Option<&Observation> {
        if self.observations.len() >= 2 {
            self.observations.get(self.observations.len() - 2)
        } else {
            None
        }
    }

    /// Get current stage number (1-indexed)
    pub fn current_stage(&self) -> usize {
        self.observations.len() + 1
    }

    /// Get elapsed time since start
    pub fn elapsed(&self) -> Duration {
        self.start_time
            .elapsed()
            .unwrap_or(Duration::from_secs(0))
    }

    /// Build a summary of all observations for next stage
    pub fn build_summary(&self) -> String {
        if self.observations.is_empty() {
            return String::new();
        }

        let mut summary = String::from("Previous observations:\n\n");

        for obs in &self.observations {
            summary.push_str(&format!(
                "Stage {}: (confidence: {:.2})\n{}\n\n",
                obs.stage, obs.confidence, obs.content
            ));
        }

        summary
    }

    /// Get aggregated cognitive layers from all observations
    ///
    /// Returns unique cognitive layers mentioned across all observations,
    /// ordered by frequency of mention.
    pub fn aggregated_layers(&self) -> Vec<crate::DimensionId> {
        use std::collections::HashMap;

        let mut layer_counts: HashMap<crate::DimensionId, usize> = HashMap::new();

        for obs in &self.observations {
            for layer in &obs.cognitive_layers {
                *layer_counts.entry(*layer).or_insert(0) += 1;
            }
        }

        let mut layers: Vec<_> = layer_counts.into_iter().collect();
        layers.sort_by(|a, b| b.1.cmp(&a.1)); // Sort by count descending

        layers.into_iter().map(|(layer, _)| layer).collect()
    }

    /// Get average confidence across all observations
    pub fn average_confidence(&self) -> f32 {
        if self.observations.is_empty() {
            return 0.0;
        }

        let sum: f32 = self.observations.iter().map(|o| o.confidence).sum();
        sum / self.observations.len() as f32
    }

    /// Format conversation history for prompt inclusion
    pub fn format_conversation_history(&self) -> String {
        if self.conversation_history.is_empty() {
            return String::new();
        }

        let mut formatted = String::from("CONVERSATION HISTORY:\n");
        formatted.push_str("(Previous messages in this conversation)\n\n");

        for (idx, msg) in self.conversation_history.iter().enumerate() {
            let role_label = match msg.role.as_str() {
                "user" => "User",
                "assistant" => "Assistant",
                _ => "Unknown",
            };

            formatted.push_str(&format!("[{}] {}:\n{}\n\n",
                idx + 1, role_label, msg.content));
        }

        formatted
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;

    fn make_observation(
        stage: usize,
        layers: Vec<u8>,
        content: &str,
        confidence: f32,
    ) -> Observation {
        Observation::new(
            stage,
            layers.into_iter().map(DimensionId).collect(),
            content.to_string(),
            confidence,
        )
    }

    #[test]
    fn test_context_from_query() {
        let ctx = ChainContext::from_query("What is consciousness?");

        assert_eq!(ctx.query, "What is consciousness?");
        assert_eq!(ctx.observations.len(), 0);
        assert_eq!(ctx.current_stage(), 1);
    }

    #[test]
    fn test_add_observation() {
        let mut ctx = ChainContext::from_query("test query");
        let obs = make_observation(1, vec![1, 2], "analysis", 0.8);

        ctx.add_observation(obs);

        assert_eq!(ctx.observations.len(), 1);
        assert_eq!(ctx.current_stage(), 2);
    }

    #[test]
    fn test_latest_and_previous() {
        let mut ctx = ChainContext::from_query("test");
        let obs1 = make_observation(1, vec![1], "first", 0.7);
        let obs2 = make_observation(2, vec![2], "second", 0.8);

        ctx.add_observation(obs1);
        ctx.add_observation(obs2);

        assert_eq!(ctx.latest_observation().unwrap().stage, 2);
        assert_eq!(ctx.previous_observation().unwrap().stage, 1);
    }

    #[test]
    fn test_build_summary() {
        let mut ctx = ChainContext::from_query("test");
        let obs1 = make_observation(1, vec![1], "First analysis", 0.7);
        let obs2 = make_observation(2, vec![2], "Second analysis", 0.8);

        ctx.add_observation(obs1);
        ctx.add_observation(obs2);

        let summary = ctx.build_summary();

        assert!(summary.contains("Stage 1"));
        assert!(summary.contains("Stage 2"));
        assert!(summary.contains("First analysis"));
        assert!(summary.contains("Second analysis"));
        assert!(summary.contains("0.70"));
        assert!(summary.contains("0.80"));
    }

    #[test]
    fn test_aggregated_layers() {
        let mut ctx = ChainContext::from_query("test");
        let obs1 = make_observation(1, vec![1, 2, 7], "first", 0.7);
        let obs2 = make_observation(2, vec![1, 2, 3], "second", 0.8);
        let obs3 = make_observation(3, vec![1, 7], "third", 0.9);

        ctx.add_observation(obs1);
        ctx.add_observation(obs2);
        ctx.add_observation(obs3);

        let layers = ctx.aggregated_layers();

        // Layer 1 appears 3 times (most frequent)
        assert_eq!(layers[0].0, 1);
        // Layers 2 and 7 appear 2 times each
        assert!(layers[1].0 == 2 || layers[1].0 == 7);
    }

    #[test]
    fn test_average_confidence() {
        let mut ctx = ChainContext::from_query("test");
        let obs1 = make_observation(1, vec![1], "first", 0.7);
        let obs2 = make_observation(2, vec![2], "second", 0.9);

        ctx.add_observation(obs1);
        ctx.add_observation(obs2);

        let avg = ctx.average_confidence();
        assert!((avg - 0.8).abs() < 0.001, "Expected ~0.8, got {}", avg);
    }

    #[test]
    fn test_elapsed_time() {
        let ctx = ChainContext::from_query("test");
        std::thread::sleep(std::time::Duration::from_millis(10));

        let elapsed = ctx.elapsed();
        assert!(elapsed.as_millis() >= 10);
    }
}
