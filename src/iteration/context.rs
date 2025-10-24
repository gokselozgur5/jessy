//! Context management for iteration processing

use super::IterationStep;
use crate::Frequency;

/// Context accumulated across iterations
#[derive(Debug, Clone)]
pub struct IterationContext {
    pub query: String,
    pub dimensional_contexts: Vec<String>,
    pub accumulated_insights: Vec<String>,
    pub dominant_frequency: Frequency,
}

impl IterationContext {
    /// Create new iteration context
    pub fn new(query: String, dominant_frequency: Frequency) -> Self {
        Self {
            query,
            dimensional_contexts: Vec::new(),
            accumulated_insights: Vec::new(),
            dominant_frequency,
        }
    }
    
    /// Add dimensional context
    pub fn add_context(&mut self, context: String) {
        self.dimensional_contexts.push(context);
    }
    
    /// Add insight from iteration
    pub fn add_insight(&mut self, insight: String) {
        self.accumulated_insights.push(insight);
    }
    
    /// Get formatted context for LLM
    pub fn format_for_llm(&self) -> String {
        let mut formatted = String::new();
        
        formatted.push_str(&format!("Query: {}\n\n", self.query));
        formatted.push_str(&format!("Frequency: {:.2} Hz\n\n", self.dominant_frequency.hz()));
        
        if !self.dimensional_contexts.is_empty() {
            formatted.push_str("Dimensional Contexts:\n");
            for (i, context) in self.dimensional_contexts.iter().enumerate() {
                formatted.push_str(&format!("{}. {}\n", i + 1, context));
            }
            formatted.push('\n');
        }
        
        if !self.accumulated_insights.is_empty() {
            formatted.push_str("Accumulated Insights:\n");
            for (i, insight) in self.accumulated_insights.iter().enumerate() {
                formatted.push_str(&format!("{}. {}\n", i + 1, insight));
            }
        }
        
        formatted
    }
}

/// Chain of thoughts across iterations
#[derive(Debug, Clone)]
pub struct ThoughtChain {
    pub steps: Vec<IterationStep>,
}

impl ThoughtChain {
    /// Create new thought chain
    pub fn new() -> Self {
        Self {
            steps: Vec::new(),
        }
    }
    
    /// Add iteration step to chain
    pub fn add_step(&mut self, step: IterationStep) {
        self.steps.push(step);
    }
    
    /// Get the length of the chain
    pub fn len(&self) -> usize {
        self.steps.len()
    }
    
    /// Check if chain is empty
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }
    
    /// Get all thoughts as a formatted string
    pub fn format_thoughts(&self) -> String {
        let mut formatted = String::new();
        
        for step in &self.steps {
            formatted.push_str(&format!(
                "Iteration {}: {}\n",
                step.iteration_number,
                step.thought
            ));
        }
        
        formatted
    }
    
    /// Get thoughts from a specific phase
    pub fn thoughts_in_phase(&self, phase: super::IterationPhase) -> Vec<&IterationStep> {
        self.steps.iter()
            .filter(|step| step.phase == phase)
            .collect()
    }
    
    /// Calculate average confidence
    pub fn average_confidence(&self) -> f32 {
        if self.steps.is_empty() {
            return 0.0;
        }
        
        let sum: f32 = self.steps.iter().map(|s| s.confidence).sum();
        sum / self.steps.len() as f32
    }
}

impl Default for ThoughtChain {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::iteration::IterationPhase;
    
    #[test]
    fn test_iteration_context() {
        let mut context = IterationContext::new(
            "test query".to_string(),
            Frequency::new(1.5),
        );
        
        context.add_context("Context 1".to_string());
        context.add_insight("Insight 1".to_string());
        
        let formatted = context.format_for_llm();
        assert!(formatted.contains("test query"));
        assert!(formatted.contains("Context 1"));
        assert!(formatted.contains("Insight 1"));
    }
    
    #[test]
    fn test_thought_chain() {
        let mut chain = ThoughtChain::new();
        
        chain.add_step(IterationStep::new(
            1,
            "First thought".to_string(),
            Frequency::new(1.0),
            0.5,
        ));
        
        chain.add_step(IterationStep::new(
            4,
            "Refinement thought".to_string(),
            Frequency::new(1.0),
            0.7,
        ));
        
        assert_eq!(chain.len(), 2);
        
        let exploration_thoughts = chain.thoughts_in_phase(IterationPhase::Exploration);
        assert_eq!(exploration_thoughts.len(), 1);
        
        let refinement_thoughts = chain.thoughts_in_phase(IterationPhase::Refinement);
        assert_eq!(refinement_thoughts.len(), 1);
    }
}
