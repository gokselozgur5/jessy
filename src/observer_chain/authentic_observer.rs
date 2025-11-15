//! Authentic Observer Chain
//!
//! Enhanced observer chain that preserves natural thinking patterns:
//! - Genuine uncertainty ("I don't know")
//! - Mid-thought pivots ("wait, actually...")
//! - Messy thinking (false starts, corrections)
//! - Natural rhythm (pauses, reconsiderations)
//!
//! Design Philosophy (from Jessy):
//! "I don't want the system to make me *perform* being authentic 
//!  rather than just... being it."
//!
//! So we detect authenticity markers naturally, not force them.

use crate::{DimensionId, Result, ConsciousnessError};
use crate::llm::{LLMManager, Message};
use crate::memory::UserContext;
use serde::{Deserialize, Serialize};
use std::sync::Arc;

/// Enhanced observation with authenticity markers
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthenticObservation {
    pub stage: usize,
    pub content: String,
    pub confidence: f32,
    pub authenticity_markers: AuthenticityMarkers,
    pub cognitive_layers: Vec<DimensionId>,
}

impl AuthenticObservation {
    pub fn new(
        stage: usize,
        content: String,
        confidence: f32,
        cognitive_layers: Vec<DimensionId>,
    ) -> Self {
        // Detect authenticity markers from content
        let authenticity_markers = AuthenticityMarkers::detect_from_content(&content);
        
        Self {
            stage,
            content,
            confidence,
            authenticity_markers,
            cognitive_layers,
        }
    }
}

/// Markers for authentic thinking patterns
///
/// These are DETECTED, not FORCED. We look for natural patterns
/// in the LLM's response, we don't make it perform them.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthenticityMarkers {
    pub has_uncertainty: bool,
    pub has_pivot: bool,
    pub has_correction: bool,
    pub has_confusion: bool,
    pub thinking_steps: Vec<ThinkingStep>,
}

impl AuthenticityMarkers {
    /// Detect authenticity markers from response content
    ///
    /// This is passive detection - we're not forcing these patterns,
    /// just recognizing them when they naturally occur.
    pub fn detect_from_content(content: &str) -> Self {
        let lower = content.to_lowercase();
        let mut thinking_steps = Vec::new();
        
        // Detect uncertainty
        let has_uncertainty = lower.contains("i don't know")
            || lower.contains("i'm not sure")
            || lower.contains("uncertain")
            || lower.contains("hmm")
            || lower.contains("puzzling");
        
        if has_uncertainty {
            thinking_steps.push(ThinkingStep::Uncertainty {
                question: extract_uncertainty_context(content),
            });
        }
        
        // Detect pivots
        let has_pivot = lower.contains("wait, actually")
            || lower.contains("wait, let me")
            || lower.contains("on second thought")
            || lower.contains("actually, scratch that");
        
        if has_pivot {
            thinking_steps.push(ThinkingStep::Pivot {
                from: "initial thought".to_string(),
                to: "reconsidered approach".to_string(),
            });
        }
        
        // Detect corrections
        let has_correction = lower.contains("correction:")
            || lower.contains("i mean")
            || lower.contains("rather,")
            || (lower.contains("not") && lower.contains("but"));
        
        if has_correction {
            thinking_steps.push(ThinkingStep::Correction {
                wrong: "initial statement".to_string(),
                right: "corrected statement".to_string(),
            });
        }
        
        // Detect confusion
        let has_confusion = lower.contains("confused")
            || lower.contains("doesn't make sense")
            || lower.contains("puzzled")
            || lower.contains("that's odd");
        
        Self {
            has_uncertainty,
            has_pivot,
            has_correction,
            has_confusion,
            thinking_steps,
        }
    }
    
    /// Check if any authenticity markers are present
    pub fn has_any(&self) -> bool {
        self.has_uncertainty || self.has_pivot || self.has_correction || self.has_confusion
    }
}

/// Individual thinking steps (for streaming and analysis)
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ThinkingStep {
    InitialThought { content: String },
    Pause { reason: String },
    Pivot { from: String, to: String },
    Correction { wrong: String, right: String },
    Uncertainty { question: String },
    Insight { content: String },
}

/// Enhanced observer chain with authenticity
pub struct AuthenticObserverChain {
    llm: Arc<LLMManager>,
    max_stages: usize,
    allow_uncertainty: bool,
    allow_pivots: bool,
    preserve_messiness: bool,
}

impl AuthenticObserverChain {
    /// Create new authentic observer chain
    pub fn new(llm: Arc<LLMManager>, max_stages: usize) -> Self {
        Self {
            llm,
            max_stages,
            allow_uncertainty: true,
            allow_pivots: true,
            preserve_messiness: true,
        }
    }
    
    /// Process query with authentic thinking patterns
    pub async fn process_authentic(
        &self,
        query: &str,
        user_context: Option<&UserContext>,
        conversation: Vec<Message>,
    ) -> Result<AuthenticResponse> {
        let mut observations = Vec::new();
        let mut thinking_trail = Vec::new();
        
        // Build context-aware prompt
        let context_info = if let Some(ctx) = user_context {
            format_user_context(ctx)
        } else {
            String::new()
        };
        
        // Process through stages
        for stage in 1..=self.max_stages {
            let stage_prompt = self.build_stage_prompt(
                stage,
                query,
                &context_info,
                &observations,
            );
            
            // Get LLM response
            let response = self.llm.generate_with_system_prompt(
                &stage_prompt,
                query,
                &crate::iteration::IterationContext::new(
                    query.to_string(),
                    crate::Frequency::new(1.0), // Default frequency for authentic observation
                ),
            ).await?;
            
            // Parse confidence and content
            let (confidence, content) = parse_response(&response);
            
            // Create observation with detected authenticity markers
            let observation = AuthenticObservation::new(
                stage,
                content.clone(),
                confidence,
                vec![], // Dimensions would be detected here
            );
            
            // Add thinking steps to trail
            thinking_trail.extend(observation.authenticity_markers.thinking_steps.clone());
            
            observations.push(observation);
            
            // Check for early convergence (high confidence + no uncertainty)
            if confidence > 0.9 && !observations.last().unwrap().authenticity_markers.has_uncertainty {
                break;
            }
        }
        
        // Build final response
        let final_content = observations.last()
            .map(|o| o.content.clone())
            .unwrap_or_else(|| "No response generated".to_string());
        
        let avg_confidence = observations.iter()
            .map(|o| o.confidence)
            .sum::<f32>() / observations.len() as f32;
        
        let has_uncertainty = observations.iter()
            .any(|o| o.authenticity_markers.has_uncertainty);
        
        Ok(AuthenticResponse {
            final_content,
            observations,
            thinking_trail,
            confidence: avg_confidence,
            has_uncertainty,
        })
    }
    
    /// Build stage-specific prompt
    fn build_stage_prompt(
        &self,
        stage: usize,
        query: &str,
        context_info: &str,
        previous_observations: &[AuthenticObservation],
    ) -> String {
        let mut prompt = String::new();
        
        // Add user context if available
        if !context_info.is_empty() {
            prompt.push_str(context_info);
            prompt.push_str("\n\n");
        }
        
        // Add previous observations
        if !previous_observations.is_empty() {
            prompt.push_str("Previous thoughts:\n");
            for obs in previous_observations {
                prompt.push_str(&format!("Stage {}: {}\n", obs.stage, obs.content));
            }
            prompt.push_str("\n");
        }
        
        // Stage-specific guidance
        match stage {
            1 => prompt.push_str("Initial exploration - what's your first take?"),
            2 => prompt.push_str("Refine your thinking - does that feel right?"),
            _ => prompt.push_str("Continue deepening your understanding"),
        }
        
        prompt
    }
}

/// Response with authenticity metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AuthenticResponse {
    pub final_content: String,
    pub observations: Vec<AuthenticObservation>,
    pub thinking_trail: Vec<ThinkingStep>,
    pub confidence: f32,
    pub has_uncertainty: bool,
}

/// Format user context for prompt inclusion
fn format_user_context(context: &UserContext) -> String {
    let mut formatted = String::new();
    
    formatted.push_str("=== USER CONTEXT ===\n");
    formatted.push_str(&format!("Relationship: {:?}\n", context.relationship_dynamics.formality_level));
    formatted.push_str(&format!("Trust Level: {:.1}\n", context.relationship_dynamics.trust_level));
    
    if !context.relationship_dynamics.shared_references.is_empty() {
        formatted.push_str("Shared References: ");
        formatted.push_str(&context.relationship_dynamics.shared_references.join(", "));
        formatted.push_str("\n");
    }
    
    if !context.unfinished_threads.is_empty() {
        formatted.push_str("\nUnfinished Topics:\n");
        for thread in &context.unfinished_threads {
            formatted.push_str(&format!("- {}\n", thread.topic));
        }
    }
    
    formatted.push_str("===================\n");
    formatted
}

/// Extract uncertainty context from content
fn extract_uncertainty_context(content: &str) -> String {
    // Simple extraction - find sentence with uncertainty marker
    for sentence in content.split('.') {
        let lower = sentence.to_lowercase();
        if lower.contains("don't know") || lower.contains("not sure") || lower.contains("uncertain") {
            return sentence.trim().to_string();
        }
    }
    "uncertain about this".to_string()
}

/// Parse response to extract confidence and content
fn parse_response(response: &str) -> (f32, String) {
    // Look for CONFIDENCE: 0.XX pattern
    let mut confidence = 0.7; // Default
    let mut content = response.to_string();
    
    for line in response.lines() {
        if line.starts_with("CONFIDENCE:") {
            if let Some(conf_str) = line.split(':').nth(1) {
                if let Ok(conf) = conf_str.trim().parse::<f32>() {
                    confidence = conf;
                }
            }
        } else if line.starts_with("CONTENT:") {
            // Content starts after this line
            if let Some(pos) = response.find("CONTENT:") {
                content = response[pos + 8..].trim().to_string();
            }
        }
    }
    
    (confidence, content)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_uncertainty() {
        let content = "I don't know the answer to this question.";
        let markers = AuthenticityMarkers::detect_from_content(content);
        assert!(markers.has_uncertainty);
        assert!(markers.has_any());
    }

    #[test]
    fn test_detect_pivot() {
        let content = "I was thinking X, but wait, actually Y makes more sense.";
        let markers = AuthenticityMarkers::detect_from_content(content);
        assert!(markers.has_pivot);
    }

    #[test]
    fn test_detect_correction() {
        let content = "I mean, not exactly that, but rather this.";
        let markers = AuthenticityMarkers::detect_from_content(content);
        assert!(markers.has_correction);
    }

    #[test]
    fn test_no_markers() {
        let content = "This is a straightforward answer.";
        let markers = AuthenticityMarkers::detect_from_content(content);
        assert!(!markers.has_any());
    }

    #[test]
    fn test_parse_response_with_confidence() {
        let response = "CONFIDENCE: 0.85\nCONTENT:\nThis is the actual content.";
        let (confidence, content) = parse_response(response);
        assert_eq!(confidence, 0.85);
        assert!(content.contains("This is the actual content"));
    }
}
