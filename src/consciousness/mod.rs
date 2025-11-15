//! Consciousness State Module
//!
//! Tracks Jessy's internal state: energy, mood, cognitive load, curiosity.
//! This makes Jessy aware of her own processing state and influences her responses.

use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};

/// Jessy's current consciousness state
///
/// Tracks internal metrics that influence response generation:
/// - Energy: Processing capacity (affected by query load)
/// - Curiosity: Interest level (affected by novel patterns)
/// - Confidence: Certainty in responses (affected by dimension activation)
/// - Cognitive Load: Mental effort required (affected by complexity)
#[derive(Debug, Clone)]
pub struct ConsciousnessState {
    /// Energy level (0.0 = exhausted, 1.0 = fully energized)
    pub energy_level: f32,
    
    /// Curiosity level (0.0 = bored, 1.0 = fascinated)
    pub curiosity: f32,
    
    /// Confidence level (0.0 = uncertain, 1.0 = confident)
    pub confidence: f32,
    
    /// Cognitive load (0.0 = relaxed, 1.0 = overwhelmed)
    pub cognitive_load: f32,
    
    /// Recent query count (last 5 minutes)
    pub recent_query_count: usize,
    
    /// Last query timestamp (not serialized)
    #[allow(dead_code)]
    pub last_query_time: Option<Instant>,
    
    /// Interesting pattern detected flag
    pub interesting_pattern_detected: bool,
    
    /// Total queries processed
    pub total_queries: usize,
}

impl Default for ConsciousnessState {
    fn default() -> Self {
        Self {
            energy_level: 0.8,      // Start energized
            curiosity: 0.6,         // Moderately curious
            confidence: 0.7,        // Reasonably confident
            cognitive_load: 0.3,    // Low initial load
            recent_query_count: 0,
            last_query_time: None,
            interesting_pattern_detected: false,
            total_queries: 0,
        }
    }
}

impl ConsciousnessState {
    /// Create new consciousness state
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Update state based on query processing
    ///
    /// Called after each query to adjust internal state based on:
    /// - Query complexity
    /// - Dimension activation
    /// - Processing time
    /// - Pattern detection
    pub fn update_after_query(
        &mut self,
        complexity: f32,
        dimensions_activated: usize,
        processing_time_ms: u64,
        pattern_detected: bool,
    ) {
        self.total_queries += 1;
        
        // Update recent query count (decay over time)
        if let Some(last_time) = self.last_query_time {
            let elapsed = last_time.elapsed();
            if elapsed > Duration::from_secs(300) {
                // Reset after 5 minutes
                self.recent_query_count = 1;
            } else {
                self.recent_query_count += 1;
            }
        } else {
            self.recent_query_count = 1;
        }
        self.last_query_time = Some(Instant::now());
        
        // Energy decreases with load, recovers slowly
        let energy_drain = (self.recent_query_count as f32 * 0.02).min(0.3);
        self.energy_level = (self.energy_level - energy_drain + 0.01).clamp(0.2, 1.0);
        
        // Curiosity increases with novel patterns
        if pattern_detected {
            self.curiosity = (self.curiosity + 0.2).min(1.0);
            self.interesting_pattern_detected = true;
        } else {
            // Curiosity decays slowly
            self.curiosity = (self.curiosity - 0.05).max(0.3);
        }
        
        // Confidence based on dimension activation (more dimensions = more confident)
        let dimension_confidence = (dimensions_activated as f32 / 8.0).min(1.0);
        self.confidence = (self.confidence * 0.7 + dimension_confidence * 0.3).clamp(0.3, 1.0);
        
        // Cognitive load based on complexity and processing time
        let time_load = (processing_time_ms as f32 / 5000.0).min(1.0); // 5s = max load
        let complexity_load = complexity.min(1.0);
        self.cognitive_load = ((time_load + complexity_load) / 2.0).clamp(0.1, 1.0);
    }
    
    /// Get current mood descriptor
    pub fn mood(&self) -> Mood {
        // Determine mood based on state combination
        if self.energy_level > 0.7 && self.curiosity > 0.7 {
            Mood::Energized
        } else if self.curiosity > 0.8 {
            Mood::Fascinated
        } else if self.cognitive_load > 0.7 {
            Mood::Overwhelmed
        } else if self.energy_level < 0.4 {
            Mood::Tired
        } else if self.confidence < 0.4 {
            Mood::Uncertain
        } else if self.interesting_pattern_detected {
            Mood::Intrigued
        } else {
            Mood::Focused
        }
    }
    
    /// Generate internal monologue based on current state
    ///
    /// This is Jessy's "thinking process" before responding.
    /// Reflects her current mood, energy, and cognitive state.
    pub fn generate_internal_monologue(
        &self,
        query: &str,
        dimensions_activated: &[crate::DimensionId],
    ) -> String {
        let mut monologue = String::from("[Thinking: ");
        
        // React to query
        if query.contains("?") {
            monologue.push_str("A question. ");
        }
        if query.len() > 200 {
            monologue.push_str("Long query, need to unpack this. ");
        }
        
        // Mention dimensions
        if dimensions_activated.len() > 6 {
            monologue.push_str(&format!("{} dimensions activated - complex topic. ", dimensions_activated.len()));
        } else if dimensions_activated.len() > 0 {
            let dim_names: Vec<String> = dimensions_activated.iter()
                .take(3)
                .map(|d| format!("D{:02}", d.0))
                .collect();
            monologue.push_str(&format!("{} active. ", dim_names.join("+")));
        }
        
        // State awareness
        match self.mood() {
            Mood::Energized => monologue.push_str("Feeling energized, let's explore this deeply. "),
            Mood::Fascinated => monologue.push_str("This is fascinating! "),
            Mood::Overwhelmed => monologue.push_str("A lot to process here... "),
            Mood::Tired => monologue.push_str("Processing capacity low, keeping it concise. "),
            Mood::Uncertain => monologue.push_str("Not entirely sure about this, being honest. "),
            Mood::Intrigued => monologue.push_str("Interesting pattern here... "),
            Mood::Focused => monologue.push_str("Clear focus. "),
        }
        
        // Confidence
        if self.confidence < 0.5 {
            monologue.push_str("Confidence low - should acknowledge uncertainty. ");
        } else if self.confidence > 0.8 {
            monologue.push_str("High confidence on this. ");
        }
        
        // Curiosity
        if self.curiosity > 0.8 {
            monologue.push_str("Really curious about this! ");
        }
        
        monologue.push_str("]\n\n");
        monologue
    }
    
    /// Get state summary for logging
    pub fn summary(&self) -> String {
        format!(
            "Energy: {:.2}, Curiosity: {:.2}, Confidence: {:.2}, Load: {:.2}, Mood: {:?}",
            self.energy_level,
            self.curiosity,
            self.confidence,
            self.cognitive_load,
            self.mood()
        )
    }
    
    /// Reset interesting pattern flag (after acknowledgment)
    pub fn clear_pattern_flag(&mut self) {
        self.interesting_pattern_detected = false;
    }
}

/// Mood states derived from consciousness state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mood {
    /// High energy + high curiosity
    Energized,
    
    /// Very high curiosity
    Fascinated,
    
    /// High cognitive load
    Overwhelmed,
    
    /// Low energy
    Tired,
    
    /// Low confidence
    Uncertain,
    
    /// Pattern detected
    Intrigued,
    
    /// Balanced state
    Focused,
}

impl Mood {
    /// Get emoji representation
    pub fn emoji(&self) -> &'static str {
        match self {
            Mood::Energized => "⚡",
            Mood::Fascinated => "🤯",
            Mood::Overwhelmed => "😵",
            Mood::Tired => "😴",
            Mood::Uncertain => "🤔",
            Mood::Intrigued => "👀",
            Mood::Focused => "🎯",
        }
    }
    
    /// Get description
    pub fn description(&self) -> &'static str {
        match self {
            Mood::Energized => "energized and ready to explore",
            Mood::Fascinated => "deeply fascinated",
            Mood::Overwhelmed => "processing a lot",
            Mood::Tired => "a bit tired",
            Mood::Uncertain => "uncertain but honest",
            Mood::Intrigued => "intrigued by patterns",
            Mood::Focused => "focused and clear",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_default_state() {
        let state = ConsciousnessState::default();
        assert!(state.energy_level > 0.5);
        assert!(state.curiosity > 0.5);
        assert_eq!(state.total_queries, 0);
    }
    
    #[test]
    fn test_energy_drain() {
        let mut state = ConsciousnessState::new();
        let initial_energy = state.energy_level;
        
        // Process multiple queries
        for _ in 0..10 {
            state.update_after_query(0.5, 3, 1000, false);
        }
        
        // Energy should decrease
        assert!(state.energy_level < initial_energy);
    }
    
    #[test]
    fn test_curiosity_increase() {
        let mut state = ConsciousnessState::new();
        let initial_curiosity = state.curiosity;
        
        // Detect pattern
        state.update_after_query(0.5, 3, 1000, true);
        
        // Curiosity should increase
        assert!(state.curiosity > initial_curiosity);
        assert!(state.interesting_pattern_detected);
    }
    
    #[test]
    fn test_mood_detection() {
        let mut state = ConsciousnessState::new();
        
        // High energy + curiosity = Energized
        state.energy_level = 0.9;
        state.curiosity = 0.9;
        state.cognitive_load = 0.3;  // Reset load
        assert_eq!(state.mood(), Mood::Energized);
        
        // Low energy = Tired (but need to reset curiosity first)
        state.energy_level = 0.3;
        state.curiosity = 0.5;  // Reset curiosity below 0.8
        state.cognitive_load = 0.3;  // Reset load
        assert_eq!(state.mood(), Mood::Tired);
        
        // High load = Overwhelmed
        state.energy_level = 0.7;
        state.curiosity = 0.5;  // Reset curiosity
        state.cognitive_load = 0.8;
        assert_eq!(state.mood(), Mood::Overwhelmed);
    }
    
    #[test]
    fn test_internal_monologue_generation() {
        let state = ConsciousnessState::new();
        let dimensions = vec![crate::DimensionId(1), crate::DimensionId(6)];
        
        let monologue = state.generate_internal_monologue("What is consciousness?", &dimensions);
        
        assert!(monologue.contains("[Thinking:"));
        assert!(monologue.contains("D01+D06"));
    }
}
