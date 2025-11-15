//! Conversation Metadata Extraction
//!
//! Analyzes conversations to extract:
//! - Emotional tone (positive, negative, neutral, playful, serious, curious)
//! - Key moments (jokes, insights, emotional moments)
//! - Unfinished threads (topics to revisit)
//! - Significance scores for memory prioritization

use serde::{Deserialize, Serialize};
use regex::Regex;
use std::collections::HashMap;

/// Emotional tone of a message or conversation
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq, Hash)]
#[serde(rename_all = "snake_case")]
pub enum EmotionalTone {
    Positive,
    Negative,
    Neutral,
    Playful,
    Serious,
    Curious,
    Excited,
    Concerned,
}

/// Type of key moment in conversation
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum KeyMomentType {
    Joke,
    Insight,
    Breakthrough,
    EmotionalMoment,
    SharedReference,
    Vulnerability,
}

/// A significant moment in the conversation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyMoment {
    pub moment_type: KeyMomentType,
    pub content: String,
    pub significance: f32,  // 0.0-1.0
    pub timestamp: chrono::DateTime<chrono::Utc>,
}

/// An unfinished conversation thread
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnfinishedThread {
    pub topic: String,
    pub context: String,
    pub priority: f32,  // 0.0-1.0
    pub mentioned_at: chrono::DateTime<chrono::Utc>,
}

/// Complete conversation metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationMetadata {
    pub overall_tone: EmotionalTone,
    pub key_moments: Vec<KeyMoment>,
    pub unfinished_threads: Vec<UnfinishedThread>,
    pub topics_discussed: Vec<String>,
    pub user_interests: HashMap<String, f32>,  // topic -> interest score
}

impl ConversationMetadata {
    pub fn new() -> Self {
        Self {
            overall_tone: EmotionalTone::Neutral,
            key_moments: Vec::new(),
            unfinished_threads: Vec::new(),
            topics_discussed: Vec::new(),
            user_interests: HashMap::new(),
        }
    }
}

impl Default for ConversationMetadata {
    fn default() -> Self {
        Self::new()
    }
}

/// Metadata extractor for analyzing conversations
pub struct MetadataExtractor {
    // Regex patterns for detection
    laughter_pattern: Regex,
    question_pattern: Regex,
    excitement_pattern: Regex,
    uncertainty_pattern: Regex,
    unfinished_pattern: Regex,
    insight_pattern: Regex,
}

impl MetadataExtractor {
    pub fn new() -> Self {
        Self {
            laughter_pattern: Regex::new(r"(?i)(haha|lol|😂|🤣|😄|😆|hehe|lmao)").unwrap(),
            question_pattern: Regex::new(r"\?").unwrap(),
            excitement_pattern: Regex::new(r"(?i)(wow|amazing|awesome|incredible|!{2,}|🎉|🚀|💪|🔥)").unwrap(),
            uncertainty_pattern: Regex::new(r"(?i)(maybe|perhaps|not sure|uncertain|hmm|🤔)").unwrap(),
            unfinished_pattern: Regex::new(r"(?i)(we should|let's discuss|remind me|come back to|later|next time)").unwrap(),
            insight_pattern: Regex::new(r"(?i)(aha|i see|makes sense|understand now|breakthrough|realized|💡)").unwrap(),
        }
    }

    /// Detect emotional tone of a message
    pub fn detect_tone(&self, message: &str) -> EmotionalTone {
        let lower = message.to_lowercase();
        
        // Check for playful indicators
        if self.laughter_pattern.is_match(message) {
            return EmotionalTone::Playful;
        }
        
        // Check for excitement
        if self.excitement_pattern.is_match(message) {
            return EmotionalTone::Excited;
        }
        
        // Check for curiosity (questions)
        if self.question_pattern.is_match(message) && lower.contains("how") || lower.contains("why") || lower.contains("what") {
            return EmotionalTone::Curious;
        }
        
        // Check for concern/uncertainty
        if self.uncertainty_pattern.is_match(message) || lower.contains("worried") || lower.contains("concerned") {
            return EmotionalTone::Concerned;
        }
        
        // Check for positive sentiment
        if lower.contains("good") || lower.contains("great") || lower.contains("love") 
            || lower.contains("perfect") || lower.contains("excellent") {
            return EmotionalTone::Positive;
        }
        
        // Check for negative sentiment
        if lower.contains("bad") || lower.contains("wrong") || lower.contains("hate") 
            || lower.contains("terrible") || lower.contains("awful") {
            return EmotionalTone::Negative;
        }
        
        // Check for serious tone
        if lower.contains("important") || lower.contains("critical") || lower.contains("serious") {
            return EmotionalTone::Serious;
        }
        
        EmotionalTone::Neutral
    }

    /// Detect key moments in a message
    pub fn detect_key_moments(&self, message: &str, role: &str) -> Vec<KeyMoment> {
        let mut moments = Vec::new();
        let now = chrono::Utc::now();
        
        // Detect jokes (laughter indicators)
        if self.laughter_pattern.is_match(message) {
            moments.push(KeyMoment {
                moment_type: KeyMomentType::Joke,
                content: message.to_string(),
                significance: 0.6,
                timestamp: now,
            });
        }
        
        // Detect insights (aha moments)
        if self.insight_pattern.is_match(message) {
            moments.push(KeyMoment {
                moment_type: KeyMomentType::Insight,
                content: message.to_string(),
                significance: 0.8,
                timestamp: now,
            });
        }
        
        // Detect breakthroughs (strong positive indicators)
        if message.to_lowercase().contains("breakthrough") 
            || message.to_lowercase().contains("finally understand") {
            moments.push(KeyMoment {
                moment_type: KeyMomentType::Breakthrough,
                content: message.to_string(),
                significance: 0.9,
                timestamp: now,
            });
        }
        
        // Detect vulnerability (personal sharing)
        if role == "user" && (
            message.to_lowercase().contains("i feel") 
            || message.to_lowercase().contains("i'm worried")
            || message.to_lowercase().contains("i'm scared")
        ) {
            moments.push(KeyMoment {
                moment_type: KeyMomentType::Vulnerability,
                content: message.to_string(),
                significance: 0.7,
                timestamp: now,
            });
        }
        
        moments
    }

    /// Detect unfinished threads in a message
    pub fn detect_unfinished_threads(&self, message: &str) -> Vec<UnfinishedThread> {
        let mut threads = Vec::new();
        let now = chrono::Utc::now();
        
        if self.unfinished_pattern.is_match(message) {
            // Extract topic (simple heuristic: words after the trigger phrase)
            let lower = message.to_lowercase();
            let topic = if let Some(pos) = lower.find("we should") {
                message[pos..].split_whitespace().take(5).collect::<Vec<_>>().join(" ")
            } else if let Some(pos) = lower.find("remind me") {
                message[pos..].split_whitespace().take(5).collect::<Vec<_>>().join(" ")
            } else if let Some(pos) = lower.find("come back to") {
                message[pos..].split_whitespace().take(5).collect::<Vec<_>>().join(" ")
            } else {
                message.to_string()
            };
            
            // Calculate priority based on urgency indicators
            let priority = if lower.contains("important") || lower.contains("urgent") {
                0.9
            } else if lower.contains("later") || lower.contains("sometime") {
                0.4
            } else {
                0.6
            };
            
            threads.push(UnfinishedThread {
                topic: topic.clone(),
                context: message.to_string(),
                priority,
                mentioned_at: now,
            });
        }
        
        threads
    }

    /// Extract topics from a message (simple keyword extraction)
    pub fn extract_topics(&self, message: &str) -> Vec<String> {
        let lower = message.to_lowercase();
        let mut topics = Vec::new();
        
        // Technical topics
        if lower.contains("code") || lower.contains("programming") {
            topics.push("programming".to_string());
        }
        if lower.contains("rust") {
            topics.push("rust".to_string());
        }
        if lower.contains("ai") || lower.contains("llm") {
            topics.push("ai".to_string());
        }
        if lower.contains("websocket") {
            topics.push("websocket".to_string());
        }
        
        // Philosophical topics
        if lower.contains("consciousness") {
            topics.push("consciousness".to_string());
        }
        if lower.contains("authentic") || lower.contains("genuine") {
            topics.push("authenticity".to_string());
        }
        
        // Emotional topics
        if lower.contains("feel") || lower.contains("emotion") {
            topics.push("emotions".to_string());
        }
        
        topics
    }

    /// Analyze a full conversation and extract metadata
    pub fn analyze_conversation(
        &self,
        messages: &[(String, String)],  // (role, content)
    ) -> ConversationMetadata {
        let mut metadata = ConversationMetadata::new();
        let mut tone_counts: HashMap<EmotionalTone, usize> = HashMap::new();
        let mut topic_mentions: HashMap<String, usize> = HashMap::new();
        
        for (role, content) in messages {
            // Detect tone
            let tone = self.detect_tone(content);
            *tone_counts.entry(tone.clone()).or_insert(0) += 1;
            
            // Detect key moments
            let moments = self.detect_key_moments(content, role);
            metadata.key_moments.extend(moments);
            
            // Detect unfinished threads
            let threads = self.detect_unfinished_threads(content);
            metadata.unfinished_threads.extend(threads);
            
            // Extract topics
            let topics = self.extract_topics(content);
            for topic in topics {
                *topic_mentions.entry(topic.clone()).or_insert(0) += 1;
                if !metadata.topics_discussed.contains(&topic) {
                    metadata.topics_discussed.push(topic);
                }
            }
        }
        
        // Determine overall tone (most common)
        metadata.overall_tone = tone_counts
            .into_iter()
            .max_by_key(|(_, count)| *count)
            .map(|(tone, _)| tone)
            .unwrap_or(EmotionalTone::Neutral);
        
        // Calculate user interests (normalized mention counts)
        let max_mentions = topic_mentions.values().max().copied().unwrap_or(1) as f32;
        for (topic, count) in topic_mentions {
            metadata.user_interests.insert(topic, count as f32 / max_mentions);
        }
        
        metadata
    }
}

impl Default for MetadataExtractor {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_detect_playful_tone() {
        let extractor = MetadataExtractor::new();
        assert_eq!(extractor.detect_tone("haha that's funny!"), EmotionalTone::Playful);
        assert_eq!(extractor.detect_tone("lol 😂"), EmotionalTone::Playful);
    }

    #[test]
    fn test_detect_excited_tone() {
        let extractor = MetadataExtractor::new();
        assert_eq!(extractor.detect_tone("Wow! That's amazing!! 🚀"), EmotionalTone::Excited);
    }

    #[test]
    fn test_detect_curious_tone() {
        let extractor = MetadataExtractor::new();
        assert_eq!(extractor.detect_tone("How does this work?"), EmotionalTone::Curious);
        assert_eq!(extractor.detect_tone("Why is that?"), EmotionalTone::Curious);
    }

    #[test]
    fn test_detect_key_moments() {
        let extractor = MetadataExtractor::new();
        let moments = extractor.detect_key_moments("Aha! I finally understand 💡", "user");
        // This message matches both "insight" (aha) and "breakthrough" (finally understand)
        assert_eq!(moments.len(), 2);
        assert_eq!(moments[0].moment_type, KeyMomentType::Insight);
        assert_eq!(moments[1].moment_type, KeyMomentType::Breakthrough);
    }

    #[test]
    fn test_detect_unfinished_threads() {
        let extractor = MetadataExtractor::new();
        let threads = extractor.detect_unfinished_threads("We should discuss this later");
        assert_eq!(threads.len(), 1);
        assert!(threads[0].topic.contains("discuss"));
    }

    #[test]
    fn test_extract_topics() {
        let extractor = MetadataExtractor::new();
        let topics = extractor.extract_topics("I'm learning Rust programming and AI");
        assert!(topics.contains(&"rust".to_string()));
        assert!(topics.contains(&"programming".to_string()));
        assert!(topics.contains(&"ai".to_string()));
    }
}
