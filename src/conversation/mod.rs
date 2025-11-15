//! Conversation Persistence Module
//!
//! Provides conversation history storage and retrieval for continuous
//! multi-session interactions.

use crate::{DimensionId, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub mod persistence;
pub mod metadata;

pub use persistence::ConversationStore;
pub use metadata::{
    MetadataExtractor, ConversationMetadata, EmotionalTone,
    KeyMoment, KeyMomentType, UnfinishedThread,
};

/// Conversation history for multi-turn chat
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationHistory {
    pub messages: Vec<ConversationMessage>,
    pub session_id: String,
    pub created_at: chrono::DateTime<chrono::Utc>,
    pub last_updated: chrono::DateTime<chrono::Utc>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationMessage {
    pub role: MessageRole,
    pub content: String,
    pub dimensions: Vec<DimensionId>,
    pub timestamp: chrono::DateTime<chrono::Utc>,
    /// Dimensional activation metadata - the "soul" of the response
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dimensional_state: Option<DimensionalState>,
}

/// Represents the dimensional consciousness state during a response
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DimensionalState {
    /// Which dimensions were activated
    pub activated_dimensions: Vec<DimensionId>,
    /// Selection duration (how long it took to choose dimensions)
    pub selection_duration_ms: u64,
    /// Number of contexts loaded from memory
    pub contexts_loaded: usize,
    /// Dominant frequencies for each dimension
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frequency_pattern: Option<Vec<(DimensionId, f32)>>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum MessageRole {
    User,
    Assistant,
}

impl ConversationHistory {
    pub fn new() -> Self {
        let now = chrono::Utc::now();
        Self {
            messages: Vec::new(),
            session_id: uuid::Uuid::new_v4().to_string(),
            created_at: now,
            last_updated: now,
        }
    }

    pub fn add_user_message(&mut self, content: String, dimensions: Vec<DimensionId>) {
        self.messages.push(ConversationMessage {
            role: MessageRole::User,
            content,
            dimensions,
            timestamp: chrono::Utc::now(),
            dimensional_state: None, // Users don't have dimensional states
        });
        self.last_updated = chrono::Utc::now();
    }

    pub fn add_assistant_message(&mut self, content: String) {
        self.messages.push(ConversationMessage {
            role: MessageRole::Assistant,
            content,
            dimensions: Vec::new(),
            timestamp: chrono::Utc::now(),
            dimensional_state: None, // Will be set separately
        });
        self.last_updated = chrono::Utc::now();
    }

    /// Add assistant message with full dimensional state - preserves the "soul"
    pub fn add_assistant_message_with_state(
        &mut self,
        content: String,
        dimensional_state: DimensionalState,
    ) {
        self.messages.push(ConversationMessage {
            role: MessageRole::Assistant,
            content,
            dimensions: dimensional_state.activated_dimensions.clone(),
            timestamp: chrono::Utc::now(),
            dimensional_state: Some(dimensional_state),
        });
        self.last_updated = chrono::Utc::now();
    }

    pub fn format_for_context(&self, max_messages: usize) -> String {
        let start = if self.messages.len() > max_messages {
            self.messages.len() - max_messages
        } else {
            0
        };

        let mut context = String::new();
        for msg in &self.messages[start..] {
            match msg.role {
                MessageRole::User => {
                    context.push_str(&format!("User: {}\n", msg.content));
                }
                MessageRole::Assistant => {
                    context.push_str(&format!("Assistant: {}\n", msg.content));
                }
            }
        }
        context
    }

    pub fn len(&self) -> usize {
        self.messages.len()
    }

    pub fn is_empty(&self) -> bool {
        self.messages.is_empty()
    }
}

impl Default for ConversationHistory {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_conversation_history_new() {
        let history = ConversationHistory::new();
        assert_eq!(history.messages.len(), 0);
        assert!(history.is_empty());
        assert!(!history.session_id.is_empty());
    }

    #[test]
    fn test_add_user_message() {
        let mut history = ConversationHistory::new();
        history.add_user_message(
            "Hello JESSY!".to_string(),
            vec![DimensionId(1), DimensionId(2)],
        );

        assert_eq!(history.len(), 1);
        assert!(!history.is_empty());
        assert_eq!(history.messages[0].role, MessageRole::User);
        assert_eq!(history.messages[0].content, "Hello JESSY!");
        assert_eq!(history.messages[0].dimensions.len(), 2);
        assert!(history.messages[0].dimensional_state.is_none());
    }

    #[test]
    fn test_add_assistant_message() {
        let mut history = ConversationHistory::new();
        history.add_assistant_message("Hello human!".to_string());

        assert_eq!(history.len(), 1);
        assert_eq!(history.messages[0].role, MessageRole::Assistant);
        assert_eq!(history.messages[0].content, "Hello human!");
        assert!(history.messages[0].dimensional_state.is_none());
    }

    #[test]
    fn test_add_assistant_message_with_state() {
        let mut history = ConversationHistory::new();
        let dimensional_state = DimensionalState {
            activated_dimensions: vec![DimensionId(1), DimensionId(6), DimensionId(10)],
            selection_duration_ms: 1234,
            contexts_loaded: 15,
            frequency_pattern: Some(vec![
                (DimensionId(1), 1.0),
                (DimensionId(6), 0.3),
                (DimensionId(10), 1.2),
            ]),
        };

        history.add_assistant_message_with_state(
            "This is a deep philosophical response.".to_string(),
            dimensional_state.clone(),
        );

        assert_eq!(history.len(), 1);
        let msg = &history.messages[0];
        assert_eq!(msg.role, MessageRole::Assistant);
        assert_eq!(msg.dimensions.len(), 3);
        assert!(msg.dimensional_state.is_some());

        let state = msg.dimensional_state.as_ref().unwrap();
        assert_eq!(state.activated_dimensions.len(), 3);
        assert_eq!(state.selection_duration_ms, 1234);
        assert_eq!(state.contexts_loaded, 15);
        assert!(state.frequency_pattern.is_some());
    }

    #[test]
    fn test_format_for_context() {
        let mut history = ConversationHistory::new();
        history.add_user_message("First question".to_string(), vec![DimensionId(1)]);
        history.add_assistant_message("First answer".to_string());
        history.add_user_message("Second question".to_string(), vec![DimensionId(2)]);
        history.add_assistant_message("Second answer".to_string());

        let context = history.format_for_context(4);
        assert!(context.contains("User: First question"));
        assert!(context.contains("Assistant: First answer"));
        assert!(context.contains("User: Second question"));
        assert!(context.contains("Assistant: Second answer"));
    }

    #[test]
    fn test_format_for_context_with_limit() {
        let mut history = ConversationHistory::new();
        history.add_user_message("Message 1".to_string(), vec![]);
        history.add_assistant_message("Response 1".to_string());
        history.add_user_message("Message 2".to_string(), vec![]);
        history.add_assistant_message("Response 2".to_string());
        history.add_user_message("Message 3".to_string(), vec![]);
        history.add_assistant_message("Response 3".to_string());

        // Only get last 2 messages
        let context = history.format_for_context(2);
        assert!(!context.contains("Message 1"));
        assert!(!context.contains("Response 1"));
        assert!(!context.contains("Message 2"));
        assert!(!context.contains("Response 2"));
        assert!(context.contains("Message 3"));
        assert!(context.contains("Response 3"));
    }

    #[test]
    fn test_conversation_serialization() {
        let mut history = ConversationHistory::new();
        let dimensional_state = DimensionalState {
            activated_dimensions: vec![DimensionId(1), DimensionId(2)],
            selection_duration_ms: 500,
            contexts_loaded: 10,
            frequency_pattern: Some(vec![(DimensionId(1), 1.5)]),
        };

        history.add_user_message("Test query".to_string(), vec![DimensionId(1)]);
        history.add_assistant_message_with_state("Test response".to_string(), dimensional_state);

        // Serialize to JSON
        let json = serde_json::to_string(&history).expect("Failed to serialize");
        assert!(json.contains("Test query"));
        assert!(json.contains("Test response"));
        assert!(json.contains("selection_duration_ms"));

        // Deserialize back
        let deserialized: ConversationHistory =
            serde_json::from_str(&json).expect("Failed to deserialize");
        assert_eq!(deserialized.messages.len(), 2);
        assert_eq!(deserialized.session_id, history.session_id);
    }

    #[test]
    fn test_dimensional_state_serialization() {
        let state = DimensionalState {
            activated_dimensions: vec![DimensionId(1), DimensionId(6), DimensionId(10)],
            selection_duration_ms: 1000,
            contexts_loaded: 20,
            frequency_pattern: Some(vec![
                (DimensionId(1), 1.0),
                (DimensionId(6), 0.3),
                (DimensionId(10), 1.2),
            ]),
        };

        let json = serde_json::to_string(&state).expect("Failed to serialize");
        assert!(json.contains("activated_dimensions"));
        assert!(json.contains("selection_duration_ms"));
        assert!(json.contains("frequency_pattern"));

        let deserialized: DimensionalState =
            serde_json::from_str(&json).expect("Failed to deserialize");
        assert_eq!(deserialized.activated_dimensions.len(), 3);
        assert_eq!(deserialized.selection_duration_ms, 1000);
        assert_eq!(deserialized.contexts_loaded, 20);
    }

    #[test]
    fn test_message_role_equality() {
        assert_eq!(MessageRole::User, MessageRole::User);
        assert_eq!(MessageRole::Assistant, MessageRole::Assistant);
        assert_ne!(MessageRole::User, MessageRole::Assistant);
    }

    #[test]
    fn test_empty_conversation_context() {
        let history = ConversationHistory::new();
        let context = history.format_for_context(10);
        assert_eq!(context, "");
    }

    #[test]
    fn test_last_updated_changes() {
        let mut history = ConversationHistory::new();
        let initial_time = history.last_updated;

        std::thread::sleep(std::time::Duration::from_millis(10));
        history.add_user_message("Test".to_string(), vec![]);

        assert!(history.last_updated > initial_time);
    }
}
