//! Conversation Persistence Module
//!
//! Provides conversation history storage and retrieval for continuous
//! multi-session interactions.

use crate::{DimensionId, Result};
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::{Path, PathBuf};

pub mod persistence;

pub use persistence::ConversationStore;

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
