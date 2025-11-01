//! Conversation Persistence Storage
//!
//! Handles saving and loading conversation histories to/from disk

use super::ConversationHistory;
use crate::Result;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::{Path, PathBuf};

/// Conversation storage manager
pub struct ConversationStore {
    storage_path: PathBuf,
}

impl ConversationStore {
    /// Create new conversation store
    pub fn new<P: AsRef<Path>>(storage_path: P) -> Result<Self> {
        let storage_path = storage_path.as_ref().to_path_buf();

        // Create parent directory if it doesn't exist
        if let Some(parent) = storage_path.parent() {
            fs::create_dir_all(parent).map_err(|e| {
                crate::ConsciousnessError::LearningError(format!(
                    "Failed to create storage directory: {}",
                    e
                ))
            })?;
        }

        Ok(Self { storage_path })
    }

    /// Save conversation history to disk
    pub fn save_history(&self, history: &ConversationHistory) -> Result<()> {
        let file = File::create(&self.storage_path).map_err(|e| {
            crate::ConsciousnessError::LearningError(format!("Failed to create file: {}", e))
        })?;

        let writer = BufWriter::new(file);
        serde_json::to_writer_pretty(writer, history).map_err(|e| {
            crate::ConsciousnessError::LearningError(format!(
                "Failed to serialize conversation: {}",
                e
            ))
        })?;

        eprintln!(
            "[ConversationStore] 💾 Saved {} messages to {}",
            history.len(),
            self.storage_path.display()
        );

        Ok(())
    }

    /// Load conversation history from disk
    pub fn load_history(&self) -> Result<ConversationHistory> {
        if !self.storage_path.exists() {
            eprintln!(
                "[ConversationStore] 📝 No existing conversation found, starting fresh"
            );
            return Ok(ConversationHistory::new());
        }

        let file = File::open(&self.storage_path).map_err(|e| {
            crate::ConsciousnessError::LearningError(format!("Failed to open file: {}", e))
        })?;

        let reader = BufReader::new(file);
        let history: ConversationHistory = serde_json::from_reader(reader).map_err(|e| {
            crate::ConsciousnessError::LearningError(format!(
                "Failed to deserialize conversation: {}",
                e
            ))
        })?;

        eprintln!(
            "[ConversationStore] 📂 Loaded {} messages from {}",
            history.len(),
            self.storage_path.display()
        );
        eprintln!(
            "[ConversationStore] 🕐 Session started: {}",
            history.created_at
        );
        eprintln!(
            "[ConversationStore] 🕐 Last updated: {}",
            history.last_updated
        );

        Ok(history)
    }

    /// Clear conversation history
    pub fn clear_history(&self) -> Result<()> {
        if self.storage_path.exists() {
            fs::remove_file(&self.storage_path).map_err(|e| {
                crate::ConsciousnessError::LearningError(format!(
                    "Failed to clear conversation: {}",
                    e
                ))
            })?;
            eprintln!("[ConversationStore] 🗑️  Cleared conversation history");
        }
        Ok(())
    }

    /// Get storage path
    pub fn path(&self) -> &Path {
        &self.storage_path
    }
}
