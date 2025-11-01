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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;
    use crate::conversation::DimensionalState;

    #[test]
    fn test_conversation_store_new() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_store_new.json");

        let store = ConversationStore::new(&store_path).expect("Failed to create store");
        assert_eq!(store.path(), store_path.as_path());

        // Cleanup
        let _ = std::fs::remove_file(store_path);
    }

    #[test]
    fn test_save_and_load_conversation() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_save_load.json");
        let store = ConversationStore::new(&store_path).expect("Failed to create store");

        // Create conversation
        let mut history = ConversationHistory::new();
        history.add_user_message("Hello!".to_string(), vec![DimensionId(1)]);
        history.add_assistant_message("Hi there!".to_string());

        // Save
        store.save_history(&history).expect("Failed to save");
        assert!(store_path.exists());

        // Load
        let loaded = store.load_history().expect("Failed to load");
        assert_eq!(loaded.messages.len(), 2);
        assert_eq!(loaded.session_id, history.session_id);
        assert_eq!(loaded.messages[0].content, "Hello!");
        assert_eq!(loaded.messages[1].content, "Hi there!");

        // Cleanup
        let _ = std::fs::remove_file(store_path);
    }

    #[test]
    fn test_save_with_dimensional_state() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_dimensional_state.json");
        let store = ConversationStore::new(&store_path).expect("Failed to create store");

        let mut history = ConversationHistory::new();
        let dimensional_state = DimensionalState {
            activated_dimensions: vec![DimensionId(1), DimensionId(6), DimensionId(10)],
            selection_duration_ms: 1500,
            contexts_loaded: 25,
            frequency_pattern: Some(vec![
                (DimensionId(1), 1.0),
                (DimensionId(6), 0.3),
                (DimensionId(10), 1.2),
            ]),
        };

        history.add_user_message("Deep question".to_string(), vec![DimensionId(1)]);
        history.add_assistant_message_with_state(
            "Philosophical answer".to_string(),
            dimensional_state,
        );

        // Save and load
        store.save_history(&history).expect("Failed to save");
        let loaded = store.load_history().expect("Failed to load");

        // Verify dimensional state preserved
        assert_eq!(loaded.messages.len(), 2);
        let assistant_msg = &loaded.messages[1];
        assert!(assistant_msg.dimensional_state.is_some());

        let state = assistant_msg.dimensional_state.as_ref().unwrap();
        assert_eq!(state.activated_dimensions.len(), 3);
        assert_eq!(state.selection_duration_ms, 1500);
        assert_eq!(state.contexts_loaded, 25);
        assert!(state.frequency_pattern.is_some());

        // Cleanup
        let _ = std::fs::remove_file(store_path);
    }

    #[test]
    fn test_load_nonexistent_file() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("nonexistent_file.json");

        // Make sure file doesn't exist
        let _ = std::fs::remove_file(&store_path);

        let store = ConversationStore::new(&store_path).expect("Failed to create store");
        let loaded = store.load_history().expect("Should return empty history");

        assert_eq!(loaded.messages.len(), 0);
        assert!(loaded.is_empty());
    }

    #[test]
    fn test_clear_history() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_clear.json");
        let store = ConversationStore::new(&store_path).expect("Failed to create store");

        // Create and save
        let mut history = ConversationHistory::new();
        history.add_user_message("Test".to_string(), vec![]);
        store.save_history(&history).expect("Failed to save");
        assert!(store_path.exists());

        // Clear
        store.clear_history().expect("Failed to clear");
        assert!(!store_path.exists());
    }

    #[test]
    fn test_clear_nonexistent_history() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_clear_nonexistent.json");
        let _ = std::fs::remove_file(&store_path);

        let store = ConversationStore::new(&store_path).expect("Failed to create store");

        // Should not error when clearing nonexistent file
        store.clear_history().expect("Should handle nonexistent file");
    }

    #[test]
    fn test_multiple_save_overwrites() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_overwrite.json");
        let store = ConversationStore::new(&store_path).expect("Failed to create store");

        // First save
        let mut history1 = ConversationHistory::new();
        history1.add_user_message("First".to_string(), vec![]);
        store.save_history(&history1).expect("Failed to save first");

        // Second save (should overwrite)
        let mut history2 = ConversationHistory::new();
        history2.add_user_message("Second".to_string(), vec![]);
        history2.add_assistant_message("Response".to_string());
        store.save_history(&history2).expect("Failed to save second");

        // Load should get second history
        let loaded = store.load_history().expect("Failed to load");
        assert_eq!(loaded.messages.len(), 2);
        assert_eq!(loaded.messages[0].content, "Second");

        // Cleanup
        let _ = std::fs::remove_file(store_path);
    }

    #[test]
    fn test_persistence_across_sessions() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_multi_session.json");

        // Session 1: Create and save
        {
            let store1 = ConversationStore::new(&store_path).expect("Failed to create store");
            let mut history = ConversationHistory::new();
            let session_id = history.session_id.clone();

            history.add_user_message("Message 1".to_string(), vec![DimensionId(1)]);
            history.add_assistant_message("Response 1".to_string());
            store1.save_history(&history).expect("Failed to save");
        }

        // Session 2: Load and add more
        {
            let store2 = ConversationStore::new(&store_path).expect("Failed to create store");
            let mut history = store2.load_history().expect("Failed to load");

            assert_eq!(history.messages.len(), 2);

            history.add_user_message("Message 2".to_string(), vec![DimensionId(2)]);
            history.add_assistant_message("Response 2".to_string());
            store2.save_history(&history).expect("Failed to save");
        }

        // Session 3: Verify all messages preserved
        {
            let store3 = ConversationStore::new(&store_path).expect("Failed to create store");
            let history = store3.load_history().expect("Failed to load");

            assert_eq!(history.messages.len(), 4);
            assert_eq!(history.messages[0].content, "Message 1");
            assert_eq!(history.messages[1].content, "Response 1");
            assert_eq!(history.messages[2].content, "Message 2");
            assert_eq!(history.messages[3].content, "Response 2");
        }

        // Cleanup
        let _ = std::fs::remove_file(store_path);
    }

    #[test]
    fn test_json_format_human_readable() {
        let temp_dir = std::env::temp_dir();
        let store_path = temp_dir.join("test_human_readable.json");
        let store = ConversationStore::new(&store_path).expect("Failed to create store");

        let mut history = ConversationHistory::new();
        history.add_user_message("Test".to_string(), vec![DimensionId(1)]);
        store.save_history(&history).expect("Failed to save");

        // Read raw JSON
        let json_content = std::fs::read_to_string(&store_path).expect("Failed to read file");

        // Should be pretty-printed (contains newlines and indentation)
        assert!(json_content.contains('\n'));
        assert!(json_content.contains("  ")); // Indentation
        assert!(json_content.contains("messages"));
        assert!(json_content.contains("session_id"));

        // Cleanup
        let _ = std::fs::remove_file(store_path);
    }
}
