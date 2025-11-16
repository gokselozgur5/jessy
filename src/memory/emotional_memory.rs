//! Emotional Memory System
//!
//! Tracks the "texture" of conversations - not just what was said,
//! but how it felt, the energy, the dynamics, the emotional fingerprint.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use chrono::{DateTime, Utc};
use crate::{DimensionId, Result, ConsciousnessError};

/// Full conversation record with emotional context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationRecord {
    pub session_id: String,
    pub user_id: String,
    pub started_at: DateTime<Utc>,
    pub ended_at: Option<DateTime<Utc>>,
    pub messages: Vec<MessageRecord>,
    pub emotional_arc: EmotionalArc,
    pub key_moments: Vec<KeyMoment>,
    pub energy_pattern: EnergyPattern,
}

/// Individual message with full context
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MessageRecord {
    pub role: String,  // "user" or "assistant"
    pub content: String,
    pub timestamp: DateTime<Utc>,
    pub dimensions_activated: Vec<DimensionId>,
    pub jessy_mood: Option<String>,  // Jessy's mood at response time
    pub jessy_energy: Option<f32>,   // Jessy's energy level
    pub emotional_tone: Option<String>,
}

/// Emotional arc of a conversation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionalArc {
    pub start_tone: String,
    pub end_tone: String,
    pub peak_moments: Vec<String>,  // Timestamps of emotional peaks
    pub overall_energy: f32,
}

/// Key moment in conversation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyMoment {
    pub timestamp: DateTime<Utc>,
    pub moment_type: String,  // "insight", "breakthrough", "joke", "excitement"
    pub description: String,
    pub emotional_intensity: f32,
}

/// Energy pattern throughout conversation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnergyPattern {
    pub average_energy: f32,
    pub peak_energy: f32,
    pub low_energy: f32,
    pub energy_trajectory: Vec<(DateTime<Utc>, f32)>,  // Timeline of energy
}

/// Aggregated emotional memory for a user
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmotionalMemory {
    pub user_id: String,
    pub first_interaction: DateTime<Utc>,
    pub last_interaction: DateTime<Utc>,
    pub total_conversations: usize,
    pub communication_patterns: CommunicationPatterns,
    pub shared_references: Vec<SharedReference>,
    pub inside_jokes: Vec<InsideJoke>,
    pub topic_energy_map: HashMap<String, f32>,  // Which topics excite this user
    pub relationship_dynamics: RelationshipDynamics,
}

/// Communication patterns between user and Jessy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationPatterns {
    pub preferred_style: String,  // "direct", "playful", "philosophical"
    pub typical_energy: f32,
    pub common_topics: Vec<String>,
    pub conversation_rhythm: String,  // "quick back-and-forth", "deep dives", "exploratory"
    pub humor_style: Option<String>,
}

/// Shared reference that emerged from conversations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SharedReference {
    pub reference: String,
    pub first_mentioned: DateTime<Utc>,
    pub times_referenced: usize,
    pub context: String,
}

/// Inside joke between user and Jessy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InsideJoke {
    pub joke: String,
    pub origin_session: String,
    pub created_at: DateTime<Utc>,
    pub times_used: usize,
}

/// Relationship dynamics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelationshipDynamics {
    pub rapport_level: f32,  // 0.0-1.0
    pub trust_level: f32,
    pub playfulness: f32,
    pub intellectual_depth: f32,
    pub emotional_openness: f32,
}

/// Emotional memory manager
pub struct EmotionalMemoryManager {
    base_path: PathBuf,
}

impl EmotionalMemoryManager {
    /// Create new emotional memory manager
    pub fn new(base_path: PathBuf) -> Result<Self> {
        std::fs::create_dir_all(&base_path)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to create emotional memory dir: {}", e)))?;
        
        Ok(Self { base_path })
    }
    
    /// Save full conversation record
    pub async fn save_conversation(&self, record: &ConversationRecord) -> Result<()> {
        let user_dir = self.base_path.join(&record.user_id);
        std::fs::create_dir_all(&user_dir)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to create user dir: {}", e)))?;
        
        let file_path = user_dir.join(format!("{}.json", record.session_id));
        let json = serde_json::to_string_pretty(record)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to serialize conversation: {}", e)))?;
        
        tokio::fs::write(&file_path, json).await
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to write conversation: {}", e)))?;
        
        eprintln!("[EmotionalMemory] Saved conversation {} for user {}", record.session_id, record.user_id);
        Ok(())
    }
    
    /// Load conversation record
    pub async fn load_conversation(&self, user_id: &str, session_id: &str) -> Result<ConversationRecord> {
        let file_path = self.base_path.join(user_id).join(format!("{}.json", session_id));
        
        let json = tokio::fs::read_to_string(&file_path).await
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to read conversation: {}", e)))?;
        
        let record = serde_json::from_str(&json)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to parse conversation: {}", e)))?;
        
        Ok(record)
    }
    
    /// Load all conversations for a user
    pub async fn load_user_conversations(&self, user_id: &str) -> Result<Vec<ConversationRecord>> {
        let user_dir = self.base_path.join(user_id);
        
        if !user_dir.exists() {
            return Ok(vec![]);
        }
        
        let mut conversations = Vec::new();
        let mut entries = tokio::fs::read_dir(&user_dir).await
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to read user dir: {}", e)))?;
        
        while let Some(entry) = entries.next_entry().await
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to read entry: {}", e)))? {
            
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) == Some("json") {
                if let Some(filename) = path.file_stem().and_then(|s| s.to_str()) {
                    if filename != "emotional_memory" {
                        if let Ok(record) = self.load_conversation(user_id, filename).await {
                            conversations.push(record);
                        }
                    }
                }
            }
        }
        
        Ok(conversations)
    }
    
    /// Save aggregated emotional memory
    pub async fn save_emotional_memory(&self, memory: &EmotionalMemory) -> Result<()> {
        let user_dir = self.base_path.join(&memory.user_id);
        std::fs::create_dir_all(&user_dir)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to create user dir: {}", e)))?;
        
        let file_path = user_dir.join("emotional_memory.json");
        let json = serde_json::to_string_pretty(memory)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to serialize emotional memory: {}", e)))?;
        
        tokio::fs::write(&file_path, json).await
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to write emotional memory: {}", e)))?;
        
        eprintln!("[EmotionalMemory] Saved emotional memory for user {}", memory.user_id);
        Ok(())
    }
    
    /// Load aggregated emotional memory
    pub async fn load_emotional_memory(&self, user_id: &str) -> Result<EmotionalMemory> {
        let file_path = self.base_path.join(user_id).join("emotional_memory.json");
        
        let json = tokio::fs::read_to_string(&file_path).await
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to read emotional memory: {}", e)))?;
        
        let memory = serde_json::from_str(&json)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to parse emotional memory: {}", e)))?;
        
        Ok(memory)
    }
    
    /// Update emotional memory from conversation
    pub async fn update_emotional_memory(&self, record: &ConversationRecord) -> Result<()> {
        // Load existing or create new
        let mut memory = self.load_emotional_memory(&record.user_id).await
            .unwrap_or_else(|_| EmotionalMemory {
                user_id: record.user_id.clone(),
                first_interaction: record.started_at,
                last_interaction: record.started_at,
                total_conversations: 0,
                communication_patterns: CommunicationPatterns {
                    preferred_style: "direct".to_string(),
                    typical_energy: 0.5,
                    common_topics: vec![],
                    conversation_rhythm: "balanced".to_string(),
                    humor_style: None,
                },
                shared_references: vec![],
                inside_jokes: vec![],
                topic_energy_map: HashMap::new(),
                relationship_dynamics: RelationshipDynamics {
                    rapport_level: 0.5,
                    trust_level: 0.5,
                    playfulness: 0.5,
                    intellectual_depth: 0.5,
                    emotional_openness: 0.5,
                },
            });
        
        // Update stats
        memory.last_interaction = record.ended_at.unwrap_or(Utc::now());
        memory.total_conversations += 1;
        
        // Update energy patterns
        memory.communication_patterns.typical_energy = 
            (memory.communication_patterns.typical_energy * 0.8) + (record.emotional_arc.overall_energy * 0.2);
        
        // Update relationship dynamics based on conversation
        if record.emotional_arc.overall_energy > 0.7 {
            memory.relationship_dynamics.rapport_level = 
                (memory.relationship_dynamics.rapport_level * 0.9 + 0.1).min(1.0);
        }
        
        // Save updated memory
        self.save_emotional_memory(&memory).await?;
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[tokio::test]
    async fn test_save_and_load_conversation() {
        let temp_dir = std::env::temp_dir().join("jessy_emotional_memory_test");
        let manager = EmotionalMemoryManager::new(temp_dir.clone()).unwrap();
        
        let record = ConversationRecord {
            session_id: "test_session".to_string(),
            user_id: "test_user".to_string(),
            started_at: Utc::now(),
            ended_at: None,
            messages: vec![],
            emotional_arc: EmotionalArc {
                start_tone: "curious".to_string(),
                end_tone: "satisfied".to_string(),
                peak_moments: vec![],
                overall_energy: 0.7,
            },
            key_moments: vec![],
            energy_pattern: EnergyPattern {
                average_energy: 0.7,
                peak_energy: 0.9,
                low_energy: 0.5,
                energy_trajectory: vec![],
            },
        };
        
        manager.save_conversation(&record).await.unwrap();
        let loaded = manager.load_conversation("test_user", "test_session").await.unwrap();
        
        assert_eq!(loaded.session_id, "test_session");
        assert_eq!(loaded.user_id, "test_user");
        
        // Cleanup
        let _ = std::fs::remove_dir_all(temp_dir);
    }
}
