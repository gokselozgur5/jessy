//! Persistent User Context System
//!
//! Manages cross-conversation memory for authentic relationship building.
//! Stores conversation flavor, relationship dynamics, and fuzzy memory recall.

use crate::{DimensionId, Result, ConsciousnessError};
use serde::{Deserialize, Serialize};
use chrono::{DateTime, Utc};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use tokio::sync::Mutex;

/// User-specific persistent context across conversations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserContext {
    pub user_id: String,
    pub conversations: Vec<ConversationSummary>,
    pub relationship_dynamics: RelationshipDynamics,
    pub conversation_flavor: ConversationFlavor,
    pub unfinished_threads: Vec<UnfinishedThread>,
    pub created_at: DateTime<Utc>,
    pub last_interaction: DateTime<Utc>,
}

impl UserContext {
    /// Create new user context
    pub fn new(user_id: String) -> Self {
        let now = Utc::now();
        Self {
            user_id,
            conversations: Vec::new(),
            relationship_dynamics: RelationshipDynamics::default(),
            conversation_flavor: ConversationFlavor::default(),
            unfinished_threads: Vec::new(),
            created_at: now,
            last_interaction: now,
        }
    }

    /// Update last interaction timestamp
    pub fn touch(&mut self) {
        self.last_interaction = Utc::now();
    }

    /// Add conversation summary
    pub fn add_conversation(&mut self, summary: ConversationSummary) {
        self.conversations.push(summary);
        self.touch();
    }

    /// Get recent conversations (last N)
    pub fn recent_conversations(&self, count: usize) -> &[ConversationSummary] {
        let start = self.conversations.len().saturating_sub(count);
        &self.conversations[start..]
    }
}

/// Rich conversation metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationSummary {
    pub session_id: String,
    pub timestamp: DateTime<Utc>,
    pub emotional_tone: EmotionalTone,
    pub key_moments: Vec<KeyMoment>,
    pub topics: Vec<String>,
    pub message_count: usize,
}

/// Emotional tone of conversation
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum EmotionalTone {
    Playful,
    Serious,
    Curious,
    Warm,
    Analytical,
    Excited,
    Contemplative,
    Mixed,
}

impl Default for EmotionalTone {
    fn default() -> Self {
        Self::Warm
    }
}

/// Key moments worth remembering
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeyMoment {
    pub moment_type: MomentType,
    pub content: String,
    pub timestamp: DateTime<Utc>,
    pub significance: f32,  // 0.0-1.0
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum MomentType {
    Joke,
    Insight,
    Emotional,
    Breakthrough,
    SharedReference,
}

/// Relationship dynamics between user and Jessy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RelationshipDynamics {
    pub formality_level: FormalityLevel,
    pub shared_references: Vec<String>,
    pub communication_style: CommunicationStyle,
    pub trust_level: f32,  // 0.0-1.0
}

impl Default for RelationshipDynamics {
    fn default() -> Self {
        Self {
            formality_level: FormalityLevel::Casual,
            shared_references: Vec::new(),
            communication_style: CommunicationStyle::DirectHonest,
            trust_level: 0.5,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum FormalityLevel {
    Formal,
    Professional,
    Casual,
    Intimate,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum CommunicationStyle {
    DirectHonest,
    Playful,
    Analytical,
    Supportive,
    Mixed,
}

/// Conversation flavor - the "vibe" of interactions
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConversationFlavor {
    pub humor_style: Option<HumorStyle>,
    pub emotional_baseline: EmotionalTone,
    pub energy_level: EnergyLevel,
    pub language_mix: LanguageMix,
}

impl Default for ConversationFlavor {
    fn default() -> Self {
        Self {
            humor_style: None,
            emotional_baseline: EmotionalTone::Warm,
            energy_level: EnergyLevel::Medium,
            language_mix: LanguageMix::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum HumorStyle {
    Sarcastic,
    Playful,
    Dry,
    Witty,
    Absurd,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum EnergyLevel {
    Low,
    Medium,
    High,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LanguageMix {
    pub turkish_ratio: f32,  // 0.0-1.0
    pub english_ratio: f32,  // 0.0-1.0
}

impl Default for LanguageMix {
    fn default() -> Self {
        Self {
            turkish_ratio: 0.0,
            english_ratio: 1.0,
        }
    }
}

/// Unfinished topics to revisit
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UnfinishedThread {
    pub topic: String,
    pub context: String,
    pub started_at: DateTime<Utc>,
    pub priority: ThreadPriority,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum ThreadPriority {
    Low,
    Medium,
    High,
}

/// Persistent context manager with LRU cache
pub struct PersistentContextManager {
    storage_path: PathBuf,
    cache: Arc<Mutex<HashMap<String, UserContext>>>,
    max_cache_size: usize,
    retention_days: i64,
}

impl PersistentContextManager {
    /// Create new persistent context manager
    pub fn new(storage_path: PathBuf, max_cache_size: usize, retention_days: i64) -> Result<Self> {
        // Create storage directory if it doesn't exist
        std::fs::create_dir_all(&storage_path)
            .map_err(|e| ConsciousnessError::LearningError(
                format!("Failed to create storage directory: {}", e)
            ))?;

        Ok(Self {
            storage_path,
            cache: Arc::new(Mutex::new(HashMap::new())),
            max_cache_size,
            retention_days,
        })
    }

    /// Get user context file path
    fn user_file_path(&self, user_id: &str) -> PathBuf {
        self.storage_path.join(format!("{}.json", user_id))
    }

    /// Load user context from storage
    pub async fn load_user_context(&self, user_id: &str) -> Result<UserContext> {
        // Check cache first
        {
            let cache = self.cache.lock().await;
            if let Some(context) = cache.get(user_id) {
                return Ok(context.clone());
            }
        }

        // Load from file
        let file_path = self.user_file_path(user_id);
        
        let context = if file_path.exists() {
            let content = tokio::fs::read_to_string(&file_path).await
                .map_err(|e| ConsciousnessError::LearningError(
                    format!("Failed to read user context: {}", e)
                ))?;

            serde_json::from_str(&content)
                .map_err(|e| ConsciousnessError::LearningError(
                    format!("Failed to parse user context: {}", e)
                ))?
        } else {
            // Create new context for new user
            UserContext::new(user_id.to_string())
        };

        // Add to cache
        {
            let mut cache = self.cache.lock().await;
            
            // Implement simple LRU: remove oldest if cache full
            if cache.len() >= self.max_cache_size {
                if let Some(oldest_key) = cache.keys().next().cloned() {
                    cache.remove(&oldest_key);
                }
            }
            
            cache.insert(user_id.to_string(), context.clone());
        }

        Ok(context)
    }

    /// Save user context to storage
    pub async fn save_user_context(&self, context: &UserContext) -> Result<()> {
        let file_path = self.user_file_path(&context.user_id);
        
        // Serialize to JSON
        let json = serde_json::to_string_pretty(context)
            .map_err(|e| ConsciousnessError::LearningError(
                format!("Failed to serialize user context: {}", e)
            ))?;

        // Write to temp file first (atomic operation)
        let temp_path = file_path.with_extension("tmp");
        tokio::fs::write(&temp_path, json).await
            .map_err(|e| ConsciousnessError::LearningError(
                format!("Failed to write user context: {}", e)
            ))?;

        // Rename to final path (atomic)
        tokio::fs::rename(&temp_path, &file_path).await
            .map_err(|e| ConsciousnessError::LearningError(
                format!("Failed to rename user context file: {}", e)
            ))?;

        // Update cache
        {
            let mut cache = self.cache.lock().await;
            cache.insert(context.user_id.clone(), context.clone());
        }

        Ok(())
    }

    /// Get relevant past context for current query (fuzzy matching)
    pub async fn get_relevant_context(
        &self,
        user_id: &str,
        current_query: &str,
        max_conversations: usize,
    ) -> Result<Vec<ConversationSummary>> {
        let context = self.load_user_context(user_id).await?;
        
        // Simple relevance scoring based on topic matching
        let query_lower = current_query.to_lowercase();
        let query_words: Vec<&str> = query_lower.split_whitespace().collect();
        
        let mut scored_conversations: Vec<(f32, &ConversationSummary)> = context
            .conversations
            .iter()
            .map(|conv| {
                let mut score = 0.0;
                
                // Score based on topic overlap
                for topic in &conv.topics {
                    let topic_lower = topic.to_lowercase();
                    for word in &query_words {
                        if topic_lower.contains(word) {
                            score += 1.0;
                        }
                    }
                }
                
                // Boost recent conversations slightly
                let days_ago = (Utc::now() - conv.timestamp).num_days();
                let recency_boost = 1.0 / (1.0 + days_ago as f32 * 0.1);
                score += recency_boost;
                
                // Boost conversations with high-significance moments
                for moment in &conv.key_moments {
                    score += moment.significance * 0.5;
                }
                
                (score, conv)
            })
            .collect();
        
        // Sort by score descending
        scored_conversations.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        
        // Return top N
        Ok(scored_conversations
            .into_iter()
            .take(max_conversations)
            .map(|(_, conv)| conv.clone())
            .collect())
    }

    /// Update relationship dynamics based on interaction
    pub async fn update_relationship_dynamics(
        &self,
        user_id: &str,
        interaction_analysis: &InteractionAnalysis,
    ) -> Result<()> {
        let mut context = self.load_user_context(user_id).await?;
        
        // Update formality level
        if let Some(formality) = &interaction_analysis.detected_formality {
            context.relationship_dynamics.formality_level = formality.clone();
        }
        
        // Add shared references
        for reference in &interaction_analysis.shared_references {
            if !context.relationship_dynamics.shared_references.contains(reference) {
                context.relationship_dynamics.shared_references.push(reference.clone());
            }
        }
        
        // Update trust level (gradual increase with positive interactions)
        if interaction_analysis.positive_interaction {
            context.relationship_dynamics.trust_level = 
                (context.relationship_dynamics.trust_level + 0.05).min(1.0);
        }
        
        // Update communication style
        if let Some(style) = &interaction_analysis.detected_style {
            context.relationship_dynamics.communication_style = style.clone();
        }
        
        context.touch();
        self.save_user_context(&context).await?;
        
        Ok(())
    }

    /// Clean up old contexts (retention policy)
    pub async fn cleanup_old_contexts(&self) -> Result<usize> {
        let cutoff_date = Utc::now() - chrono::Duration::days(self.retention_days);
        let mut removed_count = 0;
        
        // Read all user files
        let entries = std::fs::read_dir(&self.storage_path)
            .map_err(|e| ConsciousnessError::LearningError(
                format!("Failed to read storage directory: {}", e)
            ))?;
        
        for entry in entries {
            let entry = entry.map_err(|e| ConsciousnessError::LearningError(
                format!("Failed to read directory entry: {}", e)
            ))?;
            
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) == Some("json") {
                // Load context to check last interaction
                if let Ok(content) = std::fs::read_to_string(&path) {
                    if let Ok(context) = serde_json::from_str::<UserContext>(&content) {
                        if context.last_interaction < cutoff_date {
                            std::fs::remove_file(&path).ok();
                            removed_count += 1;
                        }
                    }
                }
            }
        }
        
        Ok(removed_count)
    }
}

/// Analysis of user interaction for relationship dynamics
#[derive(Debug, Clone)]
pub struct InteractionAnalysis {
    pub detected_formality: Option<FormalityLevel>,
    pub shared_references: Vec<String>,
    pub positive_interaction: bool,
    pub detected_style: Option<CommunicationStyle>,
}

impl InteractionAnalysis {
    pub fn new() -> Self {
        Self {
            detected_formality: None,
            shared_references: Vec::new(),
            positive_interaction: true,
            detected_style: None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_user_context_creation() {
        let context = UserContext::new("test-user".to_string());
        assert_eq!(context.user_id, "test-user");
        assert_eq!(context.conversations.len(), 0);
        assert_eq!(context.relationship_dynamics.trust_level, 0.5);
    }

    #[test]
    fn test_conversation_summary() {
        let summary = ConversationSummary {
            session_id: "session-1".to_string(),
            timestamp: Utc::now(),
            emotional_tone: EmotionalTone::Playful,
            key_moments: vec![],
            topics: vec!["consciousness".to_string()],
            message_count: 5,
        };
        
        assert_eq!(summary.topics.len(), 1);
        assert_eq!(summary.emotional_tone, EmotionalTone::Playful);
    }

    #[tokio::test]
    async fn test_context_manager_new_user() {
        let temp_dir = std::env::temp_dir().join("jessy_test_contexts");
        let manager = PersistentContextManager::new(temp_dir.clone(), 10, 30).unwrap();
        
        let context = manager.load_user_context("new-user").await.unwrap();
        assert_eq!(context.user_id, "new-user");
        assert_eq!(context.conversations.len(), 0);
        
        // Cleanup
        std::fs::remove_dir_all(temp_dir).ok();
    }

    #[tokio::test]
    async fn test_context_save_and_load() {
        let temp_dir = std::env::temp_dir().join("jessy_test_contexts_2");
        let manager = PersistentContextManager::new(temp_dir.clone(), 10, 30).unwrap();
        
        let mut context = UserContext::new("test-user".to_string());
        context.relationship_dynamics.trust_level = 0.8;
        
        manager.save_user_context(&context).await.unwrap();
        
        let loaded = manager.load_user_context("test-user").await.unwrap();
        assert_eq!(loaded.relationship_dynamics.trust_level, 0.8);
        
        // Cleanup
        std::fs::remove_dir_all(temp_dir).ok();
    }
}
