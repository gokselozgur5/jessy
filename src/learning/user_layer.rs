//! User-Specific Cognitive Layers (C31+) - Tier 3
//!
//! This module implements Tier 3 of the 3-Tier cognitive layer system.
//! User layers are learned from individual user interactions and provide personalization.
//!
//! ## Architecture
//!
//! - **C31+:** Unlimited dimension IDs (starting from 31)
//! - **Memory:** 32MB User-Specific Region (MMAP region 0x1000_0000 - 0x1200_0000)
//! - **Learning:** Patterns detected per user (>85% confidence)
//! - **Benefit:** Individual users get personalized responses
//!
//! ## Examples
//!
//! - C31: "Functional Programming Preference" (user prefers FP style)
//! - C32: "Spanish Technical Language" (Spanish speaker asking tech questions)
//! - C33: "Vim Shortcuts" (user frequently asks about Vim)
//! - C34: "Gardening Interest" (user asks gardening questions)
//!
//! ## Eviction Strategy
//!
//! - Max 5 layers per user
//! - LRU eviction: Least Recently Used layer evicted when limit reached
//! - Per-user isolation: User A cannot access User B's layers

use crate::{DimensionId, LayerId, Frequency, Result, ConsciousnessError};
use crate::memory::MmapManager;
use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::SystemTime;

/// Tier 3: User-specific cognitive layer (C31+)
///
/// Represents a cognitive layer learned from individual user interactions.
/// These layers provide personalization and are isolated per user.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct UserLayer {
    /// User ID (hashed for privacy)
    pub user_id: String,

    /// Dimension ID (31+ for Tier 3)
    pub dimension_id: DimensionId,

    /// Full layer identifier (includes dimension + layer + sublayer)
    pub layer_id: LayerId,

    /// Layer content (keywords, patterns, embeddings)
    pub content: Vec<u8>,

    /// Matched keywords for this layer
    pub keywords: Vec<String>,

    /// Layer frequency (Hz) for interference calculation
    pub frequency: Frequency,

    /// Pattern confidence (0.0-1.0, must be >0.85 for Tier 3)
    pub confidence: f32,

    /// When this layer was crystallized
    pub created_at: SystemTime,

    /// Last time this layer was accessed
    pub last_accessed: SystemTime,

    /// Number of times this layer has been accessed
    pub access_count: u64,
}

impl UserLayer {
    /// Create a new user-specific layer
    ///
    /// # Arguments
    ///
    /// * `user_id` - User ID (will be hashed for storage)
    /// * `dimension_id` - Dimension ID (must be ≥31)
    /// * `layer_id` - Full layer identifier
    /// * `content` - Layer content
    /// * `keywords` - Matched keywords
    /// * `frequency` - Layer frequency (Hz)
    /// * `confidence` - Pattern confidence (must be >0.85)
    ///
    /// # Returns
    ///
    /// New UserLayer instance
    ///
    /// # Errors
    ///
    /// - If dimension_id is <31
    /// - If confidence is <0.85 (Tier 3 requires >85% confidence)
    /// - If user_id is empty
    pub fn new(
        user_id: String,
        dimension_id: DimensionId,
        layer_id: LayerId,
        content: Vec<u8>,
        keywords: Vec<String>,
        frequency: Frequency,
        confidence: f32,
    ) -> Result<Self> {
        // Validate user_id
        if user_id.is_empty() {
            return Err(ConsciousnessError::LearningError(
                "UserLayer user_id cannot be empty".to_string(),
            ));
        }

        // Validate dimension range (C31+)
        let dim_num = dimension_id.0;
        if dim_num < 31 {
            return Err(ConsciousnessError::LearningError(format!(
                "UserLayer dimension_id must be ≥31, got {}",
                dim_num
            )));
        }

        // Validate confidence threshold (Tier 3 requires >85%)
        if confidence < 0.85 {
            return Err(ConsciousnessError::LearningError(format!(
                "UserLayer confidence must be ≥0.85, got {}",
                confidence
            )));
        }

        Ok(Self {
            user_id,
            dimension_id,
            layer_id,
            content,
            keywords,
            frequency,
            confidence,
            created_at: SystemTime::now(),
            last_accessed: SystemTime::now(),
            access_count: 0,
        })
    }

    /// Update access count and timestamp
    pub fn touch(&mut self) {
        self.last_accessed = SystemTime::now();
        self.access_count += 1;
    }

    /// Get age in seconds since creation
    pub fn age_seconds(&self) -> u64 {
        self.created_at
            .elapsed()
            .unwrap_or_default()
            .as_secs()
    }

    /// Get time since last access in seconds
    pub fn idle_seconds(&self) -> u64 {
        self.last_accessed
            .elapsed()
            .unwrap_or_default()
            .as_secs()
    }

    /// Get size in bytes
    pub fn size_bytes(&self) -> usize {
        self.content.len()
            + self.keywords.iter().map(|k| k.len()).sum::<usize>()
            + self.user_id.len()
            + std::mem::size_of::<Self>()
    }
}

/// Manages Tier 3: User-specific layers (C31+)
///
/// Provides CRUD operations for user-specific cognitive layers.
/// Handles per-user LRU eviction when 5 layers per user are reached.
pub struct UserLayerManager {
    /// user_id → layers (max 5 per user)
    user_layers: HashMap<String, Vec<UserLayer>>,

    /// Memory manager for MMAP
    memory_manager: Arc<MmapManager>,

    /// Max layers per user (default: 5)
    max_layers_per_user: usize,

    /// User-specific region offset (0x1000_0000)
    user_region_base: usize,

    /// User region size (32MB)
    user_region_size: usize,
}

impl UserLayerManager {
    /// Create new user layer manager
    ///
    /// # Arguments
    ///
    /// * `memory_manager` - MMAP manager instance
    /// * `user_region_size` - Size of user region (32MB)
    /// * `user_region_base` - Base offset in MMAP (0x1000_0000)
    ///
    /// # Returns
    ///
    /// New UserLayerManager instance
    pub fn new(
        memory_manager: Arc<MmapManager>,
        user_region_size: usize,
        user_region_base: usize,
    ) -> Self {
        Self {
            user_layers: HashMap::new(),
            memory_manager,
            max_layers_per_user: 5, // C31-C35 per user
            user_region_base,
            user_region_size,
        }
    }

    /// Create a new user-specific layer
    ///
    /// # Arguments
    ///
    /// * `layer` - UserLayer to create
    ///
    /// # Returns
    ///
    /// Ok(()) if layer was created successfully
    ///
    /// # Errors
    ///
    /// - If max layers per user reached and LRU eviction fails
    pub fn create_user_layer(&mut self, layer: UserLayer) -> Result<()> {
        let user_id = layer.user_id.clone();
        let dimension_id = layer.dimension_id;

        // Check if max layers per user reached BEFORE borrowing
        let needs_eviction = self
            .user_layers
            .get(&user_id)
            .map(|vec| vec.len() >= self.max_layers_per_user)
            .unwrap_or(false);

        if needs_eviction {
            eprintln!(
                "[UserLayerManager] Max layers reached for user '{}' ({}), performing LRU eviction",
                user_id,
                self.max_layers_per_user
            );
            self.evict_least_recently_used(&user_id)?;
        }

        // Allocate space in MMAP User-Specific region (C31+ region)
        let size_bytes = layer.size_bytes();
        let _offset = self.memory_manager.allocate(size_bytes)
            .map_err(|e| {
                eprintln!("[UserLayerManager] MMAP allocation failed for user '{}' {:?}: {}", user_id, dimension_id, e);
                ConsciousnessError::LearningError(format!(
                    "Failed to allocate {}KB in User-Specific region: {}",
                    size_bytes / 1024,
                    e
                ))
            })?;

        eprintln!(
            "[UserLayerManager] Allocated {}KB in MMAP User-Specific region for user '{}' {:?}",
            size_bytes / 1024,
            user_id,
            dimension_id
        );

        // Get or create user's layer vector (now safe to borrow)
        let user_vec = self.user_layers.entry(user_id.clone()).or_insert_with(Vec::new);

        // Insert layer
        user_vec.push(layer);

        eprintln!(
            "[UserLayerManager] Created user layer {:?} for '{}' (total: {}/{})",
            dimension_id,
            user_id,
            user_vec.len(),
            self.max_layers_per_user
        );

        Ok(())
    }

    /// Get all layers for a specific user
    ///
    /// # Arguments
    ///
    /// * `user_id` - User ID
    ///
    /// # Returns
    ///
    /// Vector of references to user's layers (empty if user not found)
    pub fn get_user_layers(&self, user_id: &str) -> Vec<&UserLayer> {
        self.user_layers
            .get(user_id)
            .map(|layers| layers.iter().collect())
            .unwrap_or_default()
    }

    /// Get a specific layer for a user by dimension ID
    ///
    /// # Arguments
    ///
    /// * `user_id` - User ID
    /// * `dimension_id` - Dimension ID
    ///
    /// # Returns
    ///
    /// Reference to UserLayer if found, None otherwise
    pub fn get_user_layer(
        &mut self,
        user_id: &str,
        dimension_id: DimensionId,
    ) -> Option<&UserLayer> {
        if let Some(layers) = self.user_layers.get_mut(user_id) {
            if let Some(layer) = layers.iter_mut().find(|l| l.dimension_id == dimension_id) {
                layer.touch(); // Update access count
                return Some(layer);
            }
        }
        None
    }

    /// Remove a specific layer for a user
    ///
    /// # Arguments
    ///
    /// * `user_id` - User ID
    /// * `dimension_id` - Dimension ID to remove
    ///
    /// # Returns
    ///
    /// Ok(()) if layer was removed, Err if not found
    pub fn remove_user_layer(&mut self, user_id: &str, dimension_id: DimensionId) -> Result<()> {
        if let Some(layers) = self.user_layers.get_mut(user_id) {
            if let Some(pos) = layers.iter().position(|l| l.dimension_id == dimension_id) {
                layers.remove(pos);

                eprintln!(
                    "[UserLayerManager] Removed layer {:?} for user '{}'",
                    dimension_id,
                    user_id
                );

                return Ok(());
            }
        }

        Err(ConsciousnessError::LearningError(format!(
            "UserLayer {:?} not found for user '{}'",
            dimension_id,
            user_id
        )))
    }

    /// Clear all layers for a specific user
    ///
    /// # Arguments
    ///
    /// * `user_id` - User ID
    pub fn clear_user(&mut self, user_id: &str) {
        if let Some(layers) = self.user_layers.remove(user_id) {
            eprintln!(
                "[UserLayerManager] Cleared {} layers for user '{}'",
                layers.len(),
                user_id
            );
        }
    }

    /// Evict least recently used layer for a specific user
    ///
    /// # Arguments
    ///
    /// * `user_id` - User ID
    ///
    /// # Returns
    ///
    /// Ok(()) if eviction succeeded
    ///
    /// # Errors
    ///
    /// - If user has no layers
    fn evict_least_recently_used(&mut self, user_id: &str) -> Result<()> {
        if let Some(layers) = self.user_layers.get_mut(user_id) {
            // Find layer with oldest last_accessed
            let lru_pos = layers
                .iter()
                .enumerate()
                .min_by_key(|(_, l)| l.last_accessed)
                .map(|(pos, _)| pos);

            match lru_pos {
                Some(pos) => {
                    let removed = layers.remove(pos);

                    eprintln!(
                        "[UserLayerManager] Evicted LRU layer {:?} for user '{}' (idle: {}s)",
                        removed.dimension_id,
                        user_id,
                        removed.idle_seconds()
                    );

                    Ok(())
                }
                None => Err(ConsciousnessError::LearningError(format!(
                    "No layers to evict for user '{}'",
                    user_id
                ))),
            }
        } else {
            Err(ConsciousnessError::LearningError(format!(
                "User '{}' not found",
                user_id
            )))
        }
    }

    /// Get number of users with layers
    pub fn user_count(&self) -> usize {
        self.user_layers.len()
    }

    /// Get total number of layers across all users
    pub fn total_layer_count(&self) -> usize {
        self.user_layers.values().map(|v| v.len()).sum()
    }

    /// Get total memory usage across all users
    pub fn total_memory_usage(&self) -> usize {
        self.user_layers
            .values()
            .flat_map(|layers| layers.iter())
            .map(|layer| layer.size_bytes())
            .sum()
    }

    /// Save user layers to disk
    ///
    /// Persists all user layers to JSON files in data/user_layers/{user_id}/
    /// Each layer is saved as: data/user_layers/{user_id}/c{dimension_id}.json
    ///
    /// # Returns
    ///
    /// Ok(()) if all layers were saved successfully
    ///
    /// # Errors
    ///
    /// - If directory creation fails
    /// - If serialization or file write fails
    pub fn save(&self) -> Result<()> {
        use std::fs;
        use std::path::Path;

        let base_dir = Path::new("data/user_layers");

        // Save each user's layers
        for (user_id, layers) in &self.user_layers {
            // Create user-specific directory
            let user_dir = base_dir.join(user_id);
            fs::create_dir_all(&user_dir).map_err(|e| {
                ConsciousnessError::LearningError(format!(
                    "Failed to create user_layers directory for '{}': {}",
                    user_id, e
                ))
            })?;

            // Save each layer for this user
            for layer in layers {
                let file_path = user_dir.join(format!("c{}.json", layer.dimension_id.0));

                // Serialize to JSON
                let json = serde_json::to_string_pretty(layer).map_err(|e| {
                    ConsciousnessError::LearningError(format!(
                        "Failed to serialize UserLayer {:?} for user '{}': {}",
                        layer.dimension_id, user_id, e
                    ))
                })?;

                // Write to file
                fs::write(&file_path, json).map_err(|e| {
                    ConsciousnessError::LearningError(format!(
                        "Failed to write UserLayer {:?} for user '{}' to {:?}: {}",
                        layer.dimension_id, user_id, file_path, e
                    ))
                })?;
            }

            eprintln!(
                "[UserLayerManager] Saved {} layers for user '{}'",
                layers.len(),
                user_id
            );
        }

        eprintln!(
            "[UserLayerManager] Successfully saved {} user layers across {} users",
            self.total_layer_count(),
            self.user_count()
        );

        Ok(())
    }

    /// Load user layers from disk
    ///
    /// Loads all user layers from JSON files in data/user_layers/{user_id}/
    /// Expects directory structure: data/user_layers/{user_id}/c{dimension_id}.json
    ///
    /// # Returns
    ///
    /// Ok(()) if layers were loaded successfully
    ///
    /// # Errors
    ///
    /// - If directory doesn't exist (returns Ok with 0 layers loaded)
    /// - If deserialization fails
    /// - If layer validation fails
    pub fn load(&mut self) -> Result<()> {
        use std::fs;
        use std::path::Path;

        let base_dir = Path::new("data/user_layers");

        // If directory doesn't exist, nothing to load
        if !base_dir.exists() {
            eprintln!("[UserLayerManager] No user_layers directory found, starting fresh");
            return Ok(());
        }

        // Read all user directories
        let user_entries = fs::read_dir(base_dir).map_err(|e| {
            ConsciousnessError::LearningError(format!(
                "Failed to read user_layers directory: {}",
                e
            ))
        })?;

        let mut total_loaded = 0;

        for user_entry in user_entries {
            let user_entry = user_entry.map_err(|e| {
                ConsciousnessError::LearningError(format!(
                    "Failed to read user directory entry: {}",
                    e
                ))
            })?;

            let user_path = user_entry.path();

            // Only process directories
            if !user_path.is_dir() {
                continue;
            }

            // Get user_id from directory name
            let user_id = user_path
                .file_name()
                .and_then(|n| n.to_str())
                .ok_or_else(|| {
                    ConsciousnessError::LearningError(format!(
                        "Invalid user directory name: {:?}",
                        user_path
                    ))
                })?
                .to_string();

            // Read all .json files in user directory
            let layer_entries = fs::read_dir(&user_path).map_err(|e| {
                ConsciousnessError::LearningError(format!(
                    "Failed to read user directory {:?}: {}",
                    user_path, e
                ))
            })?;

            let mut user_layer_count = 0;

            for layer_entry in layer_entries {
                let layer_entry = layer_entry.map_err(|e| {
                    ConsciousnessError::LearningError(format!(
                        "Failed to read layer file entry: {}",
                        e
                    ))
                })?;

                let layer_path = layer_entry.path();

                // Only process .json files
                if layer_path.extension().and_then(|s| s.to_str()) != Some("json") {
                    continue;
                }

                // Read file
                let json = fs::read_to_string(&layer_path).map_err(|e| {
                    ConsciousnessError::LearningError(format!(
                        "Failed to read file {:?}: {}",
                        layer_path, e
                    ))
                })?;

                // Deserialize
                let layer: UserLayer = serde_json::from_str(&json).map_err(|e| {
                    ConsciousnessError::LearningError(format!(
                        "Failed to deserialize UserLayer from {:?}: {}",
                        layer_path, e
                    ))
                })?;

                let dimension_id = layer.dimension_id;

                // Re-allocate in MMAP (layers were serialized, need to reallocate)
                let size_bytes = layer.size_bytes();
                let _offset = self.memory_manager.allocate(size_bytes).map_err(|e| {
                    ConsciousnessError::LearningError(format!(
                        "Failed to allocate {}KB in User-Specific region for loaded layer {:?} (user '{}'): {}",
                        size_bytes / 1024,
                        dimension_id,
                        user_id,
                        e
                    ))
                })?;

                // Insert layer
                let user_vec = self.user_layers.entry(user_id.clone()).or_insert_with(Vec::new);
                user_vec.push(layer);
                user_layer_count += 1;
                total_loaded += 1;
            }

            if user_layer_count > 0 {
                eprintln!(
                    "[UserLayerManager] Loaded {} layers for user '{}'",
                    user_layer_count, user_id
                );
            }
        }

        eprintln!(
            "[UserLayerManager] Successfully loaded {} user layers across {} users",
            total_loaded,
            self.user_count()
        );

        Ok(())
    }

    /// Get statistics about user layers
    pub fn stats(&self) -> UserLayerStats {
        UserLayerStats {
            user_count: self.user_count(),
            total_layer_count: self.total_layer_count(),
            max_layers_per_user: self.max_layers_per_user,
            total_memory_bytes: self.total_memory_usage(),
            total_access_count: self
                .user_layers
                .values()
                .flat_map(|layers| layers.iter())
                .map(|l| l.access_count)
                .sum(),
        }
    }
}

/// Statistics about user layers
#[derive(Debug, Clone)]
pub struct UserLayerStats {
    /// Number of users with layers
    pub user_count: usize,

    /// Total number of layers across all users
    pub total_layer_count: usize,

    /// Maximum layers per user
    pub max_layers_per_user: usize,

    /// Total memory usage (bytes)
    pub total_memory_bytes: usize,

    /// Total access count across all layers
    pub total_access_count: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_user_layer(user_id: &str, dimension_id: u32) -> UserLayer {
        let dim_id = DimensionId(dimension_id as u8);
        UserLayer::new(
            user_id.to_string(),
            dim_id,
            LayerId { dimension: dim_id, layer: 1 },
            b"Test content".to_vec(),
            vec!["test".to_string(), "keyword".to_string()],
            Frequency(2.5),
            0.87, // Valid confidence for Tier 3
        )
        .unwrap()
    }

    #[test]
    fn test_user_layer_creation() {
        let layer = create_test_user_layer("user_123", 31);
        assert_eq!(layer.user_id, "user_123");
        assert_eq!(layer.dimension_id.0, 31);
        assert_eq!(layer.confidence, 0.87);
    }

    #[test]
    fn test_user_layer_invalid_dimension() {
        let dim_id = DimensionId(30); // Too low
        let result = UserLayer::new(
            "user_123".to_string(),
            dim_id,
            LayerId { dimension: dim_id, layer: 1 },
            b"Test".to_vec(),
            vec![],
            Frequency(2.5),
            0.87,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_user_layer_low_confidence() {
        let dim_id = DimensionId(31);
        let result = UserLayer::new(
            "user_123".to_string(),
            dim_id,
            LayerId { dimension: dim_id, layer: 1 },
            b"Test".to_vec(),
            vec![],
            Frequency(2.5),
            0.80, // Too low for Tier 3
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_user_layer_empty_user_id() {
        let dim_id = DimensionId(31);
        let result = UserLayer::new(
            "".to_string(), // Empty
            dim_id,
            LayerId { dimension: dim_id, layer: 1 },
            b"Test".to_vec(),
            vec![],
            Frequency(2.5),
            0.87,
        );
        assert!(result.is_err());
    }

    #[test]
    fn test_user_layer_touch() {
        let mut layer = create_test_user_layer("user_123", 31);
        let initial_count = layer.access_count;

        layer.touch();

        assert_eq!(layer.access_count, initial_count + 1);
    }

    #[test]
    fn test_user_layer_manager_creation() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        assert_eq!(manager.user_count(), 0);
        assert_eq!(manager.total_layer_count(), 0);
        assert_eq!(manager.max_layers_per_user, 5);
    }

    #[test]
    fn test_user_layer_manager_create() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        let layer = create_test_user_layer("user_123", 31);
        manager.create_user_layer(layer).unwrap();

        assert_eq!(manager.user_count(), 1);
        assert_eq!(manager.total_layer_count(), 1);
    }

    #[test]
    fn test_user_layer_manager_get_user_layers() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        let layer1 = create_test_user_layer("user_123", 31);
        let layer2 = create_test_user_layer("user_123", 32);

        manager.create_user_layer(layer1).unwrap();
        manager.create_user_layer(layer2).unwrap();

        let layers = manager.get_user_layers("user_123");
        assert_eq!(layers.len(), 2);
    }

    #[test]
    fn test_user_layer_manager_isolation() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        let layer_a = create_test_user_layer("user_a", 31);
        let layer_b = create_test_user_layer("user_b", 31);

        manager.create_user_layer(layer_a).unwrap();
        manager.create_user_layer(layer_b).unwrap();

        // User A should only see their layer
        let layers_a = manager.get_user_layers("user_a");
        assert_eq!(layers_a.len(), 1);
        assert_eq!(layers_a[0].user_id, "user_a");

        // User B should only see their layer
        let layers_b = manager.get_user_layers("user_b");
        assert_eq!(layers_b.len(), 1);
        assert_eq!(layers_b[0].user_id, "user_b");
    }

    #[test]
    fn test_user_layer_manager_lru_eviction() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        // Create 5 layers (max)
        for i in 31..=35 {
            let layer = create_test_user_layer("user_123", i);
            manager.create_user_layer(layer).unwrap();
        }

        assert_eq!(manager.get_user_layers("user_123").len(), 5);

        // Access layer 31 to make it recently used
        std::thread::sleep(std::time::Duration::from_millis(10));
        let _ = manager.get_user_layer("user_123", DimensionId(31));

        // Create 6th layer - should trigger LRU eviction
        let new_layer = create_test_user_layer("user_123", 36);
        manager.create_user_layer(new_layer).unwrap();

        // Should still have 5 layers
        assert_eq!(manager.get_user_layers("user_123").len(), 5);

        // Layer 31 should still exist (recently accessed)
        let layer_31 = manager.get_user_layer("user_123", DimensionId(31));
        assert!(layer_31.is_some());
    }

    #[test]
    fn test_user_layer_manager_remove() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        let layer = create_test_user_layer("user_123", 31);
        manager.create_user_layer(layer).unwrap();

        assert_eq!(manager.total_layer_count(), 1);

        manager.remove_user_layer("user_123", DimensionId(31)).unwrap();

        assert_eq!(manager.total_layer_count(), 0);
    }

    #[test]
    fn test_user_layer_manager_clear_user() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        // Create multiple layers for user
        for i in 31..=33 {
            let layer = create_test_user_layer("user_123", i);
            manager.create_user_layer(layer).unwrap();
        }

        assert_eq!(manager.get_user_layers("user_123").len(), 3);

        manager.clear_user("user_123");

        assert_eq!(manager.get_user_layers("user_123").len(), 0);
        assert_eq!(manager.user_count(), 0);
    }

    #[test]
    fn test_user_layer_manager_stats() {
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory, 32 * 1024 * 1024, 0x1000_0000);

        let layer1 = create_test_user_layer("user_a", 31);
        let layer2 = create_test_user_layer("user_b", 31);

        manager.create_user_layer(layer1).unwrap();
        manager.create_user_layer(layer2).unwrap();

        let stats = manager.stats();
        assert_eq!(stats.user_count, 2);
        assert_eq!(stats.total_layer_count, 2);
        assert_eq!(stats.max_layers_per_user, 5);
    }

    #[test]
    fn test_user_layer_persistence_save_load() {
        use std::fs;

        // Clean up test directory
        let _ = fs::remove_dir_all("data/user_layers");

        // Create manager and add layers for multiple users
        let memory = Arc::new(MmapManager::new(280).unwrap());
        let mut manager = UserLayerManager::new(memory.clone(), 32 * 1024 * 1024, 0x1000_0000);

        // User A: 2 layers
        let layer_a1 = create_test_user_layer("user_a", 31);
        let layer_a2 = create_test_user_layer("user_a", 32);

        // User B: 3 layers
        let layer_b1 = create_test_user_layer("user_b", 31);
        let layer_b2 = create_test_user_layer("user_b", 32);
        let layer_b3 = create_test_user_layer("user_b", 33);

        manager.create_user_layer(layer_a1).unwrap();
        manager.create_user_layer(layer_a2).unwrap();
        manager.create_user_layer(layer_b1).unwrap();
        manager.create_user_layer(layer_b2).unwrap();
        manager.create_user_layer(layer_b3).unwrap();

        assert_eq!(manager.user_count(), 2);
        assert_eq!(manager.total_layer_count(), 5);

        // Save to disk
        manager.save().unwrap();

        // Verify directories and files exist
        assert!(std::path::Path::new("data/user_layers/user_a").exists());
        assert!(std::path::Path::new("data/user_layers/user_b").exists());
        assert!(std::path::Path::new("data/user_layers/user_a/c31.json").exists());
        assert!(std::path::Path::new("data/user_layers/user_a/c32.json").exists());
        assert!(std::path::Path::new("data/user_layers/user_b/c31.json").exists());
        assert!(std::path::Path::new("data/user_layers/user_b/c32.json").exists());
        assert!(std::path::Path::new("data/user_layers/user_b/c33.json").exists());

        // Create new manager and load
        let memory2 = Arc::new(MmapManager::new(280).unwrap());
        let mut manager2 = UserLayerManager::new(memory2, 32 * 1024 * 1024, 0x1000_0000);

        assert_eq!(manager2.user_count(), 0); // Empty before load
        assert_eq!(manager2.total_layer_count(), 0);

        manager2.load().unwrap();

        // Verify layers were loaded
        assert_eq!(manager2.user_count(), 2);
        assert_eq!(manager2.total_layer_count(), 5);

        // Verify user A layers
        let user_a_layers = manager2.get_user_layers("user_a");
        assert_eq!(user_a_layers.len(), 2);

        // Verify user B layers
        let user_b_layers = manager2.get_user_layers("user_b");
        assert_eq!(user_b_layers.len(), 3);

        // Verify user isolation
        assert!(manager2.get_user_layer("user_a", DimensionId(31)).is_some());
        assert!(manager2.get_user_layer("user_a", DimensionId(32)).is_some());
        assert!(manager2.get_user_layer("user_b", DimensionId(31)).is_some());
        assert!(manager2.get_user_layer("user_b", DimensionId(32)).is_some());
        assert!(manager2.get_user_layer("user_b", DimensionId(33)).is_some());

        // Cleanup
        let _ = fs::remove_dir_all("data/user_layers");
    }
}
