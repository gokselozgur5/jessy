//! Synesthetic Learning: Keyword association strengthening
//!
//! This module implements the synesthetic learning system that strengthens
//! associations between keywords that co-occur frequently in queries.
//!
//! # Overview
//!
//! When two keywords appear together in a query, their association is strengthened.
//! Unused associations decay over time. This creates a dynamic network of
//! keyword relationships that improves navigation accuracy.
//!
//! # Example
//!
//! ```text
//! Query: "emotional intelligence"
//! → Strengthen association: "emotional" ↔ "intelligence"
//! → Strength increases by 10% (learning rate = 1.1)
//!
//! After 24 hours unused:
//! → Strength decays by 5% (decay rate = 0.95)
//!
//! When strength < 0.1:
//! → Association removed
//! ```

use std::collections::HashMap;
use std::time::SystemTime;
use serde::{Serialize, Deserialize};
use std::path::Path;
use std::fs;
use crate::{Result, ConsciousnessError};

/// Association between two keywords
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct KeywordAssociation {
    /// First keyword
    pub keyword1: String,
    
    /// Second keyword
    pub keyword2: String,
    
    /// Association strength (0.0-∞)
    pub strength: f32,
    
    /// Number of times activated
    pub activation_count: usize,
    
    /// Last activation timestamp
    pub last_activated: SystemTime,
    
    /// Creation timestamp
    pub created_at: SystemTime,
}

impl KeywordAssociation {
    /// Create new association
    pub fn new(keyword1: String, keyword2: String) -> Self {
        let now = SystemTime::now();
        Self {
            keyword1,
            keyword2,
            strength: 1.0, // Initial strength
            activation_count: 1,
            last_activated: now,
            created_at: now,
        }
    }
    
    /// Strengthen association
    pub fn strengthen(&mut self, learning_rate: f32) {
        self.strength *= learning_rate;
        self.activation_count += 1;
        self.last_activated = SystemTime::now();
    }
    
    /// Decay association
    pub fn decay(&mut self, decay_rate: f32) {
        self.strength *= decay_rate;
    }
    
    /// Check if association is weak enough to remove
    pub fn is_weak(&self) -> bool {
        self.strength < 0.1
    }
    
    /// Get days since last activation
    pub fn days_since_activation(&self) -> f64 {
        let now = SystemTime::now();
        let duration = now.duration_since(self.last_activated)
            .unwrap_or_default();
        duration.as_secs() as f64 / 86400.0 // Convert to days
    }
}

/// Synesthetic learner for keyword associations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SynestheticLearner {
    /// Associations indexed by keyword pair
    /// Key format: "keyword1:keyword2" (alphabetically sorted)
    associations: HashMap<String, KeywordAssociation>,

    /// Learning rate (multiplier for strengthening)
    learning_rate: f32,

    /// Decay rate (multiplier for decay per day)
    decay_rate: f32,
}

impl SynestheticLearner {
    /// Create new synesthetic learner
    pub fn new(learning_rate: f32, decay_rate: f32) -> Self {
        Self {
            associations: HashMap::new(),
            learning_rate,
            decay_rate,
        }
    }
    
    /// Strengthen association between two keywords
    ///
    /// If association doesn't exist, creates it with strength 1.0.
    /// If it exists, multiplies strength by learning_rate.
    ///
    /// # Arguments
    ///
    /// * `keyword1` - First keyword
    /// * `keyword2` - Second keyword
    ///
    /// # Performance
    ///
    /// O(1) - HashMap lookup and update
    pub fn strengthen_association(&mut self, keyword1: &str, keyword2: &str) {
        let key = Self::make_key(keyword1, keyword2);
        
        self.associations
            .entry(key.clone())
            .and_modify(|assoc| assoc.strengthen(self.learning_rate))
            .or_insert_with(|| {
                KeywordAssociation::new(
                    keyword1.to_string(),
                    keyword2.to_string(),
                )
            });
    }
    
    /// Get associated keywords for a given keyword
    ///
    /// Returns keywords sorted by association strength (descending).
    ///
    /// # Arguments
    ///
    /// * `keyword` - Keyword to find associations for
    ///
    /// # Returns
    ///
    /// List of (keyword, strength) tuples sorted by strength
    ///
    /// # Performance
    ///
    /// O(n) where n is total number of associations
    pub fn get_associations(&self, keyword: &str) -> Vec<(String, f32)> {
        let mut results: Vec<(String, f32)> = self.associations
            .values()
            .filter_map(|assoc| {
                if assoc.keyword1 == keyword {
                    Some((assoc.keyword2.clone(), assoc.strength))
                } else if assoc.keyword2 == keyword {
                    Some((assoc.keyword1.clone(), assoc.strength))
                } else {
                    None
                }
            })
            .collect();
        
        // Sort by strength descending
        results.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        
        results
    }
    
    /// Decay unused associations
    ///
    /// Applies decay to associations that haven't been activated recently.
    /// Removes associations with strength < 0.1.
    ///
    /// # Performance
    ///
    /// O(n) where n is total number of associations
    pub fn decay_unused(&mut self) {
        // Apply decay to associations unused for >24 hours
        for assoc in self.associations.values_mut() {
            let days_unused = assoc.days_since_activation();
            if days_unused >= 1.0 {
                // Apply decay for each day
                let decay_factor = self.decay_rate.powi(days_unused.floor() as i32);
                assoc.decay(decay_factor);
            }
        }
        
        // Remove weak associations
        self.associations.retain(|_, assoc| !assoc.is_weak());
    }
    
    /// Get total number of associations
    pub fn association_count(&self) -> usize {
        self.associations.len()
    }
    
    /// Get association strength between two keywords
    pub fn get_strength(&self, keyword1: &str, keyword2: &str) -> Option<f32> {
        let key = Self::make_key(keyword1, keyword2);
        self.associations.get(&key).map(|assoc| assoc.strength)
    }
    
    /// Make key for association lookup (alphabetically sorted)
    fn make_key(keyword1: &str, keyword2: &str) -> String {
        if keyword1 < keyword2 {
            format!("{}:{}", keyword1, keyword2)
        } else {
            format!("{}:{}", keyword2, keyword1)
        }
    }

    /// Save synesthetic associations to JSON file
    ///
    /// # Arguments
    ///
    /// * `path` - Path to save file (e.g., "data/synesthetic_associations.json")
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Parent directory doesn't exist or can't be created
    /// - File can't be written
    /// - Serialization fails
    pub fn save<P: AsRef<Path>>(&self, path: P) -> Result<()> {
        let path = path.as_ref();

        // Create parent directory if it doesn't exist
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent).map_err(|e| {
                ConsciousnessError::LearningError(format!(
                    "Failed to create directory {:?}: {}",
                    parent, e
                ))
            })?;
        }

        // Serialize to JSON with pretty formatting
        let json = serde_json::to_string_pretty(self).map_err(|e| {
            ConsciousnessError::LearningError(format!(
                "Failed to serialize synesthetic associations: {}",
                e
            ))
        })?;

        // Write to file
        fs::write(path, json).map_err(|e| {
            ConsciousnessError::LearningError(format!(
                "Failed to write synesthetic associations to {:?}: {}",
                path, e
            ))
        })?;

        eprintln!(
            "[SynestheticLearner] Saved {} associations to {:?}",
            self.association_count(),
            path
        );

        Ok(())
    }

    /// Load synesthetic associations from JSON file
    ///
    /// # Arguments
    ///
    /// * `path` - Path to load file from
    ///
    /// # Returns
    ///
    /// Returns loaded learner if file exists, or new learner if file doesn't exist
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - File exists but can't be read
    /// - Deserialization fails
    pub fn load<P: AsRef<Path>>(path: P) -> Result<Self> {
        let path = path.as_ref();

        // If file doesn't exist, return new learner
        if !path.exists() {
            eprintln!(
                "[SynestheticLearner] No saved associations found at {:?}, starting fresh",
                path
            );
            return Ok(Self::new(1.1, 0.95));
        }

        // Read file
        let json = fs::read_to_string(path).map_err(|e| {
            ConsciousnessError::LearningError(format!(
                "Failed to read synesthetic associations from {:?}: {}",
                path, e
            ))
        })?;

        // Deserialize
        let learner: Self = serde_json::from_str(&json).map_err(|e| {
            ConsciousnessError::LearningError(format!(
                "Failed to deserialize synesthetic associations: {}",
                e
            ))
        })?;

        eprintln!(
            "[SynestheticLearner] Loaded {} associations from {:?}",
            learner.association_count(),
            path
        );

        Ok(learner)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_new_association() {
        // Given: Two keywords
        let assoc = KeywordAssociation::new("emotion".to_string(), "feeling".to_string());
        
        // Then: Should initialize correctly
        assert_eq!(assoc.keyword1, "emotion");
        assert_eq!(assoc.keyword2, "feeling");
        assert_eq!(assoc.strength, 1.0);
        assert_eq!(assoc.activation_count, 1);
    }
    
    #[test]
    fn test_strengthen_association() {
        // Given: Association
        let mut assoc = KeywordAssociation::new("test".to_string(), "example".to_string());
        let initial_strength = assoc.strength;
        
        // When: Strengthening
        assoc.strengthen(1.1);
        
        // Then: Strength should increase by 10%
        assert!((assoc.strength - initial_strength * 1.1).abs() < 0.001);
        assert_eq!(assoc.activation_count, 2);
    }
    
    #[test]
    fn test_decay_association() {
        // Given: Association with strength 1.0
        let mut assoc = KeywordAssociation::new("test".to_string(), "example".to_string());
        
        // When: Decaying
        assoc.decay(0.95);
        
        // Then: Strength should decrease by 5%
        assert!((assoc.strength - 0.95).abs() < 0.001);
    }
    
    #[test]
    fn test_is_weak() {
        // Given: Associations with different strengths
        let mut strong = KeywordAssociation::new("a".to_string(), "b".to_string());
        strong.strength = 0.5;
        
        let mut weak = KeywordAssociation::new("c".to_string(), "d".to_string());
        weak.strength = 0.05;
        
        // Then: Should correctly identify weak associations
        assert!(!strong.is_weak());
        assert!(weak.is_weak());
    }
    
    #[test]
    fn test_synesthetic_learner_creation() {
        // Given: Learning parameters
        let learner = SynestheticLearner::new(1.1, 0.95);
        
        // Then: Should initialize correctly
        assert_eq!(learner.learning_rate, 1.1);
        assert_eq!(learner.decay_rate, 0.95);
        assert_eq!(learner.association_count(), 0);
    }
    
    #[test]
    fn test_strengthen_new_association() {
        // Given: Learner
        let mut learner = SynestheticLearner::new(1.1, 0.95);
        
        // When: Strengthening new association
        learner.strengthen_association("emotion", "feeling");
        
        // Then: Should create association with strength 1.0
        assert_eq!(learner.association_count(), 1);
        let strength = learner.get_strength("emotion", "feeling");
        assert!(strength.is_some());
        assert_eq!(strength.unwrap(), 1.0);
    }
    
    #[test]
    fn test_strengthen_existing_association() {
        // Given: Learner with existing association
        let mut learner = SynestheticLearner::new(1.1, 0.95);
        learner.strengthen_association("emotion", "feeling");
        
        // When: Strengthening again
        learner.strengthen_association("emotion", "feeling");
        
        // Then: Strength should increase
        let strength = learner.get_strength("emotion", "feeling").unwrap();
        assert!((strength - 1.1).abs() < 0.001);
    }
    
    #[test]
    fn test_get_associations() {
        // Given: Learner with multiple associations
        let mut learner = SynestheticLearner::new(1.1, 0.95);
        learner.strengthen_association("emotion", "feeling");
        learner.strengthen_association("emotion", "sentiment");
        learner.strengthen_association("emotion", "mood");
        
        // Strengthen "feeling" more
        learner.strengthen_association("emotion", "feeling");
        
        // When: Getting associations for "emotion"
        let associations = learner.get_associations("emotion");
        
        // Then: Should return all associated keywords sorted by strength
        assert_eq!(associations.len(), 3);
        assert_eq!(associations[0].0, "feeling"); // Strongest (1.1)
        assert!(associations[0].1 > associations[1].1);
    }
    
    #[test]
    fn test_get_associations_empty() {
        // Given: Learner with no associations
        let learner = SynestheticLearner::new(1.1, 0.95);
        
        // When: Getting associations
        let associations = learner.get_associations("unknown");
        
        // Then: Should return empty list
        assert_eq!(associations.len(), 0);
    }
    
    #[test]
    fn test_key_ordering() {
        // Given: Two keywords in different orders
        let key1 = SynestheticLearner::make_key("alpha", "beta");
        let key2 = SynestheticLearner::make_key("beta", "alpha");
        
        // Then: Should produce same key
        assert_eq!(key1, key2);
        assert_eq!(key1, "alpha:beta");
    }
    
    #[test]
    fn test_decay_unused_removes_weak() {
        // Given: Learner with weak association
        let mut learner = SynestheticLearner::new(1.1, 0.95);
        learner.strengthen_association("test", "example");
        
        // Manually set strength to weak value
        let key = SynestheticLearner::make_key("test", "example");
        if let Some(assoc) = learner.associations.get_mut(&key) {
            assoc.strength = 0.05; // Below 0.1 threshold
        }
        
        // When: Decaying
        learner.decay_unused();
        
        // Then: Weak association should be removed
        assert_eq!(learner.association_count(), 0);
    }
}
