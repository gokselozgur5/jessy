//! Pattern Cache for Instant Responses
//!
//! This module implements a cache for frequently asked questions
//! to provide instant responses without LLM calls.
//!
//! # Overview
//!
//! When the same or similar questions are asked repeatedly,
//! we cache the response and return it instantly (<100ms).
//!
//! # Strategy
//!
//! - Key: Normalized query (lowercase, trimmed, stopwords removed)
//! - Value: Cached response + metadata
//! - TTL: 24 hours (configurable)
//! - Max size: 1000 entries (LRU eviction)
//!
//! # Example
//!
//! ```text
//! Query 1: "What is consciousness?"
//! → Cache miss, call LLM, store response
//!
//! Query 2: "what is consciousness"
//! → Cache hit! Return instantly
//!
//! Query 3: "tell me about consciousness"
//! → Cache miss (different wording), call LLM
//! ```

use std::collections::HashMap;
use std::time::{SystemTime, Duration};
use serde::{Serialize, Deserialize};

/// Cached response entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CachedResponse {
    /// The response text
    pub response: String,

    /// When it was cached
    pub cached_at: SystemTime,

    /// Hit count (how many times served from cache)
    pub hit_count: usize,

    /// Dimensions that were activated
    pub dimensions: Vec<crate::DimensionId>,

    /// Confidence score (0.0-1.0)
    pub confidence: f32,
}

impl CachedResponse {
    /// Create new cached response
    pub fn new(
        response: String,
        dimensions: Vec<crate::DimensionId>,
        confidence: f32,
    ) -> Self {
        Self {
            response,
            cached_at: SystemTime::now(),
            hit_count: 0,
            dimensions,
            confidence,
        }
    }

    /// Check if entry is expired
    pub fn is_expired(&self, ttl: Duration) -> bool {
        SystemTime::now()
            .duration_since(self.cached_at)
            .unwrap_or_default()
            > ttl
    }

    /// Increment hit count
    pub fn increment_hits(&mut self) {
        self.hit_count += 1;
    }
}

/// Pattern cache configuration
#[derive(Debug, Clone)]
pub struct PatternCacheConfig {
    /// Maximum number of cached entries
    pub max_entries: usize,

    /// Time-to-live for cached entries
    pub ttl: Duration,

    /// Enable cache (can be disabled for testing)
    pub enabled: bool,
}

impl Default for PatternCacheConfig {
    fn default() -> Self {
        Self {
            max_entries: 1000,
            ttl: Duration::from_secs(24 * 60 * 60), // 24 hours
            enabled: true,
        }
    }
}

/// Pattern cache for instant responses
pub struct PatternCache {
    /// Cache storage (normalized_query -> cached_response)
    cache: HashMap<String, CachedResponse>,

    /// Configuration
    config: PatternCacheConfig,

    /// Cache statistics
    hits: usize,
    misses: usize,
}

impl PatternCache {
    /// Create new pattern cache
    pub fn new(config: PatternCacheConfig) -> Self {
        Self {
            cache: HashMap::new(),
            config,
            hits: 0,
            misses: 0,
        }
    }

    /// Create with default config
    pub fn default() -> Self {
        Self::new(PatternCacheConfig::default())
    }

    /// Normalize query for cache key
    ///
    /// - Lowercase
    /// - Trim whitespace
    /// - Remove extra spaces
    /// - Remove common stopwords (optional)
    fn normalize_query(&self, query: &str) -> String {
        query
            .to_lowercase()
            .trim()
            .split_whitespace()
            .collect::<Vec<_>>()
            .join(" ")
    }

    /// Get cached response if available
    ///
    /// Returns None if:
    /// - Cache disabled
    /// - Entry not found
    /// - Entry expired
    pub fn get(&mut self, query: &str) -> Option<CachedResponse> {
        if !self.config.enabled {
            return None;
        }

        let key = self.normalize_query(query);

        // Check if entry exists and not expired
        let should_remove = if let Some(entry) = self.cache.get(&key) {
            entry.is_expired(self.config.ttl)
        } else {
            self.misses += 1;
            return None;
        };

        if should_remove {
            self.cache.remove(&key);
            self.misses += 1;
            return None;
        }

        // Now we can safely get_mut and increment
        if let Some(entry) = self.cache.get_mut(&key) {
            entry.increment_hits();
            self.hits += 1;

            let hit_count = entry.hit_count;
            let result = entry.clone();

            // Calculate hit rate after we're done with the mutable borrow
            let hit_rate = self.hit_rate() * 100.0;

            eprintln!(
                "[PatternCache] HIT: '{}' (hits: {}, total hit rate: {:.1}%)",
                key,
                hit_count,
                hit_rate
            );

            Some(result)
        } else {
            self.misses += 1;
            None
        }
    }

    /// Store response in cache
    ///
    /// If cache is full, evicts least recently used entry.
    pub fn put(
        &mut self,
        query: &str,
        response: String,
        dimensions: Vec<crate::DimensionId>,
        confidence: f32,
    ) {
        if !self.config.enabled {
            return;
        }

        let key = self.normalize_query(query);

        // Evict if at capacity
        if self.cache.len() >= self.config.max_entries {
            // Simple LRU: remove oldest entry (by cached_at)
            if let Some(oldest_key) = self.cache
                .iter()
                .min_by_key(|(_, entry)| entry.cached_at)
                .map(|(k, _)| k.clone())
            {
                self.cache.remove(&oldest_key);
                eprintln!("[PatternCache] Evicted oldest entry: '{}'", oldest_key);
            }
        }

        // Store new entry
        let entry = CachedResponse::new(response, dimensions, confidence);
        self.cache.insert(key.clone(), entry);

        eprintln!(
            "[PatternCache] Cached: '{}' (size: {}/{})",
            key,
            self.cache.len(),
            self.config.max_entries
        );
    }

    /// Clear all cached entries
    pub fn clear(&mut self) {
        self.cache.clear();
        eprintln!("[PatternCache] Cleared all entries");
    }

    /// Get cache size
    pub fn size(&self) -> usize {
        self.cache.len()
    }

    /// Get hit rate (0.0-1.0)
    pub fn hit_rate(&self) -> f32 {
        let total = self.hits + self.misses;
        if total == 0 {
            0.0
        } else {
            self.hits as f32 / total as f32
        }
    }

    /// Get cache statistics
    pub fn stats(&self) -> PatternCacheStats {
        PatternCacheStats {
            size: self.cache.len(),
            max_size: self.config.max_entries,
            hits: self.hits,
            misses: self.misses,
            hit_rate: self.hit_rate(),
        }
    }
}

/// Pattern cache statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PatternCacheStats {
    /// Current cache size
    pub size: usize,

    /// Maximum cache size
    pub max_size: usize,

    /// Total cache hits
    pub hits: usize,

    /// Total cache misses
    pub misses: usize,

    /// Hit rate (0.0-1.0)
    pub hit_rate: f32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::DimensionId;

    #[test]
    fn test_normalize_query() {
        let cache = PatternCache::default();

        assert_eq!(
            cache.normalize_query("What is consciousness?"),
            "what is consciousness?"
        );

        assert_eq!(
            cache.normalize_query("  WHAT   IS   CONSCIOUSNESS  "),
            "what is consciousness"
        );
    }

    #[test]
    fn test_cache_miss() {
        let mut cache = PatternCache::default();

        let result = cache.get("unknown query");
        assert!(result.is_none());
    }

    #[test]
    fn test_cache_hit() {
        let mut cache = PatternCache::default();

        // Store
        cache.put(
            "test query",
            "test response".to_string(),
            vec![DimensionId(1)],
            0.9,
        );

        // Retrieve
        let result = cache.get("test query");
        assert!(result.is_some());

        let cached = result.unwrap();
        assert_eq!(cached.response, "test response");
        assert_eq!(cached.hit_count, 1);
    }

    #[test]
    fn test_case_insensitive() {
        let mut cache = PatternCache::default();

        cache.put(
            "Test Query",
            "response".to_string(),
            vec![],
            0.9,
        );

        // Should hit with different case
        let result = cache.get("test query");
        assert!(result.is_some());
    }

    #[test]
    fn test_hit_count_increments() {
        let mut cache = PatternCache::default();

        cache.put("query", "response".to_string(), vec![], 0.9);

        cache.get("query");
        cache.get("query");
        cache.get("query");

        let result = cache.get("query").unwrap();
        assert_eq!(result.hit_count, 4); // 3 gets + 1 final get
    }

    #[test]
    fn test_lru_eviction() {
        let config = PatternCacheConfig {
            max_entries: 2,
            ..Default::default()
        };
        let mut cache = PatternCache::new(config);

        // Fill cache
        cache.put("query1", "response1".to_string(), vec![], 0.9);
        std::thread::sleep(std::time::Duration::from_millis(10));
        cache.put("query2", "response2".to_string(), vec![], 0.9);

        assert_eq!(cache.size(), 2);

        // Add third entry (should evict oldest)
        cache.put("query3", "response3".to_string(), vec![], 0.9);

        assert_eq!(cache.size(), 2);
        assert!(cache.get("query1").is_none()); // Evicted
        assert!(cache.get("query2").is_some());
        assert!(cache.get("query3").is_some());
    }

    #[test]
    fn test_hit_rate() {
        let mut cache = PatternCache::default();

        cache.put("query", "response".to_string(), vec![], 0.9);

        cache.get("query"); // Hit
        cache.get("unknown"); // Miss
        cache.get("query"); // Hit

        assert_eq!(cache.stats().hits, 2);
        assert_eq!(cache.stats().misses, 1);
        assert!((cache.hit_rate() - 0.666).abs() < 0.01);
    }

    #[test]
    fn test_disabled_cache() {
        let config = PatternCacheConfig {
            enabled: false,
            ..Default::default()
        };
        let mut cache = PatternCache::new(config);

        cache.put("query", "response".to_string(), vec![], 0.9);

        // Should not store when disabled
        assert_eq!(cache.size(), 0);
        assert!(cache.get("query").is_none());
    }
}
