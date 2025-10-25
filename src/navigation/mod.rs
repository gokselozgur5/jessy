//! Multiverse navigation system for consciousness dimensions
//!
//! This module implements parallel dimension scanning, synesthetic keyword matching,
//! and path selection for navigating through consciousness layers.

pub mod types;
pub mod registry;
pub mod navigator;
pub mod synesthetic;
pub mod path_selector;
pub mod query_analyzer;

// Re-export core types
pub use types::{
    NavigationConfig, NavigationError, ProtoDimensionId, QuestionType, SystemState, UrgencyLevel,
};

pub use registry::{DimensionMetadata, LayerMetadata, DimensionRegistry};
pub use navigator::MultiverseNavigator;
pub use synesthetic::SynestheticEngine;
pub use path_selector::PathSelector;
pub use query_analyzer::QueryAnalyzer;

use crate::{DimensionId, LayerId, Frequency};
use std::collections::HashMap;

/// Query analysis result
#[derive(Debug, Clone)]
pub struct QueryAnalysis {
    pub raw_query: String,
    pub keywords: Vec<String>,
    pub estimated_complexity: f32,
    pub emotional_indicators: Vec<String>,
    pub technical_indicators: Vec<String>,
    pub question_type: QuestionType,
    pub urgency_level: UrgencyLevel,
    pub estimated_frequency: f32,
}

/// Navigation path through dimensional layers
#[derive(Debug, Clone)]
pub struct NavigationPath {
    pub dimension_id: DimensionId,
    pub layer_sequence: Vec<LayerId>,
    pub confidence: f32,
    pub frequency: Frequency,
    pub keywords_matched: Vec<String>,
    pub synesthetic_score: f32,
}

impl NavigationPath {
    /// Create new navigation path
    pub fn new(dimension_id: DimensionId, frequency: Frequency) -> Self {
        Self {
            dimension_id,
            layer_sequence: Vec::new(),
            confidence: 0.0,
            frequency,
            keywords_matched: Vec::new(),
            synesthetic_score: 0.0,
        }
    }
    
    /// Add layer to the path
    pub fn add_layer(&mut self, layer_id: LayerId, confidence_boost: f32) {
        self.layer_sequence.push(layer_id);
        self.confidence += confidence_boost;
    }
    
    /// Add matched keyword
    pub fn add_keyword(&mut self, keyword: String, synesthetic_score: f32) {
        self.keywords_matched.push(keyword);
        self.synesthetic_score += synesthetic_score;
    }
    
    /// Get the depth of this path
    pub fn depth(&self) -> usize {
        self.layer_sequence.len()
    }
    
    /// Check if path is strong enough for activation
    pub fn is_viable(&self, min_confidence: f32) -> bool {
        self.confidence >= min_confidence && !self.layer_sequence.is_empty()
    }
    
    /// Get the final layer in the path
    pub fn final_layer(&self) -> Option<LayerId> {
        self.layer_sequence.last().copied()
    }
}

/// Result of navigation across all dimensions
#[derive(Debug)]
pub struct NavigationResult {
    pub query_analysis: QueryAnalysis,
    pub paths: Vec<NavigationPath>,
    pub dimensions: Vec<DimensionId>,
    pub frequencies: Vec<Frequency>,
    pub total_confidence: f32,
    pub complexity_score: f32,
    pub return_to_source_triggered: bool,
}

impl NavigationResult {
    /// Create new navigation result
    pub fn new(query_analysis: QueryAnalysis) -> Self {
        Self {
            query_analysis,
            paths: Vec::new(),
            dimensions: Vec::new(),
            frequencies: Vec::new(),
            total_confidence: 0.0,
            complexity_score: 0.0,
            return_to_source_triggered: false,
        }
    }
    
    /// Add navigation path to result
    pub fn add_path(&mut self, path: NavigationPath) {
        if path.is_viable(0.3) {
            self.dimensions.push(path.dimension_id);
            self.frequencies.push(path.frequency);
            self.total_confidence += path.confidence;
            self.paths.push(path);
        }
    }
    
    /// Calculate complexity score based on active dimensions and depth
    pub fn calculate_complexity(&mut self) {
        let dimension_count = self.dimensions.len() as f32;
        let avg_depth = if self.paths.is_empty() {
            0.0
        } else {
            self.paths.iter().map(|p| p.depth() as f32).sum::<f32>() / self.paths.len() as f32
        };
        
        // Complexity increases with both dimension count and average depth
        self.complexity_score = dimension_count * 0.5 + avg_depth * 0.3;
    }
    
    /// Check if return-to-source should be triggered
    pub fn should_return_to_source(&self, threshold: usize) -> bool {
        self.dimensions.len() > threshold || self.complexity_score > 5.0
    }
    
    /// Get paths sorted by confidence
    pub fn paths_by_confidence(&self) -> Vec<&NavigationPath> {
        let mut sorted = self.paths.iter().collect::<Vec<_>>();
        sorted.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());
        sorted
    }
    
    /// Get the dominant frequency from all activated paths
    pub fn dominant_frequency(&self) -> Option<Frequency> {
        if self.frequencies.is_empty() {
            return None;
        }
        
        // Simple average for now - would be replaced by interference calculation
        let avg = self.frequencies.iter().map(|f| f.hz()).sum::<f32>() / self.frequencies.len() as f32;
        Some(Frequency::new(avg))
    }
}

/// Keyword matching result
#[derive(Debug, Clone)]
pub struct KeywordMatch {
    pub keyword: String,
    pub match_type: MatchType,
    pub confidence: f32,
    pub synesthetic_associations: Vec<String>,
}

/// Type of keyword match
#[derive(Debug, Clone, PartialEq)]
pub enum MatchType {
    Literal,        // Direct keyword match
    Synesthetic,    // Cross-sensory association match
    Semantic,       // Meaning-based match
    Contextual,     // Context-dependent match
}

impl KeywordMatch {
    /// Create new keyword match
    pub fn new(keyword: String, match_type: MatchType, confidence: f32) -> Self {
        Self {
            keyword,
            match_type,
            confidence,
            synesthetic_associations: Vec::new(),
        }
    }
    
    /// Add synesthetic association
    pub fn add_association(&mut self, association: String) {
        self.synesthetic_associations.push(association);
    }
    
    /// Check if this is a strong match
    pub fn is_strong(&self) -> bool {
        self.confidence > 0.7
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_navigation_path() {
        let mut path = NavigationPath::new(DimensionId(1), Frequency::new(1.5));
        
        assert_eq!(path.depth(), 0);
        assert!(!path.is_viable(0.5));
        
        path.add_layer(LayerId { dimension: DimensionId(1), layer: 0 }, 0.8);
        path.add_keyword("test".to_string(), 0.5);
        
        assert_eq!(path.depth(), 1);
        assert!(path.is_viable(0.5));
        assert_eq!(path.keywords_matched.len(), 1);
    }
    
    #[test]
    fn test_navigation_result() {
        let analysis = QueryAnalysis {
            raw_query: "test query".to_string(),
            keywords: vec!["test".to_string()],
            estimated_complexity: 1.0,
            emotional_indicators: vec![],
            technical_indicators: vec![],
            question_type: QuestionType::Factual,
            urgency_level: UrgencyLevel::Low,
            estimated_frequency: 0.5,
        };
        
        let mut result = NavigationResult::new(analysis);
        
        let mut path = NavigationPath::new(DimensionId(1), Frequency::new(1.5));
        path.add_layer(LayerId { dimension: DimensionId(1), layer: 0 }, 0.8);
        
        result.add_path(path);
        result.calculate_complexity();
        
        assert_eq!(result.dimensions.len(), 1);
        assert_eq!(result.paths.len(), 1);
        assert!(!result.should_return_to_source(6));
    }
    
    #[test]
    fn test_keyword_match() {
        let mut keyword_match = KeywordMatch::new(
            "empathy".to_string(),
            MatchType::Literal,
            0.9,
        );
        
        assert!(keyword_match.is_strong());
        
        keyword_match.add_association("understanding".to_string());
        assert_eq!(keyword_match.synesthetic_associations.len(), 1);
    }
}