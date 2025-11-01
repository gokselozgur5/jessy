//! Core types for the navigation system
//!
//! This module defines all fundamental types used throughout the navigation system,
//! including identifiers, enums, error types, and configuration structures.

use crate::{DimensionId, LayerId};
use std::fmt;
use thiserror::Error;

/// Identifier for proto-dimensions (emergent dimensions not yet promoted to permanent status)
/// Proto-dimensions use IDs starting from 100 to avoid conflicts with core dimensions (1-14)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct ProtoDimensionId(pub u16);

impl ProtoDimensionId {
    /// Create a new proto-dimension ID
    /// Proto-dimension IDs must be >= 100 to avoid conflicts with core dimensions
    pub fn new(id: u16) -> Result<Self, NavigationError> {
        if id < 100 {
            return Err(NavigationError::InvalidProtoDimensionId { 
                id, 
                reason: "Proto-dimension IDs must be >= 100".to_string() 
            });
        }
        Ok(Self(id))
    }
    
    /// Get the raw ID value
    pub fn value(&self) -> u16 {
        self.0
    }
}

/// Type of question being asked
/// 
/// Classification is based on indicator analysis:
/// - Emotional: >50% emotional indicators
/// - Technical: >50% technical indicators  
/// - Philosophical: Contains philosophical keywords, no dominant type
/// - Factual: Contains interrogative words, no dominant type
/// - Mixed: Multiple types with equal dominance
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum QuestionType {
    /// Emotional query (e.g., "I feel anxious")
    Emotional,
    
    /// Technical query (e.g., "Debug this code")
    Technical,
    
    /// Philosophical query (e.g., "What is the meaning of consciousness?")
    Philosophical,
    
    /// Factual query (e.g., "What is X?")
    Factual,
    
    /// Mixed query with multiple dominant types
    Mixed,
}

impl fmt::Display for QuestionType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            QuestionType::Emotional => write!(f, "Emotional"),
            QuestionType::Technical => write!(f, "Technical"),
            QuestionType::Philosophical => write!(f, "Philosophical"),
            QuestionType::Factual => write!(f, "Factual"),
            QuestionType::Mixed => write!(f, "Mixed"),
        }
    }
}

/// Urgency level of a query
///
/// Determines base frequency and processing priority:
/// - Low: 0.5 Hz base frequency (contemplative)
/// - Medium: 2.0 Hz base frequency (balanced)
/// - High: 3.5 Hz base frequency (urgent)
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum UrgencyLevel {
    /// Low urgency - general curiosity, learning (0.5 Hz base)
    Low,
    
    /// Medium urgency - problem-solving, planning (2.0 Hz base)
    Medium,
    
    /// High urgency - urgent problems, critical situations (3.5 Hz base)
    High,
}

impl UrgencyLevel {
    /// Get the base frequency for this urgency level
    pub fn base_frequency(&self) -> f32 {
        match self {
            UrgencyLevel::Low => 0.5,
            UrgencyLevel::Medium => 2.0,
            UrgencyLevel::High => 3.5,
        }
    }
}

impl fmt::Display for UrgencyLevel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            UrgencyLevel::Low => write!(f, "Low"),
            UrgencyLevel::Medium => write!(f, "Medium"),
            UrgencyLevel::High => write!(f, "High"),
        }
    }
}

/// System state for lifecycle management
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum SystemState {
    /// System has not been initialized yet
    Uninitialized,
    
    /// System is currently initializing (loading registries, vocabularies, etc.)
    Initializing,
    
    /// System is ready to process navigation requests
    Ready,
    
    /// System is shutting down (persisting state, cleaning up resources)
    ShuttingDown,
    
    /// System initialization or operation failed
    Failed,
}

impl fmt::Display for SystemState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SystemState::Uninitialized => write!(f, "Uninitialized"),
            SystemState::Initializing => write!(f, "Initializing"),
            SystemState::Ready => write!(f, "Ready"),
            SystemState::ShuttingDown => write!(f, "ShuttingDown"),
            SystemState::Failed => write!(f, "Failed"),
        }
    }
}

/// Navigation system errors
///
/// Comprehensive error types covering all failure modes in the navigation system.
/// Each error includes context to aid debugging and error recovery.
#[derive(Error, Debug, Clone)]
pub enum NavigationError {
    /// Query validation failed - empty query
    #[error("Invalid query: query string is empty")]
    EmptyQuery,
    
    /// Query validation failed - exceeds maximum length
    #[error("Invalid query: length {length} exceeds maximum {max_length} characters")]
    QueryTooLong { length: usize, max_length: usize },
    
    /// Query validation failed - contains invalid characters
    #[error("Invalid query: contains invalid characters: {details}")]
    InvalidCharacters { details: String },
    
    /// Dimension scan operation timed out
    #[error("Dimension scan timeout after {duration_ms}ms, completed {completed_count}/{total_count} dimensions")]
    ScanTimeout {
        duration_ms: u64,
        completed_count: usize,
        total_count: usize,
    },
    
    /// No dimensions activated above confidence threshold
    #[error("Insufficient dimension matches: no activations above threshold {threshold} for query: {query}")]
    InsufficientMatches { threshold: f32, query: String },
    
    /// Single dimension scan failed
    #[error("Dimension scan failed for dimension {dimension_id}: {reason}")]
    DimensionScanFailed {
        dimension_id: u8,
        reason: String,
    },
    
    /// Dimension not found in registry
    #[error("Dimension {dimension_id} not found in registry")]
    DimensionNotFound { dimension_id: u8 },
    
    /// Layer not found in dimension
    #[error("Layer {layer_id} not found in dimension {dimension_id}")]
    LayerNotFound {
        dimension_id: u8,
        layer_id: u16,
    },
    
    /// Invalid proto-dimension ID
    #[error("Invalid proto-dimension ID {id}: {reason}")]
    InvalidProtoDimensionId { id: u16, reason: String },
    
    /// System initialization failed
    #[error("Initialization failed: {reason}")]
    InitializationFailed { reason: String },
    
    /// System not in ready state
    #[error("System not ready: current state is {state}")]
    SystemNotReady { state: SystemState },
    
    /// Vocabulary loading failed
    #[error("Failed to load vocabulary '{vocabulary_name}': {reason}")]
    VocabularyLoadFailed {
        vocabulary_name: String,
        reason: String,
    },
    
    /// Vocabulary validation failed
    #[error("Vocabulary validation failed for '{vocabulary_name}': {reason}")]
    VocabularyValidationFailed {
        vocabulary_name: String,
        reason: String,
    },
    
    /// Dimension registry loading failed
    #[error("Failed to load dimension registry: {reason}")]
    RegistryLoadFailed { reason: String },
    
    /// Dimension registry validation failed
    #[error("Registry validation failed: {reason}")]
    RegistryValidationFailed { reason: String },
    
    /// Configuration error
    #[error("Configuration error: {reason}")]
    ConfigurationError { reason: String },
    
    /// Association memory limit exceeded
    #[error("Association memory limit exceeded: {current_count} >= {max_count}")]
    AssociationLimitExceeded {
        current_count: usize,
        max_count: usize,
    },
    
    /// Frequency calculation error
    #[error("Frequency calculation failed: {reason}")]
    FrequencyCalculationFailed { reason: String },
    
    /// Path selection error
    #[error("Path selection failed: {reason}")]
    PathSelectionFailed { reason: String },
    
    /// Depth navigation error
    #[error("Depth navigation failed for dimension {dimension_id}: {reason}")]
    DepthNavigationFailed {
        dimension_id: u8,
        reason: String,
    },
    
    /// Generic navigation error with context
    #[error("Navigation error: {message}")]
    NavigationFailed { message: String },
    
    /// I/O error during file operations
    #[error("I/O error: {message}")]
    IoError { message: String },
    
    /// Serialization/deserialization error
    #[error("Serialization error: {message}")]
    SerializationError { message: String },

    /// Invalid dimension count for LLM selector
    #[error("Invalid dimension count: {count}, expected {expected}")]
    InvalidDimensionCount { count: usize, expected: String },

    /// External service (LLM API) error
    #[error("External service '{service}' error: {details}")]
    ExternalServiceError { service: String, details: String },

    /// Response parsing error
    #[error("Parsing error: {details}")]
    ParsingError { details: String },
}

impl NavigationError {
    /// Check if this error is recoverable
    pub fn is_recoverable(&self) -> bool {
        matches!(
            self,
            NavigationError::ScanTimeout { .. }
                | NavigationError::DimensionScanFailed { .. }
                | NavigationError::InsufficientMatches { .. }
        )
    }
    
    /// Check if this error indicates a validation failure
    pub fn is_validation_error(&self) -> bool {
        matches!(
            self,
            NavigationError::EmptyQuery
                | NavigationError::QueryTooLong { .. }
                | NavigationError::InvalidCharacters { .. }
        )
    }
    
    /// Check if this error indicates an initialization failure
    pub fn is_initialization_error(&self) -> bool {
        matches!(
            self,
            NavigationError::InitializationFailed { .. }
                | NavigationError::VocabularyLoadFailed { .. }
                | NavigationError::VocabularyValidationFailed { .. }
                | NavigationError::RegistryLoadFailed { .. }
                | NavigationError::RegistryValidationFailed { .. }
        )
    }
}

/// Navigation system configuration
///
/// Controls all tunable parameters for the navigation system.
/// Default values are optimized for the Phase 1 MVP requirements.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NavigationConfig {
    /// Scan timeout in milliseconds (default: 100ms)
    /// Requirement 2.2: Complete scan within 100ms for 95% of requests
    pub scan_timeout_ms: u64,
    
    /// Minimum confidence threshold for dimension activation (default: 0.1)
    /// Lowered from 0.3 to allow more dimension matches for common queries
    pub confidence_threshold: f32,
    
    /// Maximum number of dimensions to activate (default: 8)
    /// Requirement 4.3: Limit selection to max 8 dimensions
    pub max_dimensions: usize,
    
    /// Complexity threshold for return-to-source (default: 6)
    /// Requirement 6.2: Trigger return-to-source if >6 dimensions activated
    pub complexity_threshold: usize,
    
    /// Maximum depth for layer navigation (default: 4)
    /// Requirement 5.3: Limit depth to 4 layers (L0→L1→L2→L3)
    pub max_depth: usize,
    
    /// Maximum keywords to extract from query (default: 50)
    /// Requirement 1.5: Limit to 50 keywords
    pub max_keywords: usize,
    
    /// Maximum query length in characters (default: 10,000)
    /// Requirement 9.7: Reject queries exceeding 10,000 characters
    pub max_query_length: usize,
    
    /// Keyword match weight in confidence calculation (default: 0.5)
    /// Requirement 4.7: 50% weight for keyword match score
    pub keyword_match_weight: f32,
    
    /// Synesthetic score weight in confidence calculation (default: 0.3)
    /// Requirement 4.7: 30% weight for synesthetic score
    pub synesthetic_weight: f32,
    
    /// Frequency alignment weight in confidence calculation (default: 0.2)
    /// Requirement 4.7: 20% weight for frequency alignment
    pub frequency_alignment_weight: f32,
    
    /// Association strength multiplier for learning (default: 1.1)
    /// Requirement 3.3: Multiply by 1.1 when strengthening
    pub association_strengthen_factor: f32,
    
    /// Association strength multiplier for decay (default: 0.95)
    /// Requirement 3.4: Multiply by 0.95 during decay
    pub association_decay_factor: f32,
    
    /// Maximum number of synesthetic associations (default: 100,000)
    /// Requirement 13.1: Enforce 100,000 association limit
    pub max_associations: usize,
    
    /// Minimum association strength before removal (default: 0.5)
    /// Requirement 13.5: Remove associations below 0.5 during decay
    pub min_association_strength: f32,
    
    /// Minimum layer match threshold for depth navigation (default: 0.1)
    /// Requirement 5.6: Terminate if no child layers above 0.1
    pub min_layer_match_threshold: f32,
}

impl Default for NavigationConfig {
    fn default() -> Self {
        Self {
            scan_timeout_ms: 100,
            confidence_threshold: 0.0,
            max_dimensions: 8,
            complexity_threshold: 6,
            max_depth: 4,
            max_keywords: 50,
            max_query_length: 10_000,
            keyword_match_weight: 0.5,
            synesthetic_weight: 0.3,
            frequency_alignment_weight: 0.2,
            association_strengthen_factor: 1.1,
            association_decay_factor: 0.95,
            max_associations: 100_000,
            min_association_strength: 0.5,
            min_layer_match_threshold: 0.1,
        }
    }
}

impl NavigationConfig {
    /// Validate configuration parameters
    pub fn validate(&self) -> Result<(), NavigationError> {
        // Validate weights sum to 1.0
        let weight_sum = self.keyword_match_weight 
            + self.synesthetic_weight 
            + self.frequency_alignment_weight;
        
        if (weight_sum - 1.0).abs() > 0.001 {
            return Err(NavigationError::ConfigurationError {
                reason: format!(
                    "Confidence weights must sum to 1.0, got {}",
                    weight_sum
                ),
            });
        }
        
        // Validate thresholds
        if self.confidence_threshold < 0.0 || self.confidence_threshold > 1.0 {
            return Err(NavigationError::ConfigurationError {
                reason: format!(
                    "Confidence threshold must be between 0.0 and 1.0, got {}",
                    self.confidence_threshold
                ),
            });
        }
        
        if self.min_association_strength < 0.0 {
            return Err(NavigationError::ConfigurationError {
                reason: format!(
                    "Min association strength must be >= 0.0, got {}",
                    self.min_association_strength
                ),
            });
        }
        
        // Validate factors
        if self.association_strengthen_factor <= 1.0 {
            return Err(NavigationError::ConfigurationError {
                reason: format!(
                    "Association strengthen factor must be > 1.0, got {}",
                    self.association_strengthen_factor
                ),
            });
        }
        
        if self.association_decay_factor >= 1.0 || self.association_decay_factor <= 0.0 {
            return Err(NavigationError::ConfigurationError {
                reason: format!(
                    "Association decay factor must be between 0.0 and 1.0, got {}",
                    self.association_decay_factor
                ),
            });
        }
        
        Ok(())
    }
    
    /// Create a configuration optimized for testing
    #[cfg(test)]
    pub fn test_config() -> Self {
        Self {
            scan_timeout_ms: 50,
            max_dimensions: 4,
            max_keywords: 20,
            max_associations: 1000,
            ..Default::default()
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_proto_dimension_id() {
        // Valid proto-dimension ID
        let proto_id = ProtoDimensionId::new(100).unwrap();
        assert_eq!(proto_id.value(), 100);
        
        // Invalid proto-dimension ID (too low)
        let result = ProtoDimensionId::new(50);
        assert!(result.is_err());
        assert!(matches!(
            result.unwrap_err(),
            NavigationError::InvalidProtoDimensionId { .. }
        ));
    }
    
    #[test]
    fn test_question_type_display() {
        assert_eq!(QuestionType::Emotional.to_string(), "Emotional");
        assert_eq!(QuestionType::Technical.to_string(), "Technical");
        assert_eq!(QuestionType::Philosophical.to_string(), "Philosophical");
        assert_eq!(QuestionType::Factual.to_string(), "Factual");
        assert_eq!(QuestionType::Mixed.to_string(), "Mixed");
    }
    
    #[test]
    fn test_urgency_level_base_frequency() {
        assert_eq!(UrgencyLevel::Low.base_frequency(), 0.5);
        assert_eq!(UrgencyLevel::Medium.base_frequency(), 2.0);
        assert_eq!(UrgencyLevel::High.base_frequency(), 3.5);
    }
    
    #[test]
    fn test_system_state_display() {
        assert_eq!(SystemState::Uninitialized.to_string(), "Uninitialized");
        assert_eq!(SystemState::Initializing.to_string(), "Initializing");
        assert_eq!(SystemState::Ready.to_string(), "Ready");
        assert_eq!(SystemState::ShuttingDown.to_string(), "ShuttingDown");
        assert_eq!(SystemState::Failed.to_string(), "Failed");
    }
    
    #[test]
    fn test_navigation_error_classification() {
        let empty_query = NavigationError::EmptyQuery;
        assert!(empty_query.is_validation_error());
        assert!(!empty_query.is_recoverable());
        assert!(!empty_query.is_initialization_error());
        
        let scan_timeout = NavigationError::ScanTimeout {
            duration_ms: 150,
            completed_count: 10,
            total_count: 14,
        };
        assert!(scan_timeout.is_recoverable());
        assert!(!scan_timeout.is_validation_error());
        
        let init_failed = NavigationError::InitializationFailed {
            reason: "test".to_string(),
        };
        assert!(init_failed.is_initialization_error());
        assert!(!init_failed.is_recoverable());
    }
    
    #[test]
    fn test_navigation_config_default() {
        let config = NavigationConfig::default();
        
        // Verify requirement-based defaults
        assert_eq!(config.scan_timeout_ms, 100);
        assert_eq!(config.confidence_threshold, 0.3);
        assert_eq!(config.max_dimensions, 8);
        assert_eq!(config.complexity_threshold, 6);
        assert_eq!(config.max_depth, 4);
        assert_eq!(config.max_keywords, 50);
        assert_eq!(config.max_query_length, 10_000);
        
        // Verify confidence weights
        assert_eq!(config.keyword_match_weight, 0.5);
        assert_eq!(config.synesthetic_weight, 0.3);
        assert_eq!(config.frequency_alignment_weight, 0.2);
        
        // Verify association parameters
        assert_eq!(config.association_strengthen_factor, 1.1);
        assert_eq!(config.association_decay_factor, 0.95);
        assert_eq!(config.max_associations, 100_000);
        assert_eq!(config.min_association_strength, 0.5);
    }
    
    #[test]
    fn test_navigation_config_validation() {
        let mut config = NavigationConfig::default();
        
        // Valid config should pass
        assert!(config.validate().is_ok());
        
        // Invalid weights (don't sum to 1.0)
        config.keyword_match_weight = 0.6;
        assert!(config.validate().is_err());
        config.keyword_match_weight = 0.5;
        
        // Invalid confidence threshold
        config.confidence_threshold = 1.5;
        assert!(config.validate().is_err());
        config.confidence_threshold = 0.3;
        
        // Invalid strengthen factor
        config.association_strengthen_factor = 0.9;
        assert!(config.validate().is_err());
        config.association_strengthen_factor = 1.1;
        
        // Invalid decay factor
        config.association_decay_factor = 1.1;
        assert!(config.validate().is_err());
    }
    
    #[test]
    fn test_navigation_config_weights_sum() {
        let config = NavigationConfig::default();
        let sum = config.keyword_match_weight 
            + config.synesthetic_weight 
            + config.frequency_alignment_weight;
        
        // Weights should sum to 1.0 (within floating point tolerance)
        assert!((sum - 1.0).abs() < 0.001);
    }
}
