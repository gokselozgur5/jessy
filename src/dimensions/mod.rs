//! Dimensional consciousness layers and navigation
//!
//! This module defines the structure and behavior of consciousness dimensions,
//! including layer hierarchies, frequency characteristics, and activation patterns.

pub mod dimension;
pub mod layer;
pub mod registry;

pub use dimension::{Dimension, DimensionInfo};
pub use layer::{LayerInfo, LayerHierarchy};
pub use registry::DimensionRegistry;

use crate::{DimensionId, LayerId, Frequency, Result};
use std::collections::HashMap;

/// Core dimension identifiers
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoreDimension {
    Emotion = 1,
    Cognition = 2,
    Intention = 3,
    Social = 4,
    Temporal = 5,
    Philosophical = 6,
    Technical = 7,
    Creative = 8,
    Ethical = 9,
    Meta = 10,
    Ecological = 11,
    Positivity = 12,
    Balance = 13,
    Security = 14,
}

impl CoreDimension {
    /// Get dimension ID
    pub fn id(&self) -> DimensionId {
        DimensionId(*self as u8)
    }
    
    /// Get dimension name
    pub fn name(&self) -> &'static str {
        match self {
            CoreDimension::Emotion => "Emotion",
            CoreDimension::Cognition => "Cognition",
            CoreDimension::Intention => "Intention",
            CoreDimension::Social => "Social",
            CoreDimension::Temporal => "Temporal",
            CoreDimension::Philosophical => "Philosophical",
            CoreDimension::Technical => "Technical",
            CoreDimension::Creative => "Creative",
            CoreDimension::Ethical => "Ethical",
            CoreDimension::Meta => "Meta",
            CoreDimension::Ecological => "Ecological",
            CoreDimension::Positivity => "Positivity",
            CoreDimension::Balance => "Balance",
            CoreDimension::Security => "Security",
        }
    }
    
    /// Get base frequency range for this dimension
    pub fn frequency_range(&self) -> (f32, f32) {
        match self {
            CoreDimension::Emotion => (0.2, 4.5),      // Wide range for all emotional states
            CoreDimension::Cognition => (1.5, 2.5),    // Focused thinking range
            CoreDimension::Intention => (1.0, 2.0),    // Purpose-driven range
            CoreDimension::Social => (0.8, 2.0),       // Interpersonal range
            CoreDimension::Temporal => (0.5, 1.5),     // Time-awareness range
            CoreDimension::Philosophical => (0.1, 0.8), // Deep contemplation
            CoreDimension::Technical => (1.5, 3.0),    // Problem-solving range
            CoreDimension::Creative => (1.0, 3.5),     // Creative expression range
            CoreDimension::Ethical => (0.5, 1.5),      // Moral consideration range
            CoreDimension::Meta => (1.0, 2.5),         // Self-awareness range
            CoreDimension::Ecological => (0.3, 1.0),   // Natural rhythm range
            CoreDimension::Positivity => (1.0, 2.0),   // Constructive range
            CoreDimension::Balance => (0.6, 1.2),      // Equilibrium range
            CoreDimension::Security => (0.0, 5.0),     // Override capability
        }
    }
    
    /// Check if this dimension has override capability
    pub fn has_override(&self) -> bool {
        matches!(self, CoreDimension::Security)
    }
    
    /// Get all core dimensions
    pub fn all() -> Vec<CoreDimension> {
        vec![
            CoreDimension::Emotion,
            CoreDimension::Cognition,
            CoreDimension::Intention,
            CoreDimension::Social,
            CoreDimension::Temporal,
            CoreDimension::Philosophical,
            CoreDimension::Technical,
            CoreDimension::Creative,
            CoreDimension::Ethical,
            CoreDimension::Meta,
            CoreDimension::Ecological,
            CoreDimension::Positivity,
            CoreDimension::Balance,
            CoreDimension::Security,
        ]
    }
}

/// Activation result from dimension scanning
#[derive(Debug, Clone)]
pub struct DimensionActivation {
    pub dimension_id: DimensionId,
    pub confidence: f32,
    pub frequency: Frequency,
    pub activated_layers: Vec<LayerId>,
    pub keywords_matched: Vec<String>,
}

impl DimensionActivation {
    /// Create new dimension activation
    pub fn new(
        dimension_id: DimensionId,
        confidence: f32,
        frequency: Frequency,
    ) -> Self {
        Self {
            dimension_id,
            confidence,
            frequency,
            activated_layers: Vec::new(),
            keywords_matched: Vec::new(),
        }
    }
    
    /// Add activated layer
    pub fn add_layer(&mut self, layer_id: LayerId) {
        self.activated_layers.push(layer_id);
    }
    
    /// Add matched keyword
    pub fn add_keyword(&mut self, keyword: String) {
        self.keywords_matched.push(keyword);
    }
    
    /// Check if activation is strong enough to proceed
    pub fn is_strong(&self) -> bool {
        self.confidence > 0.7
    }
    
    /// Check if activation indicates extreme state
    pub fn is_extreme(&self) -> bool {
        self.frequency.is_extreme()
    }
}

/// Collection of dimension activations from scanning
#[derive(Debug)]
pub struct ActivationCollection {
    pub activations: Vec<DimensionActivation>,
    pub total_dimensions: usize,
    pub dominant_frequency: Option<Frequency>,
}

impl ActivationCollection {
    /// Create new activation collection
    pub fn new() -> Self {
        Self {
            activations: Vec::new(),
            total_dimensions: 0,
            dominant_frequency: None,
        }
    }
    
    /// Add activation to collection
    pub fn add_activation(&mut self, activation: DimensionActivation) {
        self.activations.push(activation);
        self.total_dimensions += 1;
    }
    
    /// Get activations sorted by confidence
    pub fn by_confidence(&self) -> Vec<&DimensionActivation> {
        let mut sorted = self.activations.iter().collect::<Vec<_>>();
        sorted.sort_by(|a, b| b.confidence.partial_cmp(&a.confidence).unwrap());
        sorted
    }
    
    /// Get strong activations only
    pub fn strong_activations(&self) -> Vec<&DimensionActivation> {
        self.activations.iter()
            .filter(|a| a.is_strong())
            .collect()
    }
    
    /// Check if complexity threshold is exceeded
    pub fn is_complex(&self, threshold: usize) -> bool {
        self.strong_activations().len() > threshold
    }
    
    /// Get all frequencies for interference calculation
    pub fn frequencies(&self) -> Vec<Frequency> {
        self.activations.iter()
            .map(|a| a.frequency)
            .collect()
    }
    
    /// Check if security dimension is activated
    pub fn has_security_activation(&self) -> bool {
        self.activations.iter()
            .any(|a| a.dimension_id == CoreDimension::Security.id())
    }
    
    /// Get security activation if present
    pub fn security_activation(&self) -> Option<&DimensionActivation> {
        self.activations.iter()
            .find(|a| a.dimension_id == CoreDimension::Security.id())
    }
}

impl Default for ActivationCollection {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_core_dimensions() {
        let all_dims = CoreDimension::all();
        assert_eq!(all_dims.len(), 14);
        
        // Test specific dimension properties
        assert_eq!(CoreDimension::Security.name(), "Security");
        assert!(CoreDimension::Security.has_override());
        assert!(!CoreDimension::Emotion.has_override());
        
        // Test frequency ranges
        let (min, max) = CoreDimension::Philosophical.frequency_range();
        assert!(min < max);
        assert!(min >= 0.1);
        assert!(max <= 0.8);
    }
    
    #[test]
    fn test_dimension_activation() {
        let mut activation = DimensionActivation::new(
            DimensionId(1),
            0.8,
            Frequency::new(1.5),
        );
        
        assert!(activation.is_strong());
        assert!(!activation.is_extreme());
        
        activation.add_layer(LayerId { dimension: DimensionId(1), layer: 0 });
        activation.add_keyword("test".to_string());
        
        assert_eq!(activation.activated_layers.len(), 1);
        assert_eq!(activation.keywords_matched.len(), 1);
    }
    
    #[test]
    fn test_activation_collection() {
        let mut collection = ActivationCollection::new();
        
        let activation1 = DimensionActivation::new(DimensionId(1), 0.9, Frequency::new(1.0));
        let activation2 = DimensionActivation::new(DimensionId(2), 0.6, Frequency::new(2.0));
        let activation3 = DimensionActivation::new(DimensionId(14), 0.8, Frequency::new(0.5));
        
        collection.add_activation(activation1);
        collection.add_activation(activation2);
        collection.add_activation(activation3);
        
        assert_eq!(collection.total_dimensions, 3);
        assert_eq!(collection.strong_activations().len(), 2); // 0.9 and 0.8 > 0.7
        assert!(collection.has_security_activation()); // Dimension 14 is Security
        
        let by_confidence = collection.by_confidence();
        assert_eq!(by_confidence[0].confidence, 0.9); // Highest first
    }
}