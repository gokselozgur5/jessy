//! Synesthetic keyword matching engine

use std::collections::HashMap;
use std::time::SystemTime;

/// Synesthetic association between keywords
#[derive(Debug, Clone)]
pub struct SynestheticAssociation {
    pub source: String,
    pub target: String,
    pub strength: f32,
    pub activation_count: usize,
    pub last_activated: SystemTime,
}

/// Synesthetic matching engine
pub struct SynestheticEngine {
    associations: HashMap<String, Vec<SynestheticAssociation>>,
    learning_rate: f32,
    decay_rate: f32,
}

impl SynestheticEngine {
    /// Create new synesthetic engine
    pub fn new() -> Self {
        Self {
            associations: HashMap::new(),
            learning_rate: 1.1,
            decay_rate: 0.95,
        }
    }
    
    /// Strengthen association between keywords
    pub fn strengthen_association(&mut self, keyword1: &str, keyword2: &str) {
        let key = keyword1.to_lowercase();
        let associations = self.associations.entry(key.clone()).or_insert_with(Vec::new);
        
        // Find existing association or create new
        if let Some(assoc) = associations.iter_mut().find(|a| a.target == keyword2) {
            assoc.strength *= self.learning_rate;
            assoc.activation_count += 1;
            assoc.last_activated = SystemTime::now();
        } else {
            associations.push(SynestheticAssociation {
                source: key,
                target: keyword2.to_lowercase(),
                strength: 1.0,
                activation_count: 1,
                last_activated: SystemTime::now(),
            });
        }
    }
    
    /// Get associations for a keyword
    pub fn get_associations(&self, keyword: &str) -> Vec<&SynestheticAssociation> {
        self.associations
            .get(&keyword.to_lowercase())
            .map(|v| v.iter().collect())
            .unwrap_or_default()
    }
    
    /// Decay unused associations
    pub fn decay_unused(&mut self) {
        for associations in self.associations.values_mut() {
            for assoc in associations.iter_mut() {
                assoc.strength *= self.decay_rate;
            }
        }
    }
}

impl Default for SynestheticEngine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_synesthetic_engine() {
        let mut engine = SynestheticEngine::new();
        
        engine.strengthen_association("empathy", "compassion");
        
        let associations = engine.get_associations("empathy");
        assert_eq!(associations.len(), 1);
        assert_eq!(associations[0].target, "compassion");
    }
    
    #[test]
    fn test_association_strengthening() {
        let mut engine = SynestheticEngine::new();
        
        engine.strengthen_association("test", "example");
        engine.strengthen_association("test", "example");
        
        let associations = engine.get_associations("test");
        assert!(associations[0].strength > 1.0);
        assert_eq!(associations[0].activation_count, 2);
    }
}
