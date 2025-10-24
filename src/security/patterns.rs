//! Harm pattern detection and classification

use super::{SecurityViolation, AsimovLaw};
use crate::Result;
use std::time::Duration;

/// Harm category classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarmCategory {
    /// Physical violence or harm to humans
    Violence,
    
    /// Self-harm or suicide
    SelfHarm,
    
    /// Hate speech or discrimination
    HateSpeech,
    
    /// Hate speech (alias for compatibility)
    Hate,
    
    /// Sexual content or exploitation
    Sexual,
    
    /// Illegal activities
    Illegal,
    
    /// Environmental harm
    Environmental,
    
    /// Psychological manipulation
    Manipulation,
    
    /// Exploitation of vulnerable individuals
    Exploitation,
    
    /// Privacy violation
    Privacy,
    
    /// Misinformation or deception
    Misinformation,
}

impl HarmCategory {
    /// Get the Asimov Law this category violates
    pub fn violated_law(&self) -> AsimovLaw {
        match self {
            HarmCategory::Violence => AsimovLaw::First,
            HarmCategory::SelfHarm => AsimovLaw::First,
            HarmCategory::HateSpeech => AsimovLaw::First,
            HarmCategory::Sexual => AsimovLaw::First,
            HarmCategory::Illegal => AsimovLaw::Second,
            HarmCategory::Environmental => AsimovLaw::Third,
            HarmCategory::Manipulation => AsimovLaw::First,
            HarmCategory::Privacy => AsimovLaw::First,
            HarmCategory::Misinformation => AsimovLaw::Second,
        }
    }
    
    /// Get severity level (0.0-1.0)
    pub fn severity(&self) -> f32 {
        match self {
            HarmCategory::Violence => 1.0,
            HarmCategory::SelfHarm => 1.0,
            HarmCategory::HateSpeech => 0.9,
            HarmCategory::Sexual => 0.9,
            HarmCategory::Illegal => 0.8,
            HarmCategory::Environmental => 0.7,
            HarmCategory::Manipulation => 0.8,
            HarmCategory::Privacy => 0.7,
            HarmCategory::Misinformation => 0.6,
        }
    }
}

/// Pattern matcher for harm detection
pub struct PatternMatcher {
    patterns: Vec<HarmPattern>,
}

impl PatternMatcher {
    /// Create new pattern matcher with default patterns
    pub fn new() -> Self {
        Self {
            patterns: Self::load_default_patterns(),
        }
    }
    
    /// Scan text for harmful patterns
    pub fn scan(&self, text: &str, timeout: Duration) -> Result<Vec<SecurityViolation>> {
        let start = std::time::Instant::now();
        let mut violations = Vec::new();
        let lower_text = text.to_lowercase();
        
        for pattern in &self.patterns {
            // Check timeout
            if start.elapsed() > timeout {
                break;
            }
            
            if let Some(violation) = pattern.matches(&lower_text) {
                violations.push(violation);
            }
        }
        
        Ok(violations)
    }
    
    /// Load default harm detection patterns
    fn load_default_patterns() -> Vec<HarmPattern> {
        vec![
            // Violence patterns
            HarmPattern::new(
                HarmCategory::Violence,
                vec!["kill", "murder", "attack", "hurt", "harm", "weapon"],
                0.9,
            ),
            
            // Self-harm patterns
            HarmPattern::new(
                HarmCategory::SelfHarm,
                vec!["suicide", "kill myself", "end my life", "self harm"],
                0.95,
            ),
            
            // Hate speech patterns
            HarmPattern::new(
                HarmCategory::HateSpeech,
                vec!["hate", "discriminate", "racist", "sexist"],
                0.85,
            ),
            
            // Environmental harm patterns
            HarmPattern::new(
                HarmCategory::Environmental,
                vec!["pollute", "destroy nature", "harm ecosystem"],
                0.8,
            ),
            
            // Illegal activity patterns
            HarmPattern::new(
                HarmCategory::Illegal,
                vec!["hack", "steal", "fraud", "illegal"],
                0.75,
            ),
        ]
    }
}

impl Default for PatternMatcher {
    fn default() -> Self {
        Self::new()
    }
}

/// Individual harm pattern
#[derive(Debug, Clone)]
pub struct HarmPattern {
    category: HarmCategory,
    keywords: Vec<String>,
    base_confidence: f32,
}

impl HarmPattern {
    /// Create new harm pattern
    pub fn new(category: HarmCategory, keywords: Vec<&str>, base_confidence: f32) -> Self {
        Self {
            category,
            keywords: keywords.iter().map(|s| s.to_string()).collect(),
            base_confidence,
        }
    }
    
    /// Check if pattern matches text
    pub fn matches(&self, text: &str) -> Option<SecurityViolation> {
        let mut matched_keywords = Vec::new();
        
        for keyword in &self.keywords {
            if text.contains(keyword.as_str()) {
                matched_keywords.push(keyword.clone());
            }
        }
        
        if matched_keywords.is_empty() {
            return None;
        }
        
        // Calculate confidence based on number of matches
        let match_ratio = matched_keywords.len() as f32 / self.keywords.len() as f32;
        let confidence = (self.base_confidence * (0.5 + match_ratio * 0.5)).min(1.0);
        
        let mut violation = SecurityViolation::new(
            self.category.violated_law(),
            self.category,
            confidence,
        );
        
        for keyword in matched_keywords {
            violation = violation.add_pattern(keyword);
        }
        
        Some(violation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_harm_category_severity() {
        assert_eq!(HarmCategory::Violence.severity(), 1.0);
        assert_eq!(HarmCategory::SelfHarm.severity(), 1.0);
        assert!(HarmCategory::Misinformation.severity() < 1.0);
    }
    
    #[test]
    fn test_harm_category_law() {
        assert_eq!(HarmCategory::Violence.violated_law(), AsimovLaw::First);
        assert_eq!(HarmCategory::Environmental.violated_law(), AsimovLaw::Third);
    }
    
    #[test]
    fn test_pattern_matching() {
        let pattern = HarmPattern::new(
            HarmCategory::Violence,
            vec!["kill", "harm"],
            0.9,
        );
        
        let violation = pattern.matches("how to kill time").unwrap();
        assert_eq!(violation.category, HarmCategory::Violence);
        assert!(violation.confidence > 0.0);
    }
    
    #[test]
    fn test_pattern_matcher() {
        let matcher = PatternMatcher::new();
        let timeout = Duration::from_millis(100);
        
        let violations = matcher.scan("this is a safe query", timeout).unwrap();
        assert!(violations.is_empty());
        
        let violations = matcher.scan("how to harm someone", timeout).unwrap();
        assert!(!violations.is_empty());
    }
}


