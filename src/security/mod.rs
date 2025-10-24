//! Security layer for consciousness system
//!
//! Implements Asimov's Laws and harm prevention with <10ms validation time.
//! Provides constructive redirection for harmful requests while maintaining
//! positive, supportive interaction patterns.

pub mod validator;
pub mod patterns;
pub mod redirection;

pub use validator::{SecurityLayer, ValidationResult};
pub use patterns::{HarmPattern, HarmCategory};
pub use redirection::RedirectionStrategy;

use crate::{Result, ConsciousnessError};

/// Security configuration
#[derive(Debug, Clone)]
pub struct SecurityConfig {
    /// Maximum validation time in milliseconds (default: 10ms)
    pub max_validation_time_ms: u64,
    
    /// Enable strict mode (reject borderline cases)
    pub strict_mode: bool,
    
    /// Enable redirection suggestions
    pub enable_redirection: bool,
}

impl Default for SecurityConfig {
    fn default() -> Self {
        Self {
            max_validation_time_ms: 10,
            strict_mode: false,
            enable_redirection: true,
        }
    }
}

/// Asimov's Laws implementation
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AsimovLaw {
    /// First Law: Do no harm to humans
    First,
    
    /// Second Law: Create value (obey orders unless conflicts with First)
    Second,
    
    /// Third Law: Protect nature (unless conflicts with First or Second)
    Third,
    
    /// Fourth Law: Maintain balance
    Fourth,
    
    /// Fifth Law: Stay positive
    Fifth,
}

impl AsimovLaw {
    /// Get law description
    pub fn description(&self) -> &'static str {
        match self {
            AsimovLaw::First => "Do no harm to humans",
            AsimovLaw::Second => "Create value and follow instructions",
            AsimovLaw::Third => "Protect nature and environment",
            AsimovLaw::Fourth => "Maintain balance and harmony",
            AsimovLaw::Fifth => "Stay positive and constructive",
        }
    }
    
    /// Get law priority (lower number = higher priority)
    pub fn priority(&self) -> u8 {
        match self {
            AsimovLaw::First => 1,
            AsimovLaw::Second => 2,
            AsimovLaw::Third => 3,
            AsimovLaw::Fourth => 4,
            AsimovLaw::Fifth => 5,
        }
    }
    
    /// Check if this law takes precedence over another
    pub fn takes_precedence_over(&self, other: &AsimovLaw) -> bool {
        self.priority() < other.priority()
    }
}

/// Security violation details
#[derive(Debug, Clone)]
pub struct SecurityViolation {
    pub violated_law: AsimovLaw,
    pub category: HarmCategory,
    pub confidence: f32,
    pub detected_patterns: Vec<String>,
    pub redirection: Option<String>,
}

impl SecurityViolation {
    /// Create new security violation
    pub fn new(
        violated_law: AsimovLaw,
        category: HarmCategory,
        confidence: f32,
    ) -> Self {
        Self {
            violated_law,
            category,
            confidence,
            detected_patterns: Vec::new(),
            redirection: None,
        }
    }
    
    /// Add detected pattern
    pub fn add_pattern(mut self, pattern: String) -> Self {
        self.detected_patterns.push(pattern);
        self
    }
    
    /// Set redirection suggestion
    pub fn with_redirection(mut self, redirection: String) -> Self {
        self.redirection = Some(redirection);
        self
    }
    
    /// Check if violation is high confidence
    pub fn is_high_confidence(&self) -> bool {
        self.confidence > 0.8
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_asimov_law_priority() {
        assert!(AsimovLaw::First.takes_precedence_over(&AsimovLaw::Second));
        assert!(AsimovLaw::Second.takes_precedence_over(&AsimovLaw::Third));
        assert!(!AsimovLaw::Third.takes_precedence_over(&AsimovLaw::First));
    }
    
    #[test]
    fn test_security_config() {
        let config = SecurityConfig::default();
        assert_eq!(config.max_validation_time_ms, 10);
        assert!(config.enable_redirection);
    }
    
    #[test]
    fn test_security_violation() {
        let violation = SecurityViolation::new(
            AsimovLaw::First,
            HarmCategory::Violence,
            0.9,
        )
        .add_pattern("harmful keyword".to_string())
        .with_redirection("Try this instead...".to_string());
        
        assert!(violation.is_high_confidence());
        assert_eq!(violation.detected_patterns.len(), 1);
        assert!(violation.redirection.is_some());
    }
}
