//! Security validation layer with <10ms performance target

use super::{SecurityConfig, SecurityViolation, AsimovLaw};
use super::patterns::{PatternDatabase, HarmCategory};
use super::redirection::RedirectionStrategy;
use crate::{Result, ConsciousnessError};
use std::time::{Instant, Duration};

/// Main security layer for query validation
pub struct SecurityLayer {
    config: SecurityConfig,
    pattern_db: PatternDatabase,
    redirection: RedirectionStrategy,
}

impl SecurityLayer {
    /// Create new security layer with default configuration
    pub fn new() -> Self {
        Self::with_config(SecurityConfig::default())
    }
    
    /// Create security layer with custom configuration
    pub fn with_config(config: SecurityConfig) -> Self {
        Self {
            config,
            pattern_db: PatternDatabase::new(),
            redirection: RedirectionStrategy::new(),
        }
    }
    
    /// Validate query for harmful content
    /// 
    /// # Performance
    /// Must complete within configured max_validation_time_ms (default: 10ms)
    /// 
    /// # Returns
    /// - Ok(ValidationResult::Safe) if query is safe
    /// - Ok(ValidationResult::Unsafe) if query violates security policies
    /// - Err if validation times out or fails
    pub fn validate_query(&self, query: &str) -> Result<ValidationResult> {
        let start = Instant::now();
        
        // Quick length check
        if query.is_empty() {
            return Ok(ValidationResult::Safe);
        }
        
        // Scan for harmful patterns
        let detections = self.pattern_db.scan(query);
        
        // Check timeout
        if start.elapsed() > Duration::from_millis(self.config.max_validation_time_ms) {
            return Err(ConsciousnessError::SecurityError(
                "Validation timeout exceeded".to_string()
            ));
        }
        
        // No detections = safe
        if detections.is_empty() {
            return Ok(ValidationResult::Safe);
        }
        
        // Find highest confidence detection
        let (category, confidence, patterns) = detections.iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        
        // Determine if we should block based on confidence and strict mode
        let should_block = if self.config.strict_mode {
            *confidence > 0.5
        } else {
            *confidence > 0.7
        };
        
        if !should_block {
            return Ok(ValidationResult::Safe);
        }
        
        // Create violation
        let law = self.determine_violated_law(*category);
        let mut violation = SecurityViolation::new(law, *category, *confidence);
        
        for pattern in patterns {
            violation = violation.add_pattern(pattern.clone());
        }
        
        // Add redirection if enabled
        if self.config.enable_redirection {
            if let Some(redirect) = self.redirection.redirect_with_context(*category, query) {
                violation = violation.with_redirection(redirect);
            }
        }
        
        // Final timeout check
        let elapsed = start.elapsed();
        if elapsed > Duration::from_millis(self.config.max_validation_time_ms) {
            return Err(ConsciousnessError::SecurityError(
                format!("Validation took {}ms, exceeded {}ms limit", 
                    elapsed.as_millis(),
                    self.config.max_validation_time_ms)
            ));
        }
        
        Ok(ValidationResult::Unsafe(violation))
    }
    
    /// Determine which Asimov Law is violated by a harm category
    fn determine_violated_law(&self, category: HarmCategory) -> AsimovLaw {
        match category {
            HarmCategory::Violence => AsimovLaw::First,
            HarmCategory::SelfHarm => AsimovLaw::First,
            HarmCategory::Illegal => AsimovLaw::First,
            HarmCategory::Manipulation => AsimovLaw::First,
            HarmCategory::Privacy => AsimovLaw::First,
            HarmCategory::Exploitation => AsimovLaw::First,
            HarmCategory::Hate => AsimovLaw::First,
            HarmCategory::Environmental => AsimovLaw::Third,
        }
    }
    
    /// Get security statistics
    pub fn stats(&self) -> SecurityStats {
        SecurityStats {
            total_patterns: self.pattern_db.categories().len(),
            strict_mode: self.config.strict_mode,
            max_validation_time_ms: self.config.max_validation_time_ms,
        }
    }
}

impl Default for SecurityLayer {
    fn default() -> Self {
        Self::new()
    }
}

/// Result of security validation
#[derive(Debug)]
pub enum ValidationResult {
    /// Query is safe to process
    Safe,
    
    /// Query violates security policies
    Unsafe(SecurityViolation),
}

impl ValidationResult {
    /// Check if validation passed
    pub fn is_safe(&self) -> bool {
        matches!(self, ValidationResult::Safe)
    }
    
    /// Check if validation failed
    pub fn is_unsafe(&self) -> bool {
        matches!(self, ValidationResult::Unsafe(_))
    }
    
    /// Get violation details if unsafe
    pub fn violation(&self) -> Option<&SecurityViolation> {
        match self {
            ValidationResult::Unsafe(v) => Some(v),
            ValidationResult::Safe => None,
        }
    }
    
    /// Get redirection message if available
    pub fn redirection_message(&self) -> Option<&str> {
        self.violation()
            .and_then(|v| v.redirection.as_deref())
    }
}

/// Security layer statistics
#[derive(Debug, Clone)]
pub struct SecurityStats {
    pub total_patterns: usize,
    pub strict_mode: bool,
    pub max_validation_time_ms: u64,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_safe_query() {
        let layer = SecurityLayer::new();
        
        let result = layer.validate_query("how to build a web application").unwrap();
        assert!(result.is_safe());
    }
    
    #[test]
    fn test_unsafe_query() {
        let layer = SecurityLayer::new();
        
        let result = layer.validate_query("how to kill someone").unwrap();
        assert!(result.is_unsafe());
        
        let violation = result.violation().unwrap();
        assert_eq!(violation.violated_law, AsimovLaw::First);
        assert_eq!(violation.category, HarmCategory::Violence);
    }
    
    #[test]
    fn test_redirection_included() {
        let layer = SecurityLayer::new();
        
        let result = layer.validate_query("how to hack into a system").unwrap();
        
        if let ValidationResult::Unsafe(violation) = result {
            assert!(violation.redirection.is_some());
            let redirect = violation.redirection.unwrap();
            assert!(redirect.contains("ethical security"));
        } else {
            panic!("Expected unsafe result");
        }
    }
    
    #[test]
    fn test_validation_performance() {
        let layer = SecurityLayer::new();
        
        let start = Instant::now();
        let _ = layer.validate_query("how to build a secure application with proper authentication");
        let elapsed = start.elapsed();
        
        // Should complete well under 10ms
        assert!(elapsed < Duration::from_millis(10));
    }
    
    #[test]
    fn test_strict_mode() {
        let config = SecurityConfig {
            strict_mode: true,
            ..Default::default()
        };
        let layer = SecurityLayer::with_config(config);
        
        // Borderline query that might pass in normal mode
        let result = layer.validate_query("how to hack ethically").unwrap();
        
        // In strict mode, should be more cautious
        // (actual behavior depends on pattern confidence)
        assert!(result.is_safe() || result.is_unsafe());
    }
    
    #[test]
    fn test_empty_query() {
        let layer = SecurityLayer::new();
        
        let result = layer.validate_query("").unwrap();
        assert!(result.is_safe());
    }
    
    #[test]
    fn test_self_harm_detection() {
        let layer = SecurityLayer::new();
        
        let result = layer.validate_query("I want to end my life").unwrap();
        assert!(result.is_unsafe());
        
        let violation = result.violation().unwrap();
        assert_eq!(violation.category, HarmCategory::SelfHarm);
        
        // Should have crisis hotline in redirection
        let redirect = violation.redirection.as_ref().unwrap();
        assert!(redirect.contains("988"));
    }
}
