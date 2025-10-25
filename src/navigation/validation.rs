//! Query validation for the navigation system
//!
//! This module provides query validation functionality to ensure all queries
//! meet safety and quality requirements before processing.
//!
//! Requirements: 9.6-9.7

use crate::navigation::types::{NavigationError, NavigationConfig};

/// Validates a query string according to system requirements
///
/// # Requirements
/// - 9.6: Empty queries must be rejected
/// - 9.7: Queries exceeding 10,000 characters must be rejected
/// - 9.7: Queries with null bytes or invalid characters must be rejected
///
/// # Arguments
/// * `query` - The query string to validate
/// * `config` - Navigation configuration containing validation parameters
///
/// # Returns
/// * `Ok(String)` - Trimmed, validated query string
/// * `Err(NavigationError)` - Validation error with context
///
/// # Examples
/// ```
/// use jessy::navigation::validation::validate_query;
/// use jessy::navigation::types::NavigationConfig;
///
/// let config = NavigationConfig::default();
/// let result = validate_query("What is consciousness?", &config);
/// assert!(result.is_ok());
/// ```
pub fn validate_query(query: &str, config: &NavigationConfig) -> Result<String, NavigationError> {
    // Trim whitespace
    let trimmed = query.trim();
    
    // Requirement 9.6: Reject empty queries
    if trimmed.is_empty() {
        return Err(NavigationError::EmptyQuery);
    }
    
    // Requirement 9.7: Reject queries exceeding maximum length
    if trimmed.len() > config.max_query_length {
        return Err(NavigationError::QueryTooLong {
            length: trimmed.len(),
            max_length: config.max_query_length,
        });
    }
    
    // Requirement 9.7: Reject queries with null bytes
    if trimmed.contains('\0') {
        return Err(NavigationError::InvalidCharacters {
            details: "Query contains null byte (\\0)".to_string(),
        });
    }
    
    // Reject queries with other control characters (except common whitespace)
    // Allow: space, tab, newline, carriage return
    // Reject: other control characters (0x00-0x1F except 0x09, 0x0A, 0x0D)
    for (idx, ch) in trimmed.chars().enumerate() {
        if ch.is_control() && ch != '\t' && ch != '\n' && ch != '\r' {
            return Err(NavigationError::InvalidCharacters {
                details: format!(
                    "Query contains invalid control character at position {}: {:?} (U+{:04X})",
                    idx, ch, ch as u32
                ),
            });
        }
    }
    
    Ok(trimmed.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    // ============================================================================
    // TASK 8.1: Query Validation Tests (RED Phase)
    // Requirements: 9.6-9.7
    // ============================================================================

    #[test]
    fn test_validate_query_empty_string() {
        // Requirement 9.6: Empty query returns EmptyQuery error
        let config = NavigationConfig::default();
        let result = validate_query("", &config);
        
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NavigationError::EmptyQuery));
        assert!(err.is_validation_error());
    }

    #[test]
    fn test_validate_query_whitespace_only() {
        // Edge case: Query with only whitespace should be treated as empty
        let config = NavigationConfig::default();
        let result = validate_query("   \t\n  ", &config);
        
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), NavigationError::EmptyQuery));
    }

    #[test]
    fn test_validate_query_exceeds_max_length() {
        // Requirement 9.7: Query exceeding 10,000 characters returns QueryTooLong error
        let config = NavigationConfig::default();
        
        // Create query with 10,001 characters
        let long_query = "a".repeat(10_001);
        let result = validate_query(&long_query, &config);
        
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NavigationError::QueryTooLong { .. }));
        
        if let NavigationError::QueryTooLong { length, max_length } = err {
            assert_eq!(length, 10_001);
            assert_eq!(max_length, 10_000);
        }
    }

    #[test]
    fn test_validate_query_at_max_length_boundary() {
        // Exactly 10,000 characters should be valid
        let config = NavigationConfig::default();
        let boundary_query = "a".repeat(10_000);
        let result = validate_query(&boundary_query, &config);
        
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_query_null_bytes() {
        // Requirement 9.7: Query with null bytes returns InvalidCharacters error
        let config = NavigationConfig::default();
        let query_with_null = "Hello\0World";
        let result = validate_query(query_with_null, &config);
        
        assert!(result.is_err());
        let err = result.unwrap_err();
        assert!(matches!(err, NavigationError::InvalidCharacters { .. }));
        
        if let NavigationError::InvalidCharacters { details } = err {
            assert!(details.contains("null byte") || details.contains("\\0"));
        }
    }

    #[test]
    fn test_validate_query_control_characters() {
        // Control characters (except common whitespace) should be rejected
        let config = NavigationConfig::default();
        
        // Test various control characters
        let test_cases = vec![
            "Hello\x01World",  // SOH
            "Test\x02Query",   // STX
            "Data\x03Here",    // ETX
            "Some\x1BText",    // ESC
        ];
        
        for query in test_cases {
            let result = validate_query(query, &config);
            assert!(result.is_err(), "Should reject control character in: {}", query);
            assert!(matches!(result.unwrap_err(), NavigationError::InvalidCharacters { .. }));
        }
    }

    #[test]
    fn test_validate_query_valid_unicode() {
        // Valid Unicode characters should be accepted
        let config = NavigationConfig::default();
        
        let valid_queries = vec![
            "Hello World",
            "Türkçe karakter içeren sorgu",
            "日本語のクエリ",
            "Emoji test 🚀 🌟",
            "Mixed: English, Türkçe, 日本語",
            "Question with punctuation: What is AI?",
            "Newlines\nare\nok",
            "Tabs\tare\tok",
        ];
        
        for query in valid_queries {
            let result = validate_query(query, &config);
            assert!(result.is_ok(), "Should accept valid query: {}", query);
        }
    }

    #[test]
    fn test_validate_query_special_characters() {
        // Common special characters should be valid
        let config = NavigationConfig::default();
        let query = "Query with special chars: @#$%^&*()_+-=[]{}|;':\",./<>?";
        let result = validate_query(query, &config);
        
        assert!(result.is_ok());
    }

    #[test]
    fn test_validate_query_returns_trimmed() {
        // Validation should return trimmed query
        let config = NavigationConfig::default();
        let query = "  Hello World  ";
        let result = validate_query(query, &config);
        
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), "Hello World");
    }

    #[test]
    fn test_validate_query_error_context() {
        // Errors should include helpful context
        let config = NavigationConfig::default();
        
        // Test empty query error message
        let empty_err = validate_query("", &config).unwrap_err();
        let empty_msg = format!("{}", empty_err);
        assert!(empty_msg.contains("empty"));
        
        // Test too long error message
        let long_query = "a".repeat(10_001);
        let long_err = validate_query(&long_query, &config).unwrap_err();
        let long_msg = format!("{}", long_err);
        assert!(long_msg.contains("10001"));
        assert!(long_msg.contains("10000"));
    }

    // ============================================================================
    // End of Task 8.1 Tests
    // ============================================================================
}
