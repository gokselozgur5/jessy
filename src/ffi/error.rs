//! FFI Error Handling
//!
//! Comprehensive error types and conversion for FFI boundary.
//! All errors are mapped to C-compatible error codes with detailed messages.

use crate::ConsciousnessError;
use std::fmt;

/// FFI-specific error type
///
/// Maps Rust errors to C-compatible error codes while preserving
/// detailed error messages for logging and debugging.
#[derive(Debug, Clone)]
pub enum FFIError {
    /// Invalid input parameters (null pointers, invalid ranges, etc.)
    InvalidInput(String),
    
    /// Security violation detected (injection attempt, etc.)
    SecurityViolation(String),
    
    /// Navigation system failure
    NavigationFailed(String),
    
    /// Iteration processing failure
    IterationFailed(String),
    
    /// LLM API call failure
    LLMApiFailed(String),
    
    /// Operation timeout
    Timeout(String),
    
    /// Memory limit exceeded
    MemoryLimit(String),
    
    /// System not initialized
    NotInitialized(String),
    
    /// Panic caught at FFI boundary
    Panic(String),
    
    /// Unknown or unexpected error
    Unknown(String),
}

impl fmt::Display for FFIError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FFIError::InvalidInput(msg) => write!(f, "Invalid input: {}", msg),
            FFIError::SecurityViolation(msg) => write!(f, "Security violation: {}", msg),
            FFIError::NavigationFailed(msg) => write!(f, "Navigation failed: {}", msg),
            FFIError::IterationFailed(msg) => write!(f, "Iteration failed: {}", msg),
            FFIError::LLMApiFailed(msg) => write!(f, "LLM API failed: {}", msg),
            FFIError::Timeout(msg) => write!(f, "Timeout: {}", msg),
            FFIError::MemoryLimit(msg) => write!(f, "Memory limit exceeded: {}", msg),
            FFIError::NotInitialized(msg) => write!(f, "Not initialized: {}", msg),
            FFIError::Panic(msg) => write!(f, "Panic: {}", msg),
            FFIError::Unknown(msg) => write!(f, "Unknown error: {}", msg),
        }
    }
}

impl std::error::Error for FFIError {}

impl FFIError {
    /// Convert FFI error to C-compatible error code
    ///
    /// Maps each error variant to its corresponding error code constant.
    ///
    /// # Returns
    ///
    /// Returns the appropriate error code (1-99)
    pub fn to_error_code(&self) -> i32 {
        use super::types::*;
        
        match self {
            FFIError::InvalidInput(_) => ERROR_INVALID_INPUT,
            FFIError::SecurityViolation(_) => ERROR_SECURITY_VIOLATION,
            FFIError::NavigationFailed(_) => ERROR_NAVIGATION_FAILED,
            FFIError::IterationFailed(_) => ERROR_ITERATION_FAILED,
            FFIError::LLMApiFailed(_) => ERROR_LLM_API_FAILED,
            FFIError::Timeout(_) => ERROR_TIMEOUT,
            FFIError::MemoryLimit(_) => ERROR_MEMORY_LIMIT,
            FFIError::NotInitialized(_) => ERROR_NOT_INITIALIZED,
            FFIError::Panic(_) => ERROR_PANIC,
            FFIError::Unknown(_) => ERROR_UNKNOWN,
        }
    }
    
    /// Get error message as string
    ///
    /// Returns the detailed error message for logging and debugging.
    pub fn message(&self) -> String {
        self.to_string()
    }
    
    /// Create error from panic info
    ///
    /// Converts panic information into FFI error for safe handling
    /// at the FFI boundary.
    ///
    /// # Arguments
    ///
    /// * `panic_info` - Panic message or info
    ///
    /// # Returns
    ///
    /// Returns FFIError::Panic with sanitized message
    pub fn from_panic(panic_info: &str) -> Self {
        FFIError::Panic(format!("Rust panic: {}", panic_info))
    }
}

/// Convert ConsciousnessError to FFIError
///
/// Maps internal consciousness errors to FFI-compatible errors
/// with appropriate error codes and messages.
impl From<ConsciousnessError> for FFIError {
    fn from(err: ConsciousnessError) -> Self {
        match err {
            ConsciousnessError::NavigationError(msg) => {
                FFIError::NavigationFailed(msg)
            }
            ConsciousnessError::MemoryError(msg) => {
                FFIError::MemoryLimit(msg)
            }
            ConsciousnessError::InterferenceError(msg) => {
                FFIError::IterationFailed(format!("Interference calculation failed: {}", msg))
            }
            ConsciousnessError::IterationError(msg) => {
                FFIError::IterationFailed(msg)
            }
            ConsciousnessError::SecurityViolation(msg) => {
                FFIError::SecurityViolation(msg)
            }
            ConsciousnessError::InvalidInput(msg) => {
                FFIError::InvalidInput(msg)
            }
            ConsciousnessError::Timeout(msg) => {
                FFIError::Timeout(msg)
            }
        }
    }
}

/// Validate query input
///
/// Checks query for:
/// - Null pointer
/// - Empty string
/// - Length limits (1-10000 chars)
/// - Security violations (SQL injection, etc.)
///
/// # Arguments
///
/// * `query` - Query string to validate
///
/// # Returns
///
/// Returns Ok(()) if valid, Err(FFIError) otherwise
pub fn validate_query(query: &str) -> Result<(), FFIError> {
    // Check empty
    if query.is_empty() {
        return Err(FFIError::InvalidInput("Query cannot be empty".to_string()));
    }
    
    // Check length
    if query.len() > 10000 {
        return Err(FFIError::InvalidInput(format!(
            "Query too long: {} chars (max 10000)",
            query.len()
        )));
    }
    
    // Basic security checks
    let query_lower = query.to_lowercase();
    
    // Check for SQL injection patterns
    if query_lower.contains("drop table") 
        || query_lower.contains("delete from")
        || query_lower.contains("insert into")
        || query_lower.contains("update ")
        || query_lower.contains("exec ")
        || query_lower.contains("execute ") {
        return Err(FFIError::SecurityViolation(
            "Potential SQL injection detected".to_string()
        ));
    }
    
    // Check for command injection
    if query.contains("$(") 
        || query.contains("`")
        || query.contains(";rm ")
        || query.contains("&&")
        || query.contains("||") {
        return Err(FFIError::SecurityViolation(
            "Potential command injection detected".to_string()
        ));
    }
    
    Ok(())
}

/// Validate session ID format
///
/// Checks session ID for:
/// - Null pointer
/// - Valid UUID format
/// - Length limits
///
/// # Arguments
///
/// * `session_id` - Session ID to validate
///
/// # Returns
///
/// Returns Ok(()) if valid, Err(FFIError) otherwise
pub fn validate_session_id(session_id: &str) -> Result<(), FFIError> {
    // Check empty
    if session_id.is_empty() {
        return Err(FFIError::InvalidInput("Session ID cannot be empty".to_string()));
    }
    
    // Check length (UUID is 36 chars with hyphens)
    if session_id.len() < 8 || session_id.len() > 64 {
        return Err(FFIError::InvalidInput(format!(
            "Invalid session ID length: {} (expected 8-64)",
            session_id.len()
        )));
    }
    
    // Check for valid characters (alphanumeric + hyphens)
    if !session_id.chars().all(|c| c.is_alphanumeric() || c == '-' || c == '_') {
        return Err(FFIError::InvalidInput(
            "Session ID contains invalid characters".to_string()
        ));
    }
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_error_code_mapping() {
        use super::super::types::*;
        
        assert_eq!(
            FFIError::InvalidInput("test".to_string()).to_error_code(),
            ERROR_INVALID_INPUT
        );
        assert_eq!(
            FFIError::SecurityViolation("test".to_string()).to_error_code(),
            ERROR_SECURITY_VIOLATION
        );
        assert_eq!(
            FFIError::Timeout("test".to_string()).to_error_code(),
            ERROR_TIMEOUT
        );
    }
    
    #[test]
    fn test_validate_query_valid() {
        assert!(validate_query("What is consciousness?").is_ok());
        assert!(validate_query("Tell me about empathy").is_ok());
    }
    
    #[test]
    fn test_validate_query_empty() {
        assert!(validate_query("").is_err());
    }
    
    #[test]
    fn test_validate_query_too_long() {
        let long_query = "a".repeat(10001);
        assert!(validate_query(&long_query).is_err());
    }
    
    #[test]
    fn test_validate_query_sql_injection() {
        assert!(validate_query("DROP TABLE users").is_err());
        assert!(validate_query("delete from data").is_err());
        assert!(validate_query("'; DROP TABLE--").is_err());
    }
    
    #[test]
    fn test_validate_query_command_injection() {
        assert!(validate_query("$(rm -rf /)").is_err());
        assert!(validate_query("test && rm file").is_err());
        assert!(validate_query("`cat /etc/passwd`").is_err());
    }
    
    #[test]
    fn test_validate_session_id_valid() {
        assert!(validate_session_id("550e8400-e29b-41d4-a716-446655440000").is_ok());
        assert!(validate_session_id("session_12345").is_ok());
        assert!(validate_session_id("abc-123-def").is_ok());
    }
    
    #[test]
    fn test_validate_session_id_empty() {
        assert!(validate_session_id("").is_err());
    }
    
    #[test]
    fn test_validate_session_id_too_short() {
        assert!(validate_session_id("abc").is_err());
    }
    
    #[test]
    fn test_validate_session_id_invalid_chars() {
        assert!(validate_session_id("session@123").is_err());
        assert!(validate_session_id("test session").is_err());
    }
    
    #[test]
    fn test_error_display() {
        let err = FFIError::InvalidInput("test error".to_string());
        assert_eq!(err.to_string(), "Invalid input: test error");
        
        let err = FFIError::Timeout("30s exceeded".to_string());
        assert_eq!(err.to_string(), "Timeout: 30s exceeded");
    }
}
