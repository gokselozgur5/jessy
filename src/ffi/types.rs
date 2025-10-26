//! C-Compatible Types for FFI
//!
//! All types use `#[repr(C)]` to ensure C-compatible memory layout.

use std::os::raw::c_char;

/// Error codes returned by FFI functions
pub const SUCCESS: i32 = 0;
pub const ERROR_INVALID_INPUT: i32 = 1;
pub const ERROR_SECURITY_VIOLATION: i32 = 2;
pub const ERROR_NAVIGATION_FAILED: i32 = 3;
pub const ERROR_ITERATION_FAILED: i32 = 4;
pub const ERROR_LLM_API_FAILED: i32 = 5;
pub const ERROR_TIMEOUT: i32 = 6;
pub const ERROR_MEMORY_LIMIT: i32 = 7;
pub const ERROR_NOT_INITIALIZED: i32 = 8;
pub const ERROR_PANIC: i32 = 9;
pub const ERROR_UNKNOWN: i32 = 99;

/// C-compatible query request
#[repr(C)]
pub struct CQueryRequest {
    /// Query string (null-terminated C string)
    pub query: *const c_char,
    
    /// Session ID (null-terminated C string)
    pub session_id: *const c_char,
    
    /// Maximum iterations (1-9)
    pub max_iterations: u32,
}

/// C-compatible query response
#[repr(C)]
pub struct CQueryResponse {
    /// Session ID (caller must free)
    pub session_id: *mut c_char,
    
    /// Final answer (caller must free)
    pub answer: *mut c_char,
    
    /// Dominant frequency (0.1-4.5 Hz)
    pub dominant_frequency: f32,
    
    /// Array of dimension names (caller must free each string and array)
    pub dimensions_activated: *mut *mut c_char,
    
    /// Number of dimensions in array
    pub dimensions_count: usize,
    
    /// Number of iterations completed
    pub iterations_completed: u32,
    
    /// Whether return-to-source was triggered
    pub return_to_source_triggered: bool,
    
    /// Processing time in milliseconds
    pub processing_time_ms: i64,
    
    /// Error code (0 = success)
    pub error_code: i32,
    
    /// Error message if error_code != 0 (caller must free)
    pub error_message: *mut c_char,
}

/// C-compatible iteration update for streaming
#[repr(C)]
pub struct CIterationUpdate {
    /// Session ID (caller must free)
    pub session_id: *mut c_char,
    
    /// Current iteration number
    pub iteration: u32,
    
    /// Maximum iterations
    pub max_iterations: u32,
    
    /// Thought for this iteration (caller must free)
    pub thought: *mut c_char,
    
    /// Current frequency
    pub frequency: f32,
    
    /// Array of dimension names (caller must free)
    pub dimensions: *mut *mut c_char,
    
    /// Number of dimensions
    pub dimensions_count: usize,
    
    /// Whether processing is complete
    pub is_complete: bool,
}

/// C-compatible learning metrics
#[repr(C)]
pub struct CMetrics {
    /// Total observations recorded
    pub observation_count: usize,
    
    /// Total patterns detected
    pub pattern_count: usize,
    
    /// Current proto-dimension count
    pub proto_dimension_count: usize,
    
    /// Crystallization success rate (0.0-1.0)
    pub crystallization_success_rate: f32,
    
    /// Current memory usage in bytes
    pub memory_usage: usize,
    
    /// Memory limit in bytes
    pub memory_limit: usize,
}

impl Default for CQueryResponse {
    fn default() -> Self {
        Self {
            session_id: std::ptr::null_mut(),
            answer: std::ptr::null_mut(),
            dominant_frequency: 0.0,
            dimensions_activated: std::ptr::null_mut(),
            dimensions_count: 0,
            iterations_completed: 0,
            return_to_source_triggered: false,
            processing_time_ms: 0,
            error_code: SUCCESS,
            error_message: std::ptr::null_mut(),
        }
    }
}

impl Default for CMetrics {
    fn default() -> Self {
        Self {
            observation_count: 0,
            pattern_count: 0,
            proto_dimension_count: 0,
            crystallization_success_rate: 0.0,
            memory_usage: 0,
            memory_limit: 0,
        }
    }
}
