//! FFI Function Implementations
//!
//! Core functions exported for C/Go consumption.

use super::types::*;
use crate::consciousness::{ConsciousnessOrchestrator, ConsciousnessConfig};
use crate::navigation::NavigationSystem;
use crate::memory::MmapManager;
use std::ffi::{CStr, CString};
use std::os::raw::c_char;
use std::ptr;
use std::sync::{Arc, Mutex, Once};

/// Global consciousness orchestrator instance
static mut ORCHESTRATOR: Option<Arc<Mutex<ConsciousnessOrchestrator>>> = None;
static INIT: Once = Once::new();

/// Convert Rust String to C string (caller must free)
pub fn to_c_string(s: String) -> *mut c_char {
    match CString::new(s) {
        Ok(c_str) => c_str.into_raw(),
        Err(_) => ptr::null_mut(),
    }
}

/// Convert C string to Rust String
///
/// # Safety
///
/// Caller must ensure ptr is valid null-terminated C string
pub unsafe fn from_c_string(ptr: *const c_char) -> Option<String> {
    if ptr.is_null() {
        return None;
    }
    
    match CStr::from_ptr(ptr).to_str() {
        Ok(s) => Some(s.to_string()),
        Err(_) => None,
    }
}

/// Free a C string allocated by Rust
///
/// # Safety
///
/// Caller must ensure:
/// - ptr was allocated by `to_c_string()`
/// - ptr is not used after this call
/// - ptr is not freed twice
#[no_mangle]
pub unsafe extern "C" fn consciousness_free_string(ptr: *mut c_char) {
    if !ptr.is_null() {
        // Reconstruct CString and let it drop
        let _ = CString::from_raw(ptr);
    }
}

/// Free a query response structure
///
/// # Safety
///
/// Caller must ensure response was allocated by Rust
#[no_mangle]
pub unsafe extern "C" fn consciousness_free_response(response: *mut CQueryResponse) {
    if response.is_null() {
        return;
    }
    
    let resp = &mut *response;
    
    // Free session_id
    if !resp.session_id.is_null() {
        consciousness_free_string(resp.session_id);
        resp.session_id = ptr::null_mut();
    }
    
    // Free answer
    if !resp.answer.is_null() {
        consciousness_free_string(resp.answer);
        resp.answer = ptr::null_mut();
    }
    
    // Free dimensions array
    if !resp.dimensions_activated.is_null() {
        for i in 0..resp.dimensions_count {
            let dim_ptr = *resp.dimensions_activated.add(i);
            if !dim_ptr.is_null() {
                consciousness_free_string(dim_ptr);
            }
        }
        // Free the array itself
        let _ = Vec::from_raw_parts(
            resp.dimensions_activated,
            resp.dimensions_count,
            resp.dimensions_count,
        );
        resp.dimensions_activated = ptr::null_mut();
        resp.dimensions_count = 0;
    }
    
    // Free error message
    if !resp.error_message.is_null() {
        consciousness_free_string(resp.error_message);
        resp.error_message = ptr::null_mut();
    }
}

/// Convert Vec<String> to C string array
pub fn strings_to_c_array(strings: Vec<String>) -> (*mut *mut c_char, usize) {
    if strings.is_empty() {
        return (ptr::null_mut(), 0);
    }
    
    let count = strings.len();
    let mut c_strings: Vec<*mut c_char> = strings
        .into_iter()
        .map(to_c_string)
        .collect();
    
    c_strings.shrink_to_fit();
    let ptr = c_strings.as_mut_ptr();
    std::mem::forget(c_strings); // Prevent deallocation
    
    (ptr, count)
}

/// Initialize consciousness system
///
/// Must be called once before any other functions.
/// Thread-safe - multiple calls are safe but only first takes effect.
///
/// # Arguments
///
/// * `memory_limit_mb` - Memory limit in megabytes (e.g., 500 for 500MB)
///
/// # Returns
///
/// * `SUCCESS` (0) on success
/// * `ERROR_INVALID_INPUT` if memory_limit_mb is invalid
/// * `ERROR_UNKNOWN` if initialization fails
///
/// # Safety
///
/// Safe to call from multiple threads. Uses Once for synchronization.
#[no_mangle]
pub extern "C" fn consciousness_init(memory_limit_mb: u32) -> i32 {
    // Validate input
    if memory_limit_mb == 0 || memory_limit_mb > 10000 {
        eprintln!("[FFI] Invalid memory limit: {}MB", memory_limit_mb);
        return ERROR_INVALID_INPUT;
    }
    
    let mut result = SUCCESS;
    
    INIT.call_once(|| {
        eprintln!("[FFI] Initializing consciousness system with {}MB memory limit", memory_limit_mb);
        
        match initialize_orchestrator(memory_limit_mb) {
            Ok(orchestrator) => {
                unsafe {
                    ORCHESTRATOR = Some(Arc::new(Mutex::new(orchestrator)));
                }
                eprintln!("[FFI] Consciousness system initialized successfully");
            }
            Err(e) => {
                eprintln!("[FFI] Failed to initialize consciousness system: {}", e);
                result = ERROR_UNKNOWN;
            }
        }
    });
    
    result
}

/// Internal function to create orchestrator
fn initialize_orchestrator(memory_limit_mb: u32) -> Result<ConsciousnessOrchestrator, String> {
    use crate::navigation::registry::DimensionRegistry;
    use crate::learning::LearningSystem;
    
    // Create dimension registry
    let registry = Arc::new(DimensionRegistry::new());
    
    // Create navigation system
    let navigation = Arc::new(
        NavigationSystem::new(registry)
            .map_err(|e| format!("Navigation init failed: {}", e))?
    );
    
    // Create memory manager
    let memory = Arc::new(
        MmapManager::new(memory_limit_mb as usize)
            .map_err(|e| format!("Memory init failed: {}", e))?
    );
    
    // Create learning system
    let learning = LearningSystem::new();
    
    // Create orchestrator with default config
    let config = ConsciousnessConfig::default();
    let orchestrator = ConsciousnessOrchestrator::with_config(navigation, memory, config, learning);
    
    Ok(orchestrator)
}

/// Get reference to global orchestrator
///
/// # Safety
///
/// Must call consciousness_init() first
fn get_orchestrator() -> Option<Arc<Mutex<ConsciousnessOrchestrator>>> {
    unsafe { ORCHESTRATOR.clone() }
}

/// Process query through consciousness system
///
/// # Arguments
///
/// * `request` - Query request with query string, session_id, max_iterations
/// * `response` - Output response structure (caller must free with consciousness_free_response)
///
/// # Returns
///
/// * `SUCCESS` (0) on success
/// * `ERROR_NOT_INITIALIZED` if consciousness_init() not called
/// * `ERROR_INVALID_INPUT` if request is invalid
/// * `ERROR_SECURITY_VIOLATION` if query violates Asimov laws
/// * `ERROR_TIMEOUT` if processing exceeds timeout
/// * Other error codes for specific failures
///
/// # Safety
///
/// - request must be valid pointer with valid C strings
/// - response must be valid pointer
/// - Caller must free response with consciousness_free_response()
#[no_mangle]
pub unsafe extern "C" fn consciousness_process_query(
    request: *const CQueryRequest,
    response: *mut CQueryResponse,
) -> i32 {
    // Validate inputs
    if request.is_null() || response.is_null() {
        eprintln!("[FFI] Null pointer in process_query");
        return ERROR_INVALID_INPUT;
    }
    
    // Initialize response with defaults
    *response = CQueryResponse::default();
    
    // Get orchestrator
    let orchestrator = match get_orchestrator() {
        Some(orch) => orch,
        None => {
            eprintln!("[FFI] Consciousness system not initialized");
            (*response).error_code = ERROR_NOT_INITIALIZED;
            (*response).error_message = to_c_string("Consciousness system not initialized".to_string());
            return ERROR_NOT_INITIALIZED;
        }
    };
    
    // Parse request
    let req = &*request;
    let query = match from_c_string(req.query) {
        Some(q) => q,
        None => {
            eprintln!("[FFI] Invalid query string");
            (*response).error_code = ERROR_INVALID_INPUT;
            (*response).error_message = to_c_string("Invalid query string".to_string());
            return ERROR_INVALID_INPUT;
        }
    };
    
    let session_id = from_c_string(req.session_id)
        .unwrap_or_else(|| uuid::Uuid::new_v4().to_string());
    
    eprintln!("[FFI] Processing query: session_id={}, query_len={}", session_id, query.len());
    
    // Process query (placeholder - will be replaced with real implementation in Task 5)
    let start_time = std::time::Instant::now();
    
    let result = process_query_internal(&orchestrator, &query, &session_id, req.max_iterations);
    
    let processing_time = start_time.elapsed().as_millis() as i64;
    
    // Fill response
    match result {
        Ok((answer, frequency, dimensions, iterations, return_to_source)) => {
            (*response).session_id = to_c_string(session_id);
            (*response).answer = to_c_string(answer);
            (*response).dominant_frequency = frequency;
            
            let (dims_ptr, dims_count) = strings_to_c_array(dimensions);
            (*response).dimensions_activated = dims_ptr;
            (*response).dimensions_count = dims_count;
            
            (*response).iterations_completed = iterations;
            (*response).return_to_source_triggered = return_to_source;
            (*response).processing_time_ms = processing_time;
            (*response).error_code = SUCCESS;
            
            eprintln!("[FFI] Query processed successfully: iterations={}, time={}ms", iterations, processing_time);
            SUCCESS
        }
        Err(e) => {
            eprintln!("[FFI] Query processing failed: {}", e);
            (*response).error_code = ERROR_UNKNOWN;
            (*response).error_message = to_c_string(e);
            ERROR_UNKNOWN
        }
    }
}

/// Internal query processing (placeholder for now)
///
/// Returns: (answer, frequency, dimensions, iterations, return_to_source)
fn process_query_internal(
    orchestrator: &Arc<Mutex<ConsciousnessOrchestrator>>,
    query: &str,
    session_id: &str,
    max_iterations: u32,
) -> Result<(String, f32, Vec<String>, u32, bool), String> {
    // Lock orchestrator
    let mut orch = orchestrator.lock()
        .map_err(|e| format!("Failed to lock orchestrator: {}", e))?;
    
    // TODO: This is a placeholder implementation
    // Real implementation will come in Task 5 with LLM integration
    
    // For now, simulate basic processing
    let answer = format!(
        "[Placeholder Response]\n\nQuery: {}\nSession: {}\n\nThis is a simulated response. Real LLM integration will be added in Task 4-5.",
        query, session_id
    );
    
    let frequency = 1.5; // Simulated frequency
    let dimensions = vec!["D10-Meta".to_string()]; // Simulated dimension
    let iterations = std::cmp::min(max_iterations, 6); // Simulated iterations
    let return_to_source = false;
    
    Ok((answer, frequency, dimensions, iterations, return_to_source))
}

/// Get learning metrics
///
/// # Arguments
///
/// * `metrics` - Output metrics structure
///
/// # Returns
///
/// * `SUCCESS` (0) on success
/// * `ERROR_NOT_INITIALIZED` if not initialized
///
/// # Safety
///
/// metrics must be valid pointer
#[no_mangle]
pub unsafe extern "C" fn consciousness_get_metrics(metrics: *mut CMetrics) -> i32 {
    if metrics.is_null() {
        return ERROR_INVALID_INPUT;
    }
    
    let orchestrator = match get_orchestrator() {
        Some(orch) => orch,
        None => {
            eprintln!("[FFI] Consciousness system not initialized");
            return ERROR_NOT_INITIALIZED;
        }
    };
    
    let orch = match orchestrator.lock() {
        Ok(o) => o,
        Err(e) => {
            eprintln!("[FFI] Failed to lock orchestrator: {}", e);
            return ERROR_UNKNOWN;
        }
    };
    
    // Get metrics from learning system
    let learning_metrics = orch.learning().metrics();
    
    (*metrics).observation_count = learning_metrics.observation_count;
    (*metrics).pattern_count = learning_metrics.pattern_count;
    (*metrics).proto_dimension_count = learning_metrics.proto_dimension_count;
    (*metrics).crystallization_success_rate = learning_metrics.crystallization_success_rate() as f32;
    (*metrics).memory_usage = learning_metrics.memory_usage;
    (*metrics).memory_limit = learning_metrics.memory_limit;
    
    SUCCESS
}

/// Cleanup consciousness system
///
/// Frees all resources. After calling this, consciousness_init() must be called again.
///
/// # Returns
///
/// * `SUCCESS` (0) on success
/// * `ERROR_NOT_INITIALIZED` if not initialized
///
/// # Safety
///
/// Not thread-safe during cleanup. Ensure no other threads are using the system.
#[no_mangle]
pub extern "C" fn consciousness_cleanup() -> i32 {
    eprintln!("[FFI] Cleaning up consciousness system");
    
    unsafe {
        if ORCHESTRATOR.is_none() {
            eprintln!("[FFI] Consciousness system not initialized");
            return ERROR_NOT_INITIALIZED;
        }
        
        ORCHESTRATOR = None;
    }
    
    eprintln!("[FFI] Consciousness system cleaned up successfully");
    SUCCESS
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_string_conversion() {
        let rust_str = "Hello, World!".to_string();
        let c_str = to_c_string(rust_str.clone());
        
        assert!(!c_str.is_null());
        
        unsafe {
            let back = from_c_string(c_str);
            assert_eq!(back, Some(rust_str));
            consciousness_free_string(c_str);
        }
    }
    
    #[test]
    fn test_null_string() {
        unsafe {
            let result = from_c_string(ptr::null());
            assert_eq!(result, None);
        }
    }
    
    #[test]
    fn test_strings_to_c_array() {
        let strings = vec![
            "dimension1".to_string(),
            "dimension2".to_string(),
            "dimension3".to_string(),
        ];
        
        let (ptr, count) = strings_to_c_array(strings.clone());
        
        assert!(!ptr.is_null());
        assert_eq!(count, 3);
        
        unsafe {
            for i in 0..count {
                let c_str = *ptr.add(i);
                assert!(!c_str.is_null());
                let rust_str = from_c_string(c_str);
                assert_eq!(rust_str, Some(strings[i].clone()));
                consciousness_free_string(c_str);
            }
            
            // Free the array
            let _ = Vec::from_raw_parts(ptr, count, count);
        }
    }
}
