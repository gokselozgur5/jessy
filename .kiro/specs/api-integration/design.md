# API Integration Design

## Overview

This design document describes the architecture for integrating JESSY's Rust consciousness core with the Go API layer, enabling full end-to-end query processing with external LLM providers.

**Key Design Principles:**
- **Zero-copy where possible**: Minimize data copying between Rust and Go
- **Thread-safe**: Support concurrent query processing
- **Fail-safe**: Graceful error handling at all boundaries
- **Observable**: Comprehensive logging and metrics
- **Testable**: Clear interfaces for unit and integration testing

---

## Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                         User/Client                          │
└────────────────┬────────────────────────────────────────────┘
                 │ HTTP/WebSocket
                 ▼
┌─────────────────────────────────────────────────────────────┐
│                      Go API Layer                            │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  HTTP Handlers (Fiber)                               │  │
│  │  - POST /api/v1/query                                │  │
│  │  - GET  /api/v1/stream (WebSocket)                   │  │
│  │  - GET  /api/v1/status                               │  │
│  └──────────────────┬───────────────────────────────────┘  │
│                     │                                        │
│  ┌──────────────────▼───────────────────────────────────┐  │
│  │  CGO Binding Layer                                    │  │
│  │  - Go → C type conversion                            │  │
│  │  - Memory management                                 │  │
│  │  - Error propagation                                 │  │
│  └──────────────────┬───────────────────────────────────┘  │
└────────────────────┬┼───────────────────────────────────────┘
                     ││ CGO/FFI
┌────────────────────▼▼───────────────────────────────────────┐
│                   Rust FFI Layer                             │
│  ┌──────────────────────────────────────────────────────┐  │
│  │  C-Compatible API                                     │  │
│  │  - consciousness_process_query()                      │  │
│  │  - consciousness_init()                               │  │
│  │  - consciousness_cleanup()                            │  │
│  └──────────────────┬───────────────────────────────────┘  │
│                     │                                        │
│  ┌──────────────────▼───────────────────────────────────┐  │
│  │  Consciousness Orchestrator                           │  │
│  │  - Security validation                                │  │
│  │  - Navigation                                         │  │
│  │  - Iteration (9 cycles)                               │  │
│  │  - Learning system                                    │  │
│  └──────────────────┬───────────────────────────────────┘  │
└────────────────────┬┼───────────────────────────────────────┘
                     ││
                     ▼▼
┌─────────────────────────────────────────────────────────────┐
│                    LLM Provider API                          │
│  - OpenAI (GPT-4, GPT-4-turbo)                              │
│  - Anthropic (Claude 3.5 Sonnet)                            │
└─────────────────────────────────────────────────────────────┘
```

---

## Components and Interfaces

### 1. Rust FFI Layer

**File**: `src/ffi/mod.rs`

#### C-Compatible Types

```rust
#[repr(C)]
pub struct CQueryRequest {
    query: *const c_char,
    session_id: *const c_char,
    max_iterations: u32,
}

#[repr(C)]
pub struct CQueryResponse {
    session_id: *const c_char,
    answer: *const c_char,
    dominant_frequency: f32,
    dimensions_activated: *const *const c_char,
    dimensions_count: usize,
    iterations_completed: u32,
    return_to_source_triggered: bool,
    processing_time_ms: i64,
    error_code: i32,
    error_message: *const c_char,
}

#[repr(C)]
pub struct CIterationUpdate {
    session_id: *const c_char,
    iteration: u32,
    max_iterations: u32,
    thought: *const c_char,
    frequency: f32,
    dimensions: *const *const c_char,
    dimensions_count: usize,
    is_complete: bool,
}

#[repr(C)]
pub struct CMetrics {
    observation_count: usize,
    pattern_count: usize,
    proto_dimension_count: usize,
    crystallization_success_rate: f32,
    memory_usage: usize,
    memory_limit: usize,
}
```

#### FFI Functions

```rust
#[no_mangle]
pub extern "C" fn consciousness_init(memory_limit_mb: u32) -> i32;

#[no_mangle]
pub extern "C" fn consciousness_process_query(
    request: *const CQueryRequest,
    response: *mut CQueryResponse,
) -> i32;

#[no_mangle]
pub extern "C" fn consciousness_get_metrics(
    metrics: *mut CMetrics,
) -> i32;

#[no_mangle]
pub extern "C" fn consciousness_cleanup() -> i32;

#[no_mangle]
pub extern "C" fn consciousness_free_string(ptr: *mut c_char);

#[no_mangle]
pub extern "C" fn consciousness_free_response(response: *mut CQueryResponse);
```

#### Error Codes

```rust
pub const SUCCESS: i32 = 0;
pub const ERROR_INVALID_INPUT: i32 = 1;
pub const ERROR_SECURITY_VIOLATION: i32 = 2;
pub const ERROR_NAVIGATION_FAILED: i32 = 3;
pub const ERROR_ITERATION_FAILED: i32 = 4;
pub const ERROR_LLM_API_FAILED: i32 = 5;
pub const ERROR_TIMEOUT: i32 = 6;
pub const ERROR_MEMORY_LIMIT: i32 = 7;
pub const ERROR_NOT_INITIALIZED: i32 = 8;
```

### 2. CGO Binding Layer

**File**: `api/consciousness_bridge.go`

```go
package main

/*
#cgo LDFLAGS: -L../target/release -ljessy
#include <stdlib.h>

typedef struct {
    const char* query;
    const char* session_id;
    unsigned int max_iterations;
} CQueryRequest;

typedef struct {
    const char* session_id;
    const char* answer;
    float dominant_frequency;
    const char** dimensions_activated;
    size_t dimensions_count;
    unsigned int iterations_completed;
    bool return_to_source_triggered;
    long long processing_time_ms;
    int error_code;
    const char* error_message;
} CQueryResponse;

typedef struct {
    size_t observation_count;
    size_t pattern_count;
    size_t proto_dimension_count;
    float crystallization_success_rate;
    size_t memory_usage;
    size_t memory_limit;
} CMetrics;

extern int consciousness_init(unsigned int memory_limit_mb);
extern int consciousness_process_query(const CQueryRequest* request, CQueryResponse* response);
extern int consciousness_get_metrics(CMetrics* metrics);
extern int consciousness_cleanup();
extern void consciousness_free_string(char* ptr);
extern void consciousness_free_response(CQueryResponse* response);
*/
import "C"
import (
    "errors"
    "unsafe"
)

// Initialize consciousness system
func InitConsciousness(memoryLimitMB uint32) error {
    result := C.consciousness_init(C.uint(memoryLimitMB))
    if result != 0 {
        return errors.New("failed to initialize consciousness system")
    }
    return nil
}

// Process query through consciousness system
func ProcessQueryNative(query string, sessionID string, maxIterations uint32) (*QueryResponse, error) {
    // Convert Go strings to C strings
    cQuery := C.CString(query)
    defer C.free(unsafe.Pointer(cQuery))
    
    cSessionID := C.CString(sessionID)
    defer C.free(unsafe.Pointer(cSessionID))
    
    // Create request
    request := C.CQueryRequest{
        query: cQuery,
        session_id: cSessionID,
        max_iterations: C.uint(maxIterations),
    }
    
    // Create response
    var response C.CQueryResponse
    
    // Call Rust
    result := C.consciousness_process_query(&request, &response)
    defer C.consciousness_free_response(&response)
    
    if result != 0 {
        errorMsg := C.GoString(response.error_message)
        return nil, errors.New(errorMsg)
    }
    
    // Convert C response to Go
    goResponse := &QueryResponse{
        SessionID: C.GoString(response.session_id),
        Answer: C.GoString(response.answer),
        DominantFrequency: float32(response.dominant_frequency),
        DimensionsActivated: convertCStringArray(
            response.dimensions_activated,
            int(response.dimensions_count),
        ),
        IterationsCompleted: int(response.iterations_completed),
        ReturnToSourceTriggered: bool(response.return_to_source_triggered),
        ProcessingTimeMs: int64(response.processing_time_ms),
        Status: "completed",
    }
    
    return goResponse, nil
}

// Get learning metrics
func GetLearningMetrics() (*LearningMetrics, error) {
    var metrics C.CMetrics
    
    result := C.consciousness_get_metrics(&metrics)
    if result != 0 {
        return nil, errors.New("failed to get metrics")
    }
    
    return &LearningMetrics{
        ObservationCount: int(metrics.observation_count),
        PatternCount: int(metrics.pattern_count),
        ProtoDimensionCount: int(metrics.proto_dimension_count),
        CrystallizationSuccessRate: float32(metrics.crystallization_success_rate),
        MemoryUsage: int(metrics.memory_usage),
        MemoryLimit: int(metrics.memory_limit),
    }, nil
}

// Cleanup consciousness system
func CleanupConsciousness() error {
    result := C.consciousness_cleanup()
    if result != 0 {
        return errors.New("failed to cleanup consciousness system")
    }
    return nil
}

// Helper to convert C string array to Go slice
func convertCStringArray(arr **C.char, count int) []string {
    if count == 0 {
        return []string{}
    }
    
    // Convert C array to Go slice
    cArray := (*[1 << 30]*C.char)(unsafe.Pointer(arr))[:count:count]
    
    result := make([]string, count)
    for i := 0; i < count; i++ {
        result[i] = C.GoString(cArray[i])
    }
    
    return result
}
```

### 3. LLM Provider Integration

**File**: `src/llm/mod.rs`

```rust
pub mod openai;
pub mod anthropic;

pub trait LLMProvider: Send + Sync {
    async fn generate(
        &self,
        prompt: &str,
        context: &IterationContext,
    ) -> Result<String>;
    
    fn name(&self) -> &str;
    fn model(&self) -> &str;
}

pub struct LLMConfig {
    pub provider: String,  // "openai" or "anthropic"
    pub model: String,     // "gpt-4", "claude-3-5-sonnet"
    pub api_key: String,
    pub timeout_secs: u64,
    pub max_retries: u32,
}

pub struct LLMManager {
    provider: Box<dyn LLMProvider>,
    config: LLMConfig,
}

impl LLMManager {
    pub fn new(config: LLMConfig) -> Result<Self> {
        let provider: Box<dyn LLMProvider> = match config.provider.as_str() {
            "openai" => Box::new(openai::OpenAIProvider::new(&config)?),
            "anthropic" => Box::new(anthropic::AnthropicProvider::new(&config)?),
            _ => return Err(Error::InvalidProvider(config.provider.clone())),
        };
        
        Ok(Self { provider, config })
    }
    
    pub async fn generate(
        &self,
        prompt: &str,
        context: &IterationContext,
    ) -> Result<String> {
        self.provider.generate(prompt, context).await
    }
}
```

### 4. Query Processing Pipeline

**Flow Diagram:**

```
Query Received
    │
    ▼
Security Validation (Asimov Laws)
    │
    ├─ Violation → Redirect Response
    │
    ▼
Navigation (Dimension Activation)
    │
    ▼
Iteration Loop (1-9 cycles)
    │
    ├─ Build Context
    ├─ Generate Prompt
    ├─ Call LLM API
    ├─ Process Response
    ├─ Check Convergence
    │   ├─ Converged → Exit Loop
    │   └─ Not Converged → Continue
    │
    ▼
Synthesis (Final Response)
    │
    ▼
Learning (Record Observation)
    │
    ▼
Return Response
```

---

## Data Models

### Query Request

```rust
pub struct QueryRequest {
    pub query: String,
    pub session_id: String,
    pub max_iterations: u32,
    pub options: HashMap<String, String>,
}
```

### Query Response

```rust
pub struct QueryResponse {
    pub session_id: String,
    pub answer: String,
    pub dominant_frequency: f32,
    pub dimensions_activated: Vec<String>,
    pub iterations_completed: u32,
    pub return_to_source_triggered: bool,
    pub processing_time_ms: i64,
    pub iterations: Vec<IterationResult>,
}
```

### Iteration Context

```rust
pub struct IterationContext {
    pub iteration: u32,
    pub query: String,
    pub dimensions: Vec<DimensionId>,
    pub frequency: f32,
    pub previous_thoughts: Vec<String>,
    pub navigation_result: NavigationResult,
}
```

---

## Error Handling

### Error Propagation Strategy

```
Rust Error → FFI Error Code → CGO Error → Go Error → HTTP Error Response
```

### Error Types

1. **Input Validation Errors** (400 Bad Request)
   - Empty query
   - Query too long
   - Invalid characters

2. **Security Errors** (403 Forbidden)
   - Asimov law violation
   - Harmful content detected

3. **Processing Errors** (500 Internal Server Error)
   - Navigation failed
   - Iteration failed
   - LLM API failed

4. **Timeout Errors** (504 Gateway Timeout)
   - Processing exceeded 30s
   - LLM API timeout

5. **Resource Errors** (507 Insufficient Storage)
   - Memory limit exceeded
   - Too many concurrent queries

---

## Testing Strategy

### Unit Tests

1. **FFI Layer Tests** (`src/ffi/tests.rs`)
   - Test C type conversions
   - Test memory management
   - Test error code propagation

2. **CGO Binding Tests** (`api/consciousness_bridge_test.go`)
   - Test Go → C → Rust calls
   - Test string conversions
   - Test concurrent calls

3. **LLM Provider Tests** (`src/llm/tests.rs`)
   - Test API calls (mocked)
   - Test retry logic
   - Test timeout handling

### Integration Tests

1. **End-to-End Tests** (`tests/integration/api_integration_test.rs`)
   - Test full query pipeline
   - Test WebSocket streaming
   - Test learning system integration

2. **Performance Tests** (`benches/api_benchmarks.rs`)
   - Test FFI call overhead
   - Test concurrent query throughput
   - Test memory usage

---

## Performance Considerations

### Optimization Strategies

1. **Zero-Copy String Passing**
   - Use pointers instead of copying
   - Caller manages memory lifecycle

2. **Connection Pooling**
   - Reuse HTTP connections to LLM APIs
   - Pool size: 10 connections

3. **Async Processing**
   - Use Tokio for async Rust
   - Use goroutines for concurrent Go

4. **Caching**
   - Cache dimension metadata
   - Cache navigation results for similar queries

### Performance Targets

- FFI call overhead: <1ms
- Navigation: <100ms
- Single iteration: <3s
- Full query: <30s
- Concurrent queries: 10+
- Memory per query: <500MB

---

## Security Considerations

1. **API Key Management**
   - Load from environment variables only
   - Never log API keys
   - Validate on startup

2. **Input Validation**
   - Sanitize all user input
   - Enforce length limits
   - Check for injection attacks

3. **Memory Safety**
   - Rust's ownership prevents memory bugs
   - Careful FFI boundary management
   - No buffer overflows

4. **Rate Limiting**
   - Limit queries per IP
   - Limit concurrent queries per user
   - Respect LLM API rate limits

---

## Deployment Architecture

### Docker Compose Setup

```yaml
services:
  jessy-core:
    build: .
    environment:
      - RUST_LOG=info
      - MEMORY_LIMIT_MB=500
    volumes:
      - ./data/mmap:/app/data/mmap
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 10s
      timeout: 5s
      retries: 3
  
  jessy-api:
    build: ./api
    environment:
      - OPENAI_API_KEY=${OPENAI_API_KEY}
      - ANTHROPIC_API_KEY=${ANTHROPIC_API_KEY}
      - LLM_PROVIDER=openai
      - LLM_MODEL=gpt-4-turbo
      - GO_ENV=production
    depends_on:
      jessy-core:
        condition: service_healthy
    ports:
      - "3000:3000"
```

---

## Monitoring and Observability

### Metrics to Track

1. **Query Metrics**
   - Total queries processed
   - Average processing time
   - Success/failure rate
   - Convergence rate

2. **LLM API Metrics**
   - API calls per minute
   - API latency (p50, p95, p99)
   - API error rate
   - Token usage

3. **Learning Metrics**
   - Observations recorded
   - Patterns detected
   - Proto-dimensions created
   - Crystallizations completed

4. **System Metrics**
   - Memory usage
   - CPU usage
   - Concurrent queries
   - FFI call overhead

### Logging Strategy

- **Structured JSON logging** in production
- **Pretty console logging** in development
- **Log levels**: ERROR, WARN, INFO, DEBUG, TRACE
- **Context**: session_id, query (truncated), timing

---

*Design Version: 1.0*  
*Date: 2025-10-26*  
*Status: Ready for Implementation*
