# API Integration Implementation Plan

This plan transforms the API integration design into actionable coding tasks. Each task builds incrementally toward a fully integrated system where Go API calls Rust consciousness core with real LLM providers.

---

## Task Breakdown

- [ ] 1. Set up Rust FFI foundation
  - [x] 1.1 Create FFI module structure
    - Create `src/ffi/mod.rs` with module declaration
    - Create `src/ffi/types.rs` for C-compatible types
    - Create `src/ffi/functions.rs` for exported functions
    - Update `Cargo.toml` with `crate-type = ["cdylib"]`
    - _Requirements: 1.1-1.8_

  - [ ] 1.2 Define C-compatible types
    - Define `CQueryRequest` struct with `#[repr(C)]`
    - Define `CQueryResponse` struct with `#[repr(C)]`
    - Define `CIterationUpdate` struct with `#[repr(C)]`
    - Define `CMetrics` struct with `#[repr(C)]`
    - Define error code constants
    - _Requirements: 1.1, 1.5, 1.7_

  - [ ] 1.3 Implement string conversion utilities
    - Implement `to_c_string()` for Rust String → C char*
    - Implement `from_c_string()` for C char* → Rust String
    - Implement `free_c_string()` for memory cleanup
    - Add safety checks for null pointers
    - _Requirements: 1.5, 1.6_

- [ ] 2. Implement core FFI functions
  - [x] 2.1 Implement consciousness_init()
    - Create global `CONSCIOUSNESS_ORCHESTRATOR` static
    - Initialize with memory limit parameter
    - Return error code on failure
    - Add thread-safe initialization (Once)
    - _Requirements: 1.2, 1.7, 1.8_

  - [x] 2.2 Implement consciousness_process_query()
    - Parse `CQueryRequest` to Rust types
    - Call `ConsciousnessOrchestrator::process_query()`
    - Convert result to `CQueryResponse`
    - Handle errors and set error codes
    - _Requirements: 1.1, 1.7, 4.1-4.8_

  - [x] 2.3 Implement consciousness_get_metrics()
    - Get metrics from learning system
    - Convert to `CMetrics` struct
    - Return success/error code
    - _Requirements: 1.4, 6.6_

  - [x] 2.4 Implement consciousness_cleanup()
    - Cleanup global orchestrator
    - Free all allocated memory
    - Return success code
    - _Requirements: 1.3_

  - [x] 2.5 Implement memory management functions
    - Implement `consciousness_free_string()`
    - Implement `consciousness_free_response()`
    - Add safety checks for double-free
    - _Requirements: 1.6_

- [ ] 3. Create CGO binding layer
  - [x] 3.1 Create Go bridge file
    - Create `api/consciousness_bridge.go`
    - Add CGO import with C declarations
    - Add build tags for CGO
    - Configure LDFLAGS for Rust library
    - _Requirements: 2.1-2.7_

  - [x] 3.2 Implement Go wrapper functions
    - Implement `InitConsciousness()`
    - Implement `ProcessQueryNative()`
    - Implement `GetLearningMetrics()`
    - Implement `CleanupConsciousness()`
    - _Requirements: 2.1-2.6_

  - [x] 3.3 Implement type conversion helpers
    - Implement `convertCStringArray()`
    - Implement Go string → C string conversion
    - Implement C string → Go string conversion
    - Add defer cleanup for all C allocations
    - _Requirements: 2.2, 2.3, 2.4_

  - [x] 3.4 Implement error handling
    - Convert C error codes to Go errors
    - Add error context (session_id, query)
    - Propagate Rust errors to Go
    - _Requirements: 2.5, 7.1-7.7_

- [ ] 4. Integrate LLM providers
  - [x] 4.1 Create LLM module structure
    - Create `src/llm/mod.rs`
    - Create `src/llm/openai.rs`
    - Create `src/llm/anthropic.rs`
    - Define `LLMProvider` trait
    - _Requirements: 3.1-3.8_

  - [x] 4.2 Implement OpenAI provider
    - Create `OpenAIProvider` struct
    - Implement API client with reqwest
    - Implement `generate()` method
    - Add retry logic with exponential backoff
    - Add timeout handling (30s)
    - _Requirements: 3.1, 3.5, 3.6, 3.7_

  - [x] 4.3 Implement Anthropic provider
    - Create `AnthropicProvider` struct
    - Implement API client with reqwest
    - Implement `generate()` method
    - Add retry logic with exponential backoff
    - Add timeout handling (30s)
    - _Requirements: 3.2, 3.5, 3.6, 3.7_

  - [x] 4.4 Implement LLM manager
    - Create `LLMManager` struct
    - Implement provider selection logic
    - Load API keys from environment
    - Validate API keys on initialization
    - _Requirements: 3.3, 3.4, 9.1-9.8_

  - [x] 4.5 Add LLM logging
    - Log all API calls with latency
    - Log API errors with context
    - Log token usage
    - Never log API keys
    - _Requirements: 3.8, 10.7_

- [ ] 5. Implement query processing pipeline
  - [ ] 5.1 Update ConsciousnessOrchestrator
    - Add `llm_manager: LLMManager` field
    - Initialize LLM manager in constructor
    - Pass LLM manager to iteration processor
    - _Requirements: 4.1-4.8_

  - [ ] 5.2 Update IterationProcessor
    - Add `llm_manager: &LLMManager` parameter
    - Build prompt from iteration context
    - Call `llm_manager.generate()`
    - Process LLM response
    - _Requirements: 4.4, 4.5_

  - [ ] 5.3 Implement prompt building
    - Create `PromptBuilder` struct
    - Build context from navigation result
    - Include previous iteration thoughts
    - Include dimensional context
    - Format for LLM consumption
    - _Requirements: 4.4_

  - [ ] 5.4 Implement response processing
    - Parse LLM response
    - Extract key insights
    - Update iteration context
    - Check for convergence signals
    - _Requirements: 4.5, 4.6_

  - [ ] 5.5 Add timeout handling
    - Wrap processing in timeout (30s)
    - Cancel LLM calls on timeout
    - Return timeout error
    - _Requirements: 4.8, 7.7, 8.3_

- [ ] 6. Implement real-time streaming
  - [ ] 6.1 Add streaming callback to orchestrator
    - Define `StreamCallback` trait
    - Add optional callback to `process_query()`
    - Call callback after each iteration
    - Include iteration details in callback
    - _Requirements: 5.1-5.6_

  - [ ] 6.2 Update Go WebSocket handler
    - Replace simulated streaming with real callbacks
    - Convert Rust callbacks to WebSocket messages
    - Handle callback errors gracefully
    - _Requirements: 5.1-5.6_

  - [ ] 6.3 Implement iteration update conversion
    - Convert `IterationResult` to `CIterationUpdate`
    - Convert `CIterationUpdate` to Go struct
    - Send via WebSocket
    - _Requirements: 5.3, 5.4_

- [ ] 7. Integrate learning system
  - [ ] 7.1 Update process_query to record observations
    - Call `learning_system.observe_interaction()` after each query
    - Pass query, navigation result, iteration result
    - Handle observation errors gracefully
    - _Requirements: 6.1_

  - [ ] 7.2 Add periodic pattern detection
    - Track query count
    - Trigger pattern detection every 100 queries
    - Run in background (non-blocking)
    - _Requirements: 6.2_

  - [ ] 7.3 Add proto-dimension creation
    - Check patterns for confidence >0.85
    - Create proto-dimensions automatically
    - Log creation events
    - _Requirements: 6.3_

  - [ ] 7.4 Add synesthetic enhancement
    - Apply synesthetic learning to navigation
    - Enhance keywords with associations
    - Filter by strength threshold
    - _Requirements: 6.5_

  - [ ] 7.5 Expose learning metrics via FFI
    - Implement `consciousness_get_metrics()`
    - Return observation, pattern, proto-dimension counts
    - Return crystallization success rate
    - _Requirements: 6.6_

- [ ] 8. Add comprehensive error handling
  - [x] 8.1 Define error types
    - Create `FFIError` enum
    - Map to error codes
    - Include error messages
    - _Requirements: 7.1, 7.2_

  - [ ] 8.2 Implement error logging
    - Log all errors with context
    - Include session_id, query, stack trace
    - Use structured logging
    - _Requirements: 7.2_

  - [ ] 8.3 Add panic handling
    - Catch Rust panics at FFI boundary
    - Convert to error codes
    - Prevent Go process crash
    - _Requirements: 7.4_

  - [ ] 8.4 Add input validation
    - Validate query length (1-10000 chars)
    - Check for null pointers
    - Validate session_id format
    - _Requirements: 7.6_

  - [ ] 8.5 Add timeout handling
    - Wrap all operations in timeouts
    - Cancel long-running operations
    - Return timeout errors
    - _Requirements: 7.7, 8.3_

- [ ] 9. Add configuration management
  - [ ] 9.1 Create configuration module
    - Create `src/config/mod.rs`
    - Define `SystemConfig` struct
    - Load from environment variables
    - _Requirements: 9.1-9.8_

  - [ ] 9.2 Add LLM configuration
    - Add `OPENAI_API_KEY` env var
    - Add `ANTHROPIC_API_KEY` env var
    - Add `LLM_PROVIDER` selection
    - Add `LLM_MODEL` selection
    - _Requirements: 9.2-9.5_

  - [ ] 9.3 Add system configuration
    - Add `MEMORY_LIMIT_MB` env var
    - Add `MAX_ITERATIONS` env var (default: 9)
    - Add `QUERY_TIMEOUT_SECS` env var (default: 30)
    - _Requirements: 9.6, 9.7_

  - [ ] 9.4 Add configuration validation
    - Validate all required env vars present
    - Validate API keys format
    - Validate numeric ranges
    - Return clear error messages
    - _Requirements: 9.8_

- [ ] 10. Add monitoring and observability
  - [ ] 10.1 Add query metrics
    - Track total queries processed
    - Track average processing time
    - Track success/failure rate
    - Track convergence rate
    - _Requirements: 10.1_

  - [ ] 10.2 Add LLM API metrics
    - Track API calls per minute
    - Track API latency (p50, p95, p99)
    - Track API error rate
    - Track token usage
    - _Requirements: 10.2, 10.3_

  - [ ] 10.3 Expose metrics endpoint
    - Update Go `/api/v1/status` endpoint
    - Include query metrics
    - Include LLM metrics
    - Include learning metrics
    - _Requirements: 10.1-10.6_

  - [ ] 10.4 Add structured logging
    - Use JSON logging in production
    - Use pretty logging in development
    - Log all queries with session_id
    - Log all API calls with latency
    - _Requirements: 10.6, 10.7, 10.8_

- [ ] 11. Write integration tests
  - [ ] 11.1 Test FFI layer
    - Test `consciousness_init()`
    - Test `consciousness_process_query()`
    - Test `consciousness_get_metrics()`
    - Test `consciousness_cleanup()`
    - Test memory management
    - _Requirements: All_

  - [ ] 11.2 Test CGO bindings
    - Test Go → Rust calls
    - Test string conversions
    - Test error propagation
    - Test concurrent calls
    - _Requirements: All_

  - [ ] 11.3 Test end-to-end pipeline
    - Test full query processing
    - Test with real LLM API (mocked)
    - Test WebSocket streaming
    - Test learning system integration
    - _Requirements: All_

  - [ ] 11.4 Test error scenarios
    - Test invalid input
    - Test security violations
    - Test LLM API failures
    - Test timeouts
    - Test memory limits
    - _Requirements: 7.1-7.7_

- [ ] 12. Create examples and documentation
  - [ ] 12.1 Create API integration example
    - Create `examples/api_integration.rs`
    - Demonstrate FFI usage
    - Demonstrate query processing
    - Demonstrate error handling
    - _Requirements: All_

  - [ ] 12.2 Update API documentation
    - Document FFI functions
    - Document CGO bindings
    - Document configuration
    - Document error codes
    - _Requirements: All_

  - [ ] 12.3 Create deployment guide
    - Document Docker setup
    - Document environment variables
    - Document API key management
    - Document monitoring setup
    - _Requirements: 9.1-9.8, 10.1-10.8_

---

## Implementation Notes

### Build Configuration

**Cargo.toml additions:**
```toml
[lib]
crate-type = ["cdylib", "rlib"]

[dependencies]
reqwest = { version = "0.11", features = ["json"] }
tokio = { version = "1", features = ["full"] }
```

**Go build command:**
```bash
CGO_ENABLED=1 go build -tags cgo
```

### Testing Strategy

1. **Unit tests**: Test each component in isolation
2. **Integration tests**: Test FFI boundary
3. **End-to-end tests**: Test full pipeline with mocked LLM
4. **Manual tests**: Test with real LLM API

### Performance Targets

- FFI call overhead: <1ms
- Navigation: <100ms
- Single iteration: <3s
- Full query: <30s
- Concurrent queries: 10+

---

*Implementation Plan Version: 1.0*  
*Date: 2025-10-26*  
*Estimated Duration: 3-4 days*
