# Implementation Plan

- [ ] 1. Add Timeout error variant to ConsciousnessError
  - Add `Timeout(String)` variant to `ConsciousnessError` enum in `src/lib.rs`
  - _Requirements: 1.2, 2.5_

- [ ] 2. Update LLMConfig default timeout
  - Change default timeout from 30 to 45 seconds in `src/llm/config.rs`
  - Update `from_env()` to use 45 as default
  - _Requirements: 1.1, 1.5_

- [ ] 3. Add timeout wrapper to LLMManager
  - [ ] 3.1 Add `timeout_duration` field to LLMManager struct
    - Store `Duration::from_secs(config.timeout_secs)` in LLMManager
    - _Requirements: 1.4_
  
  - [ ] 3.2 Wrap generate_with_system_prompt with tokio::time::timeout
    - Use `tokio::time::timeout(self.timeout_duration, provider_call).await`
    - Handle `Err(_)` case as timeout error
    - Return `ConsciousnessError::Timeout` with descriptive message
    - _Requirements: 1.2, 1.4_
  
  - [ ] 3.3 Add timeout logging
    - Log timeout events with query context
    - Log successful call durations for monitoring
    - _Requirements: 2.1, 2.3, 2.4_

- [ ] 4. Add fallback observation to Observation struct
  - Add `fallback()` constructor method to `src/observer_chain/observation.rs`
  - Create observation with low confidence (0.3) and D02 (Cognition) layer
  - _Requirements: 3.1, 3.2_

- [ ] 5. Add timeout handling to ObserverChain
  - [ ] 5.1 Catch timeout errors in observe() method
    - Match on `ConsciousnessError::Timeout` from LLM call
    - Log timeout with stage and query context
    - _Requirements: 1.3, 2.2_
  
  - [ ] 5.2 Implement Stage 1 fallback
    - Return `Observation::fallback()` if Stage 1 times out
    - Use message: "I understand your question. Let me think about this..."
    - _Requirements: 3.1_
  
  - [ ] 5.3 Implement Stage 2+ error propagation
    - Propagate timeout error if Stage 2+ times out
    - Allow orchestrator to handle with partial results
    - _Requirements: 3.2_

- [ ] 6. Add timeout recovery to Orchestrator
  - [ ] 6.1 Catch timeout errors from observer chain
    - Match on `ConsciousnessError::Timeout` in process() method
    - Log timeout event with context
    - _Requirements: 1.3, 2.1_
  
  - [ ] 6.2 Implement partial result recovery
    - Check if any observations exist in context
    - Return latest observation as partial result if available
    - _Requirements: 3.2, 3.4_
  
  - [ ] 6.3 Implement complete failure handling
    - Return user-facing timeout error if no partial results
    - Include retry suggestion in error message
    - _Requirements: 1.2, 2.5, 3.3_

- [ ] 7. Update WebSocket error handling
  - Catch timeout errors in websocket handler (`src/api/websocket.rs`)
  - Send error message to client with type "error"
  - Include retry suggestion in message
  - _Requirements: 2.5, 3.3_

- [ ] 8. Update REST API error handling
  - Catch timeout errors in chat endpoint (`src/api/chat.rs`)
  - Return 504 Gateway Timeout status
  - Include retry_after field in JSON response
  - _Requirements: 2.5, 3.3_

- [ ] 9. Build and deploy
  - [ ] 9.1 Build project locally
    - Run `cargo build --release`
    - Verify no compilation errors
  
  - [ ] 9.2 Test timeout locally
    - Set `LLM_TIMEOUT_SECS=5` for quick testing
    - Send test request and verify timeout triggers
    - Verify error message displayed correctly
  
  - [ ] 9.3 Deploy to Fly.io
    - Run `fly deploy`
    - Verify deployment successful
    - Test with real requests
