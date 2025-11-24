# Design Document

## Overview

Fix Jessy's infinite waiting issue by adding explicit timeout handling to all LLM API calls. The root cause is that reqwest's built-in timeout doesn't catch cases where the connection stays open but no data is sent. We'll wrap all LLM calls with `tokio::time::timeout` to enforce hard deadlines.

## Architecture

### Current Flow (Problematic)
```
User Request → WebSocket/API
  → Orchestrator.process()
    → ObserverChain.observe()
      → LLMManager.generate_with_system_prompt()
        → AnthropicProvider.call_api_with_system()
          → reqwest.send().await  ❌ Hangs here if no data
```

### New Flow (With Timeout)
```
User Request → WebSocket/API
  → Orchestrator.process()
    → ObserverChain.observe()
      → LLMManager.generate_with_system_prompt()
        → tokio::time::timeout(45s, async {  ✅ Hard timeout
            AnthropicProvider.call_api_with_system()
              → reqwest.send().await
          })
        → Handle timeout error gracefully
```

## Components and Interfaces

### 1. LLMManager Timeout Wrapper

**Location:** `src/llm/mod.rs`

**Changes:**
- Add `timeout_duration` field to `LLMManager`
- Wrap `generate_with_system_prompt()` with `tokio::time::timeout`
- Return descriptive error on timeout

**Interface:**
```rust
impl LLMManager {
    pub async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        context: &IterationContext,
    ) -> Result<String> {
        // Wrap provider call with timeout
        match tokio::time::timeout(
            self.timeout_duration,
            self.provider.generate_with_system_prompt(system_prompt, user_prompt, context)
        ).await {
            Ok(Ok(response)) => Ok(response),
            Ok(Err(e)) => Err(e),  // Provider error
            Err(_) => Err(ConsciousnessError::Timeout(format!(
                "LLM call timed out after {:?}",
                self.timeout_duration
            ))),
        }
    }
}
```

### 2. ObserverChain Timeout Handling

**Location:** `src/observer_chain/chain.rs`

**Changes:**
- Catch timeout errors from LLM calls
- Log timeout with context (stage, query, duration)
- Return graceful fallback response

**Fallback Strategy:**
- **Stage 1 timeout:** Return simple acknowledgment response
- **Stage 2 timeout:** Return Stage 1 observation as final answer

**Interface:**
```rust
async fn observe(&self, stage: usize, context: &ChainContext) -> Result<Observation> {
    match self.llm.generate_with_system_prompt(...).await {
        Ok(response) => Observation::parse(response, stage),
        Err(ConsciousnessError::Timeout(msg)) => {
            eprintln!("[ObserverChain] Stage {} timed out: {}", stage, msg);
            
            // Fallback response
            if stage == 1 {
                // Stage 1: Simple acknowledgment
                Ok(Observation::fallback(stage, "I understand your question. Let me think about this..."))
            } else {
                // Stage 2+: Return previous observation
                Err(ConsciousnessError::Timeout(msg))
            }
        }
        Err(e) => Err(e),
    }
}
```

### 3. Orchestrator Timeout Recovery

**Location:** `src/processing/orchestrator.rs`

**Changes:**
- Catch timeout errors from observer chain
- Return partial response if Stage 1 succeeded
- Log timeout for monitoring

**Recovery Strategy:**
```rust
let (final_answer, chain_length, converged) = if let Some(ref observer_chain) = self.observer_chain {
    match observer_chain.process(query, conversation).await {
        Ok(result) => (result.final_observation.content, result.chain_length, ...),
        Err(ConsciousnessError::Timeout(msg)) => {
            eprintln!("[Orchestrator] Observer chain timed out: {}", msg);
            
            // Check if we have any observations from earlier stages
            if let Some(partial) = context.latest_observation() {
                (partial.content, 1, false)  // Return partial result
            } else {
                // Complete failure - return error to user
                return Err(ConsciousnessError::Timeout(format!(
                    "Response generation timed out after 45 seconds. Please try again."
                )));
            }
        }
        Err(e) => return Err(e),
    }
} else {
    // Fallback without observer chain
    ...
};
```

### 4. Configuration

**Environment Variable:**
- `LLM_TIMEOUT_SECS` - Timeout duration in seconds (default: 45)

**Location:** `src/llm/config.rs`

**Changes:**
```rust
pub struct LLMConfig {
    pub timeout_secs: u64,  // Already exists
    // ... other fields
}

impl LLMConfig {
    pub fn from_env() -> Result<Self> {
        let timeout_secs = env::var("LLM_TIMEOUT_SECS")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(45);  // Change default from 30 to 45
        
        // ... rest of config
    }
}
```

## Data Models

### New Error Variant

**Location:** `src/lib.rs`

```rust
#[derive(Debug, thiserror::Error)]
pub enum ConsciousnessError {
    // ... existing variants
    
    #[error("Operation timed out: {0}")]
    Timeout(String),
}
```

### Observation Fallback

**Location:** `src/observer_chain/observation.rs`

```rust
impl Observation {
    /// Create fallback observation for timeout scenarios
    pub fn fallback(stage: usize, content: impl Into<String>) -> Self {
        Self {
            stage,
            cognitive_layers: vec![DimensionId(2)],  // D02: Cognition
            content: content.into(),
            confidence: 0.3,  // Low confidence for fallback
        }
    }
}
```

## Error Handling

### Timeout Error Flow

1. **LLM Call Timeout:**
   - `tokio::time::timeout` expires
   - Return `ConsciousnessError::Timeout`
   - Log: `[LLM] Call timed out after 45s`

2. **Observer Chain Timeout:**
   - Catch timeout from LLM
   - Log: `[ObserverChain] Stage X timed out`
   - Return fallback observation (Stage 1) or propagate error (Stage 2+)

3. **Orchestrator Timeout:**
   - Catch timeout from observer chain
   - Check for partial results
   - Return partial result or user-facing error

4. **API Response:**
   - WebSocket: Send error message to client
   - REST API: Return 504 Gateway Timeout with message

### User-Facing Error Messages

**WebSocket:**
```json
{
  "type": "error",
  "message": "Response generation timed out after 45 seconds. The AI is experiencing high load. Please try again."
}
```

**REST API:**
```json
{
  "error": "Response generation timed out after 45 seconds. Please try again.",
  "code": "TIMEOUT",
  "retry_after": 5
}
```

## Testing Strategy

### Unit Tests

1. **LLMManager Timeout Test:**
   - Mock provider that sleeps for 60 seconds
   - Verify timeout triggers after 45 seconds
   - Verify correct error type returned

2. **ObserverChain Fallback Test:**
   - Mock LLM that times out on Stage 1
   - Verify fallback observation created
   - Mock LLM that times out on Stage 2
   - Verify error propagated

3. **Orchestrator Recovery Test:**
   - Mock observer chain that times out after Stage 1
   - Verify partial result returned
   - Mock observer chain that times out on Stage 1
   - Verify error returned to user

### Integration Tests

1. **End-to-End Timeout Test:**
   - Send request to WebSocket endpoint
   - Mock Anthropic API to never respond
   - Verify timeout error received within 45 seconds
   - Verify connection remains stable

2. **Partial Response Test:**
   - Mock Anthropic to succeed on Stage 1, timeout on Stage 2
   - Verify Stage 1 response returned to user
   - Verify metadata indicates incomplete processing

3. **Load Test:**
   - Send 10 concurrent requests
   - Mock 5 to timeout
   - Verify all requests complete (success or timeout)
   - Verify no resource leaks

### Manual Testing

1. **Real API Timeout:**
   - Set `LLM_TIMEOUT_SECS=5`
   - Send complex query
   - Verify timeout triggers
   - Verify error message displayed

2. **Recovery Test:**
   - Trigger timeout
   - Send new request immediately
   - Verify system responds normally

## Performance Considerations

### Timeout Duration

- **45 seconds** chosen as balance between:
  - User patience (most users abandon after 30-60s)
  - API response time (Claude typically responds in 5-20s)
  - Complex queries (may need 30-40s for deep thinking)

### Resource Cleanup

- `tokio::time::timeout` automatically cancels the future on timeout
- No manual cleanup needed
- Locks released automatically via RAII

### Monitoring

Add metrics for:
- Timeout rate (timeouts / total requests)
- Average response time
- Timeout by stage (Stage 1 vs Stage 2)

## Deployment

### Environment Variables

```bash
# Production
LLM_TIMEOUT_SECS=45

# Development (faster feedback)
LLM_TIMEOUT_SECS=30

# Testing (quick timeouts)
LLM_TIMEOUT_SECS=5
```

### Rollout Plan

1. Deploy to staging with `LLM_TIMEOUT_SECS=30`
2. Monitor timeout rate for 24 hours
3. Adjust timeout if needed
4. Deploy to production with `LLM_TIMEOUT_SECS=45`
5. Monitor and alert on timeout rate > 5%

### Rollback Plan

If timeout causes issues:
1. Increase `LLM_TIMEOUT_SECS` to 90
2. Investigate root cause
3. Fix and redeploy

## Open Questions

None - design is straightforward and low-risk.
