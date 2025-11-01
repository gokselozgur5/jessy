# Design Document

## Overview

This design implements LLM-based dimension selection to replace keyword matching. The LLM analyzes query semantics and selects 3-7 relevant dimensions from the 14 available consciousness layers. This enables universal language support and semantic understanding.

## Architecture

### High-Level Flow

```
Query
  ↓
[LLM Dimension Selector]
  ↓
Dimension IDs + Confidence
  ↓
[OWL Encoder] → Binary Pattern
  ↓
[Memory Loader] → Load selected dimensions
  ↓
[Iteration Engine] → 9 iterations
  ↓
Response
```

### Component Diagram

```
┌─────────────────────────────────────────────────────┐
│           Consciousness Orchestrator                 │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│         LLM Dimension Selector (NEW)                 │
│  ┌─────────────────────────────────────────────┐   │
│  │  1. Build prompt with dimension descriptions │   │
│  │  2. Call LLM API (fast model)                │   │
│  │  3. Parse response (JSON/list)               │   │
│  │  4. Validate dimension IDs                   │   │
│  │  5. Assign confidence scores                 │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│              OWL Pattern Encoder                     │
│  Dimensions [2,4,10] → "01010000010000"             │
└─────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────┐
│              Memory Manager                          │
│  Load contexts only for selected dimensions         │
└─────────────────────────────────────────────────────┘
```

## Components and Interfaces

### 1. LLMDimensionSelector

**Purpose:** Analyzes queries using LLM to select relevant dimensions.

**Interface:**
```rust
pub struct LLMDimensionSelector {
    llm_client: Arc<dyn LLMProvider>,
    dimension_registry: Arc<DimensionRegistry>,
    prompt_template: String,
    cache: Arc<RwLock<LRUCache<String, DimensionSelection>>>,
}

pub struct DimensionSelection {
    pub dimension_ids: Vec<DimensionId>,
    pub confidences: HashMap<DimensionId, f32>,
    pub reasoning: String,
    pub owl_pattern: String, // "01010000010000"
}

impl LLMDimensionSelector {
    pub async fn select_dimensions(&self, query: &str) -> Result<DimensionSelection>;
    pub fn encode_owl_pattern(&self, dimensions: &[DimensionId]) -> String;
    pub fn decode_owl_pattern(&self, pattern: &str) -> Vec<DimensionId>;
}
```

**Responsibilities:**
- Build dimension selection prompt
- Call LLM API with timeout
- Parse LLM response (JSON or list)
- Validate dimension IDs (1-14)
- Assign confidence scores
- Encode/decode OWL patterns
- Cache recent selections

### 2. Dimension Selection Prompt

**Template:**
```
You are JESSY's dimension selector. Analyze this query and select 3-7 relevant dimensions.

Available Dimensions:
1. Emotion - feelings, moods, emotional states
2. Cognition - thinking, reasoning, understanding
3. Intention - goals, purposes, desires
4. Social Context - relationships, interactions
5. Temporal State - time, timing, sequences
6. Philosophical Depth - existential, metaphysical
7. Technical Level - engineering, systems
8. Creative Mode - imagination, innovation
9. Ethical Framework - morals, values
10. Meta-Awareness - self-reference, consciousness
11. Ecological - nature, environment, systems
12. Positivity - optimism, hope, growth
13. Balance - harmony, equilibrium
14. Security - safety, protection, trust

Query: "{query}"

Respond with JSON:
{
  "dimensions": [2, 4, 10],
  "reasoning": "This query involves thinking (2), social interaction (4), and self-awareness (10)"
}

Select 3-7 dimensions. Focus on semantic meaning, not keywords.
```

**Examples:**
```
Query: "Can a person be muslim and gay?"
Response: {
  "dimensions": [6, 9, 4, 10],
  "reasoning": "Philosophical (6) and ethical (9) question about identity, involving social context (4) and self-awareness (10)"
}

Query: "How do I fix this bug in my code?"
Response: {
  "dimensions": [7, 2, 3],
  "reasoning": "Technical problem (7) requiring cognitive analysis (2) with goal of fixing (3)"
}

Query: "I feel sad today"
Response: {
  "dimensions": [1, 5, 10],
  "reasoning": "Emotional state (1) with temporal context (5) and self-awareness (10)"
}
```

### 3. OWL Pattern Encoder

**Purpose:** Binary encoding of active dimensions for efficient representation.

**Interface:**
```rust
pub struct OwlPattern {
    pattern: String, // "01010000010000"
    dimensions: Vec<DimensionId>,
}

impl OwlPattern {
    pub fn encode(dimensions: &[DimensionId]) -> Self;
    pub fn decode(pattern: &str) -> Vec<DimensionId>;
    pub fn to_u16(&self) -> u16; // Binary representation
    pub fn from_u16(value: u16) -> Self;
}
```

**Encoding Rules:**
- 14-bit string, one bit per dimension
- Bit position = dimension ID - 1
- '1' = active, '0' = inactive
- Example: [2,4,10] → "01010000010000"

### 4. Integration with Orchestrator

**Modified Flow:**
```rust
impl ConsciousnessOrchestrator {
    pub async fn process(&mut self, query: &str) -> Result<Response> {
        // Phase 0: LLM Dimension Selection (NEW)
        let selection = if self.config.use_llm_selection {
            self.dimension_selector.select_dimensions(query).await?
        } else {
            // Fallback to keyword-based navigation
            self.navigation.navigate(query).await?
        };
        
        // Phase 1: Memory Loading (only selected dimensions)
        let contexts = self.memory.load_contexts(&selection.dimension_ids)?;
        
        // Phase 2-4: Existing iteration, interference, learning
        // ...
    }
}
```

## Data Models

### DimensionSelection
```rust
pub struct DimensionSelection {
    /// Selected dimension IDs (3-7 typically)
    pub dimension_ids: Vec<DimensionId>,
    
    /// Confidence score per dimension (0.0-1.0)
    pub confidences: HashMap<DimensionId, f32>,
    
    /// LLM reasoning for selection
    pub reasoning: String,
    
    /// Binary OWL pattern
    pub owl_pattern: String,
    
    /// Selection duration in ms
    pub duration_ms: u64,
    
    /// Whether fallback was used
    pub is_fallback: bool,
}
```

### LLMDimensionConfig
```rust
pub struct LLMDimensionConfig {
    /// Enable LLM-based selection
    pub enabled: bool,
    
    /// LLM model to use (fast model preferred)
    pub model: String,
    
    /// Timeout for dimension selection
    pub timeout_ms: u64,
    
    /// Cache size for recent selections
    pub cache_size: usize,
    
    /// Minimum confidence threshold
    pub min_confidence: f32,
    
    /// Default dimensions for fallback
    pub fallback_dimensions: Vec<DimensionId>,
}
```

## Error Handling

### Error Types
```rust
pub enum DimensionSelectionError {
    /// LLM API call failed
    LLMCallFailed { reason: String },
    
    /// LLM response parsing failed
    ParseError { response: String },
    
    /// Invalid dimension IDs in response
    InvalidDimensions { ids: Vec<u8> },
    
    /// Timeout exceeded
    Timeout { duration_ms: u64 },
    
    /// No dimensions selected
    NoDimensionsSelected,
}
```

### Fallback Strategy
1. **LLM call fails** → Use default dimensions [2, 4, 10]
2. **Parse error** → Use default dimensions
3. **Invalid IDs** → Filter out invalid, use remaining
4. **No dimensions** → Activate all 14 dimensions
5. **Timeout** → Use cached result or default

## Testing Strategy

### Unit Tests
- OWL pattern encoding/decoding
- Dimension ID validation
- Confidence score normalization
- Prompt template rendering
- Cache hit/miss logic

### Integration Tests
- LLM dimension selection end-to-end
- Fallback scenarios
- Multi-language queries
- Performance under load
- Cache effectiveness

### BDD Scenarios
```gherkin
Scenario: LLM selects relevant dimensions
  Given a query "Can a person be muslim and gay?"
  When LLM dimension selector analyzes the query
  Then dimensions [6, 9, 4, 10] should be selected
  And confidence scores should be > 0.5
  And OWL pattern should be "00000101100100"

Scenario: Fallback on LLM failure
  Given LLM API is unavailable
  When dimension selection is attempted
  Then default dimensions [2, 4, 10] should be used
  And is_fallback flag should be true
  And system should continue processing

Scenario: Multi-language support
  Given a query in Turkish "Merhaba nasılsın?"
  When LLM dimension selector analyzes the query
  Then dimensions should be selected based on semantics
  And language should not affect accuracy
```

## Performance Considerations

### Optimization Strategies
1. **Fast LLM Model:** Use Claude Haiku or GPT-3.5-turbo (< 1s response)
2. **Caching:** Cache dimension selections for similar queries
3. **Parallel Processing:** Don't block on dimension selection
4. **Streaming:** Use streaming responses when available
5. **Prompt Optimization:** Keep prompt < 500 tokens

### Performance Targets
- Dimension selection: < 2s (p95)
- Cache hit rate: > 30%
- Fallback rate: < 5%
- Total overhead: < 10% of query time

## Security Considerations

### Input Validation
- Sanitize query before sending to LLM
- Validate LLM response format
- Limit query length (< 1000 chars)
- Rate limit dimension selection calls

### API Key Management
- Use separate API key for dimension selection
- Rotate keys regularly
- Monitor API usage
- Set spending limits

## Migration Strategy

### Phase 1: Parallel Mode
- Run both keyword and LLM selection
- Compare results
- Log discrepancies
- Tune prompts

### Phase 2: Gradual Rollout
- Enable LLM selection for 10% of queries
- Monitor performance and accuracy
- Increase to 50%, then 100%
- Keep keyword as fallback

### Phase 3: Full Migration
- LLM selection as primary
- Keyword matching deprecated
- Remove old code
- Update documentation

## Future Enhancements

1. **Learned Dimension Selection:** Train model on historical selections
2. **Context-Aware Selection:** Consider conversation history
3. **Dynamic Confidence:** Adjust based on query complexity
4. **Dimension Relationships:** Learn which dimensions co-activate
5. **User Preferences:** Allow users to bias certain dimensions
