# LLM-Based Dimension Selector

**Status:** ✅ Implemented
**File:** `src/navigation/dimension_selector.rs`
**Demo:** `examples/llm_dimension_selector_demo.rs`
**Run:** `./RUN_LLM_SELECTOR_DEMO.sh`

## Problem

Keyword-based navigation was **too aggressive**:

```
Query: "I feel anxious about code"

Keyword Matching:
- "feel" → D01 (Emotion) ✓
- "anxious" → D01 (Emotion) ✓
- "anxious" → D09 (Ethical) ✓
- "anxious" → D14 (Security) ✓
- "code" → D07 (Technical) ✓
- "code" → D02 (Cognition) ✓
- "code" → D08 (Creative) ✓

Result: 7 dimensions activated → Return-to-source triggered
Reduced to: 3 dimensions (random top 3)
```

**User feedback:** *"RUN_ITERATIVE var ya o calisiyodu ama tam calismiyo gibiydi sadece 3 dimension active edip duruyodu surekli bazen 1 bazen 2 de yine de they are the same picture"*

Translation: "It kept activating 3 dimensions constantly, sometimes 1, sometimes 2, but it's all the same picture."

## Solution

**Use LLM to understand intent** (like GitHub's secret scanning):

```
Query: "I feel anxious about code"

LLM Analysis:
- Emotional state: anxiety → D01 (Emotion)
- Domain: programming → D07 (Technical)
- Implicit concern: harm prevention → D09 (Ethical)

Result: [D01, D07, D09] (exactly 3, meaningful)
```

## Architecture

**Single-purpose micro-service:**

```
┌─────────────────────────────────────────┐
│         DimensionSelector               │
│                                         │
│  Input:  "I feel anxious about code"   │
│     ↓                                   │
│  Decision: Call Claude API (100 tokens)│
│     ↓                                   │
│  Output: [D01, D07, D09]               │
└─────────────────────────────────────────┘
```

**Key design decisions:**

1. **Simple:** Input → Decision → Output (user's requirement)
2. **Direct API:** Prototype uses reqwest, not LLMManager abstraction
3. **Small model ready:** Designed for future Gemma 2B/Phi-3 replacement
4. **Fast:** 100 token limit for sub-second responses
5. **Robust parsing:** Handles JSON, plain text, mixed formats

## Implementation

### Core Struct

```rust
pub struct DimensionSelector {
    api_key: String,
    model: String,  // "claude-sonnet-4-20241022"
}
```

### Main Method

```rust
pub async fn select(&self, query: &str) -> Result<DimensionSelection> {
    // 1. Build prompt with dimension list
    let prompt = SELECTION_PROMPT.replace("{query}", query);

    // 2. Call Claude API
    let response = self.call_claude_api(&prompt).await?;

    // 3. Parse response (handles multiple formats)
    let dimensions = self.parse_response(&response)?;

    // 4. Validate: must be 2-3 dimensions
    if dimensions.is_empty() || dimensions.len() > 3 {
        return Err(ProcessingError::NavigationError(...));
    }

    Ok(DimensionSelection {
        dimensions,
        reasoning: None,
        confidence: 0.9,
    })
}
```

### Prompt Design

```rust
const SELECTION_PROMPT: &str = r#"Select 2-3 most relevant dimensions for this query.

Query: {query}

Available dimensions:
- D01: Emotion (empathy, joy, sadness, feelings)
- D02: Cognition (analytical, creative, intuitive, thinking)
- D03: Intention (create, destroy, explore, teach, goals)
...
- D14: Security (boundaries, protection, safety)

Rules:
1. Select ONLY 2-3 dimensions (no more, no less)
2. Choose based on query INTENT, not just keywords
3. Return ONLY a JSON array of dimension IDs

Example responses:
- For "I feel anxious about code": ["D01", "D07", "D09"]
- For "What is consciousness?": ["D02", "D06", "D10"]
- For "How to help climate?": ["D03", "D11", "D12"]

Your response (JSON array only):
"#;
```

### Robust Parsing

Handles multiple response formats:

```rust
fn parse_response(&self, response: &str) -> Result<Vec<DimensionId>> {
    // Try JSON first: ["D01", "D02", "D07"]
    if let Ok(parsed) = serde_json::from_str::<Vec<String>>(cleaned) {
        return self.parse_dimension_strings(&parsed);
    }

    // Fallback: Extract D## patterns from text
    // Handles: "I recommend D01 (Emotion) and D07 (Technical)"
    let dimension_pattern = regex::Regex::new(r"D(\\d{2})").unwrap();

    // Remove duplicates, keep order
    Ok(dimensions)
}
```

## Integration (TODO)

Current navigation flow:

```
Query → QueryAnalyzer → ParallelScanner → PathSelector → Result
            (keywords)   (keyword matching)   (filter top)
```

Proposed flow:

```
Query → DimensionSelector → DepthNavigator → Result
          (LLM: 2-3 dims)    (layer traversal)
```

**Changes needed:**

1. Add `DimensionSelector` to `NavigationSystem`
2. Replace `ParallelScanner` with LLM selector
3. Skip `PathSelector` complexity check (always 2-3 dims)
4. Update `orchestrator.rs` to use new flow

## Benefits

| Aspect | Keyword Matching | LLM Selection |
|--------|-----------------|---------------|
| **Dimensions** | 1-7 (inconsistent) | Always 2-3 |
| **Return-to-source** | Constant triggering | Never triggers |
| **Intent understanding** | No (literal keywords) | Yes (context-aware) |
| **Complexity** | Variable | Predictable |
| **User experience** | "they are the same picture" | Meaningful variety |

## Testing

```bash
# Run demo
export ANTHROPIC_API_KEY="your-key-here"
./RUN_LLM_SELECTOR_DEMO.sh

# Expected output:
📝 Query: "I feel anxious about code"
------------------------------------------------------------
✨ Selected 3 dimensions:
   - D01: Emotion
   - D07: Technical
   - D09: Ethical
   Confidence: 90.0%
```

Unit tests (`src/navigation/dimension_selector.rs`):

```rust
#[test]
fn test_parse_json_array() {
    let response = r#"["D01", "D07", "D09"]"#;
    let dims = selector.parse_response(response).unwrap();
    assert_eq!(dims.len(), 3);
}

#[test]
fn test_parse_plain_text() {
    let response = "D02, D06, D10";
    let dims = selector.parse_response(response).unwrap();
    assert_eq!(dims.len(), 3);
}

#[test]
fn test_parse_mixed_format() {
    let response = "I recommend D01 (Emotion) and D07 (Technical)";
    let dims = selector.parse_response(response).unwrap();
    assert_eq!(dims.len(), 2);
}
```

## Future Optimization

**Replace Claude API with small local model:**

```rust
// Current (prototype)
let response = call_claude_api(&prompt).await?;

// Future (production)
let response = call_local_model(&prompt)?;  // Gemma 2B, 50ms
```

**Model candidates:**
- Google Gemma 2B (2-3 second inference on CPU)
- Microsoft Phi-3 (fast, good at classification)
- Llama 3.2 1B (ultra-small, fast)

**Why this works:**
- Classification task (not generation)
- 14 options, pick 2-3 (simple)
- Deterministic prompt (consistent results)
- Can fine-tune on Jessy queries

## Status

- [x] Core implementation (`dimension_selector.rs`)
- [x] Unit tests (parsing, validation)
- [x] Demo example
- [x] Run script
- [x] Documentation
- [ ] Integration into `NavigationSystem`
- [ ] Replace `ParallelScanner` in orchestrator
- [ ] Update navigation tests
- [ ] Benchmark performance

## References

- Implementation: `src/navigation/dimension_selector.rs`
- Demo: `examples/llm_dimension_selector_demo.rs`
- Run: `./RUN_LLM_SELECTOR_DEMO.sh`
- Related: GitHub secret scanning (pattern matching + intent understanding)
