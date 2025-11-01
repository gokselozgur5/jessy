# ADR-004: Parallel Dimension Scanning with Synesthetic Matching

## Status
Accepted

## Context
The consciousness system must scan 14 dimensions to find relevant layers for each query. Sequential scanning would take 14 × 10ms = 140ms, exceeding our <100ms target. Additionally:
- Literal keyword matching misses semantic relationships
- User-specific patterns should strengthen over time
- Complexity threshold (6+ dimensions) requires early detection
- Path confidence scoring determines which dimensions activate

We need a navigation system that is:
1. **Fast**: Complete scan within 100ms
2. **Accurate**: Find relevant dimensions with high confidence
3. **Adaptive**: Learn from successful navigation patterns
4. **Intelligent**: Use synesthetic associations, not just literal matches

## Decision
Implement **parallel dimension scanning with synesthetic keyword matching**:

**Architecture**:
1. **MultiverseNavigator**: Orchestrates parallel scanning across all dimensions
2. **SynestheticEngine**: Maintains cross-sensory keyword associations
3. **PathSelector**: Scores and selects top paths based on confidence
4. **NavigationPath**: Represents journey through dimensional layers

**Key Principles**:
- **Parallel execution**: Scan all 14 dimensions simultaneously using tokio
- **Synesthetic learning**: Keywords strengthen associations when co-activated
- **Confidence scoring**: Multiple factors (literal match, synesthetic, depth)
- **Complexity detection**: Return-to-source when >6 dimensions activate
- **Adaptive decay**: Unused associations weaken over time (0.95x per day)

**Algorithm**:
```rust
async fn scan_dimensions_parallel(
    &self,
    query: &QueryAnalysis,
) -> Result<Vec<DimensionActivation>> {
    let dimensions = CoreDimension::all();
    
    // Create futures for parallel scanning
    let scan_futures: Vec<_> = dimensions
        .into_iter()
        .map(|dim| self.scan_single_dimension(dim, query))
        .collect();
    
    // Execute all scans in parallel with 100ms timeout
    let timeout = Duration::from_millis(100);
    let results = tokio::time::timeout(
        timeout,
        futures::future::join_all(scan_futures),
    ).await?;
    
    // Filter by confidence threshold (0.3 minimum)
    Ok(results.into_iter()
        .filter_map(|r| r.ok())
        .filter(|a| a.confidence >= 0.3)
        .collect())
}
```

**Synesthetic Learning**:
```rust
// When "empathy" and "compassion" co-occur
synesthetic_engine.strengthen_association("empathy", "compassion");
// Future queries with "empathy" will also match "compassion" layers
// with 0.8x confidence (discounted for indirect match)
```

## Consequences

### Positive
- **Fast**: Parallel execution achieves <100ms target (typically 40-60ms)
- **Scalable**: Adding dimensions doesn't increase latency
- **Adaptive**: System learns user-specific patterns over time
- **Intelligent**: Synesthetic matching finds semantic relationships
- **Robust**: Timeout prevents any single dimension from blocking

### Negative
- **Complexity**: Parallel code harder to debug than sequential
- **Resource usage**: 14 concurrent tasks consume more memory/CPU
- **Learning overhead**: Synesthetic associations require storage and maintenance
- **Tuning required**: Confidence thresholds need calibration
- **Race conditions**: Parallel access to shared state needs synchronization

### Neutral
- **Learning rate**: 1.1x growth, 0.95x decay (configurable)
- **Confidence threshold**: 0.3 minimum (adjustable per deployment)
- **Complexity threshold**: 6 dimensions (based on cognitive load research)

## Alternatives Considered

### Alternative 1: Sequential Scanning
**Description**: Scan dimensions one at a time

**Pros**:
- Simpler implementation
- Easier to debug
- Lower resource usage
- Deterministic ordering

**Cons**:
- 140ms+ latency (violates requirement)
- Cannot meet <100ms target
- Wastes CPU on single-threaded execution

**Why not chosen**: Cannot meet performance requirement

### Alternative 2: Priority-Based Scanning
**Description**: Scan high-priority dimensions first, stop when enough found

**Pros**:
- Can achieve <100ms
- Lower resource usage than full parallel
- Focuses on likely dimensions

**Cons**:
- Misses relevant low-priority dimensions
- Requires manual priority tuning
- Biases toward common patterns
- Cannot adapt to user-specific needs

**Why not chosen**: Sacrifices accuracy for speed; parallel scanning achieves both

### Alternative 3: ML-Based Dimension Prediction
**Description**: Train model to predict relevant dimensions

**Pros**:
- Could be very fast (single inference)
- Learns complex patterns
- Adapts to data

**Cons**:
- Requires training data
- Black box (no interpretability)
- Cannot explain dimension selection
- Inference latency still 10-20ms

**Why not chosen**: Lack of interpretability; synesthetic learning provides transparency

### Alternative 4: Literal Keyword Matching Only
**Description**: Match query keywords directly to layer keywords

**Pros**:
- Simple and fast
- Deterministic
- Easy to debug

**Cons**:
- Misses semantic relationships
- Cannot learn patterns
- Brittle to vocabulary differences
- No adaptation over time

**Why not chosen**: Too simplistic; synesthetic matching significantly improves accuracy

## Implementation Notes
- Uses `tokio::spawn` for parallel dimension scanning
- `futures::future::join_all` waits for all scans to complete
- `tokio::time::timeout` enforces 100ms hard limit
- Synesthetic associations stored in `HashMap<String, Vec<Association>>`
- Learning rate: 1.1x (10% growth on successful co-activation)
- Decay rate: 0.95x (5% decay per day unused)
- Confidence calculation: `literal_match * 1.0 + synesthetic_match * 0.8`
- Complexity check: `strong_activations.len() > 6` triggers return-to-source

## References
- [Synesthesia](https://en.wikipedia.org/wiki/Synesthesia)
- [Parallel Computing Patterns](https://patterns.eecs.berkeley.edu/)
- [Tokio Async Runtime](https://tokio.rs/)
- [Navigation Module](../src/navigation/)
- [Navigation System Spec](../specs/navigation-system-spec.md)
- ADR-001: MMAP for Memory Management

## Metadata
- **Date**: 2024-10-24
- **Author**: Core Team
- **Reviewers**: Performance Team, Architecture Team
- **Related ADRs**: ADR-001, ADR-003
- **Tags**: navigation, parallel, synesthetic, performance, learning

---

*"Scan in parallel, learn from patterns, adapt over time. Intelligence emerges from iteration."*
