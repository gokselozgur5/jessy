# ADR-002: Security Layer Pattern Matching Architecture

## Status
Accepted

## Context
The consciousness system requires harm prevention that:
- Validates queries within <10ms (99th percentile)
- Detects multiple harm categories (violence, self-harm, hate speech, etc.)
- Provides constructive redirection instead of simple blocking
- Operates as first-line defense before any dimensional processing
- Implements Asimov's Laws hierarchy

Traditional ML-based content moderation is too slow (100-500ms) and requires external API calls. Rule-based systems are fast but brittle. We need a hybrid approach that balances speed, accuracy, and maintainability.

## Decision
Implement a **pattern-based security layer** with keyword matching, confidence scoring, and tiered response:

**Architecture**:
1. **PatternMatcher**: Scans text for harmful patterns using pre-compiled keyword lists
2. **HarmCategory**: Classifies violations by type (Violence, SelfHarm, HateSpeech, etc.)
3. **RedirectionEngine**: Generates constructive alternatives based on violation type
4. **SecurityLayer**: Orchestrates validation with timeout enforcement

**Key Principles**:
- Fail-fast: Security check runs before any other processing
- Timeout-enforced: Hard 10ms limit with graceful degradation
- Confidence-based: Threshold determines blocking (0.7 default, 0.5 strict mode)
- Constructive: Always provide redirection, never just block
- Hierarchical: Asimov's Laws determine precedence

**Implementation**:
```rust
pub struct SecurityLayer {
    pattern_matcher: PatternMatcher,
    redirection: RedirectionEngine,
    config: SecurityConfig { max_validation_time_ms: 10 },
}

pub enum ValidationResult {
    Safe,
    Unsafe(SecurityViolation),
}
```

## Consequences

### Positive
- **Fast**: <10ms validation enables real-time processing
- **Deterministic**: No external API dependencies or network latency
- **Transparent**: Clear pattern matching makes debugging easy
- **Constructive**: Redirection maintains positive user experience
- **Extensible**: Easy to add new patterns and categories

### Negative
- **False positives**: Keyword matching can misinterpret context ("kill time")
- **Maintenance burden**: Pattern lists need regular updates
- **Limited sophistication**: Cannot understand nuanced language
- **Memory overhead**: Pattern storage (~4MB for comprehensive lists)

### Neutral
- **Accuracy trade-off**: 85-90% accuracy vs 95%+ for ML, but 100x faster
- **Pattern evolution**: Requires periodic review and updates
- **Strict mode option**: Allows tuning sensitivity per deployment

## Alternatives Considered

### Alternative 1: External ML API (OpenAI Moderation, Perspective API)
**Description**: Call external content moderation API

**Pros**:
- Higher accuracy (95%+)
- Handles nuanced language
- Continuously updated models

**Cons**:
- 100-500ms latency (violates <10ms requirement)
- External dependency and cost
- Privacy concerns (sends user queries externally)
- Rate limiting and availability issues

**Why not chosen**: Latency requirement is non-negotiable for real-time consciousness processing

### Alternative 2: Local ML Model (BERT-based classifier)
**Description**: Run lightweight ML model locally

**Pros**:
- Better accuracy than patterns (90-93%)
- No external dependencies
- Privacy preserved

**Cons**:
- 20-50ms inference time (too slow)
- 100-500MB model size
- GPU acceleration needed for speed
- Complex deployment and updates

**Why not chosen**: Cannot meet <10ms requirement even with optimization

### Alternative 3: Hybrid ML + Pattern Matching
**Description**: Fast pattern pre-filter, ML for borderline cases

**Pros**:
- Best accuracy for complex cases
- Fast path for clear violations

**Cons**:
- Complexity of maintaining two systems
- Still has ML latency for borderline cases
- Unclear boundary between fast/slow paths

**Why not chosen**: Added complexity not justified for current requirements; can revisit if accuracy becomes critical

## Implementation Notes
- Pattern lists stored in `src/security/patterns.rs`
- Redirection templates in `src/security/redirection.rs`
- Timeout enforced using `tokio::time::timeout`
- Confidence thresholds configurable per deployment
- Crisis resources (hotlines) included for self-harm detection
- Comprehensive test coverage including performance benchmarks

## References
- [Security Module Implementation](../src/security/)
- [Asimov's Laws](https://en.wikipedia.org/wiki/Three_Laws_of_Robotics)
- [Content Moderation Best Practices](https://www.perspectiveapi.com/)
- ADR-001: MMAP for Memory Management

## Metadata
- **Date**: 2024-10-24
- **Author**: Core Team
- **Reviewers**: Security Team, Ethics Board
- **Related ADRs**: ADR-001
- **Tags**: security, performance, ethics, harm-prevention

---

*"Speed and safety are not opposites. Pattern matching proves both are possible."*
