# Spec: Learning System Implementation

## Status
- **Phase**: Design
- **Priority**: P0 (Critical Path)
- **Owner**: Core Team
- **Dependencies**: Memory Manager, Navigation System

## Problem Statement

The consciousness system needs to learn from interactions and crystallize new patterns into permanent dimensional layers. Traditional static systems cannot adapt to user-specific patterns or emerging knowledge domains.

### Why This Matters
- System must evolve with usage patterns
- User-specific dimensions enable personalization
- Pattern detection reveals emergent knowledge structures
- Crystallization converts temporary learning to permanent memory

### Constraints
- Must not exceed 500MB total memory (280MB core + 220MB learning)
- Pattern detection must complete within query processing time
- Crystallization must be background process (non-blocking)
- Minimum 50 observations before proto-dimension creation
- 85% confidence threshold for crystallization

### Success Criteria
- [ ] Pattern detection identifies recurring themes
- [ ] Proto-dimensions created in heap memory
- [ ] Crystallization migrates heap → MMAP successfully
- [ ] Synesthetic associations strengthen over time
- [ ] User-specific dimensions activate correctly
- [ ] Memory usage stays within limits

## Domain Model

### Entities

```rust
/// Learning system coordinator
struct LearningSystem {
    pattern_detector: PatternDetector,
    crystallizer: Crystallizer,
    synesthetic_engine: SynestheticLearner,
    proto_dimensions: HashMap<DimensionId, ProtoDimension>,
    observation_buffer: Vec<Observation>,
}

/// Pattern detection engine
struct PatternDetector {
    observations: Vec<Observation>,
    patterns: HashMap<PatternId, DetectedPattern>,
    confidence_threshold: f32,
    min_observations: usize,
}

/// Crystallization engine (heap → MMAP)
struct Crystallizer {
    pending_crystallizations: Vec<ProtoDimension>,
    crystallization_queue: VecDeque<CrystallizationTask>,
    max_concurrent: usize,
}

/// Synesthetic learning engine
struct SynestheticLearner {
    associations: HashMap<String, Vec<Association>>,
    learning_rate: f32,
    decay_rate: f32,
}

/// Proto-dimension (temporary in heap)
struct ProtoDimension {
    dimension_id: DimensionId,
    content: Vec<u8>,
    observations: Vec<Observation>,
    confidence: f32,
    created_at: SystemTime,
}

/// Observation from interaction
struct Observation {
    query: String,
    activated_dimensions: Vec<DimensionId>,
    keywords: Vec<String>,
    frequency: Frequency,
    timestamp: SystemTime,
}

/// Detected pattern
struct DetectedPattern {
    pattern_id: PatternId,
    keywords: Vec<String>,
    frequency_range: (f32, f32),
    observation_count: usize,
    confidence: f32,
    suggested_dimension: Option<DimensionId>,
}

/// Synesthetic association
struct Association {
    keyword: String,
    associated_keywords: Vec<(String, f32)>, // (keyword, strength)
    activation_count: usize,
    last_activated: SystemTime,
}
```

### State Machine

```
[Observing]
    ↓ 50+ observations
[Pattern Detected]
    ↓ confidence > 85%
[Proto-Dimension Created]
    ↓ validation passed
[Crystallization Queued]
    ↓ background process
[Crystallized to MMAP]
    ↓ index updated
[Active Dimension]
```

### Invariants
1. Proto-dimensions exist only in heap memory
2. Crystallization is atomic (all-or-nothing)
3. Total memory never exceeds 500MB
4. Minimum 50 observations before proto-dimension
5. Confidence must be ≥85% for crystallization
6. Synesthetic associations decay if unused

## Architecture Design

### Component Boundaries

```
┌─────────────────────────────────────┐
│   LearningSystem (Public API)       │
├─────────────────────────────────────┤
│   - observe_interaction()           │
│   - detect_patterns()               │
│   - create_proto_dimension()        │
│   - crystallize()                   │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   PatternDetector (Internal)        │
├─────────────────────────────────────┤
│   - add_observation()               │
│   - analyze_patterns()              │
│   - calculate_confidence()          │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   Crystallizer (Internal)           │
├─────────────────────────────────────┤
│   - queue_crystallization()         │
│   - execute_crystallization()       │
│   - migrate_heap_to_mmap()          │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   SynestheticLearner (Internal)     │
├─────────────────────────────────────┤
│   - strengthen_association()        │
│   - decay_unused()                  │
│   - get_associations()              │
└─────────────────────────────────────┘
```

### Interface Contracts

```rust
impl LearningSystem {
    /// Create new learning system
    pub fn new() -> Self;
    
    /// Observe an interaction for pattern learning
    /// 
    /// # Errors
    /// - LearningError if observation buffer is full
    pub fn observe_interaction(
        &mut self,
        query: &str,
        navigation_result: &NavigationResult,
        iteration_result: &IterationResult,
    ) -> Result<()>;
    
    /// Detect patterns from accumulated observations
    /// 
    /// # Returns
    /// List of detected patterns with confidence scores
    pub fn detect_patterns(&mut self) -> Result<Vec<DetectedPattern>>;
    
    /// Create proto-dimension from detected pattern
    /// 
    /// # Errors
    /// - LearningError if pattern confidence too low
    /// - LearningError if memory limit exceeded
    pub fn create_proto_dimension(
        &mut self,
        pattern: &DetectedPattern,
    ) -> Result<DimensionId>;
    
    /// Crystallize proto-dimension to MMAP (background)
    /// 
    /// # Errors
    /// - LearningError if proto-dimension not ready
    /// - MemoryError if MMAP allocation fails
    pub async fn crystallize(
        &mut self,
        dimension_id: DimensionId,
    ) -> Result<()>;
}
```

### Data Structures

```rust
// Observation buffer: circular buffer with 1000 entries
const OBSERVATION_BUFFER_SIZE: usize = 1000;

// Pattern detection thresholds
const MIN_OBSERVATIONS: usize = 50;
const CONFIDENCE_THRESHOLD: f32 = 0.85;

// Synesthetic learning rates
const LEARNING_RATE: f32 = 1.1;  // 10% growth on activation
const DECAY_RATE: f32 = 0.95;    // 5% decay per day unused

// Memory limits
const MAX_PROTO_DIMENSIONS: usize = 10;
const MAX_PROTO_DIMENSION_SIZE: usize = 16 * 1024 * 1024; // 16MB
```

### Performance Characteristics

- **Pattern Detection**: O(n) where n = observations
- **Crystallization**: O(m) where m = proto-dimension size
- **Synesthetic Lookup**: O(1) with HashMap
- **Memory**: O(k) where k = number of proto-dimensions

## Test Specification

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn test_observation_recording() {
        // Given: Learning system
        let mut system = LearningSystem::new();
        
        // When: Record observation
        let result = system.observe_interaction(
            "test query",
            &navigation_result,
            &iteration_result,
        );
        
        // Then: Success
        assert!(result.is_ok());
        assert_eq!(system.observation_count(), 1);
    }
    
    #[test]
    fn test_pattern_detection() {
        // Given: 50+ similar observations
        let mut system = setup_system_with_observations(60);
        
        // When: Detect patterns
        let patterns = system.detect_patterns().unwrap();
        
        // Then: Pattern detected
        assert!(!patterns.is_empty());
        assert!(patterns[0].confidence > 0.85);
    }
    
    #[test]
    fn test_proto_dimension_creation() {
        // Given: High-confidence pattern
        let mut system = LearningSystem::new();
        let pattern = create_high_confidence_pattern();
        
        // When: Create proto-dimension
        let dimension_id = system.create_proto_dimension(&pattern);
        
        // Then: Success and in heap
        assert!(dimension_id.is_ok());
        assert!(system.has_proto_dimension(dimension_id.unwrap()));
    }
    
    #[test]
    fn test_crystallization() {
        // Given: Proto-dimension ready
        let mut system = setup_system_with_proto_dimension();
        let dimension_id = DimensionId(99);
        
        // When: Crystallize
        let result = system.crystallize(dimension_id).await;
        
        // Then: Success and in MMAP
        assert!(result.is_ok());
        assert!(!system.has_proto_dimension(dimension_id));
        assert!(system.is_crystallized(dimension_id));
    }
    
    #[test]
    fn test_synesthetic_learning() {
        // Given: Synesthetic learner
        let mut learner = SynestheticLearner::new();
        
        // When: Strengthen association
        learner.strengthen_association("empathy", "compassion");
        learner.strengthen_association("empathy", "compassion");
        
        // Then: Association strengthened
        let associations = learner.get_associations("empathy");
        assert!(associations.iter().any(|(kw, strength)| 
            kw == "compassion" && *strength > 1.0
        ));
    }
    
    #[test]
    fn test_memory_limit_enforcement() {
        // Given: System at memory limit
        let mut system = setup_system_at_memory_limit();
        
        // When: Try to create proto-dimension
        let result = system.create_proto_dimension(&pattern);
        
        // Then: Error
        assert!(result.is_err());
        assert_eq!(
            result.unwrap_err(),
            LearningError::MemoryLimitExceeded
        );
    }
}
```

### Integration Tests

```rust
#[tokio::test]
async fn test_full_learning_cycle() {
    // Given: Complete system
    let mut system = ConsciousnessSystem::new().await.unwrap();
    
    // When: Process 60 similar queries
    for i in 0..60 {
        system.process_query(&format!("query about topic {}", i % 5))
            .await
            .unwrap();
    }
    
    // Then: Pattern detected and crystallized
    let patterns = system.learning_system.detect_patterns().unwrap();
    assert!(!patterns.is_empty());
    
    // Crystallize the pattern
    let dimension_id = system.learning_system
        .create_proto_dimension(&patterns[0])
        .unwrap();
    
    system.learning_system.crystallize(dimension_id).await.unwrap();
    
    // Verify dimension is active
    assert!(system.memory_manager.is_dimension_loaded(dimension_id));
}
```

### BDD Scenarios

```gherkin
Feature: Learning System
  As a consciousness system
  I want to learn from interactions
  So that I can adapt to user patterns

  Scenario: Pattern detection from observations
    Given 60 similar queries have been processed
    When the system analyzes patterns
    Then at least one pattern should be detected
    And the pattern confidence should exceed 85%

  Scenario: Proto-dimension creation
    Given a high-confidence pattern exists
    When the system creates a proto-dimension
    Then the proto-dimension should exist in heap memory
    And it should not exceed 16MB in size

  Scenario: Crystallization to MMAP
    Given a proto-dimension is ready for crystallization
    When the crystallization process runs
    Then the content should be migrated to MMAP
    And the heap memory should be freed
    And the dimension should be accessible

  Scenario: Synesthetic association strengthening
    Given keywords "empathy" and "compassion" co-occur
    When they appear together 10 times
    Then their association strength should increase
    And future queries with "empathy" should activate "compassion"

  Scenario: Memory limit enforcement
    Given the system is at 500MB memory limit
    When attempting to create a new proto-dimension
    Then the operation should fail gracefully
    And an appropriate error should be returned
```

## Implementation Plan

### Phase 1: Pattern Detection (Day 1)
- [ ] Create `src/learning/mod.rs` structure
- [ ] Implement `PatternDetector`
- [ ] Add observation recording
- [ ] Implement pattern analysis
- [ ] Write unit tests

### Phase 2: Proto-Dimensions (Day 2)
- [ ] Implement `ProtoDimension` type
- [ ] Add proto-dimension creation logic
- [ ] Implement memory limit checks
- [ ] Add validation logic
- [ ] Write unit tests

### Phase 3: Crystallization (Day 3)
- [ ] Implement `Crystallizer`
- [ ] Add heap → MMAP migration
- [ ] Implement background processing
- [ ] Add error recovery
- [ ] Write integration tests

### Phase 4: Synesthetic Learning (Day 4)
- [ ] Implement `SynestheticLearner`
- [ ] Add association strengthening
- [ ] Implement decay logic
- [ ] Add lookup optimization
- [ ] Write unit tests

### Phase 5: Integration (Day 5)
- [ ] Integrate with `ConsciousnessSystem`
- [ ] Add observation hooks
- [ ] Implement background crystallization
- [ ] Full system testing
- [ ] Performance validation

## Decision Records

### DR-001: Heap for Proto-Dimensions
**Decision**: Use heap memory for proto-dimensions before crystallization

**Rationale**:
- Flexible size during learning phase
- Easy to discard if pattern doesn't crystallize
- No MMAP overhead for temporary data
- Simple memory management

**Trade-offs**:
- Heap allocations slower than MMAP access
- Need to track memory usage carefully
- Migration overhead during crystallization

### DR-002: 50 Observations Minimum
**Decision**: Require minimum 50 observations before pattern detection

**Rationale**:
- Statistical significance
- Avoid false positives
- Ensure pattern stability
- Balance learning speed vs accuracy

**Trade-offs**:
- Slower initial learning
- May miss rare but valid patterns
- Fixed threshold may not suit all cases

### DR-003: Background Crystallization
**Decision**: Crystallization runs as background async task

**Rationale**:
- Non-blocking for query processing
- Can handle large migrations
- Allows for retry on failure
- Better resource utilization

**Trade-offs**:
- More complex error handling
- Need to track crystallization state
- Potential race conditions

## Success Metrics

- [ ] Pattern detection accuracy >90%
- [ ] Crystallization success rate >95%
- [ ] Memory usage <500MB total
- [ ] No performance degradation during learning
- [ ] Synesthetic associations improve navigation accuracy by 20%

---

*"Learn from every interaction. Crystallize wisdom into permanent memory."*
