# Learning System Design

## Overview

The Learning System enables the Jessy consciousness system to learn from interactions and crystallize new patterns into permanent dimensional layers. This design emerged from analyzing the need for adaptive, personalized consciousness that evolves with usage.

## Architecture

### High-Level Flow

```
┌─────────────────────────────────────────────────────────────┐
│                    Query Processing                          │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              Observation Recording                           │
│  - Capture query, dimensions, keywords, frequency            │
│  - Store in circular buffer (1000 entries)                   │
│  - <5ms overhead                                             │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              Pattern Detection (Periodic)                    │
│  - Analyze observation buffer                                │
│  - Identify keyword clusters                                 │
│  - Calculate confidence scores                               │
│  - Suggest proto-dimensions                                  │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              Proto-Dimension Creation                        │
│  - Allocate heap memory                                      │
│  - Assign dimension ID (>100)                                │
│  - Store pattern data                                        │
│  - Check memory limits                                       │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│              Crystallization (Background)                    │
│  - Queue for async processing                                │
│  - Allocate MMAP region                                      │
│  - Migrate heap → MMAP atomically                            │
│  - Update dimension registry                                 │
│  - Free heap memory                                          │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. LearningSystem (Coordinator)

**Purpose:** Main interface for learning functionality

**Structure:**
```rust
pub struct LearningSystem {
    pattern_detector: PatternDetector,
    crystallizer: Crystallizer,
    synesthetic_engine: SynestheticLearner,
    proto_dimensions: HashMap<DimensionId, ProtoDimension>,
    observation_buffer: CircularBuffer<Observation>,
    memory_tracker: MemoryTracker,
}
```

**Key Methods:**
- `observe_interaction()` - Record observation from query
- `detect_patterns()` - Analyze observations for patterns
- `create_proto_dimension()` - Create proto-dimension from pattern
- `crystallize()` - Migrate proto-dimension to MMAP
- `get_memory_usage()` - Current memory consumption

### 2. PatternDetector

**Purpose:** Detect recurring patterns from observations

**Algorithm:**
```rust
fn detect_patterns(&mut self) -> Vec<DetectedPattern> {
    // 1. Group observations by keyword similarity
    let clusters = self.cluster_by_keywords();
    
    // 2. For each cluster with ≥50 observations
    for cluster in clusters.iter().filter(|c| c.size() >= 50) {
        // 3. Calculate confidence score
        let confidence = calculate_confidence(cluster);
        
        // 4. If confidence ≥85%, create pattern
        if confidence >= 0.85 {
            patterns.push(DetectedPattern {
                keywords: cluster.common_keywords(),
                frequency_range: cluster.frequency_range(),
                observation_count: cluster.size(),
                confidence,
            });
        }
    }
    
    patterns
}
```

**Confidence Calculation:**
```rust
fn calculate_confidence(cluster: &Cluster) -> f32 {
    let keyword_consistency = cluster.keyword_overlap_ratio();
    let frequency_consistency = cluster.frequency_variance_inverse();
    let temporal_consistency = cluster.temporal_distribution_score();
    
    // Weighted average
    (keyword_consistency * 0.5) + 
    (frequency_consistency * 0.3) + 
    (temporal_consistency * 0.2)
}
```

### 3. ProtoDimension

**Purpose:** Temporary dimension in heap memory

**Structure:**
```rust
pub struct ProtoDimension {
    dimension_id: DimensionId,
    content: Vec<u8>,              // Heap-allocated content
    observations: Vec<Observation>, // Supporting observations
    confidence: f32,                // Current confidence score
    created_at: SystemTime,
    last_accessed: SystemTime,
    size_bytes: usize,
}
```

**Memory Layout:**
```
Heap Memory:
┌─────────────────────────────────────┐
│  ProtoDimension 101 (8MB)           │
├─────────────────────────────────────┤
│  ProtoDimension 102 (12MB)          │
├─────────────────────────────────────┤
│  ProtoDimension 103 (6MB)           │
└─────────────────────────────────────┘
Total: 26MB (max 10 proto-dimensions)
```

### 4. Crystallizer

**Purpose:** Migrate proto-dimensions to MMAP

**Process:**
```rust
async fn crystallize(&mut self, dimension_id: DimensionId) -> Result<()> {
    // 1. Get proto-dimension from heap
    let proto = self.proto_dimensions.get(&dimension_id)?;
    
    // 2. Allocate MMAP region
    let mmap_region = self.memory_manager
        .allocate_dimension(dimension_id, proto.size_bytes)
        .await?;
    
    // 3. Copy content atomically
    unsafe {
        std::ptr::copy_nonoverlapping(
            proto.content.as_ptr(),
            mmap_region.as_mut_ptr(),
            proto.size_bytes,
        );
    }
    
    // 4. Verify integrity
    let checksum_heap = calculate_checksum(&proto.content);
    let checksum_mmap = calculate_checksum(mmap_region.as_slice());
    if checksum_heap != checksum_mmap {
        return Err(CrystallizationError::IntegrityCheckFailed);
    }
    
    // 5. Update registry
    self.dimension_registry.add_dimension(dimension_id, mmap_region)?;
    
    // 6. Free heap memory
    self.proto_dimensions.remove(&dimension_id);
    
    Ok(())
}
```

**Error Recovery:**
```rust
// Retry logic with exponential backoff
let mut retries = 0;
while retries < 3 {
    match self.crystallize_internal(dimension_id).await {
        Ok(()) => return Ok(()),
        Err(e) if e.is_retryable() => {
            retries += 1;
            tokio::time::sleep(Duration::from_secs(2_u64.pow(retries))).await;
        }
        Err(e) => return Err(e),
    }
}
```

### 5. SynestheticLearner

**Purpose:** Learn keyword associations

**Structure:**
```rust
pub struct SynestheticLearner {
    associations: HashMap<String, Vec<Association>>,
    learning_rate: f32,  // 1.1 (10% growth)
    decay_rate: f32,     // 0.95 (5% decay per day)
    last_decay: SystemTime,
}

pub struct Association {
    keyword: String,
    strength: f32,
    activation_count: usize,
    last_activated: SystemTime,
}
```

**Learning Algorithm:**
```rust
fn strengthen_association(&mut self, keyword1: &str, keyword2: &str) {
    let entry = self.associations
        .entry(keyword1.to_string())
        .or_insert_with(Vec::new);
    
    if let Some(assoc) = entry.iter_mut().find(|a| a.keyword == keyword2) {
        // Strengthen existing association
        assoc.strength *= self.learning_rate;  // 10% increase
        assoc.activation_count += 1;
        assoc.last_activated = SystemTime::now();
    } else {
        // Create new association
        entry.push(Association {
            keyword: keyword2.to_string(),
            strength: 1.0,
            activation_count: 1,
            last_activated: SystemTime::now(),
        });
    }
}
```

**Decay Algorithm:**
```rust
fn decay_unused(&mut self) {
    let now = SystemTime::now();
    let days_since_decay = now.duration_since(self.last_decay)
        .unwrap()
        .as_secs() / 86400;
    
    if days_since_decay == 0 {
        return;
    }
    
    for associations in self.associations.values_mut() {
        associations.retain_mut(|assoc| {
            let days_unused = now.duration_since(assoc.last_activated)
                .unwrap()
                .as_secs() / 86400;
            
            // Decay by 5% per day unused
            assoc.strength *= self.decay_rate.powi(days_unused as i32);
            
            // Remove if strength < 0.1
            assoc.strength >= 0.1
        });
    }
    
    self.last_decay = now;
}
```

## Data Structures

### Observation Buffer (Circular)

```rust
struct CircularBuffer<T> {
    buffer: Vec<T>,
    capacity: usize,
    head: usize,
    size: usize,
}

impl<T> CircularBuffer<T> {
    fn push(&mut self, item: T) {
        if self.size < self.capacity {
            self.buffer.push(item);
            self.size += 1;
        } else {
            self.buffer[self.head] = item;
            self.head = (self.head + 1) % self.capacity;
        }
    }
    
    fn iter(&self) -> impl Iterator<Item = &T> {
        // Return items in chronological order
        self.buffer[self.head..].iter()
            .chain(self.buffer[..self.head].iter())
    }
}
```

### Memory Tracker

```rust
struct MemoryTracker {
    core_dimensions: usize,      // 280MB (fixed)
    proto_dimensions: usize,     // Variable
    observation_buffer: usize,   // ~1MB (fixed)
    synesthetic_data: usize,     // Variable
}

impl MemoryTracker {
    fn total_usage(&self) -> usize {
        self.core_dimensions + 
        self.proto_dimensions + 
        self.observation_buffer + 
        self.synesthetic_data
    }
    
    fn can_allocate(&self, size: usize) -> bool {
        self.total_usage() + size <= 500 * 1024 * 1024  // 500MB limit
    }
}
```

## Performance Characteristics

### Time Complexity

| Operation | Complexity | Target |
|-----------|------------|--------|
| Record observation | O(1) | <5ms |
| Pattern detection | O(n) | <100ms |
| Proto-dimension creation | O(1) | <50ms |
| Crystallization | O(m) | Background |
| Synesthetic lookup | O(1) | <1ms |
| Synesthetic strengthen | O(1) | <1ms |

Where:
- n = number of observations (max 1000)
- m = proto-dimension size (max 16MB)

### Space Complexity

| Component | Size | Notes |
|-----------|------|-------|
| Core dimensions | 280MB | Fixed |
| Proto-dimensions | 0-160MB | Max 10 × 16MB |
| Observation buffer | ~1MB | 1000 observations |
| Synesthetic data | ~10MB | Associations |
| **Total** | **≤500MB** | Hard limit |

## Integration Points

### 1. Consciousness Orchestrator

```rust
impl ConsciousnessOrchestrator {
    pub async fn process(&mut self, query: &str) -> Result<Response> {
        // Normal processing
        let nav_result = self.navigation.navigate(query).await?;
        let contexts = self.memory.load_contexts(&nav_result)?;
        let iter_result = self.iteration.process(query, &contexts).await?;
        
        // Learning observation
        self.learning.observe_interaction(
            query,
            &nav_result,
            &iter_result,
        )?;
        
        // Periodic pattern detection (every 100 queries)
        if self.query_count % 100 == 0 {
            let patterns = self.learning.detect_patterns()?;
            for pattern in patterns {
                if pattern.confidence >= 0.85 {
                    self.learning.create_proto_dimension(&pattern)?;
                }
            }
        }
        
        Ok(Response::from(iter_result))
    }
}
```

### 2. Navigation System

```rust
// Synesthetic associations enhance navigation
impl NavigationSystem {
    fn enhance_with_synesthetic(&self, keywords: &[String]) -> Vec<String> {
        let mut enhanced = keywords.to_vec();
        
        for keyword in keywords {
            if let Some(associations) = self.learning.get_associations(keyword) {
                // Add strongly associated keywords
                for assoc in associations.iter().filter(|a| a.strength > 2.0) {
                    enhanced.push(assoc.keyword.clone());
                }
            }
        }
        
        enhanced
    }
}
```

## Design Decisions

### ADR-001: Heap for Proto-Dimensions

**Decision**: Use heap memory for proto-dimensions before crystallization

**Rationale**:
- Flexible during learning phase
- Easy to discard if pattern doesn't crystallize
- No MMAP overhead for temporary data

**Trade-offs**:
- Heap allocations slower than MMAP
- Need careful memory tracking
- Migration overhead during crystallization

### ADR-002: 50 Observations Minimum

**Decision**: Require 50+ observations before proto-dimension creation

**Rationale**:
- Statistical significance
- Avoid false positives
- Ensure pattern stability

**Trade-offs**:
- Slower initial learning
- May miss rare but valid patterns

### ADR-003: Background Crystallization

**Decision**: Crystallization runs as async background task

**Rationale**:
- Non-blocking for query processing
- Can handle large migrations
- Allows retry on failure

**Trade-offs**:
- Complex error handling
- Need state tracking
- Potential race conditions

### ADR-004: Synesthetic Decay

**Decision**: Decay unused associations by 5% per day

**Rationale**:
- Prevent stale associations
- Adapt to changing patterns
- Keep memory usage bounded

**Trade-offs**:
- May lose valid long-term associations
- Requires periodic maintenance

## Future Enhancements (Phase 2)

1. **Adaptive Thresholds**: Learn optimal observation count and confidence thresholds
2. **Hierarchical Patterns**: Detect patterns within patterns
3. **Cross-User Learning**: Learn from aggregate patterns across users
4. **Active Learning**: Suggest queries to fill knowledge gaps
5. **Pattern Merging**: Combine similar proto-dimensions
6. **Incremental Crystallization**: Partial migration for large dimensions

---

*"Learn continuously. Crystallize wisdom. Evolve with every interaction."*
