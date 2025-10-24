# Spec: Memory Manager Implementation

## Status
- **Phase**: Implementation
- **Started**: 2024-10-24
- **Owner**: Core Team
- **Priority**: P0 (Critical Path)

## Problem Statement

The consciousness system requires efficient, zero-copy access to 280MB of dimensional layer data. Traditional heap allocation and serialization would introduce unacceptable latency (>100ms) and memory overhead.

### Why This Matters
- Query processing must complete in <5s total
- Dimension scanning target: <100ms
- Memory footprint must remain predictable
- Concurrent access required for parallel queries

### Constraints
- Must work on Linux, macOS, Windows
- No external dependencies beyond libc
- Must be thread-safe
- Must handle graceful degradation on memory pressure

### Success Criteria
- [ ] MMAP regions allocate successfully
- [ ] Zero-copy access verified
- [ ] Thread-safe concurrent access
- [ ] Performance: <1ms access time
- [ ] Memory: Exactly 280MB allocated
- [ ] Tests: >90% coverage

## Domain Model

### Entities

```rust
/// Memory-mapped region for a dimension
struct MmapRegion {
    id: DimensionId,
    size: usize,
    ptr: *mut u8,
    file_descriptor: RawFd,
}

/// Pool allocator for dynamic growth
struct PoolAllocator {
    pools: HashMap<usize, Vec<*mut u8>>,
    total_allocated: AtomicUsize,
}

/// Main manager coordinating regions
struct MmapManager {
    regions: HashMap<DimensionId, MmapRegion>,
    allocator: PoolAllocator,
    config: MemoryConfig,
}
```

### State Machine

```
[Uninitialized] 
    ↓ initialize()
[Allocated] 
    ↓ load_dimension()
[Loaded]
    ↓ access_layer()
[Active]
    ↓ close()
[Closed]
```

### Invariants
1. Total allocated memory never exceeds configured limit
2. All regions are page-aligned
3. No region overlaps with another
4. File descriptors are valid while region is active
5. Concurrent access is synchronized

## Architecture Design

### Component Boundaries

```
┌─────────────────────────────────────┐
│   MmapManager (Public API)          │
├─────────────────────────────────────┤
│   - initialize()                    │
│   - load_dimension()                │
│   - access_layer()                  │
│   - close()                         │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   MmapRegion (Internal)             │
├─────────────────────────────────────┤
│   - allocate()                      │
│   - map_file()                      │
│   - unmap()                         │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   PoolAllocator (Internal)          │
├─────────────────────────────────────┤
│   - allocate_block()                │
│   - free_block()                    │
│   - grow_pool()                     │
└─────────────────────────────────────┘
```

### Interface Contracts

```rust
impl MmapManager {
    /// Initialize manager with configuration
    /// 
    /// # Errors
    /// - MemoryError if allocation fails
    /// - MemoryError if file creation fails
    pub fn new(config: MemoryConfig) -> Result<Self>;
    
    /// Load dimension data into memory
    /// 
    /// # Errors
    /// - MemoryError if dimension already loaded
    /// - MemoryError if insufficient memory
    pub fn load_dimension(&mut self, id: DimensionId) -> Result<()>;
    
    /// Access layer data with zero-copy
    /// 
    /// # Safety
    /// Caller must ensure no concurrent writes
    pub unsafe fn access_layer(&self, layer: LayerId) -> Result<&[u8]>;
}
```

### Data Structures

```rust
// Memory layout: 280MB total
// D01-D14: 280MB (dimensions)
// Reserve: 112MB (future growth)
// User: 32MB (per-user data)

const DIMENSION_SIZES: [(DimensionId, usize); 14] = [
    (D01, 16 * MB),  // Emotion
    (D02, 16 * MB),  // Cognition
    (D03, 16 * MB),  // Intention
    (D04, 8 * MB),   // Social
    (D05, 8 * MB),   // Temporal
    (D06, 16 * MB),  // Philosophy
    (D07, 12 * MB),  // Technical
    (D08, 8 * MB),   // Creative
    (D09, 12 * MB),  // Ethical
    (D10, 8 * MB),   // Meta
    (D11, 8 * MB),   // Ecological
    (D12, 8 * MB),   // Positivity
    (D13, 8 * MB),   // Balance
    (D14, 4 * MB),   // Security
];
```

### Performance Characteristics

- **Allocation**: O(1) - pre-allocated regions
- **Access**: O(1) - direct pointer arithmetic
- **Memory**: O(n) - linear with dimension count
- **Concurrency**: Lock-free reads, synchronized writes

## Test Specification

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    #[test]
    fn test_manager_initialization() {
        // Given: Default configuration
        let config = MemoryConfig::default();
        
        // When: Initialize manager
        let manager = MmapManager::new(config);
        
        // Then: Success with correct allocation
        assert!(manager.is_ok());
        assert_eq!(manager.unwrap().total_allocated(), 280 * MB);
    }
    
    #[test]
    fn test_dimension_loading() {
        // Given: Initialized manager
        let mut manager = MmapManager::new(MemoryConfig::default()).unwrap();
        
        // When: Load dimension
        let result = manager.load_dimension(DimensionId(1));
        
        // Then: Success and accessible
        assert!(result.is_ok());
        assert!(manager.is_loaded(DimensionId(1)));
    }
    
    #[test]
    fn test_zero_copy_access() {
        // Given: Loaded dimension
        let mut manager = setup_manager_with_dimension();
        
        // When: Access layer
        let data = unsafe { manager.access_layer(LayerId::new(1, 0)) };
        
        // Then: Direct pointer, no copy
        assert!(data.is_ok());
        assert_eq!(data.unwrap().as_ptr(), expected_ptr);
    }
    
    #[test]
    fn test_concurrent_access() {
        // Given: Loaded dimension
        let manager = Arc::new(setup_manager_with_dimension());
        
        // When: Multiple threads access
        let handles: Vec<_> = (0..10)
            .map(|_| {
                let m = manager.clone();
                thread::spawn(move || {
                    unsafe { m.access_layer(LayerId::new(1, 0)) }
                })
            })
            .collect();
        
        // Then: All succeed without data races
        for handle in handles {
            assert!(handle.join().unwrap().is_ok());
        }
    }
    
    #[test]
    fn test_memory_limit_enforcement() {
        // Given: Manager at capacity
        let mut manager = setup_manager_at_capacity();
        
        // When: Attempt to exceed limit
        let result = manager.allocate_beyond_limit();
        
        // Then: Error returned
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), MemoryError::LimitExceeded);
    }
}
```

### Integration Tests

```rust
#[test]
fn test_full_query_flow() {
    // Given: System with loaded dimensions
    let mut system = ConsciousnessSystem::new().await.unwrap();
    
    // When: Process query requiring multiple dimensions
    let response = system.process_query("complex query").await;
    
    // Then: Success with correct dimensions accessed
    assert!(response.is_ok());
    assert!(response.unwrap().dimensions_activated.len() > 1);
}

#[test]
fn test_memory_pressure_handling() {
    // Given: System under memory pressure
    let mut system = setup_system_with_limited_memory();
    
    // When: Process queries
    let results: Vec<_> = (0..100)
        .map(|_| system.process_query("test"))
        .collect();
    
    // Then: Graceful degradation, no crashes
    assert!(results.iter().all(|r| r.is_ok() || r.is_err()));
}
```

### BDD Scenarios

```gherkin
Feature: Memory Manager
  As a consciousness system
  I want efficient memory management
  So that queries process quickly

  Scenario: Initialize memory regions
    Given a default configuration
    When I initialize the memory manager
    Then 280MB should be allocated
    And all 14 dimensions should have regions
    And regions should be page-aligned

  Scenario: Zero-copy layer access
    Given a loaded dimension
    When I access a layer
    Then data should be returned immediately
    And no memory copy should occur
    And access time should be <1ms

  Scenario: Concurrent query processing
    Given multiple active queries
    When they access the same dimension
    Then all should succeed
    And no data corruption should occur
    And performance should remain consistent
```

## Implementation Plan

### Phase 1: Core Structure (Day 1)
- [ ] Define types and traits
- [ ] Implement MmapRegion
- [ ] Add basic allocation
- [ ] Write unit tests

### Phase 2: Pool Allocator (Day 2)
- [ ] Implement PoolAllocator
- [ ] Add growth strategy
- [ ] Handle fragmentation
- [ ] Write unit tests

### Phase 3: Manager Integration (Day 3)
- [ ] Implement MmapManager
- [ ] Add dimension loading
- [ ] Implement access methods
- [ ] Write integration tests

### Phase 4: Concurrency (Day 4)
- [ ] Add thread safety
- [ ] Implement synchronization
- [ ] Test concurrent access
- [ ] Performance benchmarks

### Phase 5: Validation (Day 5)
- [ ] Full test suite
- [ ] Performance validation
- [ ] Documentation
- [ ] Code review

## Decision Records

### ADR-001: Use MMAP over Heap Allocation
**Status**: Accepted

**Context**: Need efficient access to large dimensional data.

**Decision**: Use memory-mapped files instead of heap allocation.

**Rationale**:
- Zero-copy access
- OS-managed paging
- Predictable memory usage
- Better cache locality

**Consequences**:
- Platform-specific code required
- File descriptor management needed
- More complex error handling

### ADR-002: Pool Allocator for Dynamic Growth
**Status**: Accepted

**Context**: Need dynamic allocation for learning system.

**Decision**: Implement custom pool allocator.

**Rationale**:
- Reduces fragmentation
- Predictable performance
- Better control over growth
- Aligns with MMAP regions

**Consequences**:
- Additional complexity
- Need to tune pool sizes
- Memory overhead for metadata

## Risks and Mitigations

### Risk: Platform Differences
**Mitigation**: Abstract platform-specific code, test on all targets

### Risk: Memory Leaks
**Mitigation**: RAII patterns, comprehensive testing, valgrind

### Risk: Performance Degradation
**Mitigation**: Continuous benchmarking, profiling, optimization

### Risk: Concurrent Access Bugs
**Mitigation**: Thread sanitizer, stress testing, formal verification

## Success Metrics

- [ ] All tests pass (>90% coverage)
- [ ] Performance: <1ms access time
- [ ] Memory: Exactly 280MB allocated
- [ ] Concurrency: 100+ concurrent queries
- [ ] Documentation: Complete API docs
- [ ] Review: Approved by 2+ engineers

---

*"Memory is the foundation. Build it solid, build it fast, build it right."*
