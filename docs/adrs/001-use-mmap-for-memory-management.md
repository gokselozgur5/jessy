# ADR-001: Use Memory-Mapped Files for Dimensional Layer Storage

## Status
Accepted

## Context
The consciousness system needs to manage 280MB of dimensional layer data with the following requirements:

- **Performance**: Query processing must complete in <5s, with dimension scanning <100ms
- **Memory efficiency**: Predictable memory footprint, no excessive heap allocation
- **Concurrency**: Support for parallel query processing (100+ concurrent queries)
- **Zero-copy access**: Minimize serialization/deserialization overhead
- **Scalability**: Ability to grow to 500MB+ as new dimensions crystallize

Traditional approaches have limitations:
- **Heap allocation**: Unpredictable memory usage, GC pressure, serialization overhead
- **Database**: Network latency, query overhead, complex deployment
- **In-memory cache**: Memory duplication, cache invalidation complexity

## Decision
We will use memory-mapped files (MMAP) as the primary storage mechanism for dimensional layer data.

**Key principles**:
1. Each dimension gets a dedicated MMAP region
2. Zero-copy access through direct pointer arithmetic
3. OS-managed paging for memory efficiency
4. Custom pool allocator for dynamic growth
5. Thread-safe concurrent read access

**Architecture**:
```
┌─────────────────────────────────────┐
│   MmapManager                       │
│   - Coordinates all regions         │
│   - Manages lifecycle               │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   MmapRegion (per dimension)        │
│   - File descriptor                 │
│   - Memory pointer                  │
│   - Size and offset                 │
└─────────────────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│   PoolAllocator                     │
│   - Dynamic growth                  │
│   - Fragmentation management        │
└─────────────────────────────────────┘
```

## Consequences

### Positive
- **Zero-copy access**: Direct memory access without serialization (target: <1ms)
- **Predictable memory**: Exactly 280MB allocated, OS manages paging
- **High performance**: Memory-speed access, no network/disk latency
- **Concurrent reads**: Multiple threads can read simultaneously without locks
- **OS optimization**: Kernel handles caching, prefetching, and paging
- **Persistence**: Data survives process restarts (if needed)

### Negative
- **Platform complexity**: Different implementations for Linux/macOS/Windows
- **File descriptor limits**: Need to manage FD lifecycle carefully
- **Unsafe code**: Requires careful handling of raw pointers
- **Cold start**: Initial MMAP allocation takes time (~100ms)
- **Memory commitment**: 280MB reserved even if not all used

### Neutral
- **File-backed vs anonymous**: Can choose based on persistence needs
- **Page size alignment**: Must align allocations to 4KB pages
- **Error handling**: Need robust handling of MMAP-specific errors

## Alternatives Considered

### Alternative 1: Heap Allocation with Serialization
**Description**: Store dimensional data in heap-allocated structures, serialize/deserialize on access

**Pros**:
- Simple implementation
- No platform-specific code
- Easy to debug
- Familiar patterns

**Cons**:
- Serialization overhead (10-50ms per access)
- Unpredictable memory usage
- GC pressure in managed languages
- Memory fragmentation over time

**Why not chosen**: Performance requirements cannot be met with serialization overhead. The <100ms dimension scan target would be impossible with 10-50ms per dimension access.

### Alternative 2: Embedded Database (SQLite/RocksDB)
**Description**: Use embedded database for dimensional layer storage

**Pros**:
- ACID guarantees
- Query capabilities
- Mature, battle-tested
- Good tooling

**Cons**:
- Query overhead (5-20ms per access)
- Serialization still required
- Additional dependency
- Overkill for read-heavy workload
- Complex deployment

**Why not chosen**: Query overhead makes it unsuitable for our <100ms scan target. We don't need ACID guarantees or complex queries - just fast read access.

### Alternative 3: Shared Memory (SHM)
**Description**: Use POSIX shared memory for inter-process communication

**Pros**:
- Very fast access
- True zero-copy
- Good for IPC

**Cons**:
- More complex than MMAP
- Requires explicit synchronization
- Limited to single machine
- No persistence

**Why not chosen**: MMAP provides similar performance with simpler API. We don't need IPC capabilities currently, and MMAP gives us persistence option for free.

### Alternative 4: Custom Memory Allocator on Heap
**Description**: Implement custom allocator (arena, pool) on top of heap

**Pros**:
- Predictable allocation patterns
- Reduced fragmentation
- No platform-specific code

**Cons**:
- Still uses heap memory
- No OS-level optimization
- Manual memory management complexity
- No zero-copy benefits

**Why not chosen**: Doesn't solve the fundamental issue of heap overhead. MMAP gives us OS-level optimizations (page caching, prefetching) that we can't replicate with custom allocators.

## Implementation Notes

### Phase 1: Core MMAP Wrapper
```rust
pub struct MmapRegion {
    ptr: *mut u8,
    size: usize,
    fd: RawFd,
}

impl MmapRegion {
    pub fn new(size: usize) -> Result<Self> {
        // Platform-specific MMAP allocation
    }
    
    pub unsafe fn as_slice(&self) -> &[u8] {
        std::slice::from_raw_parts(self.ptr, self.size)
    }
}
```

### Phase 2: Manager Layer
```rust
pub struct MmapManager {
    regions: HashMap<DimensionId, MmapRegion>,
    total_allocated: AtomicUsize,
}

impl MmapManager {
    pub fn load_dimension(&mut self, id: DimensionId) -> Result<()> {
        // Allocate and map region
    }
    
    pub fn access_layer(&self, layer: LayerId) -> Result<&[u8]> {
        // Zero-copy access
    }
}
```

### Phase 3: Pool Allocator
```rust
pub struct PoolAllocator {
    pools: HashMap<usize, Vec<*mut u8>>,
}

impl PoolAllocator {
    pub fn allocate(&mut self, size: usize) -> Result<*mut u8> {
        // Pool-based allocation with growth
    }
}
```

### Migration Strategy
1. Implement MMAP wrapper with comprehensive tests
2. Add manager layer with dimension loading
3. Integrate with existing navigation system
4. Performance validation and tuning
5. Gradual rollout with fallback to heap

### Timeline
- Week 1: Core MMAP implementation
- Week 2: Manager and integration
- Week 3: Pool allocator and optimization
- Week 4: Testing and validation

### Dependencies
- `memmap2` crate for cross-platform MMAP
- `libc` for low-level system calls
- Platform-specific testing on Linux/macOS/Windows

## References
- [Linux mmap(2) man page](https://man7.org/linux/man-pages/man2/mmap.2.html)
- [memmap2 crate documentation](https://docs.rs/memmap2/)
- [Zero-copy techniques in Rust](https://without.boats/blog/zero-copy-1/)
- Related: ADR-002 (Pool Allocator Design)

## Metadata
- **Date**: 2024-10-24
- **Author**: Core Team
- **Reviewers**: Architecture Team
- **Related ADRs**: ADR-002 (Pool Allocator)
- **Tags**: memory, performance, architecture, core

---

*"The right abstraction at the right level. MMAP gives us the performance we need with the simplicity we want."*
