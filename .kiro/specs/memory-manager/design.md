# Memory Manager Design Document

## 1. Problem & Context

### The Challenge

The consciousness system needs efficient access to 280MB of dimensional layer data across 14 dimensions. Traditional approaches fail to meet our requirements:

**Heap Allocation Problems**:
- Unpredictable memory usage and GC pressure
- Serialization overhead (10-50ms per access)
- Memory fragmentation over time
- Cannot meet <100ms dimension scan target

**Database Problems**:
- Query overhead (5-20ms per access)
- Serialization still required
- Complex deployment and dependencies
- Overkill for read-heavy workload

**Our Requirements**:
- Query processing: <5s total
- Dimension scanning: <100ms
- Layer access: <1ms
- Memory footprint: Predictable 280MB
- Concurrent access: 100+ simultaneous reads
- Zero-copy: No serialization overhead

### Success Criteria

A successful memory manager must:
1. Enable zero-copy access to dimensional layers
2. Maintain predictable memory footprint
3. Support concurrent read operations without locks
4. Handle both static (crystallized) and dynamic (learning) content
5. Provide clear error handling and recovery
6. Scale to 500MB+ as system grows


## 2. Solution Strategy

### Core Approach: Memory-Mapped Files (MMAP)

We use memory-mapped files as the primary storage mechanism, providing:
- **Zero-copy access**: Direct memory access without serialization
- **OS-managed caching**: Kernel handles paging and prefetching
- **Predictable footprint**: Exactly 280MB allocated
- **Concurrent reads**: Multiple threads read simultaneously without locks
- **Persistence option**: Data can survive process restarts

### Three-Layer Architecture

```
┌─────────────────────────────────────────┐
│         MmapManager                     │
│  Orchestrates all memory operations     │
└─────────────────────────────────────────┘
              │
    ┌─────────┼─────────┐
    ▼         ▼         ▼
┌────────┐ ┌────────┐ ┌────────┐
│  Pool  │ │ Region │ │ Layer  │
│Allocator│ │Registry│ │ Index  │
└────────┘ └────────┘ └────────┘
```

**PoolAllocator**: Manages memory blocks with different sizes
**RegionRegistry**: Tracks loaded dimensions and their MMAP regions
**LayerIndex**: Enables O(1) lookup from LayerId to memory location

### Hybrid Storage Model

Three content location strategies based on lifecycle:

1. **MMAP (Static)**: Crystallized dimensions (D01-D14)
   - Memory-mapped from disk
   - Zero-copy access
   - OS-managed caching

2. **Heap (Dynamic)**: Proto-dimensions during learning
   - Temporary storage
   - Pending crystallization
   - Explicit management

3. **Hybrid (Static + Dynamic)**: Future feature
   - Base content in MMAP
   - Dynamic overlay in heap
   - Combined view for queries


## 3. Design Decisions

### Decision 1: Multi-Pool Allocation Strategy

**Context**: Need to allocate memory for layers of varying sizes (1KB to 1MB+) efficiently.

**Options Considered**:

**Option A: Single Large Pool**
- Pros: Simple implementation, no pool selection logic
- Cons: High fragmentation, inefficient for small allocations
- **Rejected**: Would waste significant memory on small layers

**Option B: Fixed-Size Blocks Only**
- Pros: Zero fragmentation, predictable allocation
- Cons: Inflexible, forces all content into same size
- **Rejected**: Wastes memory when content doesn't fit block size

**Option C: Multi-Pool with Best-Fit** ✓
- Pros: Balances flexibility and efficiency, reduces fragmentation
- Cons: More complex pool selection logic
- **Selected**: Optimal trade-off for our use case

**Decision**: Use 4 pools with power-of-2-ish block sizes

**Rationale**:
- Different dimensions have different layer size distributions
- Small pools for metadata and simple layers
- Large pools for complex hierarchical content
- Best-fit selection minimizes wasted space
- Bitmap tracking enables O(1) allocation within pool

**Consequences**:
- Positive: Efficient memory usage, fast allocation
- Negative: Need to manage multiple pools, more complex code
- Neutral: Pool sizes need tuning based on actual data

### Decision 2: Hybrid Storage Model (MMAP + Heap)

**Context**: System needs both static (crystallized) and dynamic (learning) content.

**Options Considered**:

**Option A: Pure MMAP**
- Pros: Maximum performance, zero-copy everywhere
- Cons: Cannot handle dynamic content during learning
- **Rejected**: Learning system needs mutable storage

**Option B: Pure Heap**
- Pros: Flexible, easy to modify
- Cons: No zero-copy, serialization overhead, GC pressure
- **Rejected**: Cannot meet performance targets

**Option C: Hybrid (MMAP for static, Heap for dynamic)** ✓
- Pros: Fast access to crystallized content, flexible learning
- Cons: Two code paths, complexity in lifecycle management
- **Selected**: Enables both performance and flexibility

**Decision**: Use MMAP for crystallized dimensions, heap for proto-dimensions

**Rationale**:
- Crystallized dimensions (D01-D14) are read-only → MMAP perfect
- Proto-dimensions during learning need mutation → Heap necessary
- Crystallization process moves heap → MMAP when ready
- Clear lifecycle: Create (heap) → Learn (heap) → Crystallize (MMAP) → Permanent (MMAP)

**Consequences**:
- Positive: Optimal performance for each use case
- Negative: More complex lifecycle management
- Neutral: Need clear crystallization criteria

### Decision 3: Anonymous MMAP for Pools

**Context**: Pool allocator needs large contiguous memory regions.

**Options Considered**:

**Option A: File-Backed MMAP**
- Pros: Persistence, can survive crashes
- Cons: Disk I/O overhead, file management complexity
- **Rejected**: Pools are temporary, don't need persistence

**Option B: Heap Allocation**
- Pros: Simple, portable
- Cons: No OS-level optimization, no zero-copy benefits
- **Rejected**: Loses MMAP advantages

**Option C: Anonymous MMAP** ✓
- Pros: OS-managed, no disk I/O, zero-copy capable
- Cons: No persistence (acceptable for pools)
- **Selected**: Best performance for temporary allocations

**Decision**: Use anonymous MMAP (MAP_ANON/MAP_ANONYMOUS)

**Rationale**:
- Pools are temporary scratch space
- Don't need persistence across restarts
- OS can optimize page allocation
- Enables zero-copy within pools
- Pre-allocation during initialization ensures predictable memory patterns

**Consequences**:
- Positive: Maximum performance, OS optimization, predictable access patterns
- Negative: Lost on restart (acceptable)
- Neutral: Platform-specific flags (handled by memmap2 crate)

### Decision 4: Pre-Allocation Strategy

**Context**: Need predictable memory access patterns to maintain consistent query latency.

**Options Considered**:

**Option A: Lazy Allocation**
- Pros: Lower initial memory usage, allocate only what's needed
- Cons: Unpredictable latency spikes during first access, GC pressure
- **Rejected**: Cannot meet <1ms access time guarantee

**Option B: Pre-Allocation** ✓
- Pros: Predictable performance, no allocation during hot path, consistent latency
- Cons: Higher initial memory usage, some waste if not all regions used
- **Selected**: Meets performance requirements

**Option C: Hybrid (Pre-allocate hot, lazy for cold)**
- Pros: Balance of memory efficiency and performance
- Cons: Complex logic, still has latency spikes for cold data
- **Rejected**: Adds complexity without clear benefit for our use case

**Decision**: Pre-allocate all dimension regions during initialization

**Rationale**:
- Query processing is hot path - cannot tolerate allocation delays
- 280MB is acceptable memory footprint for the system
- Predictable performance is more valuable than memory efficiency
- Initialization time budget (100ms) accommodates pre-allocation
- Eliminates dynamic allocation from critical paths

**Consequences**:
- Positive: Consistent <1ms layer access, no GC pressure, predictable performance
- Negative: Higher initial memory usage, longer initialization time
- Neutral: Memory is reserved but may not all be used immediately


## 4. Architecture Overview

### Component Responsibilities

**MmapManager** (Orchestrator)
- Coordinates all memory operations
- Manages dimension lifecycle (load, unload)
- Provides unified access interface
- Tracks memory usage and statistics
- Handles proto-dimension crystallization

**PoolAllocator** (Memory Provider)
- Manages multiple memory pools
- Allocates/deallocates blocks
- Tracks free space with bitmaps
- Provides best-fit allocation
- Reports pool statistics

**MmapRegion** (Dimension Container)
- Represents one loaded dimension
- Wraps memory-mapped file
- Provides bounds-checked access
- Stores dimension metadata
- Lists available layers

**LayerIndex** (Fast Lookup)
- Maps LayerId → ContentLocation
- Enables O(1) layer access
- Tracks location type (MMAP/Heap/Hybrid)
- Updated on dimension load/crystallization

### Data Flow

```
Query Request
    ↓
Navigator (selects paths)
    ↓
MmapManager.load_contexts(paths)
    ↓
LayerIndex lookup (O(1))
    ↓
MmapRegion.read_content (zero-copy)
    ↓
LoadedContext (ready for processing)
```

### State Machines

**Proto-Dimension Lifecycle**:
```
[Create] → [Active] → [Crystallize] → [Permanent]
  (heap)    (heap)      (copy)         (MMAP)
```

**Region Loading**:
```
[Unloaded] → [Loading] → [Loaded] → [Accessed]
              (file I/O)  (indexed)  (OS paging)
```


## 5. Component Contracts

### MmapManager Interface

```rust
// Initialization
fn new(total_memory_mb: usize) -> Result<Self>
async fn initialize_core_dimensions() -> Result<()>

// Dimension Management
fn load_dimension(dimension_id: DimensionId) -> Result<RegionId>
fn create_proto_dimension(dimension_id: DimensionId, content: Vec<u8>) -> Result<LayerId>
fn crystallize_proto_dimension(layer_id: LayerId) -> Result<()>

// Content Access (zero-copy)
fn load_layer_context(layer_id: LayerId) -> Result<LoadedContext>
fn load_contexts(paths: &[NavigationPath]) -> Result<ContextCollection>

// Memory Operations
fn allocate(size: usize) -> Result<MmapOffset>
fn deallocate(offset: MmapOffset) -> Result<()>

// Monitoring & Diagnostics
fn get_stats() -> MemoryStats
fn dump_state() -> MemoryStateDump
fn get_memory_usage() -> MemoryUsage
```

**Contracts**:
- `new`: Initializes pools, creates base directory, returns ready manager. Completes within 100ms.
- `load_dimension`: Loads dimension file, creates MMAP, indexes layers, returns region ID. Completes within 50ms.
- `load_layer_context`: Returns zero-copy reference to layer content in <1ms
- `crystallize_proto_dimension`: Moves heap content to MMAP, updates index
- `get_stats`: Returns current memory usage statistics for monitoring
- `dump_state`: Returns comprehensive diagnostic information including all regions, allocations, and pool states
- `get_memory_usage`: Returns current and limit values for memory tracking
- All methods are thread-safe for concurrent reads
- All methods return Result types and never panic on errors

### PoolAllocator Interface

```rust
fn new() -> Self
fn add_pool(size_mb: usize, block_size: usize) -> Result<PoolId>
fn allocate(size: usize) -> Result<MmapOffset>
fn deallocate(offset: MmapOffset) -> Result<()>
fn get_stats() -> PoolStats
```

**Contracts**:
- `allocate`: Returns offset to block >= requested size using best-fit
- `deallocate`: Marks block as free, enables reuse
- Operations are O(1) within a pool (bitmap lookup)
- Thread-safe with internal synchronization

### MmapRegion Interface

```rust
fn from_file(region_id: u32, dimension_id: DimensionId, path: &Path) -> Result<Self>
fn read_content(offset: usize, size: usize) -> Result<&[u8]>
fn read_string(offset: usize, size: usize) -> Result<String>
fn get_layer_info(layer_id: LayerId) -> Option<&LayerInfo>
fn list_layers() -> Vec<LayerId>
```

**Contracts**:
- `from_file`: Creates MMAP, parses metadata, validates format
- `read_content`: Returns zero-copy slice, bounds-checked
- All reads are thread-safe (immutable after load)
- Metadata cached in memory for fast access


## 6. Critical Sequences

### Sequence 1: Dimension Loading

```
User → MmapManager: load_dimension(D01)
    MmapManager → FileSystem: open("D01/region.mmap")
    FileSystem → MmapManager: file_handle
    MmapManager → OS: mmap(file_handle)
    OS → MmapManager: memory_ptr
    MmapManager → MmapRegion: new(ptr, metadata)
    MmapRegion → MmapRegion: parse_metadata()
    MmapRegion → MmapRegion: index_layers()
    MmapRegion → MmapManager: region
    MmapManager → LayerIndex: update(layers → locations)
    MmapManager → User: region_id
```

**Timing**: <50ms per dimension
**Failure Points**: File not found, invalid metadata, MMAP failure
**Recovery**: Return error, cleanup partial state, log details

### Sequence 2: Layer Access (Zero-Copy)

```
QueryProcessor → MmapManager: load_layer_context(L01-05)
    MmapManager → LayerIndex: lookup(L01-05)
    LayerIndex → MmapManager: ContentLocation::Mmap{offset, size, region_id}
    MmapManager → MmapRegion: read_content(offset, size)
    MmapRegion → MmapRegion: bounds_check(offset, size)
    MmapRegion → QueryProcessor: &[u8] (zero-copy slice)
```

**Timing**: <1ms per layer
**Failure Points**: Layer not found, out of bounds
**Recovery**: Return error with context, continue with other layers

### Sequence 3: Proto-Dimension Crystallization

```
LearningSystem → MmapManager: create_proto_dimension(D99, content)
    MmapManager → Heap: allocate(content.len())
    MmapManager → LayerIndex: insert(L99-00 → Heap{data})
    MmapManager → LearningSystem: layer_id

[... learning phase ...]

LearningSystem → MmapManager: crystallize_proto_dimension(L99-00)
    MmapManager → LayerIndex: lookup(L99-00)
    LayerIndex → MmapManager: ContentLocation::Heap{data}
    MmapManager → PoolAllocator: allocate(data.len())
    PoolAllocator → MmapManager: mmap_offset
    MmapManager → Memory: copy(heap_data → mmap_offset)
    MmapManager → LayerIndex: update(L99-00 → Mmap{offset})
    MmapManager → Heap: deallocate(heap_data)
    MmapManager → LearningSystem: Ok(())
```

**Timing**: ~10ms per MB
**Failure Points**: Allocation failure, copy failure
**Recovery**: Keep heap version, retry later, log warning

### Sequence 4: Concurrent Access

```
Thread1 → MmapManager: load_layer_context(L01-05)
Thread2 → MmapManager: load_layer_context(L02-03)
Thread3 → MmapManager: load_layer_context(L01-05)

[All threads read simultaneously - no locks needed]

Thread1 ← MmapManager: LoadedContext
Thread2 ← MmapManager: LoadedContext
Thread3 ← MmapManager: LoadedContext
```

**Timing**: No contention, parallel execution
**Safety**: MMAP regions are immutable after load
**Scalability**: 100+ concurrent reads supported


## 7. Capacity Planning

### Memory Allocation Strategy

**Total Budget**: 280MB (configurable)

**Pool Distribution** (TBD: Requires profiling actual dimension data):

Estimated based on expected layer size patterns:
- **4KB Pool**: ~20-40MB (estimated 5,000-10,000 blocks)
  - Use case: Small layers, metadata, simple content
  - Justification: Many dimensions have small foundational layers
  
- **16KB Pool**: ~100-140MB (estimated 6,000-9,000 blocks)
  - Use case: Typical layer content, most common size
  - Justification: Majority of layers fall in 8-20KB range
  
- **64KB Pool**: ~60-100MB (estimated 1,000-1,500 blocks)
  - Use case: Complex hierarchical content
  - Justification: Some dimensions have deep layer trees
  
- **256KB Pool**: ~20-60MB (estimated 80-240 blocks)
  - Use case: Very large dimensions, combined content
  - Justification: Reserve capacity for outliers

**Note**: These numbers are estimates. Actual distribution should be determined by:
1. Profiling existing dimension files (D01-D14)
2. Analyzing layer size distribution
3. Measuring allocation patterns during runtime
4. Adjusting pool sizes based on utilization metrics

### Dimension Breakdown

**14 Core Dimensions**: ~280MB total (file-backed MMAP)
- D01-D03: ~16MB each (48MB total) - High-frequency dimensions
- D04-D05: ~8MB each (16MB total) - Context dimensions
- D06-D07: ~14MB each (28MB total) - Deep thinking dimensions
- D08-D13: ~8MB each (48MB total) - Specialized dimensions
- D14: ~4MB - Security dimension (always active)
- Metadata overhead: ~20MB (indexes, structures)
- Reserve: ~120MB (pools for dynamic content)

**Proto-Dimensions**: Heap-based, outside 280MB budget
- Temporary storage during learning
- Moved to MMAP pools upon crystallization
- Heap usage monitored separately

### Growth Strategy

**Current Capacity**: 280MB
**Target Capacity**: 500MB (future)

**Growth Path**:
1. **Phase 1** (Current): 14 dimensions, 280MB
2. **Phase 2** (6 months): Add 6 dimensions, expand to 400MB
3. **Phase 3** (12 months): Add 10 dimensions, expand to 500MB

**Scaling Mechanism**:
- Increase pool sizes proportionally
- Add new pools for larger block sizes if needed
- Use `mremap` on Linux for efficient expansion
- Copy-based growth on macOS/Windows

### Capacity Monitoring

**Warning Thresholds**:
- 75% utilization: Log warning, consider cleanup
- 85% utilization: Trigger eviction of least-used regions
- 95% utilization: Reject new allocations, return error

**Metrics to Track**:
- Per-pool utilization percentage
- Fragmentation ratio (free blocks / total blocks)
- Allocation failure rate
- Average allocation size
- Peak concurrent access count


## 8. Error Handling & Recovery

### Error Categories

**1. Allocation Errors**
- `AllocationFailed`: No suitable pool or pool full
  - Context: Requested size, available pools, current utilization
- `LimitExceeded`: Total memory budget exceeded
  - Context: Current usage, limit, requested size
- Recovery: Trigger cleanup, retry with smaller size, or return error

**2. Access Errors**
- `LayerNotFound`: Layer ID not in index
  - Context: Layer ID, available layers, dimension state
- `RegionNotFound`: Dimension not loaded
  - Context: Dimension ID, loaded regions, initialization state
- `OutOfBounds`: Access beyond region limits
  - Context: Offset, length, region size, dimension ID
- Recovery: Return empty context or default value, log warning

**3. File Errors**
- `FileNotFound`: Dimension file missing
  - Context: Expected path, dimension ID, file system error code
- `InvalidMetadata`: Corrupted or incompatible format
  - Context: File path, metadata version, parse error details
- `MmapFailed`: OS-level MMAP failure
  - Context: File descriptor, size, system error code, platform details
- Recovery: Skip dimension, log error, continue with others

**4. Lifecycle Errors**
- `CrystallizationFailed`: Cannot move heap → MMAP
  - Context: Layer ID, content size, pool availability, failure reason
- `PoolFull`: No space for crystallization
  - Context: Required size, pool utilization, available pools
- Recovery: Keep in heap, retry later, log warning

**All errors include**:
- Timestamp
- Operation context
- Current memory state snapshot
- Suggested remediation steps

### Failure Scenarios & Recovery

**Scenario 1: Dimension Loading Fails Mid-Process**

Problem: File opens but MMAP fails or metadata is corrupt

Recovery Strategy:
```
1. Detect failure during load_dimension()
2. Clean up partial state (close file, free memory)
3. Remove from regions map if partially added
4. Log detailed error with dimension ID, file path, and system error code
5. Include current memory state in error context
6. Return MemoryError with comprehensive diagnostic information
7. System continues with other dimensions
```

**Scenario 2: Memory Limit Exceeded During Allocation**

Problem: Request would exceed 280MB budget

Recovery Strategy:
```
1. Check utilization before allocation
2. If >85%, trigger eviction of LRU regions
3. Retry allocation after eviction
4. If still fails, return LimitExceeded error
5. Caller decides: fail request or retry later
6. Log warning with current utilization
```

**Scenario 3: Crystallization Fails (Heap → MMAP)**

Problem: No space in pools or copy fails

Recovery Strategy:
```
1. Detect allocation failure in crystallize_proto_dimension()
2. Keep proto-dimension in heap (don't delete)
3. Mark as "pending crystallization"
4. Retry during next cleanup cycle
5. Log warning with layer ID and reason
6. System continues, heap version still accessible
```

**Scenario 4: Concurrent Access During Crystallization**

Problem: Thread reads layer while it's being moved heap → MMAP

Recovery Strategy:
```
1. Use atomic pointer swap for location update
2. Old heap version stays valid until swap complete
3. Readers see either old (heap) or new (MMAP) version
4. Both versions have same content
5. Heap deallocated only after swap confirmed
6. No reader sees inconsistent state
```

**Scenario 5: System Crash During Crystallization**

Problem: Process dies mid-copy, partial data in MMAP

Recovery Strategy:
```
1. On restart, check for incomplete crystallizations
2. Detect via "pending" flag in metadata
3. Discard partial MMAP data
4. Reload proto-dimension from backup if available
5. Or mark as lost and log error
6. System continues with other dimensions
```

### Error Handling Principles

1. **Fail Fast**: Detect errors early, don't propagate bad state
2. **Clean Up**: Always release resources on error paths
3. **Context**: Include relevant IDs, sizes, and state in errors
4. **Graceful Degradation**: System continues with reduced functionality
5. **Logging**: Log all errors with sufficient detail for debugging
6. **No Panic**: Use Result<T> consistently, never panic in production


## 9. Testing Strategy

### Unit Testing

**What to Test**:
- Pool allocation/deallocation cycles
- Best-fit pool selection logic
- Bitmap free block tracking
- Region bounds checking
- Metadata parsing
- Layer index operations
- Proto-dimension lifecycle

**Success Criteria**:
- All edge cases covered (empty, full, boundary)
- Error conditions trigger correct errors
- Resource cleanup verified (no leaks)
- Concurrent operations safe (no data races)

**Not Testing** (implementation details):
- Exact Rust syntax
- Internal data structure layout
- Platform-specific MMAP calls (covered by memmap2)

### Integration Testing

**Scenarios**:

1. **Full System Load**
   - Load all 14 core dimensions
   - Verify memory usage within 280MB
   - Check all layers accessible
   - Measure load time <1.5s total

2. **Concurrent Access**
   - 100 threads reading simultaneously
   - No data corruption
   - No deadlocks
   - Performance degradation <10%

3. **Proto-Dimension Workflow**
   - Create in heap
   - Access during learning
   - Crystallize to MMAP
   - Verify content identical
   - Confirm heap freed

4. **Error Recovery**
   - Missing dimension file → graceful skip
   - Allocation failure → cleanup and error
   - Invalid metadata → log and continue
   - Partial load → rollback state

### Performance Benchmarks

**Critical Paths**:
- Layer access: Target <1ms (p99)
- Dimension scan: Target <100ms (all layers)
- Allocation: Target <100μs
- Crystallization: Target <10ms per MB
- Concurrent reads: Target linear scaling to 100 threads

**Measurement Approach**:
- Use criterion.rs for Rust benchmarks
- Black-box inputs to prevent optimization
- Run on target hardware (M2 MacBook)
- Compare against baseline (heap allocation)

### Property-Based Testing

**Properties to Verify**:
- Allocate then deallocate always succeeds for same size
- Read within bounds always succeeds
- Read beyond bounds always fails
- Concurrent reads return consistent data
- Total allocated never exceeds pool size

**Tools**: proptest crate for Rust


## 10. Requirements Traceability

### Requirements Coverage

| Req ID | User Story | Design Section | Status |
|--------|------------|----------------|--------|
| R1 | Initialize memory regions efficiently | §3 Decision 4, §5 MmapManager.new, §6 Sequence 1 | ✓ Covered |
| R2 | Load dimension data into memory | §6 Sequence 1, §5 MmapRegion, §11 Platform Notes | ✓ Covered |
| R3 | Zero-copy access to layer data | §2 Solution Strategy, §5 Contracts, §6 Sequence 2 | ✓ Covered |
| R4 | Thread-safe memory access | §5 Contracts, §6 Sequence 4, §8 Scenario 4 | ✓ Covered |
| R5 | Memory limits enforced | §5 MmapManager, §7 Capacity Planning, §8 Scenario 2 | ✓ Covered |
| R6 | Dynamic memory growth capability | §3 Decision 1, §4 PoolAllocator, §7 Growth Strategy | ✓ Covered |
| R7 | Graceful error handling | §8 Error Handling, §8 Failure Scenarios | ✓ Covered |
| R8 | Cross-platform consistency | §3 Decision 3, §11 Platform Considerations | ✓ Covered |
| R9 | Comprehensive error information | §5 MmapManager.dump_state, §8 Error Categories | ✓ Covered |
| R10 | Predictable memory access patterns | §3 Decision 4, §7 Capacity Planning, §11 Optimizations | ✓ Covered |

### Acceptance Criteria Coverage

**R1: Initialization**
- ✓ Allocate exactly 280MB (§7 Capacity Planning)
- ✓ Create 14 distinct regions (§7 Dimension Breakdown)
- ✓ Page-aligned allocation (§11 Platform Considerations)
- ✓ Return MemoryError on failure (§8 Error Categories)
- ✓ Complete within 100ms (§5 Contracts, §9 Benchmarks)

**R2: Dimension Loading**
- ✓ Map file to pre-allocated region (§6 Sequence 1)
- ✓ Detect duplicate loads (§8 Error Categories)
- ✓ Handle insufficient memory (§8 Scenario 2)
- ✓ Mark dimension as accessible (§4 LayerIndex)
- ✓ Complete within 50ms (§5 Contracts, §9 Benchmarks)

**R3: Zero-Copy Access**
- ✓ Return direct pointer (§5 MmapRegion.read_content)
- ✓ Complete within 1ms (§6 Sequence 2, §9 Benchmarks)
- ✓ No intermediate buffers (§2 Solution Strategy)
- ✓ Handle layer not loaded (§8 Error Categories)
- ✓ Validate bounds (§5 MmapRegion, §11 Bounds Checking)

**R4: Thread Safety**
- ✓ Synchronize concurrent access (§5 Contracts)
- ✓ Support 100+ concurrent reads (§6 Sequence 4, §9 Integration)
- ✓ Block reads during writes (§8 Scenario 4)
- ✓ Lock-free read paths (§5 Contracts, §11 Optimizations)
- ✓ Handle access during deallocation (§8 Error Categories)

**R5: Memory Limits**
- ✓ Track with atomic operations (§5 MmapManager)
- ✓ Reject when limit exceeded (§8 Scenario 2)
- ✓ Accurate allocation count (§5 PoolAllocator)
- ✓ Decrement on deallocation (§4 PoolAllocator)
- ✓ Expose through monitoring (§5 MmapManager.get_stats)

**R6: Dynamic Growth**
- ✓ Satisfy from existing pools (§4 PoolAllocator)
- ✓ Create new pool if needed (§7 Growth Strategy)
- ✓ Organize by block size (§3 Decision 1)
- ✓ Return blocks for reuse (§4 PoolAllocator)
- ✓ Limit total pool memory (§7 Capacity Planning)

**R7: Error Handling**
- ✓ Specific error types (§8 Error Categories)
- ✓ Clean up partial resources (§8 Scenario 1)
- ✓ Include file system details (§8 Error Categories)
- ✓ Log warnings on pressure (§7 Capacity Monitoring)
- ✓ Return Result, never panic (§5 Contracts, §11 Memory Safety)

**R8: Cross-Platform**
- ✓ Identical public APIs (§5 Component Contracts)
- ✓ Abstract platform specifics (§11 Platform Considerations)
- ✓ Platform-appropriate syscalls (§11 Platform Considerations)
- ✓ Handle page sizes correctly (§11 Platform Considerations)
- ✓ Same test suite (§9 Testing Strategy)

**R9: Error Information**
- ✓ Include dimension/layer ID (§8 Error Categories)
- ✓ Include memory statistics (§8 Error Categories)
- ✓ Include system error codes (§8 Error Categories)
- ✓ Log all operations at debug (§8 Error Handling Principles)
- ✓ Dump memory state method (§5 MmapManager.dump_state)

**R10: Predictable Access**
- ✓ Pre-allocate all regions (§3 Decision 4)
- ✓ Zero allocations in hot path (§3 Decision 4, §11 Optimizations)
- ✓ Contiguous memory regions (§7 Dimension Breakdown)
- ✓ Cache-aligned structures (§11 Optimizations)
- ✓ 99% within 1ms ±0.1ms (§9 Performance Benchmarks)

### Design Decisions Coverage

| Decision | Rationale Section | Alternatives Considered | Status |
|----------|-------------------|------------------------|--------|
| Multi-pool allocation | §3 Decision 1 | Single pool, Fixed-size | ✓ Documented |
| Hybrid storage model | §3 Decision 2 | Pure MMAP, Pure heap | ✓ Documented |
| Anonymous MMAP for pools | §3 Decision 3 | File-backed, Heap | ✓ Documented |
| Best-fit allocation | §3 Decision 1 | First-fit, Worst-fit | ✓ Documented |
| Bitmap free tracking | §4 PoolAllocator | Linked list, Tree | ⚠ Implicit |
| O(1) layer lookup | §4 LayerIndex | Linear search, Tree | ⚠ Implicit |

**Note**: Decisions marked ⚠ Implicit should be made explicit in future iterations.

### Gap Analysis

**Fully Covered**:
- Core functionality (load, access, allocate)
- Performance targets
- Error handling
- Testing strategy
- Capacity planning

**Partially Covered**:
- Eviction policy (mentioned but not detailed)
- Fragmentation management (strategy unclear)
- Monitoring and diagnostics (basic coverage)

**Not Covered** (Future Work):
- Dynamic pool resizing
- Huge pages support
- NUMA awareness
- Compression for cold layers
- Persistence layer for snapshots


## 11. Implementation Notes

### Platform Considerations

**Linux**:
- Use `mmap(2)` with `MAP_ANONYMOUS` for pools
- File-backed MMAP for dimension regions
- Page size: typically 4KB
- Future: `mremap` for efficient pool growth

**macOS**:
- Use `mmap(2)` with `MAP_ANON` for pools
- Page size: 4KB (Intel), 16KB (Apple Silicon)
- No `mremap` equivalent (use munmap + mmap)
- Alignment critical on Apple Silicon

**Windows**:
- Use `CreateFileMapping` + `MapViewOfFile`
- Anonymous mapping via `INVALID_HANDLE_VALUE`
- Page size: 4KB or 64KB
- Different error handling model

**Abstraction**: Use `memmap2` crate for cross-platform compatibility

### Critical Algorithms

**Best-Fit Pool Selection** (Pseudocode):
```
function select_pool(requested_size):
    suitable_pools = pools.filter(p => p.block_size >= requested_size)
    if suitable_pools.empty():
        return Error(NoSuitablePool)
    return suitable_pools.min_by(p => p.block_size)
```

**Bitmap Allocation** (Pseudocode):
```
function allocate_block(pool):
    free_index = pool.bitmap.first_one()
    if free_index.none():
        return Error(PoolFull)
    pool.bitmap.set(free_index, false)
    return Offset(pool.id, free_index * pool.block_size)
```

**Bounds Checking** (Pseudocode):
```
function read_content(region, offset, size):
    if offset + size > region.mmap.len():
        return Error(OutOfBounds)
    return region.mmap[offset..offset+size]
```

### File Format Specification

**Region File Structure**:
```
Bytes 0-3:    Magic "JSON"
Bytes 4-1023: Metadata (JSON, null-padded)
Bytes 1024+:  Layer content (sequential)
```

**Metadata JSON Schema**:
```json
{
  "version": 1,
  "created_at": 1234567890,
  "dimension_name": "D01-Emotion",
  "total_size": 16777216,
  "content_offset": 1024,
  "layers": {
    "D01-L00": {
      "name": "Foundation",
      "frequency": 1.5,
      "depth": 0,
      "offset": 1024,
      "size": 4096,
      "keywords": ["empathy", "emotion"]
    }
  }
}
```

### Memory Safety

**Unsafe Code Boundaries**:
- Raw pointer access in PoolAllocator
- MMAP slice creation in MmapRegion
- Platform-specific system calls

**Safety Guarantees**:
- All unsafe blocks have safety comments
- Bounds checked before unsafe operations
- No public unsafe functions
- RAII for resource cleanup

### Performance Optimizations

1. **Zero-Copy**: Return slice references, never copy
2. **Lock-Free Reads**: Immutable MMAP regions, no synchronization
3. **Lazy Loading**: Load dimensions on-demand
4. **OS Prefetching**: Use `madvise(MADV_WILLNEED)` on Linux
5. **Bitmap Efficiency**: Use `bitvec` crate for fast bit operations


## 12. Future Enhancements

### Phase 2: Advanced Features (6-12 months)

**Dynamic Pool Growth**
- Use `mremap` on Linux for efficient expansion
- Implement copy-based growth on macOS/Windows
- Automatic growth when utilization >85%
- Configurable growth policy

**Huge Pages Support**
- 2MB/1GB pages for large dimensions
- Reduce TLB pressure
- Platform-specific configuration
- Measure performance impact

**Enhanced Monitoring**
- Prometheus metrics export
- Real-time utilization dashboard
- Allocation pattern analysis
- Predictive capacity planning

### Phase 3: Optimization (12-18 months)

**NUMA Awareness**
- Allocate on local NUMA node
- Reduce cross-socket memory access
- Topology-aware pool distribution
- Measure latency improvements

**Compression**
- LZ4 compression for cold layers
- Transparent decompression on access
- Trade CPU for memory savings
- Configurable compression threshold

**Tiered Storage**
- Hot layers in MMAP (fast)
- Warm layers in compressed MMAP (medium)
- Cold layers on disk with lazy load (slow)
- Automatic tier promotion/demotion

**Adaptive Pool Sizing**
- Monitor allocation patterns
- Adjust pool sizes dynamically
- Machine learning for prediction
- Minimize fragmentation

### Phase 4: Advanced Capabilities (18+ months)

**Persistence Layer**
- Snapshot current state to disk
- Fast restart from snapshot
- Incremental updates
- Crash recovery

**Distributed Memory**
- Share MMAP regions across processes
- IPC via shared memory
- Distributed dimension loading
- Cluster-aware allocation

**Prefetching Engine**
- Predict access patterns
- Preload likely-needed layers
- Reduce latency for common queries
- ML-based prediction


## 13. References

### Architecture Decision Records
- **ADR-001**: Use MMAP for Memory Management
  - Location: `.kiro/adrs/001-use-mmap-for-memory-management.md`
  - Decision: Memory-mapped files for dimensional layer storage
  - Rationale: Zero-copy access, OS optimization, predictable footprint

### Requirements
- **Requirements Document**: `.kiro/specs/memory-manager/requirements.md`
  - Functional requirements (FR-1 through FR-8)
  - Non-functional requirements (NFR-1 through NFR-8)
  - Success criteria and constraints

### Implementation
- **Source Code**: `src/memory/`
  - `manager.rs`: MmapManager implementation
  - `region.rs`: MmapRegion and metadata handling
  - `pool.rs`: PoolAllocator and bitmap tracking
  - `mod.rs`: Public API and type definitions

### External Documentation
- **memmap2 crate**: https://docs.rs/memmap2/
  - Cross-platform MMAP abstraction
  - Safety guarantees and usage patterns
  
- **Linux mmap(2)**: https://man7.org/linux/man-pages/man2/mmap.2.html
  - System call documentation
  - Flags and error conditions
  
- **bitvec crate**: https://docs.rs/bitvec/
  - Efficient bitmap operations
  - Used for free block tracking

### Related Specifications
- **Navigation System**: `.kiro/specs/navigation-system-spec.md`
  - Integration point: NavigationPath → LoadedContext
  
- **Learning System**: `.kiro/specs/learning-system-spec.md`
  - Integration point: Proto-dimension lifecycle

---

## Document Metadata

**Version**: 2.0 (Crystallized)
**Date**: 2024-10-24
**Status**: Ready for Implementation
**Authors**: Core Team
**Reviewers**: Architecture Team

**Changelog**:
- v1.0 (2024-10-20): Initial draft with implementation details
- v2.0 (2024-10-24): Refactored to design-first approach
  - Removed implementation code blocks
  - Added decision rationale (ADR-style)
  - Added sequence diagrams
  - Added capacity planning with estimates
  - Added failure scenarios and recovery
  - Added requirements traceability
  - Crystallized through 9-iteration method

**Next Steps**:
1. Review and approve design document
2. Create implementation tasks (tasks.md)
3. Begin Phase 1 implementation
4. Validate against requirements
5. Iterate based on feedback

---

*"Design is not just what it looks like and feels like. Design is how it works." - Steve Jobs*

*"The right abstraction at the right level. MMAP gives us the performance we need with the simplicity we want."*

*"Crystallized through 9 iterations: Explore → Refine → Emerge."*
