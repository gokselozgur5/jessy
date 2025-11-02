# Memory Manager Design

## Overview

The Memory Manager provides zero-copy access to dimensional layer data through memory-mapped files (MMAP). It manages 280MB of memory across 14 dimensions, ensuring fast (<1ms) access while maintaining thread safety and predictable resource usage.

## Architecture

### Component Diagram

```
┌──────────────────────────────────────────────────────────┐
│                      MmapManager                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Region Registry (D01-D14)                         │  │
│  │  ┌──────┐ ┌──────┐ ┌──────┐       ┌──────┐       │  │
│  │  │ D01  │ │ D02  │ │ D03  │  ...  │ D14  │       │  │
│  │  │ 16MB │ │ 16MB │ │ 16MB │       │ 4MB  │       │  │
│  │  └──────┘ └──────┘ └──────┘       └──────┘       │  │
│  └────────────────────────────────────────────────────┘  │
│                                                           │
│  ┌────────────────────────────────────────────────────┐  │
│  │  PoolAllocator (Learning System Growth)            │  │
│  │  - 32MB  of 4KB blocks  (small layers)            │  │
│  │  - 128MB of 16KB blocks (medium layers)           │  │
│  │  - 80MB  of 64KB blocks (large layers)            │  │
│  │  - 40MB  of 256KB blocks (very large dimensions)  │  │
│  └────────────────────────────────────────────────────┘  │
│                                                           │
│  ┌────────────────────────────────────────────────────┐  │
│  │  Memory Statistics & Monitoring                     │  │
│  │  - Usage tracking                                   │  │
│  │  - Performance metrics                              │  │
│  │  - Threshold warnings                               │  │
│  └────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────┘
```

### Memory Layout (280MB Total)

```
Address Range           Size    Purpose
──────────────────────────────────────────────────────────
0x0000_0000-0x0100_0000  16MB   D01: Emotion
0x0100_0000-0x0200_0000  16MB   D02: Cognition
0x0200_0000-0x0300_0000  16MB   D03: Intention
0x0300_0000-0x0380_0000   8MB   D04: Social
0x0380_0000-0x0400_0000   8MB   D05: Temporal
0x0400_0000-0x0500_0000  16MB   D06: Philosophy
0x0500_0000-0x0580_0000  12MB   D07: Technical
0x0580_0000-0x0600_0000   8MB   D08: Creative
0x0600_0000-0x0680_0000  12MB   D09: Ethical (IMMUTABLE)
0x0680_0000-0x0700_0000   8MB   D10: Meta
0x0700_0000-0x0780_0000   8MB   D11: Ecological
0x0780_0000-0x0800_0000   8MB   D12: Positivity
0x0800_0000-0x0880_0000   8MB   D13: Balance
0x0880_0000-0x08C0_0000   4MB   D14: Security (IMMUTABLE)
──────────────────────────────────────────────────────────
Total Core Dimensions:  168MB

0x08C0_0000-0x1000_0000 112MB   Reserve Pool (D15+ crystallization)
0x1000_0000-0x1200_0000  32MB   User-Specific Dimensions
──────────────────────────────────────────────────────────
Total System:           280MB
```

## Core Data Structures

### MmapManager

**Location:** `src/memory/manager.rs`

**Responsibilities:**
- Initialize and manage memory-mapped regions
- Track loaded dimensions
- Provide zero-copy access to layer data
- Enforce memory limits
- Generate statistics

**Key Methods:**
```rust
pub struct MmapManager {
    config: MemoryConfig,
    regions: HashMap<DimensionId, MmapRegion>,
    pool: PoolAllocator,
    stats: RwLock<MemoryStats>,
}

impl MmapManager {
    pub fn new(total_memory_mb: usize) -> Result<Self>;
    pub fn load_dimension(&mut self, dim_id: DimensionId, path: &Path) -> Result<()>;
    pub fn access_layer(&self, layer_id: LayerId) -> Result<&[u8]>;
    pub fn get_stats(&self) -> MemoryStats;
    pub fn close(&mut self) -> Result<()>;
}
```

### MmapRegion

**Location:** `src/memory/region.rs`

**Responsibilities:**
- Map a single dimension file into memory
- Manage file descriptors
- Provide pointer access
- Handle cleanup

**Key Methods:**
```rust
pub struct MmapRegion {
    dimension_id: DimensionId,
    file_path: PathBuf,
    mmap: memmap2::Mmap,
    size_bytes: usize,
}

impl MmapRegion {
    pub fn new(dim_id: DimensionId, path: &Path) -> Result<Self>;
    pub fn access(&self, offset: usize, size: usize) -> Result<&[u8]>;
    pub fn size(&self) -> usize;
}
```

### PoolAllocator

**Location:** `src/memory/pool.rs`

**Responsibilities:**
- Manage dynamic memory pools
- Allocate blocks for learning system
- Track free/used blocks
- Enforce memory limits

**Key Methods:**
```rust
pub struct PoolAllocator {
    pools: Vec<Pool>,
    total_limit_bytes: usize,
    total_allocated_bytes: AtomicUsize,
}

impl PoolAllocator {
    pub fn new(pools: Vec<(usize, usize)>, limit_mb: usize) -> Self;
    pub fn allocate(&mut self, size: usize) -> Result<*mut u8>;
    pub fn deallocate(&mut self, ptr: *mut u8, size: usize);
    pub fn usage(&self) -> (usize, usize); // (used, limit)
}
```

### MemoryConfig

**Location:** `src/lib.rs` (core type)

**Purpose:** Configuration for memory management behavior

**Fields:**
- `total_memory_mb`: Total memory limit (default: 280MB)
- `base_path`: Directory for MMAP files (default: data/consciousness)
- `pool_configs`: Pool sizes and block sizes
- `warning_threshold`: Warning trigger percentage (default: 75%)
- `eviction_threshold`: Eviction trigger percentage (default: 85%)
- `critical_threshold`: Critical warning percentage (default: 95%)

### MemoryStats

**Location:** `src/lib.rs` (core type)

**Purpose:** Track memory usage and performance metrics

**Key Fields:**
- `total_allocated_bytes`: Total memory allocated
- `total_used_bytes`: Memory currently in use
- `dimensions_loaded`: Number of loaded dimensions
- `layers_loaded`: Number of accessible layers
- `active_regions`: Number of MMAP regions
- `peak_usage_bytes`: Peak memory usage
- `avg_access_latency_us`: Average access time
- `simulation_mode`: Whether using simulated contexts

## Thread Safety

### Concurrency Model

**Read-Heavy Workload:**
- Use `RwLock` for stats (many readers, few writers)
- MMAP regions are read-only after initialization
- Multiple queries can access same dimension simultaneously

**Synchronization Points:**
- Dimension loading: Exclusive write lock
- Stats updates: Write lock on stats
- Layer access: No lock (read-only memory)

### Lock-Free Operations

**Zero-lock access paths:**
- Direct memory access via MMAP (after initialization)
- Atomic counters for allocation tracking
- Read-only region pointers

## Performance Characteristics

### Target Metrics

| Operation              | Target Latency | Actual (Measured) |
|------------------------|----------------|-------------------|
| Initialization         | <100ms         | TBD               |
| Dimension Loading      | <50ms          | TBD               |
| Layer Access           | <1ms           | TBD               |
| Stats Retrieval        | <1ms           | TBD               |
| Pool Allocation        | <10μs          | TBD               |

### Memory Overhead

- Core dimensions: 168MB (fixed)
- Reserve pool: 112MB (dynamic)
- Metadata: ~2MB (region tracking, stats)
- **Total: 280MB**

## Error Handling

### Error Types

```rust
pub enum ConsciousnessError {
    MemoryError(String),              // General memory errors
    AllocationFailed(String),         // Allocation failures
    LimitExceeded { ... },            // Memory limit violations
    LayerNotFound { ... },            // Missing layer access
    RegionNotFound { ... },           // Missing region
    OutOfBounds { ... },              // Invalid memory access
    DimensionNotFound { ... },        // Unknown dimension
}
```

### Error Recovery

**Non-Fatal Errors:**
- Missing dimension → Skip dimension, continue processing
- Out of memory → Reject allocation, return error

**Fatal Errors:**
- MMAP initialization failure → System cannot start
- File descriptor exhaustion → Cleanup and restart required

## Simulation Mode (Development)

### Fallback Behavior

**When binary MMAP files unavailable:**
1. MmapManager detects missing files
2. Falls back to `create_simulated_contexts()`
3. Loads keywords from `data/dimensions.json`
4. Generates realistic context strings
5. Sets `simulation_mode = true` in stats

**Transition Strategy:**
- Simulation mode used during development
- Binary MMAP generation (Phase 9) creates real files
- No code changes needed - automatic detection

## File Format (Binary MMAP)

### Region File Structure

```
[Header: 256 bytes]
  - dimension_id: u32
  - dimension_name_len: u32
  - dimension_name: [u8; 64] (UTF-8, padded)
  - total_size: u64
  - layer_count: u32
  - magic: [u8; 16] = "JESSY_MMAP_V1\0\0\0"
  - reserved: [u8; 152]

[Layer Metadata Array]
  For each layer (0-3):
    - layer_num: u16
    - depth: u8
    - frequency: f32
    - keyword_count: u32
    - keywords: Vec<String> (length-prefixed)
    - content_offset: u64
    - content_size: u32

[Layer Content Blocks]
  - Layer 0 content (variable size)
  - Layer 1 content (variable size)
  - Layer 2 content (variable size)
  - Layer 3 content (variable size)
```

## Integration Points

### Navigation System

**Flow:** Navigator → MmapManager
- Navigator selects dimensions and layers
- Returns `NavigationPath` with layer sequences
- MmapManager loads contexts for paths

### Interference Engine

**Flow:** MmapManager → InterferenceEngine
- Loaded contexts include frequency information
- InterferenceEngine calculates dominant frequency
- No direct dependency on MmapManager

### Learning System

**Flow:** LearningSystem → PoolAllocator
- New proto-dimensions allocated from pool
- Crystallization migrates heap → MMAP reserve pool
- MmapManager tracks user-specific dimensions

## Testing Strategy

### Unit Tests

**MmapRegion:**
- File mapping success/failure
- Bounds checking
- Cleanup and resource release

**PoolAllocator:**
- Allocation and deallocation
- Limit enforcement
- Fragmentation handling

**MmapManager:**
- Initialization
- Dimension loading
- Access methods
- Error conditions

### Integration Tests

**End-to-End:**
- Full initialization flow
- Concurrent access patterns
- Memory limit validation
- Cleanup verification

### Performance Tests

**Benchmarks:**
- Initialization time
- Access latency
- Concurrent throughput
- Memory footprint

## Open Questions

1. **Binary format optimization:** Should we use compression for text content?
2. **Eviction strategy:** LRU vs LFU for dimension eviction under memory pressure?
3. **Checksum validation:** Add checksums to binary files for integrity checks?

## Implementation Status

- ✅ Core types defined (DimensionId, LayerId, MemoryConfig, MemoryStats)
- ✅ Simulated context fallback working
- ⏳ MmapRegion partially implemented
- ⏳ PoolAllocator not yet implemented
- ⏳ Binary MMAP generation deferred (Phase 9)
