# Memory Manager Module

Zero-copy memory-mapped file manager for consciousness system dimensional layers.

## Overview

The memory manager provides efficient access to 280MB of dimensional layer data through memory-mapped files (MMAP). It supports:

- **Zero-copy access**: Direct memory access without serialization
- **Thread-safe concurrency**: Multiple readers, exclusive writers (RwLock)
- **Performance optimized**: <1ms layer access, <100ms dimension scan
- **Cross-platform**: Linux, macOS, Windows support via memmap2
- **Comprehensive monitoring**: Detailed statistics and diagnostic snapshots

## Quick Start

### Loading Dimensions

```rust
use jessy::memory::MmapManager;
use jessy::DimensionId;

// Create manager with 280MB limit
let mut manager = MmapManager::new(280)?;

// Load a dimension
let region_id = manager.load_dimension(DimensionId(1))?;

// Initialize all core dimensions
manager.initialize_core_dimensions().await?;
```

### Accessing Layers

```rust
use jessy::LayerId;

// Load a specific layer
let layer_id = LayerId {
    dimension: DimensionId(1),
    layer: 5,
};

let context = manager.load_layer_context(layer_id)?;
println!("Content: {}", context.content);
println!("Frequency: {:.2} Hz", context.frequency.hz());
```

### Proto-Dimension Lifecycle

```rust
// Create proto-dimension in heap
let dimension_id = DimensionId(99);
let content = b"Learning content".to_vec();
let layer_id = manager.create_proto_dimension(dimension_id, content)?;

// Access while in heap
let context = manager.load_layer_context(layer_id)?;

// Crystallize to MMAP
manager.crystallize_proto_dimension(layer_id)?;

// Now in MMAP for permanent storage
```

### Memory Monitoring

```rust
// Get statistics
let stats = manager.get_stats();
println!("Memory usage: {}/{} MB ({:.1}%)",
    stats.current_allocated_mb,
    stats.total_limit_mb,
    stats.utilization_percent
);

// Check thresholds
if stats.is_at_warning_threshold() {
    println!("Warning: Memory usage above 75%");
}

// Dump complete state for diagnostics
let dump = manager.dump_state();
dump.to_json_file(Path::new("memory_state.json"))?;
println!("{}", dump.summary());
```

## Architecture

### Components

- **MmapManager**: Main orchestrator for all memory operations
- **MmapRegion**: Individual memory-mapped dimension files
- **PoolAllocator**: Multi-pool allocator for dynamic content
- **Diagnostics**: Comprehensive state dumping and monitoring

### Memory Layout

```
280MB Total:
- 4KB Pool:   32MB  (metadata, small layers)
- 16KB Pool: 128MB  (typical layer content)
- 64KB Pool:  80MB  (complex hierarchies)
- 256KB Pool: 40MB  (very large dimensions)
```

### Thread Safety

- **RwLock** on regions and layer_index: Multiple concurrent readers
- **Atomic counters**: Lock-free statistics tracking
- **Immutable MMAP regions**: No synchronization needed after load
- **Atomic pointer swap**: Safe crystallization during concurrent access

## Performance Characteristics

- **Layer access**: <1ms (p99)
- **Dimension scan**: <100ms (all layers)
- **Allocation**: <100μs
- **Initialization**: <100ms (pre-allocation)
- **Concurrent readers**: 100+ supported with <10% degradation

## Error Handling

All operations return `Result<T>` with specific error types:

```rust
use jessy::ConsciousnessError;

match manager.load_dimension(dimension_id) {
    Ok(region_id) => println!("Loaded: {}", region_id),
    Err(ConsciousnessError::DimensionNotFound { dimension }) => {
        eprintln!("Dimension {} not found", dimension);
    }
    Err(ConsciousnessError::LimitExceeded { current_mb, limit_mb, .. }) => {
        eprintln!("Memory limit exceeded: {}/{} MB", current_mb, limit_mb);
    }
    Err(e) => eprintln!("Error: {}", e),
}
```

## Platform Support

### Linux
- Uses `mmap()` with `MAP_ANONYMOUS`
- 4KB page size
- Future: `mremap` for efficient pool growth

### macOS
- Uses `mmap()` with `MAP_ANON`
- 4KB pages (Intel), 16KB pages (Apple Silicon)
- Automatic page size handling

### Windows
- Uses `CreateFileMapping` with `INVALID_HANDLE_VALUE`
- 4KB or 64KB pages
- Abstracted by memmap2 crate

## Monitoring & Diagnostics

### Statistics

```rust
let stats = manager.get_stats();

// Basic metrics
stats.total_limit_mb
stats.current_allocated_mb
stats.utilization_percent
stats.regions_loaded
stats.layers_indexed

// Detailed tracking
stats.total_allocations
stats.total_deallocations
stats.allocation_failure_count
stats.peak_concurrent_readers
stats.fragmentation_ratio
```

### State Dumps

```rust
let dump = manager.dump_state();

// JSON export for debugging
dump.to_json_file(Path::new("state.json"))?;

// Quick summary
println!("{}", dump.summary());
// Output:
// Memory State Dump @ 2024-10-25T12:00:00Z
// Regions: 14 | Layers: 280 | Pools: 4
// Memory: 180/280 MB (64.3%)
// Allocations: 1523 | Failures: 0 | Peak Readers: 47
```

## Safety Notes

### Unsafe Code

The memory manager uses `unsafe` in limited, well-documented locations:

1. **Pointer arithmetic** in `PoolAllocator::get_ptr()`
   - Bounds checked before unsafe operations
   - Safety documented in comments

2. **MMAP slice creation** in `MmapRegion::read_content()`
   - Validated against region size
   - Immutable after creation

3. **Memory copy** in `crystallize_proto_dimension()`
   - Source and destination validated
   - No aliasing possible

All unsafe blocks have safety comments explaining invariants.

## Testing

```bash
# Run all tests
cargo test --lib

# Run specific test suites
cargo test --lib memory::perf_tests
cargo test --lib memory::concurrency_tests
cargo test --lib memory::error_tests

# Run benchmarks
cargo bench --bench memory_benchmarks
```

## Future Enhancements

- Dynamic pool growth with `mremap` (Linux)
- Huge pages support (2MB/1GB)
- LRU eviction policy
- Compression for cold layers
- NUMA-aware allocation
- Prometheus metrics export

## License

AGPL-3.0 - See LICENSE file for details

## Authors

Göksel Özgür <gokselozgur5@gmail.com>
