# Memory Manager Integration

## Overview

The Navigation System produces `NavigationResult` structures that specify which dimensional content should be loaded from memory-mapped storage. The Memory Manager consumes these results to load the appropriate MMAP regions.

## Integration Flow

```
Query → NavigationSystem → NavigationResult → MemoryManager → Loaded Content
```

## NavigationResult Structure

```rust
pub struct NavigationResult {
    pub query_analysis: QueryAnalysis,
    pub paths: Vec<NavigationPath>,
    pub dimensions: Vec<DimensionId>,
    pub frequencies: Vec<Frequency>,
    pub total_confidence: f32,
    pub complexity_score: f32,
    pub return_to_source_triggered: bool,
    
    // Duration tracking
    pub query_analysis_duration_ms: u64,
    pub dimension_scan_duration_ms: u64,
    pub path_selection_duration_ms: u64,
    pub depth_navigation_duration_ms: u64,
    pub total_duration_ms: u64,
}

pub struct NavigationPath {
    pub dimension_id: DimensionId,
    pub layer_sequence: Vec<LayerId>,
    pub confidence: f32,
    pub frequency: Frequency,
    pub keywords_matched: Vec<String>,
    pub synesthetic_score: f32,
}
```

## Memory Manager API

### Core Methods

```rust
impl MmapManager {
    /// Create new memory manager with total memory limit
    pub fn new(total_memory_mb: usize) -> Result<Self>;
    
    /// Load a dimension from filesystem into MMAP
    pub fn load_dimension(&self, dimension_id: DimensionId) -> Result<u32>;
    
    /// Load contexts for navigation paths (high-level API)
    pub fn load_contexts(&self, paths: &[NavigationPath]) -> Result<ContextCollection>;
    
    /// Load context for a specific layer
    pub fn load_layer_context(&self, layer_id: LayerId) -> Result<LoadedContext>;
    
    /// Get memory usage statistics
    pub fn get_stats(&self) -> MemoryStats;
    
    /// Initialize all 14 core dimensions
    pub async fn initialize_core_dimensions(&self) -> Result<()>;
}
```

### Data Structures

```rust
/// Context loaded from a layer
pub struct LoadedContext {
    pub layer_id: LayerId,
    pub content: String,
    pub frequency: Frequency,
    pub keywords: Vec<String>,
}

/// Collection of loaded contexts
pub struct ContextCollection {
    pub contexts: Vec<LoadedContext>,
    pub total_size: usize,
}

impl ContextCollection {
    /// Format contexts for LLM with metadata
    pub fn format_with_metadata(&self) -> String;
    
    /// Format contexts for LLM (simple)
    pub fn format_for_llm(&self) -> String;
}
```

## Integration Patterns

### Pattern 1: Simple Integration (Recommended)

Use the high-level `load_contexts()` API:

```rust
// Step 1: Navigate to find relevant dimensions
let nav_result = navigation_system.navigate(query).await?;

// Step 2: Load all contexts in one call
let contexts = memory_manager.load_contexts(&nav_result.paths)?;

// Step 3: Format for LLM
let llm_input = contexts.format_with_metadata();

// Step 4: Send to LLM
let response = llm_client.complete(&llm_input).await?;
```

### Pattern 2: Manual Control

For fine-grained control over loading:

```rust
let nav_result = navigation_system.navigate(query).await?;
let mut contexts = ContextCollection::new();

for path in nav_result.paths {
    // Load dimension if not already loaded
    if !is_dimension_loaded(path.dimension_id) {
        memory_manager.load_dimension(path.dimension_id)?;
    }
    
    // Load each layer in sequence
    for layer_id in path.layer_sequence {
        let context = memory_manager.load_layer_context(layer_id)?;
        contexts.add_context(context);
    }
}
```

### Pattern 3: Selective Loading

Load only high-confidence paths:

```rust
let nav_result = navigation_system.navigate(query).await?;

// Filter to high-confidence paths only
let high_confidence_paths: Vec<_> = nav_result.paths
    .iter()
    .filter(|p| p.confidence >= 0.7)
    .collect();

// Load contexts for high-confidence paths
let contexts = memory_manager.load_contexts(&high_confidence_paths)?;
```

### Pattern 4: Parallel Loading

Load dimensions concurrently for better performance:

```rust
use tokio::task;

let nav_result = navigation_system.navigate(query).await?;

// Spawn concurrent loading tasks
let load_tasks: Vec<_> = nav_result.paths
    .iter()
    .map(|path| {
        let manager = Arc::clone(&memory_manager);
        let path = path.clone();
        task::spawn(async move {
            manager.load_contexts(&[path])
        })
    })
    .collect();

// Wait for all loads to complete
let results = futures::future::join_all(load_tasks).await;

// Combine all contexts
let mut all_contexts = ContextCollection::new();
for result in results {
    if let Ok(Ok(contexts)) = result {
        for context in contexts.contexts {
            all_contexts.add_context(context);
        }
    }
}
```

## Thread Safety

The Memory Manager is designed for concurrent access:

- **Shared Ownership**: Wrap in `Arc<MmapManager>` for multi-threaded use
- **Read-Heavy Optimized**: Uses `RwLock` internally for concurrent reads
- **Lock-Free Tracking**: Uses `AtomicUsize` for memory counters
- **Safe Mutations**: Interior mutability via `Mutex` for allocations

```rust
// Thread-safe usage
let memory_manager = Arc::new(MmapManager::new(280)?);

// Multiple threads can read concurrently
let manager_clone = Arc::clone(&memory_manager);
tokio::spawn(async move {
    let contexts = manager_clone.load_contexts(&paths).await?;
    // Process contexts...
});
```

## Expected Behavior

### Dimension Loading

1. **Lazy Loading**: Dimensions loaded on-demand, not at startup
2. **Caching**: Once loaded, dimensions stay in memory until eviction
3. **Idempotent**: Calling `load_dimension()` multiple times is safe
4. **Atomic**: Loading is atomic - readers see complete or no dimension

### Layer Access

1. **Sequential**: Access layers in the order specified by `layer_sequence`
2. **Zero-Copy**: MMAP provides direct memory access without copying
3. **Concurrent**: Multiple threads can read layers simultaneously
4. **Fast**: Layer access is <1μs (just pointer arithmetic)

### Memory Management

1. **Graduated Warnings**: 75% (warn), 85% (evict), 95% (reject)
2. **Pool-Based**: Multiple size classes reduce fragmentation
3. **Monitoring**: Detailed stats via `get_stats()`
4. **Diagnostics**: Complete state dump via `dump_state()`

### Error Handling

1. **Missing Dimension**: Returns `DimensionNotFound` error
2. **Missing Layer**: Returns `LayerNotFound` error
3. **Memory Pressure**: Returns `LimitExceeded` error
4. **Graceful Degradation**: Partial results on non-critical errors

## Performance Characteristics

### Latency Targets

- **Dimension Load**: < 10ms per dimension (MMAP file mapping)
- **Layer Access**: < 1μs (direct memory access)
- **Context Loading**: < 50ms for typical query (3-5 dimensions)
- **Initialization**: < 100ms for all 14 core dimensions

### Throughput

- **Concurrent Reads**: Unlimited (read-only operations)
- **Concurrent Writes**: Serialized via locks (rare operation)
- **Memory Overhead**: ~5% for metadata and indexes

### Scalability

- **Dimensions**: Tested with 14 core + unlimited proto-dimensions
- **Layers**: Tested with 4 layers per dimension (L0-L3)
- **Memory**: Configurable limit (default 280MB)
- **Threads**: Lock-free reads scale linearly with cores

## Complete Examples

### Example 1: Simple Integration

See `examples/complete_integration.rs` for a full working example:

```rust
use jessy::navigation::{NavigationSystem, DimensionRegistry};
use jessy::memory::MmapManager;
use std::sync::Arc;

async fn process_query(
    query: &str,
    nav_system: &NavigationSystem,
    memory_manager: &Arc<MmapManager>,
) -> Result<String> {
    // Step 1: Navigate to find relevant dimensions
    let nav_result = nav_system.navigate(query).await?;
    
    // Step 2: Load contexts (high-level API)
    let contexts = memory_manager.load_contexts(&nav_result.paths)?;
    
    // Step 3: Format for LLM
    let llm_input = contexts.format_with_metadata();
    
    // Step 4: Send to LLM (your implementation)
    // let response = llm_client.complete(&llm_input).await?;
    
    Ok(llm_input)
}
```

Run with:
```bash
cargo run --example complete_integration
```

### Example 2: Concurrent Processing

See `examples/concurrent_integration.rs` for concurrent query processing:

```rust
use std::sync::Arc;
use tokio::task;

async fn process_queries_concurrent(
    queries: &[&str],
    nav_system: &Arc<NavigationSystem>,
    memory_manager: &Arc<MmapManager>,
) -> Vec<Result<String>> {
    // Spawn concurrent tasks
    let tasks: Vec<_> = queries
        .iter()
        .map(|&query| {
            let nav = Arc::clone(nav_system);
            let mem = Arc::clone(memory_manager);
            task::spawn(async move {
                let nav_result = nav.navigate(query).await?;
                let contexts = mem.load_contexts(&nav_result.paths)?;
                Ok(contexts.format_with_metadata())
            })
        })
        .collect();
    
    // Wait for all tasks
    let results = futures::future::join_all(tasks).await;
    results.into_iter().map(|r| r.unwrap()).collect()
}
```

Run with:
```bash
cargo run --example concurrent_integration
```

### Example 3: Basic Integration

See `examples/navigation_memory_integration.rs` for a step-by-step walkthrough:

```bash
cargo run --example navigation_memory_integration
```

## Data Flow Diagram

```
┌─────────────────┐
│  User Query     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ NavigationSystem│
│  - Analyze      │
│  - Scan         │
│  - Select       │
│  - Navigate     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│NavigationResult │
│  - dimensions   │
│  - paths        │
│  - layer_seq    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  MemoryManager  │
│  - load_dim     │
│  - get_layer    │
│  - read_content │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Loaded Content  │
│  - Layer data   │
│  - Zero-copy    │
└─────────────────┘
```

## Requirements Satisfied

- **Requirement 7.1**: NavigationResult contains all dimension identifiers
- **Requirement 7.2**: NavigationResult contains complete layer sequences
- **Requirement 7.3**: NavigationResult contains confidence scores
- **Requirement 7.5**: NavigationResult contains matched keywords
- **Requirement 7.6**: NavigationResult contains total duration

## Future Enhancements (Phase 2)

- **Predictive Loading**: Pre-load likely dimensions based on query patterns
- **Adaptive Caching**: Adjust cache size based on usage patterns
- **Streaming**: Stream large layer content instead of loading all at once
- **Compression**: Compress infrequently accessed layers
