# Technical Standards & Best Practices

## Code Organization

### Module Structure
**Proverb**: "A place for everything, everything in its place."

```
src/
├── lib.rs              # Public API, core types
├── module_name/
│   ├── mod.rs          # Module public interface
│   ├── types.rs        # Domain types
│   ├── logic.rs        # Business logic
│   ├── tests.rs        # Unit tests
│   └── benchmarks.rs   # Performance tests
```

**Rules**:
- One concept per module
- Clear public/private boundaries
- Tests alongside implementation
- Benchmarks for critical paths

### File Naming
- `snake_case` for files
- Match module names
- Descriptive, not abbreviated
- Consistent across project

### Import Organization
```rust
// Standard library
use std::collections::HashMap;
use std::sync::Arc;

// External crates
use tokio::sync::RwLock;
use serde::{Deserialize, Serialize};

// Internal modules
use crate::memory::MmapManager;
use crate::types::DimensionId;
```

## Rust Standards

### Type Design
**Proverb**: "Make illegal states unrepresentable."

```rust
// ✅ Good: Type system prevents invalid states
enum ConnectionState {
    Disconnected,
    Connecting { attempt: u32 },
    Connected { session_id: Uuid },
    Failed { reason: String },
}

// ❌ Bad: Invalid states possible
struct Connection {
    connected: bool,
    session_id: Option<Uuid>,  // Can be Some when disconnected
    attempt: u32,              // Meaningless when connected
}
```

### Error Handling
**Proverb**: "Errors should never pass silently."

```rust
// ✅ Good: Specific error types
#[derive(Error, Debug)]
pub enum MemoryError {
    #[error("Allocation failed: {0}")]
    AllocationFailed(String),
    
    #[error("Region {0:?} not found")]
    RegionNotFound(DimensionId),
    
    #[error("Memory limit exceeded: {current} > {limit}")]
    LimitExceeded { current: usize, limit: usize },
}

// ❌ Bad: Generic errors
fn allocate() -> Result<(), String> {
    Err("something went wrong".to_string())
}
```

### Ownership Patterns
```rust
// Prefer borrowing
fn process_data(data: &[u8]) -> Result<()> { }

// Take ownership when needed
fn consume_data(data: Vec<u8>) -> Result<()> { }

// Return owned data
fn create_data() -> Vec<u8> { }

// Use Arc for shared ownership
fn share_data(data: Arc<Data>) -> Result<()> { }
```

### Async Patterns
```rust
// ✅ Good: Clear async boundaries
pub async fn process_query(&mut self, query: &str) -> Result<Response> {
    let data = self.load_data().await?;
    let result = self.compute(data)?;  // Sync computation
    self.save_result(result).await?;
    Ok(Response::new())
}

// ❌ Bad: Unnecessary async
pub async fn add(a: i32, b: i32) -> i32 {
    a + b  // No async work!
}
```

### Trait Design
```rust
// ✅ Good: Focused, composable traits
pub trait Allocator {
    fn allocate(&mut self, size: usize) -> Result<*mut u8>;
    fn deallocate(&mut self, ptr: *mut u8);
}

pub trait Resizable: Allocator {
    fn resize(&mut self, new_size: usize) -> Result<()>;
}

// ❌ Bad: God trait
pub trait Everything {
    fn allocate(&mut self, size: usize) -> Result<*mut u8>;
    fn process_query(&self, q: &str) -> Result<String>;
    fn save_to_disk(&self) -> Result<()>;
    // ... 20 more methods
}
```

## Go Standards

### Package Structure
```
api/
├── main.go              # Entry point
├── server.go            # HTTP server
├── handlers.go          # Request handlers
├── middleware.go        # Middleware
├── types.go             # Domain types
└── consciousness.go     # Business logic
```

### Error Handling
```go
// ✅ Good: Wrap errors with context
func LoadDimension(id DimensionID) error {
    data, err := readFile(id)
    if err != nil {
        return fmt.Errorf("loading dimension %d: %w", id, err)
    }
    return nil
}

// ❌ Bad: Swallow errors
func LoadDimension(id DimensionID) error {
    data, _ := readFile(id)  // Error ignored!
    return nil
}
```

### Concurrency Patterns
```go
// ✅ Good: Use channels for communication
func ProcessQueries(queries <-chan Query) <-chan Result {
    results := make(chan Result)
    go func() {
        defer close(results)
        for query := range queries {
            results <- process(query)
        }
    }()
    return results
}

// Use sync primitives for state
type SafeCounter struct {
    mu    sync.Mutex
    count int
}

func (c *SafeCounter) Increment() {
    c.mu.Lock()
    defer c.mu.Unlock()
    c.count++
}
```

## Testing Standards

### Unit Test Structure
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_descriptive_name() {
        // Given: Setup test state
        let manager = MmapManager::new(config);
        
        // When: Perform action
        let result = manager.allocate(1024);
        
        // Then: Assert expectations
        assert!(result.is_ok());
        assert_eq!(result.unwrap().size(), 1024);
    }
}
```

### Test Naming
- `test_<what>_<condition>_<expected>`
- Examples:
  - `test_allocate_valid_size_succeeds`
  - `test_allocate_zero_size_fails`
  - `test_concurrent_access_no_data_race`

### Test Coverage
- **Unit tests**: Pure logic, edge cases
- **Integration tests**: Component interactions
- **BDD tests**: User-facing behaviors
- **Benchmarks**: Performance critical paths

### Property-Based Testing
```rust
use proptest::prelude::*;

proptest! {
    #[test]
    fn test_allocate_any_valid_size(size in 1usize..1024*1024) {
        let mut manager = MmapManager::new(config);
        let result = manager.allocate(size);
        prop_assert!(result.is_ok());
    }
}
```

## Documentation Standards

### Module Documentation
```rust
//! Memory management subsystem.
//!
//! This module provides zero-copy access to dimensional layer data
//! through memory-mapped files. It includes:
//!
//! - [`MmapManager`]: Main interface for memory operations
//! - [`MmapRegion`]: Individual memory-mapped regions
//! - [`PoolAllocator`]: Dynamic memory allocation
//!
//! # Examples
//!
//! ```
//! use jessy::memory::MmapManager;
//!
//! let manager = MmapManager::new(config)?;
//! manager.load_dimension(DimensionId(1))?;
//! ```
```

### Function Documentation
```rust
/// Allocates a memory-mapped region for the specified dimension.
///
/// # Arguments
///
/// * `id` - The dimension identifier
///
/// # Returns
///
/// Returns `Ok(())` on success, or `MemoryError` if:
/// - The dimension is already loaded
/// - Insufficient memory is available
/// - File creation fails
///
/// # Examples
///
/// ```
/// let mut manager = MmapManager::new(config)?;
/// manager.load_dimension(DimensionId(1))?;
/// ```
///
/// # Safety
///
/// This function is safe to call concurrently from multiple threads.
pub fn load_dimension(&mut self, id: DimensionId) -> Result<()> {
    // Implementation
}
```

### Type Documentation
```rust
/// Represents a memory-mapped region for a dimension.
///
/// Each region corresponds to one dimensional layer and provides
/// zero-copy access to its data through memory mapping.
///
/// # Thread Safety
///
/// `MmapRegion` is `Send` but not `Sync`. Use `Arc<RwLock<MmapRegion>>`
/// for shared access across threads.
pub struct MmapRegion {
    // Fields
}
```

## Performance Standards

### Benchmarking
```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn benchmark_allocation(c: &mut Criterion) {
    let mut manager = setup_manager();
    
    c.bench_function("allocate_1kb", |b| {
        b.iter(|| {
            manager.allocate(black_box(1024))
        });
    });
}

criterion_group!(benches, benchmark_allocation);
criterion_main!(benches);
```

### Performance Targets
- **Memory allocation**: <1ms
- **Dimension scan**: <100ms
- **Query processing**: <5s
- **API response**: <100ms (p95)
- **Memory footprint**: <500MB

### Optimization Guidelines
1. **Measure first**: Profile before optimizing
2. **Optimize hot paths**: Focus on critical sections
3. **Avoid premature optimization**: Clarity first
4. **Use appropriate data structures**: HashMap vs Vec vs BTreeMap
5. **Minimize allocations**: Reuse buffers, use stack when possible
6. **Leverage zero-copy**: MMAP, slices, references

## Security Standards

### Input Validation
```rust
// ✅ Good: Validate all inputs
pub fn process_query(query: &str) -> Result<Response> {
    if query.is_empty() {
        return Err(ValidationError::EmptyQuery);
    }
    if query.len() > MAX_QUERY_LENGTH {
        return Err(ValidationError::QueryTooLong);
    }
    // Process validated input
}
```

### Memory Safety
```rust
// ✅ Good: Safe abstractions over unsafe code
pub struct MmapRegion {
    ptr: *mut u8,
    size: usize,
}

impl MmapRegion {
    /// # Safety
    /// Caller must ensure ptr is valid and size is correct
    unsafe fn new(ptr: *mut u8, size: usize) -> Self {
        Self { ptr, size }
    }
    
    // Safe public interface
    pub fn read(&self, offset: usize, len: usize) -> Result<&[u8]> {
        if offset + len > self.size {
            return Err(MemoryError::OutOfBounds);
        }
        unsafe {
            Ok(std::slice::from_raw_parts(
                self.ptr.add(offset),
                len
            ))
        }
    }
}
```

### Secrets Management
- Never commit secrets to git
- Use environment variables
- Rotate credentials regularly
- Use secret management services

## Code Review Standards

### What to Look For
1. **Correctness**: Does it work as intended?
2. **Clarity**: Is it easy to understand?
3. **Completeness**: Are edge cases handled?
4. **Consistency**: Does it match project style?
5. **Coverage**: Are tests sufficient?

### Review Checklist
- [ ] Code follows style guidelines
- [ ] Tests are comprehensive
- [ ] Documentation is complete
- [ ] No security vulnerabilities
- [ ] Performance is acceptable
- [ ] Error handling is robust
- [ ] Types are well-designed
- [ ] No code smells

### Giving Feedback
- Be specific and actionable
- Explain the "why"
- Suggest alternatives
- Praise good work
- Focus on code, not person

## Continuous Improvement

### Metrics to Track
- Test coverage
- Build time
- Test execution time
- Code complexity
- Documentation coverage
- Bug rate
- Performance benchmarks

### Regular Reviews
- Weekly: Code quality metrics
- Monthly: Architecture alignment
- Quarterly: Technical debt assessment
- Annually: Technology stack evaluation

---

*"Standards enable excellence. Consistency enables collaboration. Quality enables trust."*
