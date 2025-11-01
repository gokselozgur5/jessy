# Running Performance Benchmarks

## Quick Start

```bash
# Run all navigation benchmarks
cargo bench --bench navigation_benchmarks

# Or with Docker
docker-compose run --rm unit-tests cargo bench --bench navigation_benchmarks
```

## Benchmark Groups

### 1. Registry Lookup
Tests dimension and layer lookup performance.

```bash
cargo bench --bench navigation_benchmarks -- registry_lookup
```

**Expected Results:**
- Dimension lookup: <1μs
- Layer lookup: <1μs
- Child layers: <1μs

### 2. Query Analysis
Tests query parsing and classification.

```bash
cargo bench --bench navigation_benchmarks -- query_analysis
```

**Expected Results:**
- Short queries: <2ms
- Medium queries: <3ms
- Long queries: <5ms

### 3. Full Navigation
Tests end-to-end navigation performance.

```bash
cargo bench --bench navigation_benchmarks -- full_navigation
```

**Expected Results:**
- Emotional queries: <100ms
- Technical queries: <100ms
- Philosophical queries: <100ms
- All queries p95: <150ms

### 4. Concurrent Navigation
Tests scalability with multiple concurrent queries.

```bash
cargo bench --bench navigation_benchmarks -- concurrent_navigation
```

**Expected Results:**
- Linear scaling up to CPU core count
- No lock contention
- Minimal overhead per additional query

### 5. Query Types
Compares performance across different query types.

```bash
cargo bench --bench navigation_benchmarks -- query_types
```

**Expected Results:**
- Similar performance across all types
- Emotional queries may be slightly faster (fewer keywords)
- Technical queries may be slightly slower (more keywords)

### 6. Memory Integration
Tests navigation + memory loading overhead.

```bash
cargo bench --bench navigation_benchmarks -- memory_integration
```

**Expected Results:**
- Navigation only: <100ms
- Navigation + loading: <150ms
- Overhead: <50ms

## Viewing Results

### Terminal Output
Criterion prints results directly to terminal with:
- Mean time
- Standard deviation
- Confidence intervals
- Comparison with previous runs

### HTML Reports
```bash
# Generate HTML reports
cargo bench --bench navigation_benchmarks

# Open in browser
open target/criterion/report/index.html
```

HTML reports include:
- Interactive charts
- Statistical analysis
- Historical comparisons
- Detailed breakdowns

## Interpreting Results

### Time Units
- **ns** (nanoseconds): 10^-9 seconds
- **μs** (microseconds): 10^-6 seconds
- **ms** (milliseconds): 10^-3 seconds

### Statistical Measures
- **Mean**: Average time across all samples
- **Median**: Middle value (less affected by outliers)
- **Std Dev**: Variation in measurements
- **p95**: 95th percentile (worst case for 95% of requests)

### Performance Indicators
- **Green**: Meeting or exceeding targets
- **Yellow**: Close to targets (within 10%)
- **Red**: Missing targets (need optimization)

## Baseline Comparison

### Save Baseline
```bash
# Save current performance as baseline
cargo bench --bench navigation_benchmarks -- --save-baseline main
```

### Compare Against Baseline
```bash
# Compare current performance with baseline
cargo bench --bench navigation_benchmarks -- --baseline main
```

This shows:
- Performance improvements (faster)
- Performance regressions (slower)
- Statistical significance

## Continuous Integration

### CI Pipeline
Benchmarks should run:
1. **Before Release**: Validate performance targets
2. **After Changes**: Detect regressions
3. **Weekly**: Track trends over time

### Regression Detection
```bash
# Fail if performance degrades by >10%
cargo bench --bench navigation_benchmarks -- --baseline main --threshold 10
```

## Troubleshooting

### Benchmark Takes Too Long
```bash
# Reduce sample size
cargo bench --bench navigation_benchmarks -- --sample-size 10

# Reduce measurement time
cargo bench --bench navigation_benchmarks -- --measurement-time 5
```

### Inconsistent Results
- Close other applications
- Disable CPU frequency scaling
- Run multiple times and average
- Use dedicated benchmark machine

### Missing Dimension Files
Benchmarks use simulated data when dimension files don't exist.
This is expected and doesn't affect performance measurements.

## Hardware Considerations

### CPU
- More cores = better concurrent performance
- Higher clock speed = better single-query performance
- Cache size affects registry lookup speed

### Memory
- More RAM = more dimensions can be cached
- Faster RAM = better MMAP performance
- Memory bandwidth affects concurrent queries

### Disk
- SSD vs HDD: 10-100x difference in dimension loading
- NVMe vs SATA: 2-5x difference
- MMAP benefits from fast storage

## Best Practices

1. **Consistent Environment**: Run on same hardware
2. **Minimal Load**: Close other applications
3. **Multiple Runs**: Average across runs
4. **Baseline Tracking**: Compare against previous versions
5. **Document Changes**: Note what changed between runs

## Example Session

```bash
# 1. Save current baseline
cargo bench --bench navigation_benchmarks -- --save-baseline before-opt

# 2. Make optimization changes
# ... edit code ...

# 3. Compare with baseline
cargo bench --bench navigation_benchmarks -- --baseline before-opt

# 4. View detailed report
open target/criterion/report/index.html

# 5. If improved, save as new baseline
cargo bench --bench navigation_benchmarks -- --save-baseline after-opt
```

## Performance Targets Summary

| Operation | Target | Importance |
|-----------|--------|------------|
| Registry Lookup | <1μs | Critical |
| Query Analysis | <5ms | High |
| Full Navigation | <150ms p95 | Critical |
| Memory Loading | <50ms | High |
| Concurrent Scaling | Linear | Medium |

---

*For detailed results, see [benchmark-results.md](benchmark-results.md)*
