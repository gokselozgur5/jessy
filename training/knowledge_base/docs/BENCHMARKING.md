# Performance Benchmarking Guide

## Overview

Jessy uses [Criterion.rs](https://github.com/bheisler/criterion.rs) for performance benchmarking and regression detection. Benchmarks run in Docker containers to ensure consistent results across environments.

## Running Benchmarks

### Basic Usage

```bash
# Run all benchmarks
make bench

# Run specific benchmark
docker-compose run --rm jessy-test cargo bench --bench memory_benchmarks

# Run with verbose output
docker-compose run --rm jessy-test cargo bench -- --verbose
```

### Baseline Comparison

```bash
# Save current performance as baseline
make bench-baseline

# Make code changes...

# Compare against baseline
make bench-compare
```

## Available Benchmarks

### Memory Benchmarks (`benches/memory_benchmarks.rs`)

Tests memory management performance:

- **Allocation**: Tests allocation of various sizes (1KB - 64KB)
- **Read**: Tests read operations of various sizes (64B - 4KB)
- **Write**: Tests write operations of various sizes (64B - 4KB)

**Performance Targets**:
- Allocation: <1ms for sizes up to 64KB
- Read: <100μs for sizes up to 4KB
- Write: <100μs for sizes up to 4KB

### Dimension Benchmarks (`benches/dimension_benchmarks.rs`)

Tests dimensional layer operations:

- **Creation**: Tests dimension creation performance
- **Lookup**: Tests dimension lookup with varying registry sizes (10-1000 dimensions)
- **Scan**: Tests frequency-based dimension scanning (10-1000 dimensions)

**Performance Targets**:
- Creation: <10μs per dimension
- Lookup: <1μs (O(1) hash lookup)
- Scan: <100ms for 1000 dimensions

## Benchmark Results

### Viewing Reports

Criterion generates HTML reports with detailed statistics:

```bash
# After running benchmarks
open target/criterion/report/index.html
```

Reports include:
- Mean execution time
- Standard deviation
- Throughput measurements
- Comparison with previous runs
- Violin plots and histograms

### Understanding Output

```
memory_allocation/1024  time:   [245.67 ns 248.32 ns 251.24 ns]
                        change: [-2.3421% -0.8123% +0.5421%] (p = 0.23 > 0.05)
                        No change in performance detected.
```

- **time**: Mean and confidence interval
- **change**: Performance change vs previous run
- **p-value**: Statistical significance (p < 0.05 = significant change)

## Regression Detection

### Setting Baselines

Save baseline before making changes:

```bash
# Before optimization
make bench-baseline

# Make changes to code...

# Compare performance
make bench-compare
```

### Interpreting Changes

- **Green**: Performance improved (faster)
- **Red**: Performance regressed (slower)
- **Yellow**: No significant change

### Acceptable Thresholds

- **<5% change**: Acceptable noise
- **5-10% change**: Review carefully
- **>10% change**: Investigate thoroughly

## Writing Benchmarks

### Basic Structure

```rust
use criterion::{black_box, criterion_group, criterion_main, Criterion};

fn bench_my_function(c: &mut Criterion) {
    c.bench_function("my_function", |b| {
        b.iter(|| {
            // Code to benchmark
            my_function(black_box(42))
        });
    });
}

criterion_group!(benches, bench_my_function);
criterion_main!(benches);
```

### Best Practices

1. **Use `black_box`**: Prevents compiler optimizations
   ```rust
   b.iter(|| my_function(black_box(input)));
   ```

2. **Benchmark groups**: Test multiple input sizes
   ```rust
   let mut group = c.benchmark_group("my_group");
   for size in [10, 100, 1000].iter() {
       group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
           b.iter(|| my_function(black_box(size)));
       });
   }
   group.finish();
   ```

3. **Setup/teardown**: Use `iter_batched` for expensive setup
   ```rust
   b.iter_batched(
       || expensive_setup(),
       |data| my_function(data),
       BatchSize::SmallInput
   );
   ```

4. **Throughput**: Measure operations per second
   ```rust
   group.throughput(Throughput::Bytes(size as u64));
   ```

## CI/CD Integration

### Automated Benchmarking

Benchmarks run automatically in CI on:
- Pull requests (compare against main)
- Main branch commits (update baseline)
- Release tags (performance validation)

### Performance Gates

CI fails if:
- Any benchmark regresses >10%
- Critical path regresses >5%
- Memory usage increases >20%

## Troubleshooting

### Inconsistent Results

**Problem**: Benchmark results vary significantly between runs

**Solutions**:
1. Increase sample size:
   ```bash
   cargo bench -- --sample-size 1000
   ```

2. Increase measurement time:
   ```bash
   cargo bench -- --measurement-time 10
   ```

3. Check system load:
   ```bash
   # Run benchmarks on idle system
   docker-compose run --rm jessy-test cargo bench
   ```

### Slow Benchmarks

**Problem**: Benchmarks take too long to run

**Solutions**:
1. Reduce sample size for development:
   ```bash
   cargo bench -- --sample-size 10 --quick
   ```

2. Run specific benchmarks:
   ```bash
   cargo bench --bench memory_benchmarks
   ```

3. Use `--profile-time` to find slow benchmarks:
   ```bash
   cargo bench -- --profile-time 1
   ```

### Out of Memory

**Problem**: Benchmarks fail with OOM errors

**Solutions**:
1. Increase Docker memory limit
2. Reduce benchmark input sizes
3. Add cleanup between iterations:
   ```rust
   b.iter_batched(
       || setup(),
       |data| {
           let result = my_function(data);
           drop(data); // Explicit cleanup
           result
       },
       BatchSize::SmallInput
   );
   ```

## Performance Optimization Workflow

1. **Establish baseline**:
   ```bash
   make bench-baseline
   ```

2. **Profile code**:
   ```bash
   cargo flamegraph --bench memory_benchmarks
   ```

3. **Make optimization**:
   - Change algorithm
   - Reduce allocations
   - Improve cache locality

4. **Benchmark changes**:
   ```bash
   make bench-compare
   ```

5. **Validate improvement**:
   - Check statistical significance
   - Verify no regressions elsewhere
   - Review memory usage

6. **Document results**:
   - Update performance targets
   - Note optimization techniques
   - Add regression tests

## Related Documentation

- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)
- [Memory Manager Design](specifications/memory-manager/design.md)
- [Development Workflow](DEVELOPMENT_PRINCIPLES.md)

---

*"Measure twice, optimize once. Benchmarks reveal truth. ⚡"*
