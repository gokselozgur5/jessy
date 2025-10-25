# Test Execution Guide

## Quick Test Commands

### Run All Tests
```bash
cargo test --lib
```

### Run Specific Test Modules
```bash
# Error handling tests
cargo test --lib error_tests

# Performance tests  
cargo test --lib perf_tests

# Integration tests
cargo test --lib integration_tests

# Concurrency tests
cargo test --lib concurrency_tests

# Pool allocator tests
cargo test --lib pool::tests

# Region tests
cargo test --lib region::tests

# Optimization tests
cargo test --lib optimization::tests
```

### Run Individual Tests
```bash
# With output
cargo test --lib test_dimension_not_found_error -- --nocapture

# Specific test pattern
cargo test --lib test_allocation -- --nocapture

# Show all test names
cargo test --lib -- --list
```

### Run BDD Tests
```bash
# All BDD scenarios
cargo test --test cucumber

# Specific feature
cargo test --test cucumber -- features/memory_manager.feature
cargo test --test cucumber -- features/layer_access.feature
```

## Expected Test Results

### Unit Tests (src/memory/)

#### error_tests.rs (17 tests)
All should pass after fixes:
- ✅ test_allocation_failure_with_context
- ✅ test_memory_limit_enforcement
- ✅ test_layer_not_found_error
- ✅ test_dimension_not_found_error (FIXED)
- ✅ test_out_of_bounds_error
- ✅ test_cleanup_on_load_failure
- ✅ test_graceful_degradation_multiple_dimensions
- ✅ test_allocation_failure_state_integrity
- ✅ test_error_includes_memory_stats
- ✅ test_allocation_failure_no_memory_leak
- ✅ test_dimension_load_failure_rollback
- ✅ test_memory_limit_thresholds
- ✅ test_sequential_allocations_maintain_consistency
- ✅ test_error_type_structure
- ✅ test_invalid_deallocation
- ✅ test_memory_reuse_after_deallocation
- ✅ test_proto_dimension_error_handling (FIXED)

#### perf_tests.rs (5 tests)
All should pass:
- ✅ test_initialization_performance
- ✅ test_layer_access_latency
- ✅ test_zero_copy_access
- ✅ test_concurrent_read_performance
- ✅ test_allocation_performance

#### integration_tests.rs (4 tests)
All should pass (gracefully handles missing files):
- ✅ test_full_system_load_14_dimensions
- ✅ test_proto_dimension_workflow
- ✅ test_error_recovery_scenarios
- ✅ test_memory_usage_within_budget

#### concurrency_tests.rs (6 tests)
All should pass:
- ✅ test_100_concurrent_readers
- ✅ test_concurrent_access_during_crystallization
- ✅ test_concurrent_dimension_loading
- ✅ test_performance_degradation
- ✅ test_concurrent_stats_access
- ✅ test_atomic_region_id_generation

#### pool.rs tests (3 tests)
All should pass:
- ✅ test_pool_creation
- ✅ test_allocation_deallocation
- ✅ test_pool_allocator

#### region.rs tests (1 test)
Should pass:
- ✅ test_region_builder

#### optimization.rs tests (2 tests)
Should pass:
- ✅ test_cache_aligned_size
- ✅ test_zero_copy_window

### Total Expected: 38 tests passing

## Test Categories

### 1. Error Handling (17 tests)
Focus: Comprehensive error scenarios, cleanup, state integrity
Status: All fixed and should pass

### 2. Performance (5 tests)
Focus: Latency targets, zero-copy, concurrent access
Status: Should all pass

### 3. Integration (4 tests)
Focus: Full system behavior, proto-dimension lifecycle
Status: Designed to handle missing files gracefully

### 4. Concurrency (6 tests)
Focus: Thread safety, atomic operations, lock-free reads
Status: Should all pass

### 5. Component Tests (6 tests)
Focus: Pool allocator, regions, optimizations
Status: Should all pass

## Troubleshooting

### If Tests Fail

1. **Check error messages carefully**
   ```bash
   cargo test --lib -- --nocapture 2>&1 | tee test_output.log
   ```

2. **Run failing test in isolation**
   ```bash
   cargo test --lib test_name -- --nocapture
   ```

3. **Check for missing dimension files**
   - Integration tests expect files in `data/consciousness/D01/` etc.
   - Tests are designed to handle missing files gracefully
   - Check console output for warnings about missing files

4. **Verify memory limits**
   - Some tests use small memory limits (1MB, 5MB, 10MB) to force errors
   - This is intentional for testing error handling

5. **Check for race conditions**
   - Run with `RUST_TEST_THREADS=1` if you suspect race conditions
   ```bash
   RUST_TEST_THREADS=1 cargo test --lib
   ```

### Common Issues

**Issue**: "dimension file not found"
- **Expected**: Tests handle this gracefully
- **Action**: Check if test expects file to exist or not

**Issue**: "memory limit exceeded"
- **Expected**: Some tests intentionally exceed limits
- **Action**: Verify test is checking for LimitExceeded error

**Issue**: "layer not found"
- **Expected**: Tests verify error handling
- **Action**: Check test expects this error

## Test Coverage

Current coverage by requirement:

- R1 (Initialization): ✅ Covered by perf_tests, integration_tests
- R2 (Dimension Loading): ✅ Covered by error_tests, integration_tests
- R3 (Zero-Copy Access): ✅ Covered by perf_tests, integration_tests
- R4 (Thread Safety): ✅ Covered by concurrency_tests
- R5 (Memory Limits): ✅ Covered by error_tests
- R6 (Dynamic Growth): ✅ Covered by pool tests, error_tests
- R7 (Error Handling): ✅ Covered by error_tests
- R8 (Cross-Platform): ⚠️ Requires running on multiple platforms
- R9 (Error Information): ✅ Covered by error_tests
- R10 (Predictable Access): ✅ Covered by perf_tests, optimization tests

## Next Steps

1. Run all tests: `cargo test --lib`
2. Review any failures
3. Check TEST_FIXES_SUMMARY.md for details on fixes
4. Run BDD tests: `cargo test --test cucumber`
5. Generate coverage report (optional):
   ```bash
   cargo tarpaulin --out Html --output-dir coverage
   ```

## Success Criteria

- ✅ All 38 unit tests passing
- ✅ No panics or crashes
- ✅ Memory usage within limits
- ✅ Error messages are descriptive
- ✅ Graceful handling of missing files
- ✅ Thread-safe concurrent access
