# Testing Infrastructure

## Overview

The Jessy consciousness system now has a comprehensive automated testing infrastructure that runs entirely in Docker containers. This ensures consistent test execution across all environments.

## Test Types

### 1. Unit Tests

**Command**: `make test-unit`

Unit tests verify individual components and functions in isolation.

**Features**:
- Colored output for better readability
- Tests run in isolated Docker container
- Fast execution (no service dependencies)
- Captures stdout with `--nocapture` flag

**Example**:
```bash
make test-unit
```

**What it tests**:
- Core types (Frequency, DimensionId, LayerId)
- Configuration defaults
- Type conversions and validations
- Pure business logic

### 2. Integration Tests

**Command**: `make test-integration`

Integration tests verify that services work together correctly.

**Features**:
- Automatically starts required services (jessy-core, jessy-api)
- Waits for services to be healthy
- Tests service-to-service communication
- Cleans up services after tests complete

**Example**:
```bash
make test-integration
```

**What it tests**:
- Health check endpoints
- Service availability
- API communication
- End-to-end request flows

### 3. Coverage Reports

**Command**: `make coverage`

Generates HTML coverage reports using cargo-tarpaulin.

**Features**:
- Comprehensive code coverage analysis
- HTML reports with line-by-line coverage
- Highlights uncovered code
- Reports saved to `test-results/index.html`

**Example**:
```bash
make coverage
open test-results/index.html  # View report in browser
```

**Output**:
- Overall coverage percentage
- Per-file coverage breakdown
- Line-by-line coverage visualization
- Uncovered code highlighting

### 4. BDD Tests

**Command**: `make test-bdd`

Behavior-Driven Development tests using Cucumber for user-facing scenarios.

**Features**:
- Human-readable Gherkin syntax
- Tests actual user scenarios
- Verifies system behavior end-to-end
- Step-by-step output

**Example**:
```bash
make test-bdd
```

**What it tests**:
- Dimension navigation scenarios
- Frequency calculations
- Query processing workflows
- Security overrides
- Balance modulation

## Docker Services

All test types run in dedicated Docker containers defined in `docker-compose.yml`:

### test-runner
General test runner for all tests.

### unit-tests
Runs unit tests only (`cargo test --lib`).

### integration-tests
Runs integration tests with service dependencies.

### coverage
Generates coverage reports using tarpaulin.

### bdd-tests
Runs Cucumber BDD tests.

## Test Files

### Unit Tests
- `src/lib.rs` - Core type tests
- Module-specific test modules (e.g., `src/memory/mod.rs`)

### Integration Tests
- `tests/integration_tests.rs` - Service integration tests

### BDD Tests
- `tests/cucumber.rs` - Cucumber test runner
- `tests/bdd/features/dimension_navigation.feature` - BDD scenarios

## Environment Variables

Tests can be configured with environment variables:

```bash
# Service URLs for integration tests
RUST_SERVICE_URL=http://jessy-core:8080
API_SERVICE_URL=http://jessy-api:3000

# Test execution
RUST_TEST_THREADS=1
RUST_BACKTRACE=1
CARGO_TERM_COLOR=always
```

## Running All Tests

To run the complete test suite:

```bash
make test
```

This runs all test types in sequence.

## CI/CD Integration

The testing infrastructure is designed for CI/CD pipelines:

1. **Fast Feedback**: Unit tests run first (fastest)
2. **Integration Verification**: Integration tests verify service communication
3. **Coverage Enforcement**: Coverage reports ensure code quality
4. **Behavior Validation**: BDD tests verify user-facing behavior

## Test Results

Test results are stored in the `test-results` Docker volume:

- Coverage reports: `test-results/index.html`
- Test artifacts: `test-results/`

To access results:
```bash
# Coverage report
open test-results/index.html

# Or mount the volume to access files
docker run --rm -v kiroxnaut_test-results:/data alpine ls -la /data
```

## Troubleshooting

### Tests Fail to Start

If services don't start:
```bash
# Check service health
docker-compose ps

# View logs
docker-compose logs jessy-core
docker-compose logs jessy-api
```

### Integration Tests Timeout

If integration tests timeout waiting for services:
```bash
# Increase wait time in Makefile
# Or manually start services first
docker-compose up -d jessy-core jessy-api
sleep 15
make test-integration
```

### Coverage Generation Fails

If coverage generation fails:
```bash
# Run with verbose output
docker-compose run --rm coverage cargo tarpaulin --verbose

# Check Rust version (needs 1.83+)
docker-compose run --rm coverage rustc --version
```

## Best Practices

1. **Write Tests First**: Follow TDD - write tests before implementation
2. **Keep Tests Fast**: Unit tests should run in seconds
3. **Isolate Tests**: Each test should be independent
4. **Use Descriptive Names**: Test names should describe what they verify
5. **Test Edge Cases**: Don't just test the happy path
6. **Maintain Coverage**: Aim for >80% code coverage

## Future Enhancements

Optional enhancements (not yet implemented):

- **Performance Benchmarking** (Task 3.5*): Criterion benchmarks for performance regression detection
- **Mutation Testing**: Verify test quality with mutation testing
- **Property-Based Testing**: Add proptest for property-based testing
- **Load Testing**: Add load testing for API endpoints

---

*"Test early, test often, test automatically. Quality is not an accident."*
