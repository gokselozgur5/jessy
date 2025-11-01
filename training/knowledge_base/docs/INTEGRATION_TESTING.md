# Integration Testing Guide

## Overview

Integration tests verify that services work together correctly in a containerized environment. The Jessy project uses Docker Compose to orchestrate services and run tests against them.

## Running Integration Tests

### Quick Start

```bash
make test-integration
```

This command will:
1. Start required services (jessy-core and jessy-api)
2. Wait for services to be healthy
3. Run integration tests in an isolated container
4. Clean up services regardless of test outcome

### Alternative: Using the Script Directly

For more control and better error reporting:

```bash
./scripts/run-integration-tests.sh
```

This script provides:
- Detailed health check monitoring
- Better error messages with service logs
- Automatic cleanup on exit
- Color-coded output

## Test Infrastructure

### Docker Compose Configuration

The `integration-tests` service is defined in `docker-compose.yml`:

```yaml
integration-tests:
  build:
    context: .
    dockerfile: docker/Dockerfile.test
  environment:
    - RUST_SERVICE_URL=http://jessy-core:8080
    - API_SERVICE_URL=http://jessy-api:3000
  command: ["cargo", "test", "--test", "*", "--all-features", "--color=always"]
  depends_on:
    jessy-core:
      condition: service_healthy
    jessy-api:
      condition: service_healthy
  networks:
    - jessy-network
  profiles:
    - test
```

### Test Files

Integration tests are located in `tests/integration_tests.rs`. These tests:
- Use `reqwest` to make HTTP requests to services
- Verify health endpoints
- Test service-to-service communication
- Check API responses

## Writing Integration Tests

### Test Structure

```rust
#[tokio::test]
async fn test_service_endpoint() {
    let url = format!("{}/endpoint", rust_service_url());
    
    let client = reqwest::Client::new();
    let response = client.get(&url)
        .send()
        .await
        .expect("Failed to connect to service");
    
    assert!(response.status().is_success());
    
    let body = response.json::<ResponseType>().await.unwrap();
    assert_eq!(body.field, expected_value);
}
```

### Environment Variables

Tests can access service URLs through environment variables:
- `RUST_SERVICE_URL`: URL of the Rust core service (default: `http://localhost:8080`)
- `API_SERVICE_URL`: URL of the Go API service (default: `http://localhost:3000`)

### Best Practices

1. **Use async/await**: All integration tests should be async
2. **Test real endpoints**: Don't mock service responses
3. **Check status codes**: Verify both success and error cases
4. **Clean test data**: Ensure tests don't leave side effects
5. **Use descriptive names**: Test names should explain what they verify

## Health Checks

Services must implement health check endpoints:

### Rust Service
- Endpoint: `GET /health`
- Expected response: JSON with `status: "healthy"`

### Go API
- Endpoint: `GET /api/health`
- Expected response: JSON with `status: "healthy"`

## Troubleshooting

### Services Won't Start

Check service logs:
```bash
docker-compose logs jessy-core
docker-compose logs jessy-api
```

Common issues:
- Compilation errors in source code
- Port conflicts (8080 or 3000 already in use)
- Missing dependencies

### Tests Fail to Connect

1. Verify services are running:
   ```bash
   docker-compose ps
   ```

2. Check service health:
   ```bash
   curl http://localhost:8080/health
   curl http://localhost:3000/api/health
   ```

3. Verify network connectivity:
   ```bash
   docker-compose exec integration-tests ping jessy-core
   ```

### Cleanup Issues

If services don't stop properly:
```bash
docker-compose down -v
docker system prune -f
```

## CI/CD Integration

Integration tests run automatically in CI/CD pipelines:

```yaml
# .github/workflows/ci.yml
- name: Run integration tests
  run: make test-integration
```

## Performance Considerations

- **Startup time**: Services take 10-30 seconds to become healthy
- **Test execution**: Integration tests are slower than unit tests
- **Resource usage**: Requires Docker and ~500MB RAM
- **Parallel execution**: Tests run sequentially to avoid conflicts

## Requirements Mapping

This implementation satisfies requirements:
- **2.1**: Automated testing in containers
- **2.2**: Real-time output with colored formatting

---

*"Integration tests verify the symphony, not just the instruments."*
