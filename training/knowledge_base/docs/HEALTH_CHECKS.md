# Health Check Implementation

## Overview

The Jessy system implements comprehensive health checks for all services to ensure reliability and enable proper orchestration. Health checks are implemented at multiple levels:

1. **Application Level**: HTTP endpoints that report service health
2. **Docker Level**: Container health checks using HEALTHCHECK directive
3. **Compose Level**: Service dependency management using health conditions

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Docker Compose                         │
│  ┌────────────────────────────────────────────────┐    │
│  │  jessy-api (depends_on: jessy-core healthy)    │    │
│  │  - Waits for jessy-core to be healthy          │    │
│  │  - Health: /api/health (10s interval)          │    │
│  └────────────────────────────────────────────────┘    │
│                        ↓                                 │
│  ┌────────────────────────────────────────────────┐    │
│  │  jessy-core (Rust service)                      │    │
│  │  - Health: /health (10s interval)               │    │
│  │  - Must be healthy before API starts            │    │
│  └────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

## Health Endpoints

### Rust Service (jessy-core)

**Endpoint**: `GET /health`

**Response**:
```json
{
  "status": "healthy",
  "service": "jessy-core",
  "version": "0.1.0",
  "timestamp": "2024-10-24T10:30:00Z"
}
```

**Implementation** (`src/bin/jessy.rs`):
```rust
#[get("/health")]
async fn health() -> impl Responder {
    let response = HealthResponse {
        status: "healthy".to_string(),
        service: "jessy-core".to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        timestamp: chrono::Utc::now().to_rfc3339(),
    };
    
    HttpResponse::Ok().json(response)
}
```

**Health Check Command**:
```bash
curl -f http://localhost:8080/health
```

### Go Service (jessy-api)

**Endpoint**: `GET /api/health`

**Response**:
```json
{
  "status": "healthy",
  "service": "jessy-api",
  "version": "1.0.0"
}
```

**Implementation** (`api/main.go`):
```go
app.Get("/api/health", func(c *fiber.Ctx) error {
    return c.JSON(fiber.Map{
        "status":  "healthy",
        "service": "jessy-api",
        "version": "1.0.0",
    })
})
```

**Health Check Command**:
```bash
wget --no-verbose --tries=1 --spider http://localhost:3000/api/health
```

## Docker Configuration

### Rust Service Dockerfile

```dockerfile
# Development stage
FROM rust:1.75-slim as development
RUN apt-get update && apt-get install -y curl
# ... other setup ...

# Production stage
FROM debian:bookworm-slim as production
RUN apt-get update && apt-get install -y curl
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8080/health || exit 1
```

### Go Service Dockerfile

```dockerfile
# Development stage
FROM golang:1.21-alpine as development
RUN apk add --no-cache wget
# ... other setup ...

# Production stage
FROM alpine:latest as production
RUN apk --no-cache add wget
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:3000/api/health || exit 1
```

## Docker Compose Configuration

```yaml
services:
  jessy-core:
    # ... build config ...
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 10s
      timeout: 5s
      retries: 3
      start_period: 10s
    restart: unless-stopped

  jessy-api:
    # ... build config ...
    depends_on:
      jessy-core:
        condition: service_healthy
    healthcheck:
      test: ["CMD", "wget", "--no-verbose", "--tries=1", "--spider", "http://localhost:3000/api/health"]
      interval: 10s
      timeout: 5s
      retries: 3
      start_period: 10s
    restart: unless-stopped
```

## Health Check Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `interval` | 10s | Time between health checks |
| `timeout` | 5s | Maximum time for health check to complete |
| `retries` | 3 | Number of consecutive failures before unhealthy |
| `start_period` | 10s | Grace period during container startup |

## Service Dependencies

The Go API service depends on the Rust core service being healthy:

```yaml
jessy-api:
  depends_on:
    jessy-core:
      condition: service_healthy
```

This ensures:
1. `jessy-core` starts first
2. `jessy-api` waits for `jessy-core` to be healthy
3. If `jessy-core` becomes unhealthy, `jessy-api` can detect it

## Graceful Shutdown

Both services implement graceful shutdown to handle SIGTERM signals:

### Rust Service
```rust
tokio::select! {
    result = server => {
        result?;
    }
    _ = tokio::signal::ctrl_c() => {
        info!("🛑 Received shutdown signal, gracefully stopping...");
        server_handle.stop(true).await;
        info!("✅ Shutdown complete");
    }
}
```

### Go Service
```go
quit := make(chan os.Signal, 1)
signal.Notify(quit, os.Interrupt, syscall.SIGTERM)
<-quit

zlog.Info().Msg("Shutting down server gracefully...")
if err := app.ShutdownWithTimeout(30 * time.Second); err != nil {
    zlog.Error().Err(err).Msg("Server forced to shutdown")
}
```

## Testing Health Checks

### Manual Testing

**Test Rust service**:
```bash
curl http://localhost:8080/health
```

**Test Go service**:
```bash
curl http://localhost:3000/api/health
```

**Check Docker health status**:
```bash
docker inspect --format='{{.State.Health.Status}}' jessy-core
docker inspect --format='{{.State.Health.Status}}' jessy-api
```

### Automated Testing

Run the comprehensive health check test suite:

```bash
./scripts/test-health-checks.sh
```

This script tests:
- ✅ Initial health check endpoints
- ✅ Health check response format
- ✅ Docker health check status
- ✅ Service dependency ordering
- ✅ Health check failure detection (optional)
- ✅ Health check recovery (optional)

### Test Failure Scenarios

To test health check failure and recovery:

1. **Simulate service crash**:
```bash
# Kill the process inside the container
docker-compose exec jessy-core pkill -9 jessy

# Wait for health check to detect failure (30-40 seconds)
watch docker inspect --format='{{.State.Health.Status}}' jessy-core
```

2. **Verify automatic restart**:
```bash
# With restart: unless-stopped, container should restart
docker-compose ps

# Check logs
docker-compose logs jessy-core
```

3. **Test recovery**:
```bash
# Manually restart if needed
docker-compose restart jessy-core

# Verify health returns
curl http://localhost:8080/health
```

## Monitoring Health Checks

### View Health Status

```bash
# All services
docker-compose ps

# Specific service
docker inspect jessy-core | jq '.[0].State.Health'
```

### View Health Check Logs

```bash
# Last 5 health check results
docker inspect jessy-core | jq '.[0].State.Health.Log[-5:]'
```

### Continuous Monitoring

```bash
# Watch health status
watch -n 1 'docker-compose ps'

# Stream logs
docker-compose logs -f jessy-core jessy-api
```

## Troubleshooting

### Service Shows as Unhealthy

1. **Check service logs**:
```bash
docker-compose logs jessy-core
```

2. **Test endpoint manually**:
```bash
docker-compose exec jessy-core curl http://localhost:8080/health
```

3. **Check if service is running**:
```bash
docker-compose exec jessy-core ps aux
```

4. **Verify port binding**:
```bash
docker-compose exec jessy-core netstat -tlnp
```

### Health Check Times Out

1. **Increase timeout**:
```yaml
healthcheck:
  timeout: 10s  # Increase from 5s
```

2. **Check service startup time**:
```yaml
healthcheck:
  start_period: 30s  # Increase grace period
```

### Service Dependency Issues

1. **Check dependency configuration**:
```bash
docker-compose config | grep -A 5 depends_on
```

2. **Verify health check is working**:
```bash
docker inspect --format='{{.State.Health.Status}}' jessy-core
```

3. **Check startup order**:
```bash
docker-compose logs | grep "Starting"
```

## Best Practices

### Health Endpoint Design

✅ **Do**:
- Return 200 OK when healthy
- Include service name and version
- Keep checks lightweight (< 1s)
- Check critical dependencies
- Use structured JSON responses

❌ **Don't**:
- Perform expensive operations
- Check non-critical dependencies
- Return 200 when unhealthy
- Include sensitive information
- Make external API calls

### Health Check Configuration

✅ **Do**:
- Set appropriate intervals (10-30s)
- Allow grace period for startup
- Use 3+ retries to avoid false positives
- Set reasonable timeouts (3-5s)
- Configure restart policies

❌ **Don't**:
- Check too frequently (< 5s)
- Set timeout > interval
- Use 1 retry (too sensitive)
- Forget start_period
- Ignore restart policies

### Service Dependencies

✅ **Do**:
- Use `condition: service_healthy`
- Order services by dependency
- Handle dependency failures gracefully
- Implement circuit breakers
- Log dependency status

❌ **Don't**:
- Create circular dependencies
- Assume services are always available
- Fail hard on dependency issues
- Skip health checks for dependencies
- Ignore dependency health status

## Integration with CI/CD

Health checks are automatically tested in CI/CD:

```yaml
# .github/workflows/ci.yml
- name: Start services
  run: docker-compose up -d

- name: Wait for services to be healthy
  run: |
    timeout 60 bash -c 'until docker inspect --format="{{.State.Health.Status}}" jessy-core | grep -q healthy; do sleep 1; done'
    timeout 60 bash -c 'until docker inspect --format="{{.State.Health.Status}}" jessy-api | grep -q healthy; do sleep 1; done'

- name: Run health check tests
  run: ./scripts/test-health-checks.sh
```

## Metrics and Observability

Health check metrics to monitor:

- **Health check success rate**: Should be > 99%
- **Health check latency**: Should be < 100ms
- **Time to healthy**: Should be < 30s
- **Restart frequency**: Should be minimal
- **Dependency wait time**: Track startup delays

## References

- [Docker HEALTHCHECK documentation](https://docs.docker.com/engine/reference/builder/#healthcheck)
- [Docker Compose healthcheck](https://docs.docker.com/compose/compose-file/compose-file-v3/#healthcheck)
- [Service dependency conditions](https://docs.docker.com/compose/compose-file/compose-file-v3/#depends_on)
- Requirements: 5.1, 5.2 from docker-cicd-infrastructure spec

---

*"Health checks are the heartbeat of distributed systems. Monitor them well."*
