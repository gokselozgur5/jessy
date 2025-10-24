# Docker Setup and Architecture

## Overview

Jessy uses Docker and Docker Compose for development, testing, and deployment. This document provides a comprehensive guide to the Docker infrastructure.

## Architecture

### Service Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Docker Network (jessy-network)           │
│                                                              │
│  ┌──────────────┐         ┌──────────────┐                 │
│  │              │         │              │                 │
│  │  jessy-core  │◄────────┤  jessy-api   │                 │
│  │  (Rust)      │         │  (Go)        │                 │
│  │  Port: 8080  │         │  Port: 3000  │                 │
│  │              │         │              │                 │
│  └──────┬───────┘         └──────┬───────┘                 │
│         │                        │                         │
│         │                        │                         │
│         ▼                        ▼                         │
│  ┌─────────────────────────────────────┐                  │
│  │     Shared Volumes                  │                  │
│  │  - cargo-cache (Rust deps)          │                  │
│  │  - target-cache (Build artifacts)   │                  │
│  │  - go-cache (Go modules)            │                  │
│  │  - mmap-data (Dimensional layers)   │                  │
│  │  - test-results (Test outputs)      │                  │
│  └─────────────────────────────────────┘                  │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Container Details

#### jessy-core (Rust Service)
- **Base Image**: `rust:latest` (development), `alpine:latest` (production)
- **Purpose**: Core consciousness engine with dimensional layer management
- **Port**: 8080
- **Health Check**: `curl http://localhost:8080/health`
- **Hot Reload**: cargo-watch monitors source changes
- **Volumes**:
  - Source code: `./src:/app/src`
  - Dependencies: `cargo-cache:/usr/local/cargo/registry`
  - Build artifacts: `target-cache:/app/target`

#### jessy-api (Go Service)
- **Base Image**: `golang:1.23-alpine` (development), `alpine:latest` (production)
- **Purpose**: REST API gateway and HTTP interface
- **Port**: 3000
- **Health Check**: `wget http://localhost:3000/api/health`
- **Hot Reload**: air monitors source changes
- **Volumes**:
  - Source code: `./api:/app/api`
  - Go modules: `go-cache:/go/pkg/mod`
- **Dependencies**: Waits for jessy-core to be healthy

#### jessy-test (Test Runner)
- **Base Image**: `rust:latest`
- **Purpose**: Runs all test suites in isolated environment
- **Volumes**: Same as jessy-core plus test directories
- **Profiles**: Only runs with `--profile test`

## Quick Start

### Prerequisites

- Docker 20.10+
- Docker Compose 2.0+
- 4GB+ RAM available for Docker
- 10GB+ disk space

### Installation

```bash
# Clone repository
git clone https://github.com/yourusername/jessy.git
cd jessy

# Start services
make up

# View logs
make logs

# Stop services
make down
```

### Verify Installation

```bash
# Check services are running
docker-compose ps

# Test Rust service
curl http://localhost:8080/health

# Test Go API
curl http://localhost:3000/api/health
```

## Development Workflows

### Starting Development

```bash
# Start all services with hot reload
make watch

# Or start specific service
docker-compose up jessy-core
docker-compose up jessy-api
```

### Making Code Changes

1. **Edit source files** - Changes are automatically detected
2. **Watch logs** - See compilation and restart in real-time
3. **Test changes** - Service restarts automatically on success

```bash
# Watch Rust service logs
make logs-rust

# Watch Go API logs
make logs-go

# Watch all logs
make logs
```

### Running Tests

```bash
# Run all tests
make test

# Run specific test suites
make test-unit           # Unit tests only
make test-integration    # Integration tests
make test-bdd           # BDD/Cucumber tests

# Generate coverage report
make coverage
```

### Debugging

```bash
# Open shell in Rust container
make shell-rust

# Open shell in Go container
make shell-go

# View service logs
docker-compose logs -f jessy-core
docker-compose logs -f jessy-api

# Inspect container
docker inspect jessy-core
docker exec -it jessy-core ps aux
```

### Building for Production

```bash
# Build production images
docker-compose build --target production

# Test production build
docker-compose -f docker-compose.prod.yml up

# Push to registry
docker-compose push
```

## Volume Management

### Named Volumes

Jessy uses named volumes for caching and data persistence:

```yaml
volumes:
  cargo-cache:      # Rust dependency cache (~500MB)
  target-cache:     # Rust build artifacts (~2GB)
  go-cache:         # Go module cache (~100MB)
  test-results:     # Test outputs and coverage reports
  mmap-data:        # Dimensional layer data files
```

### Inspecting Volumes

```bash
# List all volumes
docker volume ls | grep jessy

# Inspect volume
docker volume inspect cargo-cache

# Check volume size
docker system df -v
```

### Cleaning Volumes

```bash
# Remove all volumes (WARNING: deletes cached data)
make clean

# Remove specific volume
docker volume rm cargo-cache

# Prune unused volumes
docker volume prune
```

### Volume Performance

**Cache Hit Rates**:
- First build: ~60s (Rust), ~10s (Go)
- Cached build: ~5s (Rust), ~2s (Go)
- Hot reload: ~2s (Rust), ~1s (Go)

**Disk Usage**:
- cargo-cache: ~500MB
- target-cache: ~2GB (debug), ~500MB (release)
- go-cache: ~100MB
- Total: ~3GB for development

## Network Configuration

### Bridge Network

All services communicate via `jessy-network` bridge:

```yaml
networks:
  jessy-network:
    driver: bridge
```

### Service Discovery

Services can reach each other by name:

```bash
# From jessy-api to jessy-core
curl http://jessy-core:8080/health

# From jessy-core to jessy-api
curl http://jessy-api:3000/api/health
```

### Port Mapping

```
Host Port → Container Port
8080      → jessy-core:8080
3000      → jessy-api:3000
```

## Health Checks

### Configuration

```yaml
healthcheck:
  test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
  interval: 10s      # Check every 10 seconds
  timeout: 5s        # Fail if no response in 5s
  retries: 3         # Try 3 times before marking unhealthy
  start_period: 10s  # Grace period for startup
```

### Checking Health

```bash
# View health status
docker-compose ps

# Inspect health check
docker inspect jessy-core --format='{{.State.Health.Status}}'

# View health check logs
docker inspect jessy-core --format='{{range .State.Health.Log}}{{.Output}}{{end}}'
```

## Restart Policies

### Configuration

```yaml
restart: unless-stopped
```

**Behavior**:
- Restarts on failure (exit code != 0)
- Exponential backoff: 10s, 20s, 40s, 60s (max)
- Does not restart if manually stopped
- Restarts on Docker daemon restart

### Managing Restarts

```bash
# Restart specific service
docker-compose restart jessy-core

# Stop without restart
docker-compose stop jessy-core

# Start stopped service
docker-compose start jessy-core
```

## Logging

### Configuration

```yaml
logging:
  driver: "json-file"
  options:
    max-size: "10m"    # Max 10MB per log file
    max-file: "3"      # Keep 3 log files
    labels: "service,environment"
```

### Viewing Logs

```bash
# Follow all logs
make logs

# Follow specific service
make logs-rust
make logs-go

# View last N lines
docker-compose logs --tail=100 jessy-core

# View logs since timestamp
docker-compose logs --since="2024-10-24T10:00:00" jessy-core

# View structured logs only
make logs-json
```

### Log Aggregation

```bash
# Test log aggregation
make logs-test

# Export logs
docker-compose logs > logs.txt

# Filter by service label
docker logs $(docker ps -qf "label=service=jessy-core")
```

## Troubleshooting

### Services Won't Start

**Problem**: `docker-compose up` fails

**Solutions**:

1. **Check Docker is running**:
   ```bash
   docker info
   ```

2. **Check port conflicts**:
   ```bash
   lsof -i :8080
   lsof -i :3000
   ```

3. **Check disk space**:
   ```bash
   df -h
   docker system df
   ```

4. **Rebuild images**:
   ```bash
   make rebuild
   ```

### Service Unhealthy

**Problem**: Service shows as unhealthy in `docker-compose ps`

**Solutions**:

1. **Check logs**:
   ```bash
   docker-compose logs jessy-core
   ```

2. **Check health endpoint**:
   ```bash
   docker exec jessy-core curl http://localhost:8080/health
   ```

3. **Increase start period**:
   ```yaml
   healthcheck:
     start_period: 30s  # Give more time to start
   ```

### Compilation Errors

**Problem**: Hot reload shows compilation errors

**Solutions**:

1. **This is normal** - Fix the code errors
2. **View full error**:
   ```bash
   docker-compose logs jessy-core | grep error
   ```

3. **Restart with clean build**:
   ```bash
   docker-compose down -v
   docker-compose up --build
   ```

### Slow Performance

**Problem**: Builds or tests are slow

**Solutions**:

1. **Check cache volumes exist**:
   ```bash
   docker volume ls | grep cache
   ```

2. **Increase Docker resources**:
   - Docker Desktop → Settings → Resources
   - Increase CPU: 4+ cores
   - Increase Memory: 4GB+

3. **Use BuildKit**:
   ```bash
   export DOCKER_BUILDKIT=1
   docker-compose build
   ```

4. **Clean old images**:
   ```bash
   docker system prune -a
   ```

### Volume Permission Issues

**Problem**: Permission denied errors in containers

**Solutions**:

1. **Check volume ownership**:
   ```bash
   docker exec jessy-core ls -la /app
   ```

2. **Fix permissions**:
   ```bash
   docker exec jessy-core chown -R $(id -u):$(id -g) /app
   ```

3. **Use user mapping** (add to docker-compose.yml):
   ```yaml
   user: "${UID}:${GID}"
   ```

### Network Issues

**Problem**: Services can't communicate

**Solutions**:

1. **Check network exists**:
   ```bash
   docker network ls | grep jessy
   ```

2. **Inspect network**:
   ```bash
   docker network inspect jessy-network
   ```

3. **Recreate network**:
   ```bash
   docker-compose down
   docker network rm jessy-network
   docker-compose up
   ```

4. **Test connectivity**:
   ```bash
   docker exec jessy-api ping jessy-core
   docker exec jessy-api curl http://jessy-core:8080/health
   ```

## Best Practices

### Development

1. **Use hot reload** - Faster iteration than rebuilding
2. **Keep containers running** - Restart is faster than recreate
3. **Use volumes for source** - Avoid copying large directories
4. **Monitor logs** - Catch issues early
5. **Clean periodically** - Remove unused images and volumes

### Testing

1. **Use test profile** - Isolate test containers
2. **Clean between runs** - Ensure test isolation
3. **Use test volumes** - Separate test data
4. **Run in CI** - Ensure consistency
5. **Check coverage** - Maintain quality

### Production

1. **Use multi-stage builds** - Smaller images
2. **Pin versions** - Reproducible builds
3. **Use health checks** - Automatic recovery
4. **Set resource limits** - Prevent resource exhaustion
5. **Use secrets** - Never commit credentials

## Advanced Topics

### Multi-Stage Builds

```dockerfile
# Stage 1: Builder
FROM rust:latest as builder
WORKDIR /app
COPY . .
RUN cargo build --release

# Stage 2: Production
FROM alpine:latest as production
COPY --from=builder /app/target/release/jessy .
CMD ["./jessy"]
```

### Build Arguments

```bash
# Pass build arguments
docker-compose build --build-arg RUST_VERSION=1.75

# In Dockerfile
ARG RUST_VERSION=1.74
FROM rust:${RUST_VERSION}
```

### Environment Overrides

```bash
# Override environment variables
RUST_LOG=debug docker-compose up

# Use different compose file
docker-compose -f docker-compose.prod.yml up
```

### Resource Limits

```yaml
services:
  jessy-core:
    deploy:
      resources:
        limits:
          cpus: '2'
          memory: 2G
        reservations:
          cpus: '1'
          memory: 1G
```

## Related Documentation

- [Hot Reload Guide](HOT_RELOAD.md)
- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)
- [CI/CD Pipeline](CI_CD.md)
- [Benchmarking Guide](BENCHMARKING.md)

---

*"Containers isolate complexity. Docker orchestrates harmony. 🐳"*
