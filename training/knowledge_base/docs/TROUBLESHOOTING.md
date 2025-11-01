# Troubleshooting Guide

## Overview

This guide provides solutions to common issues encountered when developing, testing, and deploying Jessy. Issues are organized by category with clear symptoms, causes, and solutions.

---

## Docker Issues

### Services Won't Start

**Symptoms**:
- `docker-compose up` fails immediately
- Containers exit with error codes
- "Port already in use" errors

**Common Causes & Solutions**:

#### Port Conflicts

```bash
# Check what's using the ports
lsof -i :8080
lsof -i :3000

# Kill the process or change Jessy's ports
# Edit docker-compose.yml to use different ports
```

#### Docker Not Running

```bash
# Check Docker status
docker info

# Start Docker Desktop (macOS)
open -a Docker

# Or restart Docker daemon
sudo systemctl restart docker  # Linux
```

#### Insufficient Resources

```bash
# Check available resources
docker system df

# Clean up unused resources
docker system prune -a

# Increase Docker resources in Docker Desktop:
# Settings → Resources → Increase CPU/Memory
```

#### Corrupted Images

```bash
# Remove and rebuild images
docker-compose down -v
docker-compose build --no-cache
docker-compose up
```

---

### Service Shows as Unhealthy

**Symptoms**:
- `docker-compose ps` shows service as "unhealthy"
- Health checks failing repeatedly
- Service restarts in a loop

**Diagnosis**:

```bash
# Check health check logs
docker inspect jessy-core --format='{{range .State.Health.Log}}{{.Output}}{{end}}'

# Check service logs
docker-compose logs jessy-core

# Test health endpoint manually
docker exec jessy-core curl -f http://localhost:8080/health
```

**Solutions**:

#### Service Taking Too Long to Start

```yaml
# In docker-compose.yml, increase start_period
healthcheck:
  start_period: 30s  # Give more time for initialization
```

#### Health Endpoint Not Implemented

```bash
# Verify the health endpoint exists
docker exec jessy-core curl -v http://localhost:8080/health

# Check if the service is actually running
docker exec jessy-core ps aux
```

#### Service Crashing on Startup

```bash
# View detailed logs
docker-compose logs --tail=100 jessy-core

# Check for missing dependencies
docker exec jessy-core ldd /app/jessy

# Run service manually to see errors
docker-compose run --rm jessy-core /bin/bash
```

---

### Hot Reload Not Working

**Symptoms**:
- Code changes don't trigger recompilation
- Service doesn't restart after changes
- Old code still running

**Diagnosis**:

```bash
# Check if cargo-watch is running
docker exec jessy-core ps aux | grep cargo-watch

# Check if air is running
docker exec jessy-api ps aux | grep air

# Verify volume mounts
docker inspect jessy-core --format='{{range .Mounts}}{{.Source}} → {{.Destination}}{{"\n"}}{{end}}'
```

**Solutions**:

#### File Watcher Not Installed

```bash
# Rebuild with development stage
docker-compose build jessy-core
docker-compose up jessy-core
```

#### Volume Not Mounted

```bash
# Check docker-compose.yml has volume mounts
# Should include:
volumes:
  - ./src:/app/src
  - ./Cargo.toml:/app/Cargo.toml
```

#### File System Events Not Propagating

```bash
# On macOS, this can happen with Docker Desktop
# Workaround: Touch a file to force detection
touch src/lib.rs

# Or restart the service
docker-compose restart jessy-core
```

#### Compilation Errors Blocking Restart

```bash
# This is normal - fix the compilation errors
# View full error output
docker-compose logs jessy-core | grep error

# The service will restart once code compiles successfully
```

---

### Slow Build Times

**Symptoms**:
- Initial build takes >5 minutes
- Incremental builds take >30 seconds
- Docker feels sluggish

**Diagnosis**:

```bash
# Check cache volumes exist
docker volume ls | grep cache

# Check Docker resource usage
docker stats

# Check disk space
docker system df -v
```

**Solutions**:

#### Missing Cache Volumes

```bash
# Ensure cache volumes are defined in docker-compose.yml
volumes:
  cargo-cache:
  target-cache:
  go-cache:

# Rebuild with cache
docker-compose build
```

#### Insufficient Docker Resources

```
Docker Desktop → Settings → Resources:
- CPU: 4+ cores
- Memory: 4GB+
- Disk: 10GB+
```

#### BuildKit Not Enabled

```bash
# Enable BuildKit for faster builds
export DOCKER_BUILDKIT=1
docker-compose build

# Or add to ~/.zshrc
echo 'export DOCKER_BUILDKIT=1' >> ~/.zshrc
```

#### Large Build Context

```bash
# Check .dockerignore exists and includes:
target/
node_modules/
.git/
*.log

# Verify build context size
docker-compose build --progress=plain 2>&1 | grep "transferring context"
```

---

### Volume Permission Issues

**Symptoms**:
- "Permission denied" errors in containers
- Can't write to mounted volumes
- Files owned by root

**Diagnosis**:

```bash
# Check file ownership in container
docker exec jessy-core ls -la /app

# Check your user ID
id -u
id -g
```

**Solutions**:

#### Files Owned by Root

```bash
# Fix ownership (temporary)
docker exec jessy-core chown -R $(id -u):$(id -g) /app

# Permanent fix: Add user mapping to docker-compose.yml
services:
  jessy-core:
    user: "${UID}:${GID}"
```

#### Volume Mount Permissions

```bash
# On Linux, ensure your user owns the source directories
sudo chown -R $USER:$USER .

# On macOS, this is usually not an issue
```

---

## Build & Compilation Issues

### Rust Compilation Errors

**Symptoms**:
- `cargo build` fails
- Type errors, borrow checker errors
- Missing dependencies

**Common Errors**:

#### Dependency Version Conflicts

```bash
# Update dependencies
cargo update

# Or clean and rebuild
cargo clean
cargo build

# In Docker
docker-compose run --rm jessy-core cargo clean
docker-compose run --rm jessy-core cargo build
```

#### Outdated Rust Toolchain

```bash
# Update Rust
rustup update stable

# In Docker, rebuild image
docker-compose build --no-cache jessy-core
```

#### Missing System Dependencies

```bash
# Check Dockerfile includes required packages
# For Rust, typically need:
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev
```

---

### Go Compilation Errors

**Symptoms**:
- `go build` fails
- Import errors
- Module not found

**Solutions**:

#### Module Cache Issues

```bash
# Clear module cache
go clean -modcache

# In Docker
docker-compose run --rm jessy-api go clean -modcache
docker-compose run --rm jessy-api go mod download
```

#### Outdated Dependencies

```bash
# Update dependencies
go get -u ./...
go mod tidy

# In Docker
docker-compose run --rm jessy-api go mod tidy
```

---

## Test Failures

### Unit Tests Failing

**Symptoms**:
- `cargo test` fails
- Specific test cases failing
- Flaky tests

**Diagnosis**:

```bash
# Run specific test with output
cargo test test_name -- --nocapture

# Run tests in single thread (for debugging)
cargo test -- --test-threads=1

# In Docker
docker-compose run --rm jessy-test cargo test -- --nocapture
```

**Solutions**:

#### Race Conditions

```bash
# Run tests serially
cargo test -- --test-threads=1

# Or fix the test to be thread-safe
# Use proper synchronization primitives
```

#### Environment-Specific Issues

```bash
# Check test assumes specific environment
# Use test fixtures and setup/teardown

# Ensure tests clean up after themselves
```

#### Timing Issues

```rust
// Increase timeouts in tests
#[tokio::test]
async fn test_with_timeout() {
    tokio::time::timeout(
        Duration::from_secs(10),  // Increase if needed
        async_operation()
    ).await.unwrap();
}
```

---

### Integration Tests Failing

**Symptoms**:
- Integration tests fail but unit tests pass
- Services can't communicate
- Timeout errors

**Diagnosis**:

```bash
# Check services are running
docker-compose ps

# Check service health
curl http://localhost:8080/health
curl http://localhost:3000/api/health

# Check logs for errors
docker-compose logs jessy-core
docker-compose logs jessy-api
```

**Solutions**:

#### Services Not Ready

```bash
# Ensure services are healthy before tests
# Use wait-for-services.sh script
./scripts/wait-for-services.sh

# Or add health check dependencies in docker-compose.yml
depends_on:
  jessy-core:
    condition: service_healthy
```

#### Network Issues

```bash
# Check services can reach each other
docker exec jessy-api curl http://jessy-core:8080/health

# Verify network exists
docker network ls | grep jessy

# Recreate network if needed
docker-compose down
docker-compose up
```

#### Port Conflicts

```bash
# Check if ports are already in use
lsof -i :8080
lsof -i :3000

# Stop conflicting services or change ports
```

---

### BDD Tests Failing

**Symptoms**:
- Cucumber tests fail
- Step definitions not found
- Feature file parsing errors

**Diagnosis**:

```bash
# Run BDD tests with verbose output
docker-compose run --rm jessy-test cargo test --test cucumber -- --nocapture

# Check feature files are valid
# Verify step definitions exist
```

**Solutions**:

#### Missing Step Definitions

```rust
// Ensure all steps are implemented
#[given("a dimension with frequency {float}")]
fn given_dimension(world: &mut World, freq: f64) {
    // Implementation
}
```

#### Feature File Syntax Errors

```gherkin
# Check Gherkin syntax is valid
# Common issues:
# - Missing colons after keywords
# - Incorrect indentation
# - Undefined parameters
```

---

## Performance Issues

### Slow Query Processing

**Symptoms**:
- Queries take >5 seconds
- High CPU usage
- Memory growing over time

**Diagnosis**:

```bash
# Profile the application
cargo flamegraph --bin jessy

# Check memory usage
docker stats jessy-core

# Run benchmarks
make bench
```

**Solutions**:

#### Algorithm Inefficiency

```bash
# Profile to find hot spots
cargo flamegraph

# Optimize critical paths
# Consider better data structures
# Add caching where appropriate
```

#### Memory Leaks

```bash
# Check for memory leaks with valgrind
docker-compose run --rm jessy-test valgrind --leak-check=full ./target/debug/jessy

# Review code for:
# - Unclosed resources
# - Circular references
# - Growing collections
```

#### Insufficient Resources

```bash
# Increase Docker memory limit
# Docker Desktop → Settings → Resources → Memory: 4GB+

# Or set resource limits in docker-compose.yml
deploy:
  resources:
    limits:
      memory: 2G
```

---

### High Memory Usage

**Symptoms**:
- Container using >1GB RAM
- Out of memory errors
- System becomes unresponsive

**Diagnosis**:

```bash
# Check memory usage
docker stats jessy-core

# Profile memory allocations
cargo build --release
valgrind --tool=massif ./target/release/jessy
```

**Solutions**:

#### Memory Leaks

```rust
// Check for:
// - Unclosed file handles
// - Growing caches without eviction
// - Circular Arc references

// Use weak references where appropriate
use std::sync::Weak;
```

#### Large Allocations

```bash
# Profile to find large allocations
cargo build --release
heaptrack ./target/release/jessy

# Consider:
# - Streaming instead of loading all at once
# - Memory-mapped files for large data
# - Pagination for large result sets
```

---

## CI/CD Issues

### GitHub Actions Failing

**Symptoms**:
- CI pipeline fails
- Tests pass locally but fail in CI
- Timeout errors

**Diagnosis**:

```bash
# Check GitHub Actions logs
# Look for differences between local and CI environment

# Run CI locally with act
act -j test
```

**Solutions**:

#### Environment Differences

```yaml
# Ensure CI environment matches local
# Check:
# - Rust version
# - Go version
# - System dependencies
# - Environment variables
```

#### Timeout Issues

```yaml
# Increase timeout in workflow
jobs:
  test:
    timeout-minutes: 30  # Increase if needed
```

#### Flaky Tests

```bash
# Identify flaky tests
# Run tests multiple times
for i in {1..10}; do cargo test || break; done

# Fix or mark as flaky
#[ignore]  // Temporarily ignore flaky test
```

---

### Docker Build Failing in CI

**Symptoms**:
- Docker build succeeds locally but fails in CI
- "No space left on device" errors
- Cache issues

**Solutions**:

#### Disk Space Issues

```yaml
# Add cleanup step before build
- name: Clean up disk space
  run: |
    docker system prune -af
    docker volume prune -f
```

#### Cache Issues

```yaml
# Use GitHub Actions cache
- uses: actions/cache@v4
  with:
    path: /tmp/.buildx-cache
    key: ${{ runner.os }}-buildx-${{ github.sha }}
```

---

## Network Issues

### Services Can't Communicate

**Symptoms**:
- "Connection refused" errors
- Services can't reach each other
- DNS resolution fails

**Diagnosis**:

```bash
# Check network exists
docker network ls | grep jessy

# Inspect network
docker network inspect jessy-network

# Test connectivity
docker exec jessy-api ping jessy-core
docker exec jessy-api curl http://jessy-core:8080/health
```

**Solutions**:

#### Network Not Created

```bash
# Recreate network
docker-compose down
docker-compose up
```

#### Wrong Network Configuration

```yaml
# Ensure all services use same network
services:
  jessy-core:
    networks:
      - jessy-network
  jessy-api:
    networks:
      - jessy-network

networks:
  jessy-network:
    driver: bridge
```

#### DNS Issues

```bash
# Use service names, not localhost
# ✅ Good
curl http://jessy-core:8080/health

# ❌ Bad (from another container)
curl http://localhost:8080/health
```

---

## Data & State Issues

### MMAP Files Corrupted

**Symptoms**:
- "Invalid data" errors
- Segmentation faults
- Inconsistent reads

**Diagnosis**:

```bash
# Check MMAP files exist
docker exec jessy-core ls -la /app/mmap-data/

# Check file permissions
docker exec jessy-core stat /app/mmap-data/dimension_1.mmap
```

**Solutions**:

#### Corrupted Files

```bash
# Remove and regenerate
docker-compose down
docker volume rm jessy_mmap-data
docker-compose up
```

#### Permission Issues

```bash
# Fix permissions
docker exec jessy-core chmod 644 /app/mmap-data/*.mmap
```

---

### State Not Persisting

**Symptoms**:
- Data lost after restart
- Volumes not persisting
- Fresh state on every start

**Diagnosis**:

```bash
# Check volumes exist
docker volume ls | grep jessy

# Inspect volume
docker volume inspect jessy_mmap-data
```

**Solutions**:

#### Volumes Not Defined

```yaml
# Ensure volumes are defined in docker-compose.yml
volumes:
  mmap-data:

services:
  jessy-core:
    volumes:
      - mmap-data:/app/mmap-data
```

#### Using Anonymous Volumes

```bash
# Don't use anonymous volumes
# ❌ Bad
volumes:
  - /app/mmap-data

# ✅ Good
volumes:
  - mmap-data:/app/mmap-data
```

---

## Debugging Techniques

### General Debugging Workflow

1. **Reproduce the issue**
   ```bash
   # Document exact steps to reproduce
   # Note environment details
   ```

2. **Check logs**
   ```bash
   docker-compose logs jessy-core
   docker-compose logs jessy-api
   ```

3. **Isolate the problem**
   ```bash
   # Test components individually
   # Disable features one by one
   ```

4. **Use debugging tools**
   ```bash
   # Rust debugger
   rust-gdb target/debug/jessy
   
   # Go debugger
   dlv debug
   ```

5. **Add instrumentation**
   ```rust
   // Add debug logging
   tracing::debug!("Processing query: {:?}", query);
   
   // Add assertions
   debug_assert!(size > 0, "Size must be positive");
   ```

### Interactive Debugging

```bash
# Open shell in container
docker exec -it jessy-core /bin/bash

# Run commands manually
cargo build
cargo test
./target/debug/jessy

# Check environment
env | grep RUST
ps aux
netstat -tlnp
```

### Log Analysis

```bash
# Filter logs by level
docker-compose logs jessy-core | grep ERROR

# Follow logs in real-time
docker-compose logs -f jessy-core

# Export logs for analysis
docker-compose logs > logs.txt

# Search for patterns
grep -A 5 "error" logs.txt
```

---

## Getting Help

### Before Asking for Help

1. **Search existing issues**: Check GitHub issues for similar problems
2. **Check documentation**: Review relevant docs
3. **Gather information**:
   ```bash
   # System info
   docker version
   docker-compose version
   uname -a
   
   # Service status
   docker-compose ps
   
   # Recent logs
   docker-compose logs --tail=100
   ```

### Creating a Bug Report

Include:
- **Description**: What's wrong?
- **Expected behavior**: What should happen?
- **Actual behavior**: What actually happens?
- **Steps to reproduce**: Exact steps
- **Environment**: OS, Docker version, etc.
- **Logs**: Relevant log output
- **Configuration**: docker-compose.yml, .env

### Useful Commands for Diagnostics

```bash
# Full system status
make status

# Generate diagnostic report
docker-compose ps > status.txt
docker-compose logs >> status.txt
docker system df >> status.txt

# Check all health endpoints
curl http://localhost:8080/health
curl http://localhost:3000/api/health
```

---

## Related Documentation

- [Docker Setup](DOCKER_SETUP.md)
- [CI/CD Pipeline](CI_CD.md)
- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)
- [Hot Reload Guide](HOT_RELOAD.md)

---

*"Every bug is a teacher. Every error is a lesson. Debug with patience, learn with purpose. 🔍"*
