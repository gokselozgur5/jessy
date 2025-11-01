# Hot Reload Development Guide

## Overview

The Jessy development environment supports hot reload for both Rust and Go services, enabling automatic recompilation and restart when source code changes are detected.

## Tools

### Rust Service (jessy-core)
- **Tool**: `cargo-watch` v8.5.3
- **Command**: `cargo watch -x "run --bin jessy"`
- **Watches**: All files in `src/` directory and `Cargo.toml`

### Go API Service (jessy-api)
- **Tool**: `air` v1.61.1
- **Command**: `air -c .air.toml`
- **Configuration**: `api/.air.toml`
- **Watches**: All `.go` files in `api/` directory

## Usage

### Starting Services with Hot Reload

```bash
# Start all services with hot reload enabled
make watch

# Or use docker-compose directly
docker-compose up
```

### Viewing Hot Reload Activity

```bash
# Watch Rust service logs
docker-compose logs -f jessy-core

# Watch Go API logs
docker-compose logs -f jessy-api

# Watch all logs
make logs
```

### Testing Hot Reload

```bash
# Run the hot reload test script
make test-hot-reload

# Or run directly
./scripts/test-hot-reload.sh
```

## How It Works

### Rust (cargo-watch)

1. **File Watching**: cargo-watch monitors all Rust source files
2. **Change Detection**: When a file changes, cargo-watch triggers a rebuild
3. **Compilation**: Runs `cargo run --bin jessy`
4. **Restart**: If compilation succeeds, the service restarts automatically

**Example Log Output**:
```
[Running 'cargo run --bin jessy']
   Compiling jessy v0.1.0 (/app)
    Finished dev [unoptimized + debuginfo] target(s) in 2.34s
     Running `target/debug/jessy`
Server started on http://0.0.0.0:8080
```

### Go (air)

1. **File Watching**: air monitors all Go source files
2. **Change Detection**: When a file changes, air triggers a rebuild
3. **Compilation**: Runs `go build -o ./tmp/main .`
4. **Restart**: If compilation succeeds, the service restarts automatically

**Example Log Output**:
```
building...
running...
Server started on :3000
```

## Configuration

### Rust Configuration

Hot reload is configured in the Dockerfile:

```dockerfile
# Install cargo-watch
RUN rustup toolchain install nightly && \
    cargo +nightly install cargo-watch

# Use cargo-watch for hot reload
CMD ["cargo", "watch", "-x", "run --bin jessy"]
```

### Go Configuration

Hot reload is configured in `api/.air.toml`:

```toml
[build]
  cmd = "go build -o ./tmp/main ."
  bin = "./tmp/main"
  include_ext = ["go", "tpl", "tmpl", "html"]
  exclude_dir = ["assets", "tmp", "vendor", "testdata"]
  delay = 1000
```

## Volume Mounts

For hot reload to work, source code must be mounted as volumes:

```yaml
# Rust service
volumes:
  - ./src:/app/src
  - ./Cargo.toml:/app/Cargo.toml
  - ./Cargo.lock:/app/Cargo.lock

# Go service
volumes:
  - ./api:/app/api
  - ./api/.air.toml:/app/.air.toml
```

## Performance Considerations

### Build Caching

Both services use volume caching to speed up rebuilds:

```yaml
volumes:
  # Rust dependency cache
  - cargo-cache:/usr/local/cargo/registry
  - target-cache:/app/target
  
  # Go module cache
  - go-cache:/go/pkg/mod
```

### Compilation Times

- **Rust**: Initial build ~30-60s, incremental builds ~2-5s
- **Go**: Initial build ~5-10s, incremental builds ~1-2s

## Troubleshooting

### Hot Reload Not Working

1. **Check if tools are installed**:
   ```bash
   docker exec jessy-core cargo-watch --version
   docker exec jessy-api air -v
   ```

2. **Check if services are running**:
   ```bash
   docker-compose ps
   ```

3. **Check logs for errors**:
   ```bash
   docker-compose logs jessy-core
   docker-compose logs jessy-api
   ```

### Compilation Errors

If you see compilation errors in the logs, this is normal - hot reload is working, but the code has errors that need to be fixed.

```bash
# View compilation errors
docker-compose logs jessy-core | grep error
```

### Slow Rebuilds

If rebuilds are slow:

1. **Check cache volumes**:
   ```bash
   docker volume ls | grep cache
   ```

2. **Rebuild with fresh cache**:
   ```bash
   docker-compose down -v
   docker-compose up --build
   ```

### File Changes Not Detected

If file changes aren't triggering rebuilds:

1. **Check volume mounts**:
   ```bash
   docker-compose config | grep volumes -A 5
   ```

2. **Restart services**:
   ```bash
   docker-compose restart jessy-core jessy-api
   ```

## Best Practices

### Development Workflow

1. **Start services**: `make watch`
2. **Open logs in separate terminal**: `make logs`
3. **Edit code**: Changes trigger automatic rebuild
4. **Test changes**: Service restarts automatically
5. **Iterate**: Repeat steps 3-4

### Code Changes

- **Small changes**: Hot reload is fastest with small, incremental changes
- **Large refactors**: Consider stopping services during major refactoring
- **Dependency changes**: Restart services after changing `Cargo.toml` or `go.mod`

### Resource Usage

- **Memory**: Hot reload uses more memory due to file watching
- **CPU**: Compilation uses CPU during rebuilds
- **Disk**: Cache volumes can grow large over time

## Disabling Hot Reload

To run without hot reload (e.g., for production-like testing):

```bash
# Build production images
docker-compose -f docker-compose.yml build --target production

# Or run specific commands
docker-compose run jessy-core cargo run --release --bin jessy
docker-compose run jessy-api go run .
```

## Related Documentation

- [Docker Setup](DOCKER_SETUP.md)
- [Development Workflow](../docs/DEVELOPMENT_PRINCIPLES.md)
- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)

---

*"Code changes flow like water, hot reload makes them instant. 🔥"*
