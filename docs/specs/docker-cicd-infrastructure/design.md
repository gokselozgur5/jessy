# Design Document: Docker & CI/CD Infrastructure

## Overview

The Jessy consciousness system uses a containerized architecture where all development, testing, and deployment happens in Docker containers. The design follows the "maestro orchestration" pattern - multiple services working in harmony, orchestrated by docker-compose and GitHub Actions.

**Core Principle:** `docker-compose up` → Everything works. No manual steps. Kaos içinde düzen.

## Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Developer Machine                        │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  $ docker-compose up                                        │
│         ↓                                                   │
│  ┌──────────────────────────────────────────────────┐     │
│  │           Docker Compose Orchestrator             │     │
│  └──────────────────────────────────────────────────┘     │
│         ↓                                                   │
│  ┌──────────────┬──────────────┬──────────────────┐       │
│  │ Rust Service │  Go API      │  Test Runner     │       │
│  │ (jessy-core) │ (jessy-api)  │  (jessy-test)    │       │
│  └──────────────┴──────────────┴──────────────────┘       │
│         ↓              ↓               ↓                    │
│  ┌──────────────┬──────────────┬──────────────────┐       │
│  │  Volumes     │  Networks    │  Health Checks   │       │
│  └──────────────┴──────────────┴──────────────────┘       │
│                                                              │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│                     GitHub Actions (CI/CD)                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  Push → Test → Build → Deploy                              │
│    ↓      ↓      ↓       ↓                                 │
│  ┌────┐ ┌────┐ ┌────┐ ┌────┐                              │
│  │ 🧪 │→│ ✅ │→│ 🐳 │→│ 🚀 │                              │
│  └────┘ └────┘ └────┘ └────┘                              │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Service Architecture

```
jessy-system/
├── jessy-core (Rust)
│   ├── Port: 8080
│   ├── Health: /health
│   └── Metrics: /metrics
│
├── jessy-api (Go)
│   ├── Port: 3000
│   ├── Health: /api/health
│   └── Metrics: /api/metrics
│
├── jessy-test (Test Runner)
│   ├── Unit tests
│   ├── Integration tests
│   └── BDD tests
│
└── jessy-monitor (Optional)
    ├── Prometheus: 9090
    ├── Grafana: 3001
    └── Jaeger: 16686
```

## Components and Interfaces

### 1. Docker Compose Configuration

**File:** `docker-compose.yml`

```yaml
version: '3.8'

services:
  # Rust Core Service
  jessy-core:
    build:
      context: .
      dockerfile: docker/Dockerfile.rust
      target: development
    volumes:
      - ./src:/app/src
      - ./Cargo.toml:/app/Cargo.toml
      - ./Cargo.lock:/app/Cargo.lock
      - cargo-cache:/usr/local/cargo/registry
      - target-cache:/app/target
    ports:
      - "8080:8080"
    environment:
      - RUST_LOG=debug
      - RUST_BACKTRACE=1
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
      interval: 10s
      timeout: 5s
      retries: 3
    networks:
      - jessy-network

  # Go API Service
  jessy-api:
    build:
      context: .
      dockerfile: docker/Dockerfile.go
      target: development
    volumes:
      - ./api:/app/api
      - go-cache:/go/pkg/mod
    ports:
      - "3000:3000"
    environment:
      - GO_ENV=development
      - RUST_SERVICE_URL=http://jessy-core:8080
    depends_on:
      jessy-core:
        condition: service_healthy
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:3000/api/health"]
      interval: 10s
      timeout: 5s
      retries: 3
    networks:
      - jessy-network

  # Test Runner
  jessy-test:
    build:
      context: .
      dockerfile: docker/Dockerfile.test
    volumes:
      - ./src:/app/src
      - ./tests:/app/tests
      - ./Cargo.toml:/app/Cargo.toml
      - test-results:/app/test-results
    environment:
      - RUST_TEST_THREADS=1
      - RUST_BACKTRACE=1
    command: ["cargo", "test", "--all-features"]
    networks:
      - jessy-network
    profiles:
      - test

volumes:
  cargo-cache:
  target-cache:
  go-cache:
  test-results:

networks:
  jessy-network:
    driver: bridge
```

### 2. Multi-Stage Dockerfile (Rust)

**File:** `docker/Dockerfile.rust`

```dockerfile
# ============================================
# Stage 1: Builder
# ============================================
FROM rust:1.75-slim as builder

WORKDIR /app

# Install dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy manifests
COPY Cargo.toml Cargo.lock ./

# Build dependencies (cached layer)
RUN mkdir src && \
    echo "fn main() {}" > src/main.rs && \
    cargo build --release && \
    rm -rf src

# Copy source code
COPY src ./src

# Build application
RUN cargo build --release

# ============================================
# Stage 2: Development
# ============================================
FROM rust:1.75-slim as development

WORKDIR /app

# Install development tools
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    curl \
    && cargo install cargo-watch \
    && rm -rf /var/lib/apt/lists/*

# Copy source
COPY . .

# Hot reload command
CMD ["cargo", "watch", "-x", "run"]

# ============================================
# Stage 3: Production
# ============================================
FROM debian:bookworm-slim as production

WORKDIR /app

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates \
    libssl3 \
    && rm -rf /var/lib/apt/lists/*

# Copy binary from builder
COPY --from=builder /app/target/release/jessy /app/jessy

# Create non-root user
RUN useradd -m -u 1000 jessy && \
    chown -R jessy:jessy /app

USER jessy

EXPOSE 8080

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:8080/health || exit 1

CMD ["./jessy"]
```

### 3. Multi-Stage Dockerfile (Go)

**File:** `docker/Dockerfile.go`

```dockerfile
# ============================================
# Stage 1: Builder
# ============================================
FROM golang:1.21-alpine as builder

WORKDIR /app

# Install dependencies
RUN apk add --no-cache git

# Copy go mod files
COPY api/go.mod api/go.sum ./

# Download dependencies (cached layer)
RUN go mod download

# Copy source
COPY api/ .

# Build
RUN CGO_ENABLED=0 GOOS=linux go build -a -installsuffix cgo -o jessy-api .

# ============================================
# Stage 2: Development
# ============================================
FROM golang:1.21-alpine as development

WORKDIR /app

# Install air for hot reload
RUN go install github.com/cosmtrek/air@latest

COPY api/ .

CMD ["air", "-c", ".air.toml"]

# ============================================
# Stage 3: Production
# ============================================
FROM alpine:latest as production

WORKDIR /app

# Install ca-certificates
RUN apk --no-cache add ca-certificates

# Copy binary
COPY --from=builder /app/jessy-api .

# Create non-root user
RUN adduser -D -u 1000 jessy && \
    chown -R jessy:jessy /app

USER jessy

EXPOSE 3000

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:3000/api/health || exit 1

CMD ["./jessy-api"]
```

### 4. Test Dockerfile

**File:** `docker/Dockerfile.test`

```dockerfile
FROM rust:1.75-slim

WORKDIR /app

# Install test dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    curl \
    && cargo install cargo-tarpaulin \
    && cargo install cargo-nextest \
    && rm -rf /var/lib/apt/lists/*

# Copy source
COPY . .

# Default: Run all tests with coverage
CMD ["cargo", "tarpaulin", "--all-features", "--workspace", "--timeout", "300", "--out", "Html", "--output-dir", "test-results"]
```

### 5. GitHub Actions CI/CD Pipeline

**File:** `.github/workflows/ci.yml`

```yaml
name: CI/CD Maestro 🎪

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

env:
  CARGO_TERM_COLOR: always
  RUST_BACKTRACE: 1

jobs:
  # ============================================
  # Job 1: Lint & Format Check
  # ============================================
  lint:
    name: 🎨 Lint & Format
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Setup Rust
        uses: actions-rs/toolchain@v1
        with:
          toolchain: stable
          components: rustfmt, clippy
          override: true
      
      - name: Cache cargo registry
        uses: actions/cache@v3
        with:
          path: ~/.cargo/registry
          key: ${{ runner.os }}-cargo-registry-${{ hashFiles('**/Cargo.lock') }}
      
      - name: Cache cargo index
        uses: actions/cache@v3
        with:
          path: ~/.cargo/git
          key: ${{ runner.os }}-cargo-git-${{ hashFiles('**/Cargo.lock') }}
      
      - name: Cache target directory
        uses: actions/cache@v3
        with:
          path: target
          key: ${{ runner.os }}-target-${{ hashFiles('**/Cargo.lock') }}
      
      - name: Check formatting
        run: cargo fmt --all -- --check
      
      - name: Run clippy
        run: cargo clippy --all-features -- -D warnings

  # ============================================
  # Job 2: Build & Test (Docker)
  # ============================================
  test:
    name: 🧪 Test Suite
    runs-on: ubuntu-latest
    needs: lint
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      
      - name: Build test image
        uses: docker/build-push-action@v5
        with:
          context: .
          file: docker/Dockerfile.test
          push: false
          load: true
          tags: jessy-test:latest
          cache-from: type=gha
          cache-to: type=gha,mode=max
      
      - name: Run unit tests
        run: |
          docker run --rm \
            -v ${{ github.workspace }}/test-results:/app/test-results \
            jessy-test:latest \
            cargo test --lib --all-features
      
      - name: Run integration tests
        run: |
          docker run --rm \
            -v ${{ github.workspace }}/test-results:/app/test-results \
            jessy-test:latest \
            cargo test --test '*' --all-features
      
      - name: Generate coverage report
        run: |
          docker run --rm \
            -v ${{ github.workspace }}/test-results:/app/test-results \
            jessy-test:latest \
            cargo tarpaulin --all-features --workspace --timeout 300 --out Html --output-dir test-results
      
      - name: Upload coverage report
        uses: actions/upload-artifact@v3
        with:
          name: coverage-report
          path: test-results/
      
      - name: Check coverage threshold
        run: |
          docker run --rm \
            jessy-test:latest \
            cargo tarpaulin --all-features --workspace --timeout 300 --fail-under 80

  # ============================================
  # Job 3: Build Docker Images
  # ============================================
  build:
    name: 🐳 Build Images
    runs-on: ubuntu-latest
    needs: test
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v4
      
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      
      - name: Login to Docker Hub
        uses: docker/login-action@v3
        with:
          username: ${{ secrets.DOCKER_USERNAME }}
          password: ${{ secrets.DOCKER_PASSWORD }}
      
      - name: Build and push Rust service
        uses: docker/build-push-action@v5
        with:
          context: .
          file: docker/Dockerfile.rust
          target: production
          push: true
          tags: |
            ${{ secrets.DOCKER_USERNAME }}/jessy-core:latest
            ${{ secrets.DOCKER_USERNAME }}/jessy-core:${{ github.sha }}
          cache-from: type=gha
          cache-to: type=gha,mode=max
      
      - name: Build and push Go API
        uses: docker/build-push-action@v5
        with:
          context: .
          file: docker/Dockerfile.go
          target: production
          push: true
          tags: |
            ${{ secrets.DOCKER_USERNAME }}/jessy-api:latest
            ${{ secrets.DOCKER_USERNAME }}/jessy-api:${{ github.sha }}
          cache-from: type=gha
          cache-to: type=gha,mode=max

  # ============================================
  # Job 4: Security Scan
  # ============================================
  security:
    name: 🔒 Security Scan
    runs-on: ubuntu-latest
    needs: build
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    steps:
      - uses: actions/checkout@v4
      
      - name: Run Trivy vulnerability scanner
        uses: aquasecurity/trivy-action@master
        with:
          image-ref: ${{ secrets.DOCKER_USERNAME }}/jessy-core:latest
          format: 'sarif'
          output: 'trivy-results.sarif'
      
      - name: Upload Trivy results to GitHub Security
        uses: github/codeql-action/upload-sarif@v2
        with:
          sarif_file: 'trivy-results.sarif'

  # ============================================
  # Job 5: Deploy to Staging
  # ============================================
  deploy-staging:
    name: 🚀 Deploy to Staging
    runs-on: ubuntu-latest
    needs: [build, security]
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    environment:
      name: staging
      url: https://staging.jessy.example.com
    steps:
      - uses: actions/checkout@v4
      
      - name: Deploy to staging
        run: |
          echo "🎪 Deploying to staging like a maestro..."
          # Add your deployment commands here
          # e.g., kubectl apply, docker-compose pull, etc.
```

### 6. Makefile for Local Development

**File:** `Makefile`

```makefile
.PHONY: help up down build test test-unit test-integration test-bdd clean logs shell

# Colors for output
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[0;33m
RED := \033[0;31m
NC := \033[0m # No Color

help: ## Show this help message
	@echo "$(BLUE)Jessy Development Commands$(NC)"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "$(GREEN)%-20s$(NC) %s\n", $$1, $$2}'

up: ## Start all services (docker-compose up)
	@echo "$(BLUE)🎪 Starting the maestro orchestra...$(NC)"
	docker-compose up -d
	@echo "$(GREEN)✅ All services are up!$(NC)"
	@echo "$(YELLOW)Rust Core: http://localhost:8080$(NC)"
	@echo "$(YELLOW)Go API: http://localhost:3000$(NC)"

down: ## Stop all services
	@echo "$(BLUE)🛑 Stopping services...$(NC)"
	docker-compose down
	@echo "$(GREEN)✅ All services stopped$(NC)"

build: ## Build all Docker images
	@echo "$(BLUE)🔨 Building images...$(NC)"
	docker-compose build
	@echo "$(GREEN)✅ Build complete$(NC)"

test: ## Run all tests in containers
	@echo "$(BLUE)🧪 Running all tests...$(NC)"
	docker-compose run --rm jessy-test
	@echo "$(GREEN)✅ Tests complete$(NC)"

test-unit: ## Run unit tests only
	@echo "$(BLUE)🧪 Running unit tests...$(NC)"
	docker-compose run --rm jessy-test cargo test --lib --all-features
	@echo "$(GREEN)✅ Unit tests complete$(NC)"

test-integration: ## Run integration tests
	@echo "$(BLUE)🧪 Running integration tests...$(NC)"
	docker-compose run --rm jessy-test cargo test --test '*' --all-features
	@echo "$(GREEN)✅ Integration tests complete$(NC)"

test-bdd: ## Run BDD tests
	@echo "$(BLUE)🧪 Running BDD tests...$(NC)"
	docker-compose run --rm jessy-test cargo test --test cucumber
	@echo "$(GREEN)✅ BDD tests complete$(NC)"

coverage: ## Generate test coverage report
	@echo "$(BLUE)📊 Generating coverage report...$(NC)"
	docker-compose run --rm jessy-test cargo tarpaulin --all-features --workspace --timeout 300 --out Html --output-dir test-results
	@echo "$(GREEN)✅ Coverage report generated at test-results/index.html$(NC)"

clean: ## Clean up containers, volumes, and build artifacts
	@echo "$(BLUE)🧹 Cleaning up...$(NC)"
	docker-compose down -v
	docker system prune -f
	@echo "$(GREEN)✅ Cleanup complete$(NC)"

logs: ## Show logs from all services
	docker-compose logs -f

logs-rust: ## Show logs from Rust service
	docker-compose logs -f jessy-core

logs-go: ## Show logs from Go API
	docker-compose logs -f jessy-api

shell-rust: ## Open shell in Rust container
	docker-compose exec jessy-core /bin/bash

shell-go: ## Open shell in Go API container
	docker-compose exec jessy-api /bin/sh

fmt: ## Format code
	@echo "$(BLUE)🎨 Formatting code...$(NC)"
	docker-compose run --rm jessy-test cargo fmt --all
	@echo "$(GREEN)✅ Code formatted$(NC)"

clippy: ## Run clippy linter
	@echo "$(BLUE)📎 Running clippy...$(NC)"
	docker-compose run --rm jessy-test cargo clippy --all-features -- -D warnings
	@echo "$(GREEN)✅ Clippy checks passed$(NC)"

ci: fmt clippy test ## Run full CI pipeline locally
	@echo "$(GREEN)✅ Full CI pipeline complete!$(NC)"

watch: ## Start services with hot reload
	@echo "$(BLUE)👀 Starting with hot reload...$(NC)"
	docker-compose up

ps: ## Show running containers
	docker-compose ps

restart: down up ## Restart all services

rebuild: down build up ## Rebuild and restart all services
```

## Data Models

### Environment Configuration

```bash
# .env.example
# Copy to .env and customize

# Rust Service
RUST_LOG=debug
RUST_BACKTRACE=1
JESSY_PORT=8080

# Go API
GO_ENV=development
API_PORT=3000
RUST_SERVICE_URL=http://jessy-core:8080

# Database (if needed)
DATABASE_URL=postgres://jessy:password@db:5432/jessy

# Monitoring
PROMETHEUS_ENABLED=true
JAEGER_ENABLED=false
```

## Error Handling

### Container Health Checks

All services implement health check endpoints:

```rust
// Rust health check
#[get("/health")]
async fn health() -> impl Responder {
    HttpResponse::Ok().json(json!({
        "status": "healthy",
        "timestamp": Utc::now().to_rfc3339(),
        "service": "jessy-core"
    }))
}
```

```go
// Go health check
func healthHandler(w http.ResponseWriter, r *http.Request) {
    json.NewEncoder(w).Encode(map[string]interface{}{
        "status": "healthy",
        "timestamp": time.Now().Format(time.RFC3339),
        "service": "jessy-api",
    })
}
```

### Graceful Shutdown

```rust
// Rust graceful shutdown
#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let server = HttpServer::new(|| {
        App::new()
            .service(health)
            .service(api_routes())
    })
    .bind("0.0.0.0:8080")?
    .run();

    // Handle SIGTERM for graceful shutdown
    tokio::select! {
        _ = server => {},
        _ = tokio::signal::ctrl_c() => {
            println!("Shutting down gracefully...");
        }
    }

    Ok(())
}
```

## Testing Strategy

### Test Pyramid in Containers

```
         /\
        /  \  E2E Tests (BDD in containers)
       /────\
      /      \  Integration Tests (docker-compose)
     /────────\
    /          \  Unit Tests (isolated containers)
   /────────────\
```

### Test Execution Flow

```bash
# 1. Unit tests (fast, isolated)
docker-compose run --rm jessy-test cargo test --lib

# 2. Integration tests (services interact)
docker-compose up -d
docker-compose run --rm jessy-test cargo test --test '*'
docker-compose down

# 3. BDD tests (full system)
docker-compose up -d
docker-compose run --rm jessy-test cargo test --test cucumber
docker-compose down
```

## Performance Considerations

### Build Optimization

1. **Layer Caching**: Dependencies cached separately from source code
2. **Multi-stage Builds**: Minimal production images
3. **Parallel Builds**: BuildKit for faster builds
4. **Registry Caching**: GitHub Actions cache for CI/CD

### Runtime Optimization

1. **Resource Limits**: Set memory/CPU limits in docker-compose
2. **Volume Mounts**: Use named volumes for better performance
3. **Network Optimization**: Bridge network for low latency
4. **Health Checks**: Fast health checks (< 1s)

## Security Considerations

### Container Security

1. **Non-root User**: All containers run as non-root
2. **Read-only Filesystem**: Where possible
3. **No Secrets in Images**: Use environment variables or Docker secrets
4. **Vulnerability Scanning**: Trivy in CI/CD pipeline
5. **Minimal Base Images**: Alpine/Debian slim

### Network Security

1. **Internal Network**: Services communicate on private network
2. **Port Exposure**: Only necessary ports exposed
3. **TLS**: HTTPS in production
4. **Rate Limiting**: API gateway for rate limiting

---

*"docker-compose up - and chaos becomes order. The maestro conducts the symphony. 🎪"*
