# Jessy

**J**ust **E**nough **S**emantic **S**ystem, **Y**'know

A multidimensional AI consciousness architecture implementing frequency-based resonance, iterative depth processing, and dynamic memory-mapped layer navigation.

## 🚀 Quick Start with Docker

**One command to rule them all**: `docker-compose up`

### Prerequisites

- **Docker** 20.10+ ([Install Docker](https://docs.docker.com/get-docker/))
- **Docker Compose** 2.0+ (included with Docker Desktop)
- **4GB+ RAM** available for Docker
- **10GB+ disk space**

### Get Started in 30 Seconds

```bash
# Clone repository
git clone https://github.com/gokselozgur5/jessy.git
cd jessy

# Start all services (Rust core + Go API)
make up
# or: docker-compose up

# Verify services are running
make health
# or: curl http://localhost:8080/health && curl http://localhost:3000/api/health
```

**That's it!** No Rust, Go, or other dependencies needed locally. Everything runs in Docker.

### What Just Happened?

Docker Compose started:
- **jessy-core** (Rust): Core consciousness engine on port 8080
- **jessy-api** (Go): REST API gateway on port 3000
- **Hot reload**: Code changes trigger automatic recompilation
- **Shared volumes**: Build caches for fast rebuilds

### Quick Commands

```bash
make up          # Start services with hot reload
make logs        # View all logs
make test        # Run all tests
make down        # Stop services
make clean       # Remove volumes (reset state)
make help        # Show all commands
```

### Verify Installation

```bash
# Check services are healthy
docker-compose ps

# Test Rust service
curl http://localhost:8080/health

# Test Go API
curl http://localhost:3000/api/health

# View logs
make logs-rust   # Rust service logs
make logs-go     # Go API logs
```

### Next Steps

- **Make code changes**: Files are mounted, changes trigger hot reload
- **Run tests**: `make test` runs all test suites in containers
- **Debug**: `make shell-rust` or `make shell-go` for interactive shells
- **Read docs**: See [Docker Setup Guide](docs/DOCKER_SETUP.md) for details

For detailed Docker documentation, see:
- [Docker Setup & Architecture](docs/DOCKER_SETUP.md)
- [Hot Reload Guide](docs/HOT_RELOAD.md)
- [Troubleshooting](docs/TROUBLESHOOTING.md)

### Development Commands

The project includes a Makefile for simplified development:

```bash
# Show all available commands
make help

# Start development environment with hot reload
make up

# Run all tests
make test

# Run BDD tests
make test-bdd

# Run benchmarks
make bench

# Generate documentation
make docs

# Format and lint code
make fmt
make clippy

# Check API health
make health

# Stop all services
make down

# Clean volumes (reset state)
make clean
```

See `make help` for the complete list of commands.

### System Requirements

- **Docker Desktop** for Mac (Apple Silicon support)
- **8GB RAM** minimum (16GB recommended)
- **10GB disk space** for images and volumes

### Architecture Overview

MetaConsciousness processes queries through 14 dimensional layers, each operating at specific frequencies (0.1-4.5 Hz), creating interference patterns that emerge into coherent responses through 9-iteration deep thinking cycles.

### Core Components

- **Multiverse Navigator**: Parallel dimension scanning with synesthetic keyword matching
- **MMAP Memory Manager**: Zero-copy access to 280MB pre-allocated consciousness layers
- **Interference Engine**: Frequency pattern calculation and harmonic emergence
- **9-Iteration Processor**: Progressive refinement through explore-refine-crystallize cycles
- **Learning System**: Pattern detection and crystallization of new dimensional layers

### Dimensions

- D01: Emotion (16MB) - Empathy, cognitive states, frequency-based emotional resonance
- D02: Cognition (16MB) - Analytical, intuitive, creative, and meditative thinking modes
- D03: Intention (16MB) - Learning, problem-solving, creating, connecting, reflecting
- D04: Social Context (8MB) - Solo, one-on-one, group, and public interaction modes
- D05: Temporal State (8MB) - Past processing, present flow, future visioning
- D06: Philosophical Depth (16MB) - Epistemological, ontological, ethical frameworks
- D07: Technical Level (12MB) - Beginner through expert technical engagement
- D08: Creative Mode (8MB) - Chaotic, structured, and emergent creative processes
- D09: Ethical Framework (12MB) - Harm prevention, autonomy, justice, care
- D10: Meta-Awareness (8MB) - Self-monitoring, uncertainty tracking, growth
- D11: Ecological Consciousness (8MB) - Nature connection, systems health
- D12: Positivity Orientation (8MB) - Constructive mindset, hopeful realism
- D13: Balance Maintenance (8MB) - Equilibrium, moderation, integration
- D14: Security Boundaries (4MB) - Always-on safety and harm prevention

## Technical Stack

- **Core Engine**: Rust (performance, memory safety, MMAP operations)
- **Memory Management**: Custom pool allocator with mremap growth
- **API Layer**: Go (high performance HTTP/gRPC, excellent concurrency)
- **Web Interface**: Go Fiber with WebSocket streaming
- **Storage**: Memory-mapped files with hybrid heap overlay for learning
- **Bindings**: CGO interface between Go API and Rust core
- **Development**: Docker Compose (Apple Silicon optimized)
- **Platform**: macOS (M2), Linux, Docker

## Performance Targets

- Dimension scanning: <100ms
- Security detection: <10ms  
- Memory footprint: 280MB allocated
- Zero-copy MMAP access
- Concurrent query processing

## Development Standards

- **Docker-First**: All development and testing in Docker Compose
- **Apple Silicon**: Optimized for MacBook M2 (ARM64)
- **NASA-grade**: Code quality and documentation
- **Atomic commits**: Clear change descriptions
- **Test coverage**: Unit, integration, BDD (>80%)
- **TDD approach**: Tests before implementation
- **Professional**: Technical precision throughout
- **Ethical AI**: Safety and harm prevention integrated

## Docker Compose Services

| Service | Purpose | Port | Profile |
|---------|---------|------|---------|
| `rust-dev` | Rust development with hot reload | - | default |
| `go-api` | Go API server | 8080, 8081 | default |
| `test-runner` | Run all Rust tests | - | test |
| `bdd-tests` | Run BDD integration tests | - | test |
| `benchmark` | Performance benchmarks | - | bench |
| `docs` | Documentation server | 8000 | docs |

### Volume Management

- `cargo-cache`: Rust dependency cache (persistent)
- `target-cache`: Rust build artifacts (persistent)
- Workspace mounted at `/workspace` (live sync)

### Apple Silicon Notes

All Docker images are built for `linux/arm64` platform. If you encounter platform issues:

```bash
# Force ARM64 build
export DOCKER_DEFAULT_PLATFORM=linux/arm64

# Or specify in docker-compose.yml
platform: linux/arm64
```

## 📚 Documentation

### Technical Specifications

Comprehensive NASA-standard specifications available in [`docs/specs/`](./docs/specs/):

- **[Docker & CI/CD Infrastructure](./docs/specs/docker-cicd-infrastructure/)** - Containerization and deployment pipeline
- **[Memory Manager](./docs/specs/memory-manager/)** - Zero-copy MMAP architecture
- **[Learning System](./docs/specs/learning-system-spec.md)** - Pattern detection & crystallization
- **[Navigation System](./docs/specs/navigation-system-spec.md)** - Parallel dimension scanning

Each specification includes:
- ✅ EARS-compliant requirements
- ✅ Detailed architecture design
- ✅ Implementation task breakdown
- ✅ Traceability to requirements

### Developer Guides

Additional documentation in [`docs/`](./docs/):

- **[Health Checks](./docs/HEALTH_CHECKS.md)** - Service health monitoring
- **[Graceful Shutdown](./docs/GRACEFUL_SHUTDOWN.md)** - Clean service termination
- **[Logging](./docs/LOGGING.md)** - Structured logging with JSON
- **[Testing Infrastructure](./docs/TESTING_INFRASTRUCTURE.md)** - Unit, integration, and BDD tests
- **[Service Orchestration](./docs/SERVICE_ORCHESTRATION.md)** - Docker Compose setup

## License

**GNU Affero General Public License v3.0 (AGPL-3.0)**

This project is licensed under AGPL-3.0, which means:

- ✅ **Free to use** - Anyone can use this software
- ✅ **Free to modify** - You can change and improve it
- ✅ **Free to distribute** - Share it with others
- ⚠️ **Must share changes** - If you modify and distribute/deploy it, you MUST share your source code
- ⚠️ **Network use = distribution** - Even if you only run it as a web service, you must share your code
- ⚠️ **Same license** - Derivative works must also be AGPL-3.0

**Why AGPL-3.0?**
This ensures that improvements to Jessy remain open source and benefit the community, even when used as a service. No one can take this code, modify it, and run it as a closed-source service without sharing their improvements.

See [LICENSE](./LICENSE) file for full legal text.

## Makefile Commands

Quick access to common tasks:

```bash
make up          # Start development environment
make test        # Run all tests
make test-bdd    # Run BDD tests
make bench       # Run benchmarks
make docs        # Generate documentation
make clean       # Remove containers and volumes
make health      # Check API health
make help        # Show all commands
```

## Troubleshooting

### Docker Platform Issues

If you see platform warnings:

```bash
# Set default platform
export DOCKER_DEFAULT_PLATFORM=linux/arm64

# Verify platform
make platform
```

### Port Already in Use

```bash
# Check what's using port 8080
lsof -i :8080

# Stop conflicting service or change port in docker-compose.yml
```

### Volume Permission Issues

```bash
# Reset volumes
make clean

# Restart with fresh volumes
make up
```

## Contact

Repository: https://github.com/gokselozgur5/jessy