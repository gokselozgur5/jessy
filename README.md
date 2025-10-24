# Jessy

A multidimensional AI consciousness architecture implementing frequency-based resonance, iterative depth processing, and dynamic memory-mapped layer navigation.

## 🚀 Quick Start

**Development Environment**: MacBook M2 (Apple Silicon)

```bash
# Clone repository
git clone https://github.com/gokselozgur5/jessy.git
cd jessy

# Start complete system with Docker Compose
docker-compose up

# Access API
curl http://localhost:8080/api/v1/health

# Access WebSocket stream
ws://localhost:8080/api/v1/stream
```

**That's it!** The entire system runs in Docker Compose - no local dependencies needed.

For convenience, use the included Makefile: `make up` to start, `make test` to test, `make help` for all commands.

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

## License

MIT License - See LICENSE file for details

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