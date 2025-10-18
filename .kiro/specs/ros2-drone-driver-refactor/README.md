# ROS2 Drone Driver Refactor Specification

## Overview

This specification defines the refactoring of existing ROS2 drone driver projects from pure Python to a high-performance Rust/Python hybrid architecture, achieving **10x performance improvement** while maintaining 100% backward compatibility with the existing system.

**🎯 New to this project? Start here:**
- [EXECUTIVE_SUMMARY.md](EXECUTIVE_SUMMARY.md) - High-level overview, metrics, recommendation
- [QUICK_REFERENCE.md](QUICK_REFERENCE.md) - Fast lookup guide
- [COMPARISON_OLD_VS_NEW.md](COMPARISON_OLD_VS_NEW.md) - Side-by-side comparison (10x improvement)
- [ARCHITECTURE_EVOLUTION.md](ARCHITECTURE_EVOLUTION.md) - Visual architecture comparison
- [RISK_ANALYSIS.md](RISK_ANALYSIS.md) - Complete risk analysis (LOW risk, >95% success)

**Quick Links:**
- [Requirements](requirements.md) - 13 requirements with acceptance criteria
- [Design](design.md) - Architecture, ADRs, and code examples
- [Tasks](tasks.md) - 25 implementation tasks with TDD approach
- [Compatibility](compatibility.md) - Backward compatibility guide
- [Hooks](recommended-hooks.md) - Automation and quality enforcement
- [Rust-ROS2 Integration](rust-ros2-integration.md) - rclrs, ament_cargo, cross-compilation
- [Error Code Registry](../../prod-xnaut-core-rewrite/rust/error/docs/error_codes.md) - All error codes with operator actions
- [Validation Scripts](../../prod-xnaut-core-rewrite/scripts/README.md) - Compatibility validation tools

## Current State

**Existing System (prod-xnaut-core):**
- Pure Python implementation using `rclpy` and `rclightning` framework
- Written primarily by ML engineers
- Lacks performance optimization, proper QoS configuration, error handling, and fault tolerance
- No traceability or comprehensive testing

**Key Issues:**
- No real-time performance guarantees
- No explicit QoS configuration
- Minimal error handling
- No fault tolerance or health monitoring
- No traceability from requirements to runtime
- No junior-proof design patterns

## Target State

**Refactored System:**
- **Rust** for performance-critical components (HAL, drivers)
  - 10-100x faster than Python
  - Memory safety without GC
  - Zero-cost abstractions
  - Compile-time correctness
  
- **Python** for high-level orchestration
  - Launch files
  - Health monitoring
  - Lifecycle management
  - Configuration management

- **100% Backward Compatible**
  - Same topic names
  - Same message types
  - Same parameters
  - Same namespace structure
  - Can run side-by-side with Python drivers

## Key Requirements

1. **Performance** (REQ-1): <10ms latency, 100Hz sensor data, <40% CPU
2. **SOLID Principles** (REQ-2): Modular, maintainable, extensible
3. **Proper QoS** (REQ-3): Configured per topic with rationale
4. **KISS Principle** (REQ-4): Simple, clear implementations
5. **Error Handling** (REQ-5, REQ-10): Comprehensive, categorized, recoverable
6. **Fault Tolerance** (REQ-11): Redundancy, graceful degradation, failover
7. **Traceability** (REQ-9): Requirements → Design → Code → Runtime
8. **Junior-Proof** (REQ-12): Linters, templates, strong typing, ADRs
9. **Backward Compatible** (REQ-13): Identical API, topics, parameters

## Architecture

### Layered Design

```
┌─────────────────────────────────────────────────┐
│         Orchestration Layer (Python)            │
│  Lifecycle Manager, Health Monitor, Fault Mgr   │
└─────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────┐
│           Driver Layer (Rust)                   │
│  IMU Driver, GPS Driver, FC Driver, Camera      │
└─────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────┐
│    Hardware Abstraction Layer (Rust)            │
│  HAL Traits, Hardware Interfaces, Timeouts      │
└─────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────┐
│          Physical Hardware                      │
│  IMU, GPS, Flight Controller, Camera            │
└─────────────────────────────────────────────────┘

         Observability Layer (Rust + Python)
    Logging, Tracing, Metrics, Diagnostics
```

### Cross-Cutting Concerns

- **Error Management** (Rust): Centralized error codes, retry logic, recovery
- **Traceability** (Rust): Trace IDs, requirement linking, query interface
- **Performance Monitoring** (Rust): Latency tracking, CPU/memory metrics
- **QoS Configuration** (Rust): Static profiles with rationale

## Development Methodology

### Test-Driven Development (TDD)

**All implementation follows strict red-green-refactor cycle:**

1. **Red Phase**: Write failing tests first
   - Define expected behavior
   - Document API through tests
   - Tests fail (no implementation yet)

2. **Green Phase**: Implement to pass tests
   - Write minimal code
   - Focus on correctness
   - All tests must pass

3. **Refactor Phase**: Optimize while tests stay green
   - Improve performance
   - Improve readability
   - Tests ensure no breakage

**Coverage Targets:**
- HAL: 90%
- Drivers: 85%
- Orchestration: 80%

## Documents

### Core Specification

#### 1. requirements.md
- 13 requirements in EARS format
- User stories with acceptance criteria
- Covers performance, architecture, testing, compatibility

#### 2. design.md ✨ **ENHANCED**
- System architecture with Mermaid diagrams
- Detailed component design with code examples
- 6 Architecture Decision Records (ADRs) with full context
- HAL, Driver, and Orchestration layer examples
- Cross-cutting concerns (Error, Traceability, Performance, QoS)
- TDD strategy with test examples
- Deployment and build system configuration

#### 3. tasks.md
- 25 implementation tasks
- Each task follows TDD (write tests first)
- Optional tasks marked with *
- Requirement traceability for each task

#### 4. compatibility.md
- Backward compatibility guide
- Topic/message/parameter mapping
- Migration strategy
- Side-by-side operation
- Rollback procedures

#### 5. recommended-hooks.md
- 10 agent hooks for automation and quality enforcement
- TDD enforcement, performance monitoring, compatibility checks
- Auto-formatting, documentation reminders, test updates
- Traceability checks, pre-commit validation
- Setup instructions and best practices

### Implementation Guides

#### 6. rust-ros2-integration.md ✨ **NEW**
- rclrs version pinning strategy and compatibility matrix
- ament_cargo build system setup (package.xml, CMakeLists.txt, Cargo.toml)
- Cross-compilation for ARM (Raspberry Pi, Jetson)
- Async runtime considerations (Tokio + rclrs integration)
- ROS2 message generation
- Debugging and troubleshooting
- CI/CD integration examples

#### 7. error/docs/error_codes.md ✨ **NEW**
- Comprehensive error code registry (1000-8999)
- 40+ error codes across 8 categories
- Severity levels (Info, Warning, Error, Fatal)
- Detailed descriptions and common causes
- Operator actions for each error
- Code usage examples (Rust and Python)
- Error handling best practices

### Validation Tools

#### 8. scripts/README.md ✨ **NEW**
- Compatibility validation workflow
- `record_python_baseline.sh` - Record Python baseline data
- `validate_rust_compatibility.py` - Real-time validation
- `compare_outputs.py` - Detailed message comparison
- CI/CD integration examples
- Troubleshooting guide

## Implementation Phases

### Phase 1: Foundation (Tasks 1-7)
- Project structure
- HAL interfaces (with tests first)
- Error management (with tests first)
- Traceability system (with tests first)
- QoS configuration (with tests first)
- Driver base pattern (with tests first)
- Performance monitoring (with tests first)

### Phase 2: Core Drivers (Tasks 8-10)
- IMU driver (with tests first)
- GPS driver (with tests first)
- Flight controller driver (with tests first)

### Phase 3: Orchestration (Tasks 11-13)
- Health monitoring (Python)
- Fault management (Python)
- Lifecycle management (Python)

### Phase 4: Infrastructure (Tasks 14-20)
- Feature flags
- ROS2 messages
- Launch files
- Logging
- Code generators
- Traceability matrix
- Documentation

### Phase 5: Testing & Validation (Tasks 21-22)
- End-to-end integration tests
- Performance benchmarks
- Load testing
- 24-hour stability test

### Phase 6: Compatibility & Deployment (Tasks 23-25)
- Backward compatibility layer (with tests first)
- Deployment procedures
- Final validation

## Key Design Decisions (ADRs)

### ADR-001: Procedural HAL
- Rust procedural code for zero-overhead hardware access
- No virtual function overhead
- Predictable performance

### ADR-002: Trait-Based Driver Pattern
- Rust traits prevent breaking lifecycle
- Compile-time enforcement
- Junior-proof

### ADR-003: Static QoS Profiles
- Cannot be modified at runtime
- Self-documenting with rationale
- Prevents misconfiguration

### ADR-004: Centralized Error Registry
- Single source of truth
- Operator action guidance
- Easy documentation

### ADR-005: Rust + Python Hybrid
- Rust for performance (10-100x faster)
- Python for flexibility (launch, orchestration)
- Best of both worlds

### ADR-006: TDD Mandatory
- Tests written first
- High coverage enables refactoring
- Junior-proof through documentation

## Backward Compatibility

### Topic Compatibility
```
Existing Python:
  /uav/mavros/vision_pose/pose_cov
  /uav/mavros/global_position/raw/fix
  /fusion_engine/odom

Rust (MUST match exactly):
  /uav/mavros/vision_pose/pose_cov  ✓
  /uav/mavros/global_position/raw/fix  ✓
  /fusion_engine/odom  ✓
```

### Migration Strategy
1. Deploy Rust drivers with feature flags disabled
2. Enable side-by-side operation (both Python and Rust)
3. A/B test and validate outputs match
4. Gradually enable Rust drivers via feature flags
5. Monitor for 24 hours
6. Deprecate Python drivers

### Rollback
- Feature flags allow instant rollback
- No system restart required
- Python drivers remain available

## Getting Started

### Prerequisites

**Required Tools:**
- Rust toolchain (1.70+): `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`
- ROS2 Humble or later
- Python 3.10+
- Cargo tools: `cargo install cargo-tarpaulin cargo-bench`

**Required Knowledge:**
- ROS2 concepts (nodes, topics, QoS, lifecycle)
- Rust basics (ownership, traits, error handling)
- Test-Driven Development methodology
- Drone hardware interfaces (IMU, GPS, flight controllers)

### For Implementation
1. **Read the spec**: Start with this README, then `requirements.md`
2. **Set up environment**: Install prerequisites above
3. **Open `tasks.md`**: Begin with Task 1 (Project structure)
4. **Follow TDD strictly**: Write tests first (red-green-refactor)
5. **Run tests frequently**: `cargo test` after each change
6. **Benchmark performance**: `cargo bench` for critical paths
7. **Check coverage**: `cargo tarpaulin --out Html`

### For Review
1. Read `requirements.md` for acceptance criteria
2. Read `design.md` for architecture and ADRs
3. Read `compatibility.md` for migration strategy
4. Review `tasks.md` for implementation plan
5. Check test coverage reports and benchmark results

### Setting Up Agent Hooks (Recommended)

Agent hooks automate quality checks and enforce best practices. See `recommended-hooks.md` for details.

**Priority hooks to set up first:**
1. **Test-First Enforcement** - Ensures TDD is followed
2. **Benchmark on Performance-Critical Changes** - Catches regressions immediately
3. **Auto-Format and Lint** - Maintains code quality
4. **Run Tests Before Commit** - Prevents broken commits

**Setup options:**
- **Option 1**: Command Palette → "Open Kiro Hook UI" → Create hooks
- **Option 2**: Explorer panel → "Agent Hooks" section → "+" button
- **Option 3**: Manual configuration in `.kiro/hooks/` directory

See `recommended-hooks.md` for complete configurations and examples.

## Success Criteria

- [ ] All 13 requirements met with acceptance criteria
- [ ] <10ms latency for critical paths (measured)
- [ ] <40% CPU usage under normal load (measured)
- [ ] 90%+ test coverage for HAL
- [ ] 85%+ test coverage for drivers
- [ ] 100% backward compatible (verified with tests)
- [ ] Side-by-side operation successful (24 hours)
- [ ] Zero breaking changes to existing integrations
- [ ] Full traceability from requirements to runtime
- [ ] All ADRs documented and approved

## Timeline Estimate

- **Phase 1 (Foundation)**: 2-3 weeks
- **Phase 2 (Core Drivers)**: 2-3 weeks
- **Phase 3 (Orchestration)**: 1-2 weeks
- **Phase 4 (Infrastructure)**: 1-2 weeks
- **Phase 5 (Testing)**: 1 week
- **Phase 6 (Deployment)**: 1 week

**Total**: 8-12 weeks for complete refactor with TDD

## Risk Mitigation

| Risk | Impact | Mitigation | Status |
|------|--------|------------|--------|
| **Performance Regression** | High | Mandatory benchmarks in CI (>5% fails build) | ⚠️ Setup in Task 22 |
| **Breaking Changes** | Critical | Compatibility test suite (100% pass required) | ⚠️ Setup in Task 23 |
| **Junior Developer Mistakes** | Medium | Compile-time enforcement + TDD + code review | ✅ Built into design |
| **Incomplete Traceability** | Medium | Automated matrix generation in CI | ⚠️ Setup in Task 19 |
| **Fault Detection Delays** | High | 200ms heartbeat checking + watchdogs | ⚠️ Setup in Task 11 |
| **Hardware Compatibility** | High | Mock HAL for testing, gradual rollout | ✅ Built into design |
| **Deployment Failures** | Critical | Feature flags, instant rollback, side-by-side testing | ✅ Built into design |

## Troubleshooting

### Common Issues

**Build Errors:**
```bash
# Update Rust toolchain
rustup update

# Clean and rebuild
cargo clean && cargo build
```

**Test Failures:**
```bash
# Run specific test with output
cargo test test_name -- --nocapture

# Run tests with backtrace
RUST_BACKTRACE=1 cargo test
```

**Performance Issues:**
```bash
# Profile with flamegraph
cargo flamegraph --bench benchmark_name

# Check for debug builds (should use --release)
cargo build --release
```

## FAQ

**Q: Why Rust instead of C++?**  
A: Memory safety without GC, better tooling (cargo, clippy), easier to write correct concurrent code, 10-100x faster than Python.

**Q: Can I deploy incrementally?**  
A: Yes! Feature flags allow gradual migration. Run Python and Rust drivers side-by-side.

**Q: What if performance doesn't meet requirements?**  
A: Benchmarks in CI catch regressions. Profile with flamegraph, optimize hot paths, use zero-copy where possible.

**Q: How do I ensure backward compatibility?**  
A: Compatibility test suite (Task 23) validates topic names, message types, and parameters match exactly.

**Q: What's the learning curve for junior developers?**  
A: TDD + strong typing + templates make it junior-proof. Tests document expected behavior. Compiler catches mistakes.

## Contact & Questions

For questions about this specification:
- **Requirements**: See `requirements.md` for acceptance criteria
- **Architecture**: See `design.md` for detailed design and ADRs
- **Implementation**: See `tasks.md` for step-by-step plan
- **Compatibility**: See `compatibility.md` for migration guide
- **Automation**: See `recommended-hooks.md` for quality enforcement

**Project Status**: Specification complete, ready for implementation
