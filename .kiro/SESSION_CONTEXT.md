# 🧠 Session Context - Critical Information for All Sessions

> **Purpose**: This file contains essential information that MUST be known and remembered across ALL development sessions.

---

## 🚨 CRITICAL RULES (NEVER FORGET!)

### 1. 🐳 Docker-First Development
**ALWAYS USE DOCKER FOR EVERYTHING!**

```bash
# Development
make dev

# Testing
make test

# NEVER run tests locally without Docker!
```

**Why?**
- Consistent environment
- Same as CI/CD
- No "works on my machine" issues

### 2. 🧪 Test-Driven Development (TDD)
**ALWAYS WRITE TESTS FIRST!**

**TDD Cycle:**
1. 🔴 RED: Write failing test
2. 🟢 GREEN: Write minimal code to pass
3. 🔵 REFACTOR: Improve code
4. ✅ VERIFY: Run all tests in Docker

**Never:**
- ❌ Write implementation before tests
- ❌ Skip running full test suite
- ❌ Commit failing tests

### 3. 📝 Documentation
**DON'T CREATE NEW MD FILES UNNECESSARILY!**

**Before creating new docs:**
- Check if existing doc can be updated
- Use DOCUMENTATION_MAP.md to find existing docs
- Only create for major features or ADRs

**Update instead:**
- DOCUMENTATION_MAP.md - Master index
- Existing guides in docs/
- Task status in .kiro/specs/*/tasks.md

---

## 📂 Project Structure

### Key Locations

```
prod-xnaut-core-rewrite/
├── .kiro/
│   ├── SESSION_CONTEXT.md          # This file ⭐
│   ├── DEVELOPMENT_GUIDELINES.md   # Docker + TDD workflow
│   └── specs/                      # Feature specifications
│       ├── algorithms-refactor/    # GPS odometry (CURRENT)
│       ├── failsafes-refactor/     # Failsafes (PLANNED)
│       └── utils-refactor/         # Utils (PLANNED)
│
├── DOCUMENTATION_MAP.md            # Master doc index
├── DOCKER_TESTING.md               # Docker testing guide
├── Makefile                        # Quick commands
├── README.md                       # Project overview
│
├── rust/                           # Rust components
│   ├── algorithms/                # Algorithms workspace
│   │   └── gps_odometry/          # GPS odometry (CURRENT WORK)
│   ├── drivers/                   # Drivers (COMPLETE)
│   ├── hal/                       # HAL (COMPLETE)
│   └── error/                     # Error management (COMPLETE)
│
├── python/                         # Python orchestration
│   ├── lifecycle_manager/         # Lifecycle (COMPLETE)
│   ├── fault_manager/             # Fault management (COMPLETE)
│   └── health_monitor/            # Health monitoring (COMPLETE)
│
└── docs/
    ├── adr/                       # Architecture decisions
    └── *.md                       # Various guides
```

---

## 🎯 Current Status

### Active Work: GPS Odometry Refactor

**Location**: `rust/algorithms/gps_odometry/`

**Spec Files**:
- Requirements: `.kiro/specs/algorithms-refactor/requirements.md`
- Design: `.kiro/specs/algorithms-refactor/design.md`
- Tasks: `.kiro/specs/algorithms-refactor/tasks.md`

**Progress**:
- ✅ Task 1: Project setup
- ✅ Task 2: Core types & config (32 tests passing)
- ✅ Task 3: Quaternion transformation (51 tests passing)
- ✅ Task 4: Validation module (78 tests passing)
- ✅ Task 5: Noise generator module (78 tests passing)
- ✅ Task 6: Message synchronization (88 tests passing)
- 🔄 Task 7: ROS2 node implementation (NEXT)
- ⏳ Task 8-13: Remaining tasks

**Test Status**:
- GPS Odometry: 88 tests ✅ (+10 message synchronization tests)
- Rust (all): 137 tests ✅
- Python: 21 tests ✅
- **Total**: 190 tests passing

### Completed Work

- ✅ Drivers (GPS, IMU, FC)
- ✅ HAL layer
- ✅ Error management
- ✅ Lifecycle manager
- ✅ Fault manager
- ✅ Health monitor
- ✅ Docker environment
- ✅ TDD workflow established

---

## 🏗️ Architecture Principles

### Hybrid Architecture
- **Rust**: Performance-critical paths (drivers, algorithms)
- **Python**: Orchestration (lifecycle, fault management)

### Key ADRs (Architecture Decision Records)

**Must Know**:
1. **ADR-001**: Rust for performance-critical paths
2. **ADR-002**: Python for orchestration
3. **ADR-006**: TDD mandatory
4. **ADR-010**: Nalgebra for quaternion operations
5. **ADR-011**: Approximate time message synchronization
6. **ADR-012**: Zero-allocation runtime design

**Location**: `docs/adr/`

### Design Patterns
- Trait-based HAL
- Centralized error management
- Non-blocking async orchestration
- QoS-aware communication
- Zero-copy where possible

---

## 🔧 Development Workflow

### Starting a New Task

```bash
# 1. Read task from spec
cat .kiro/specs/algorithms-refactor/tasks.md

# 2. Start Docker
make dev

# 3. Inside Docker: Write tests first (TDD)
cd /workspace/src/prod-xnaut-core-rewrite/rust/gps_odometry
# Write test
cargo test test_name  # Should fail (RED)

# 4. Implement feature
# Write minimal code
cargo test test_name  # Should pass (GREEN)

# 5. Refactor if needed
cargo test --all      # All tests pass

# 6. Exit and verify in Docker
exit
make test             # All tests pass

# 7. Update task status
# Mark task complete in tasks.md

# 8. Commit
git add .
git commit -m "feat: implement feature X"
```

### 🎓 Learning & Observation Mode

**User Preference**: Watch and learn from command execution
- **DO NOT** shorten or abbreviate commands in responses
- **SHOW FULL** commands with complete output when relevant
- **EXPLAIN** what commands do and why they're being used
- **DEMONSTRATE** the full workflow step-by-step
- User wants to understand the process, not just see results

### Task Execution Rules

**MUST DO**:
- ✅ Read requirements & design docs first
- ✅ Write tests before implementation (TDD)
- ✅ Run tests in Docker (`make test`)
- ✅ Update task status in tasks.md
- ✅ Verify all tests pass before committing
- ✅ **After each task: Review with critical eye**
  - Check requirements coverage
  - Verify ADR compliance
  - Confirm test quality
  - Identify deferred validations (note for later tasks)
- ✅ **Show full commands and outputs** - User wants to learn and observe

**MUST NOT DO**:
- ❌ Skip writing tests
- ❌ Run tests only locally
- ❌ Create unnecessary MD files
- ❌ Commit failing tests
- ❌ Work on multiple tasks simultaneously
- ❌ Test features before dependencies are ready (e.g., Python compat before ROS2 integration)
- ❌ Skip incremental validation (test each layer as you build it)
- ❌ Shorten or abbreviate commands in explanations

---

## 📊 Quick Commands

### Docker Commands
```bash
make dev              # Interactive development
make test             # Run all tests
make test-python      # Python tests only
make test-rust        # Rust tests only
make test-gps-odometry # GPS odometry tests only
make build            # Build Docker image
make clean            # Clean Docker resources
```

### Inside Docker
```bash
# Rust tests
cargo test --lib
cargo test --workspace --lib
cargo build --release

# GPS Odometry specific tests
cargo test --package gps_odometry --lib

# Python tests
pytest tests/ -v

# Check compilation
cargo check --workspace
```

---

## 📚 Essential Documentation

### Must Read (Priority Order)

1. **[SESSION_CONTEXT.md](.kiro/SESSION_CONTEXT.md)** - This file!
2. **[DEVELOPMENT_GUIDELINES.md](.kiro/DEVELOPMENT_GUIDELINES.md)** - Docker + TDD
3. **[DOCUMENTATION_MAP.md](../prod-xnaut-core-rewrite/DOCUMENTATION_MAP.md)** - Find any doc
4. **[Current Task List](.kiro/specs/algorithms-refactor/tasks.md)** - What to do

### Reference When Needed

- **Design**: `.kiro/specs/algorithms-refactor/design.md`
- **Requirements**: `.kiro/specs/algorithms-refactor/requirements.md`
- **ADRs**: `docs/adr/README.md`
- **Docker Testing**: `DOCKER_TESTING.md`

---

## 🎓 Key Concepts

### GPS Odometry Algorithm

**Purpose**: Transform GPS velocity from ENU frame to body frame

**Key Components**:
- Types & Config (✅ Done)
- Quaternion transformation (🔄 Next)
- Noise generation
- Validation
- ROS2 node

**Math Library**: nalgebra (ADR-010)
**Sync Strategy**: Approximate time (ADR-011)
**Memory**: Zero-allocation runtime (ADR-012)

### Testing Philosophy

**TDD Cycle**:
1. Red → Green → Refactor
2. Tests first, always
3. Minimal implementation
4. Refactor when green
5. All tests in Docker

**Test Types**:
- Unit tests (in src/ files)
- Integration tests (in tests/ dir)
- Benchmarks (in benches/ dir)

---

## 🚫 Common Mistakes to Avoid

### Documentation
- ❌ Creating new MD files for every small thing
- ❌ Duplicating information across files
- ✅ Update existing docs instead
- ✅ Use DOCUMENTATION_MAP.md

### Development
- ❌ Running tests locally without Docker
- ❌ Writing code before tests
- ❌ Skipping full test suite
- ✅ Always use Docker
- ✅ Always TDD
- ✅ Always run all tests

### Task Management
- ❌ Working on multiple tasks at once
- ❌ Not updating task status
- ❌ Committing incomplete work
- ✅ One task at a time
- ✅ Update tasks.md
- ✅ Commit only when complete

---

## 🔄 Session Handoff Checklist

### When Starting a New Session

- [ ] Read this SESSION_CONTEXT.md
- [ ] Check current task in `.kiro/specs/algorithms-refactor/tasks.md`
- [ ] Review last commit to understand what was done
- [ ] Verify Docker environment: `make test`
- [ ] Read relevant design/requirements docs

### When Ending a Session

- [ ] Commit all completed work
- [ ] Update task status in tasks.md
- [ ] Ensure all tests pass: `make test`
- [ ] Update DOCUMENTATION_MAP.md if needed
- [ ] Leave clear commit message for next session

---

## 📞 Getting Help

### Can't Find Something?

1. Check **DOCUMENTATION_MAP.md**
2. Check **docs/INDEX.md**
3. Search in `.kiro/specs/`
4. Check **docs/adr/** for decisions

### Confused About Workflow?

1. Read **DEVELOPMENT_GUIDELINES.md**
2. Read **DOCKER_TESTING.md**
3. Check **Makefile** for commands

### Need Architecture Context?

1. Read relevant **ADR** in `docs/adr/`
2. Check **design.md** in spec folder
3. Review **requirements.md** in spec folder

---

## 🎯 Success Criteria

### A Task is Complete When:

1. ✅ Tests written first (TDD)
2. ✅ All tests pass in Docker (`make test`)
3. ✅ Code follows ADR guidelines
4. ✅ Task marked complete in tasks.md
5. ✅ Committed with clear message
6. ✅ Documentation updated (if needed)

### A Session is Successful When:

1. ✅ At least one task completed
2. ✅ All tests passing
3. ✅ Work committed
4. ✅ Next steps clear
5. ✅ No broken state left behind

---

## 📝 Quick Reference

### File Locations (Memorize These!)

```
.kiro/SESSION_CONTEXT.md              # This file
.kiro/DEVELOPMENT_GUIDELINES.md       # Workflow
.kiro/specs/algorithms-refactor/      # Current work
prod-xnaut-core-rewrite/DOCUMENTATION_MAP.md  # Find docs
prod-xnaut-core-rewrite/Makefile      # Commands
prod-xnaut-core-rewrite/rust/algorithms/gps_odometry/    # Current code
```

### Commands (Memorize These!)

```bash
make dev              # Start development
make test             # Run all tests
make help             # See all commands
```

### Workflow (Memorize This!)

```
1. Read task
2. make dev
3. Write test (RED)
4. Write code (GREEN)
5. Refactor (REFACTOR)
6. make test (VERIFY)
7. Commit (DONE)
```

---

## 🔥 REMEMBER THESE ALWAYS!

1. **🐳 DOCKER FIRST** - Always use Docker
2. **🧪 TDD ALWAYS** - Tests before code
3. **📝 NO NEW MDS** - Update existing docs
4. **✅ ONE TASK** - Focus on one task at a time
5. **🔍 CHECK DOCS** - Use DOCUMENTATION_MAP.md

---

**Last Updated**: 2025-10-18

**Current Focus**: GPS Odometry Refactor (Task 6 COMPLETE ✅, Task 7 NEXT)

**Next Session Should**: Start Task 7 - ROS2 node implementation (subscribers, publishers, callbacks)

**Session 6 Summary - Message Synchronization COMPLETE**:
- ✅ Task 6: Message synchronization implemented (88 tests passing)
- ✅ Approximate time synchronization with 10ms tolerance
- ✅ Generic MessageSynchronizer<T1, T2> with HasTimestamp trait
- ✅ Queue-based approach with automatic trimming (max 10 messages)
- ✅ Best-match selection algorithm for closest timestamps
- ✅ Handles different message rates (GPS 10Hz, IMU 100Hz)
- ✅ 10 comprehensive synchronizer tests added
- ✅ All tests passing: 88/88 ✅
- ✅ Committed and pushed to origin/feature/task-23-backward-compatibility

**Task 6 Technical Details**:
- MessageSynchronizer<T1, T2> with generic types
- HasTimestamp trait for message abstraction
- Configurable max_queue_size (default: 10) and time_tolerance (default: 10ms)
- add_message1() and add_message2() return synchronized pairs
- find_synchronized_pair() selects best match within tolerance
- Automatic queue trimming prevents memory growth
- TDD workflow followed (RED → GREEN → REFACTOR)
- All edge cases tested (exact match, approximate, outside tolerance, overflow)
- ADR-011 compliance: Approximate Time Message Synchronization

**Task 7 Preview - What's Next**:
- Implement full ROS2 node with subscribers and publishers
- Wire up MessageSynchronizer with actual ROS2 message types
- Create GPS velocity callback (TwistStamped subscriber)
- Create IMU callback (Imu subscriber)
- Implement synchronized processing callback
- Create odometry publisher (Odometry message)
- Add diagnostic status publishing
- Integrate with error manager and performance monitor
- Note: Full ROS2 integration requires ROS2 dependencies (may need Docker or mock types)

---

**🎯 When in doubt, read this file first!**
