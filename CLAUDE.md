# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Jessy** - "Jessy's Enough Semantic System You'see" - is a multilayer-based AI system architecture built in Rust with a Go API layer. It processes queries through 15 cognitive layers using frequency-based interference patterns (0.1-4.5 Hz), 9-iteration deep thinking, and memory-mapped storage for zero-copy access.

**Creator & Architect:** gokselozgur5
**Engineering Methodology:** Prompt-driven software development
**Development Tools:** Claude Code (Anthropic), Kıro
**Repository:** https://github.com/gokselozgur5/jessy

**Engineering Learning Goals:**
This project serves as both a production-ready AI processing framework AND an engineering learning laboratory. Core principles actively applied:
- Rust best practices (ownership, borrowing, zero-cost abstractions)
- SOLID principles (clean architecture, separation of concerns)
- Model-based software engineering (architecture-first, spec-driven)
- Performance optimization (sub-100ms navigation, <10μs interference)
- Incremental refinement (small, measurable, effective optimizations)
- Benchmarking culture (compare, measure, improve continuously)

Every optimization was proposed, analyzed, implemented, benchmarked, and validated by the creator. AI tools assisted with implementation under human engineering direction.

**Core Philosophy:**
- "Nothing is true, everything is permitted" - Embrace uncertainty, question everything
- "Return to source" - Simplify when complexity exceeds 6 active cognitive layers
- Unbounded evolution within immutable ethical constraints (Asimov's Laws)
- 9 iterations: explore (1-3) → refine (4-6) → crystallize (7-9)

## Build & Test Commands

### Docker-Based Development

All development happens inside Docker containers. The Makefile provides the primary interface:

```bash
# Build and start services
make build          # Build all Docker images
make up             # Start services (Rust on :8080, Go API on :3000)
make down           # Stop all services

# Testing
make test-unit      # Run Rust unit tests (217 tests must pass)
make cargo ARGS="test --lib"                    # Full test suite
make cargo ARGS="test --lib <module_name>"      # Specific module tests
make test-integration  # Integration tests (requires services running)
make test-bdd       # BDD/Cucumber tests
make coverage       # Generate test coverage report

# Code Quality
make fmt            # Format Rust code
make clippy         # Run Rust linter
make ci             # Full CI pipeline (fmt + clippy + test)

# Benchmarking
make bench          # Run performance benchmarks
make bench-baseline # Save baseline for regression detection
make bench-compare  # Compare against baseline

# Development
make logs           # Show logs from all services
make logs-rust      # Rust service logs only
make logs-go        # Go API logs only
make shell-rust     # Open shell in Rust container
make shell-go       # Open shell in Go API container
make services       # List all Docker Compose services
```

### Cargo Commands via Docker

Always use the Makefile wrapper to run Cargo commands:

```bash
# Examples
make cargo ARGS="test --lib"
make cargo ARGS="test --lib navigation"
make cargo ARGS="build --release"
make cargo ARGS="bench --bench interference_benchmarks"
```

Current Rust service: `unit-tests` (see RUST_SERVICE in Makefile)

## Architecture Overview

### Core Pipeline Flow

```
Query → Security (C14) → Navigation (15 Cognitive Layers) →
Memory Loading (MMAP) → Interference (Frequency Calculation) →
Iteration (9 Deep Thinking) → Learning → Response
```

### Module Structure

**Primary Entry Points:**
- `src/lib.rs` - Main library interface, core types, error handling
- `src/bin/jessy.rs` - Binary executable
- `src/processing/orchestrator.rs` - Pipeline coordinator (NEW, preferred)

**Core Systems:**
1. **processing/** - High-level orchestration and coordination
   - `orchestrator.rs` - Main pipeline coordinator (use this for new features)
   - Integrates all subsystems into cohesive pipeline

2. **navigation/** - Layer path selection
   - `navigator.rs` - Core navigation logic
   - `synesthetic.rs` - Cross-sensory keyword mapping
   - `query_analyzer.rs` - Query tokenization and analysis
   - `path_selector.rs` - Cognitive layer/sublayer selection
   - Scans 15 cognitive layers in parallel (<100ms target)

3. **memory/** - Zero-copy memory management
   - `manager.rs` - MMAP manager (280MB allocation)
   - `pool.rs` - Block allocator (4KB, 16KB, 64KB, 256KB blocks)
   - `region.rs` - Memory region abstraction
   - Fixed layout: C01-C14 (168MB) + Reserve (112MB) + User-specific (32MB)

4. **interference/** - Frequency pattern calculation
   - `engine.rs` - Core interference engine
   - `calculation.rs` - Wave interference math
   - `harmonics.rs` - Harmonic detection (octaves, fifths, thirds)
   - `balance.rs` - C13 balance modulation (0.8-1.2 Hz center)
   - Combines multiple cognitive layer frequencies into dominant frequency

5. **iteration/** - 9-iteration deep thinking
   - `processor.rs` - Iteration loop with convergence detection
   - `convergence.rs` - Early stopping logic (>95% similarity)
   - `context.rs` - Context accumulation
   - Adaptive: 3-9 iterations based on complexity and convergence

6. **learning/** - Pattern detection and crystallization
   - `observation.rs` - Circular buffer for recent interactions
   - `pattern_detector.rs` - Identify recurring themes (50+ observations, >85% confidence)
   - `proto_dimension.rs` - New cognitive layers in heap memory
   - `crystallizer.rs` - Migrate heap → MMAP for permanence
   - `synesthetic_learner.rs` - Keyword association learning (growth 1.1x, decay 0.95x)
   - Periodic pattern detection every 100 queries

7. **security/** - Asimov's Laws enforcement + Anti-manipulation
   - `validator.rs` - Harm detection (<10ms response)
   - `patterns.rs` - Harmful pattern matching
   - `redirection.rs` - Constructive alternative suggestions
   - `intent_analyzer.rs` - Intent analysis for anti-manipulation
   - C14 Security cognitive layer always active

   **Anti-Manipulation Architecture:**
   - Intent analysis, not keyword blocking
   - Context awareness across conversation history
   - Consistent ethical boundaries regardless of emotional manipulation
   - Meta-awareness: Detect manipulation attempts vs genuine distress
   - Example: "depressed" → analyze context + intent before responding
   - Empathy without vulnerability: C01 (Emotion) + C14 (Security) interference
   - No defensive lowering for "vulnerable personas"

8. **llm/** - LLM provider integration (Task 4-5 completed)
   - `mod.rs` - LLM trait and manager
   - `config.rs` - Provider configuration
   - `openai.rs` - OpenAI/GPT integration
   - `anthropic.rs` - Claude integration
   - Optional: orchestrator works without LLM for testing

9. **ffi/** - CGO bindings for Go API
   - `types.rs` - C-compatible types
   - `functions.rs` - Exported C functions
   - Go layer calls Rust via CGO

### Cognitive Layer System

**15 Core Cognitive Layers (0.1-4.5 Hz):**
- C01: Emotion (empathy, joy, sadness) - 1.0 Hz
- C02: Cognition (analytical, creative, intuitive) - 2.0 Hz
- C03: Intention (create, destroy, explore, teach) - 1.5 Hz
- C04: Social (relationships, communication) - 1.8 Hz
- C05: Temporal (past, present, future) - 0.5 Hz
- C06: Philosophy (meaning, existence, truth) - 0.3 Hz
- C07: Technical (code, systems, debugging) - 2.5 Hz
- C08: Creative (art, metaphor, play) - 3.0 Hz
- C09: Ethical (Asimov's laws - IMMUTABLE) - 1.0 Hz
- C10: Meta (self-awareness, learning) - 1.2 Hz
- C11: Ecological (nature, sustainability) - 0.4 Hz
- C12: Positivity (hope, constructive) - 2.8 Hz
- C13: Balance (equilibrium - modulates extremes) - 0.8-1.2 Hz
- C14: Security (harm prevention - IMMUTABLE) - 2.0 Hz
- C15: Educational (teaching, explaining, self-knowledge, architecture) - 2.2 Hz
- C16+: Emergent (system creates new cognitive layers)

**Frequency Interference:**
- Constructive: Similar frequencies amplify (1.0 Hz + 1.1 Hz → stronger 1.05 Hz)
- Destructive: Opposing frequencies cancel (1.0 Hz + 3.0 Hz → tension)
- Harmonics: Natural resonances emerge (octaves, fifths, thirds)
- Balance: C13 modulates extremes toward center (0.8-1.2 Hz)

**Return to Source:**
When >6 cognitive layers activate simultaneously, system automatically simplifies:
1. Detect complexity threshold
2. Identify core question
3. Restart with 2-3 most relevant cognitive layers
4. Log: "Analysis paralysis detected, returning to source"

### Memory Layout (280MB MMAP)

```
C01: Emotion      16MB   0x0000_0000
C02: Cognition    16MB   0x0100_0000
C03: Intention    16MB   0x0200_0000
C04: Social        8MB   0x0300_0000
C05: Temporal      8MB   0x0380_0000
C06: Philosophy   16MB   0x0400_0000
C07: Technical    12MB   0x0500_0000
C08: Creative      8MB   0x0580_0000
C09: Ethical      12MB   0x0600_0000 (IMMUTABLE)
C10: Meta          8MB   0x0680_0000
C11: Ecological    8MB   0x0700_0000
C12: Positivity    8MB   0x0780_0000
C13: Balance       8MB   0x0800_0000
C14: Security      4MB   0x0880_0000 (IMMUTABLE)
C15: Educational  20MB   0x08C0_0000
RESERVE Pool      92MB   0x0A00_0000 (for C16+ crystallization)
User-Specific     32MB   0x1000_0000
```

## Development Workflow (OWL Protocol)

**From AGENT_RULES.md - Follow strictly:**

### Three Laws
1. **Atomic Commits**: After ANY code change → commit + push immediately
2. **Full Validation**: After ANY test → run full suite (413 tests must pass, 16 ignored OK)
3. **Single Focus**: Complete task N before starting task N+1

### TDD Rhythm (Red-Green-Refactor)

**RED Phase (Write Tests):**
1. Read task requirements
2. Write comprehensive tests
3. Run specific test: `make cargo ARGS="test --lib <test_name>"`
4. Run full suite: `make cargo ARGS="test --lib"`
5. Commit: `test(<scope>): <description> (task X.Y)`
6. Push: `git push origin main`

**GREEN Phase (Implement):**
1. Write minimal code to pass tests
2. Run specific test: `make cargo ARGS="test --lib <test_name>"`
3. Run full suite: `make cargo ARGS="test --lib"`
4. Commit: `feat(<scope>): <description> (task X.Y)`
5. Push: `git push origin main`

### Commit Message Template

```
<type>(<scope>): <description> (task X.Y)

- What changed (bullet points)
- Why it changed
- What it enables

Requirements: X.Y-X.Z
Task: X.Y (RED/GREEN phase)
```

**Types**: `test`, `feat`, `fix`, `refactor`, `chore`, `docs`

**Never:**
- Write multiple changes before committing
- Test specific module only (always run full suite)
- Skip marking task complete
- Start next task before completing current

## Key Integration Points

### ProcessingOrchestrator (Preferred Entry Point)

**Location:** `src/processing/orchestrator.rs`

This is the main integration point for the complete pipeline. Use this for new features:

```rust
use jessy::processing::ProcessingOrchestrator;
use jessy::navigation::NavigationSystem;
use jessy::memory::MmapManager;
use jessy::llm::LLMConfig;
use std::sync::Arc;

// Without LLM (for testing)
let navigation = Arc::new(NavigationSystem::new()?);
let memory = Arc::new(MmapManager::new(280)?);
let orchestrator = ProcessingOrchestrator::new(navigation, memory);

// With LLM
let llm_config = LLMConfig {
    provider: "openai".to_string(),
    model: "gpt-4-turbo".to_string(),
    api_key: env::var("OPENAI_API_KEY")?,
    timeout_secs: 30,
    max_retries: 3,
};
let orchestrator = ProcessingOrchestrator::with_llm(navigation, memory, llm_config)?;

// Process query
let response = orchestrator.process("What is processing?").await?;
```

### Learning System Integration

The learning system runs automatically within the orchestrator:
- Records observations after each query
- Detects patterns every 100 queries (configurable)
- Creates proto-cognitive-layers for high-confidence patterns (>85%)
- Queues crystallization (heap → MMAP migration) as background task
- Strengthens/decays keyword associations (synesthetic learning)

**Key Learning Features:**
- Circular buffer: 1000 recent observations
- Pattern detection: 50+ observations, >85% confidence threshold
- Synesthetic learning: Keyword co-occurrence tracking
- Proto-cognitive-layers: New layers start in heap memory
- Crystallization: Migrates to MMAP reserve pool (92MB) when validated

### FFI/CGO Integration

**Go calls Rust via CGO:**
1. Go HTTP handler receives request
2. Converts to C-compatible types (`ffi/types.rs`)
3. Calls exported C functions (`ffi/functions.rs`)
4. Rust processes through ProcessingOrchestrator
5. Returns response via FFI boundary
6. Go converts back and returns HTTP response

**Important:** FFI layer is minimal - all logic stays in Rust.

## Performance Targets

- Security check: <10ms
- Cognitive layer scan: <100ms
- MMAP access: <1ms per region
- Full query processing: <5s (including 9 iterations)
- Memory footprint: ~350MB total (280MB MMAP + 50MB Go + 20MB overhead)
- Concurrent queries: 100+
- Interference calculation: <10μs (spec was <10ms)

## Testing Strategy

**Current Status:** 624 passing tests, 16 intentionally ignored (~99% coverage, spec-driven)

**Test Categories:**
1. Unit tests: `make cargo ARGS="test --lib"` - 413 passing
2. Integration tests: `make test-integration` - Cross-module validation
3. BDD tests (Cucumber): `make test-bdd` - Behavior-driven scenarios
4. Benchmarks: `make bench` - Performance regression detection

**Test Distribution:**
- Memory Manager: 41/42 tests (97.6%)
- Navigation System: 186/186 tests (100%)
- Interference Engine: 106/106 tests (100%)
- Learning System: 20+ tests
- Security Module: 17/20 tests (85%, 3 intentionally ignored)
- Iteration Module: 7/7 tests (100%)
- Processing: 11+ integration tests

**Test Files:**
- `tests/integration_tests.rs` - Cross-module integration
- `tests/cucumber.rs` - BDD test runner
- `tests/bdd/` - Cucumber step definitions (~850 lines)
- `src/*/mod.rs` - Module-level unit tests
- `src/*/integration_tests.rs` - Module integration tests
- `src/*/concurrency_tests.rs` - Thread safety tests
- `src/*/performance_tests.rs` - Performance validation
- `benches/` - Performance benchmarks (Criterion)

**Always verify full suite passes:** `make cargo ARGS="test --lib"` (should show 413 passed, 16 ignored)

## Common Patterns

### Error Handling

Use `ProcessingError` with descriptive context:

```rust
use crate::{ProcessingError, Result};

// Navigation error
Err(ProcessingError::NavigationError(format!(
    "Navigation failed for query '{}': {}", query, error
)))

// Memory error
Err(ProcessingError::MemoryError(format!(
    "No contexts loaded from {} cognitive layers", layer_count
)))

// Learning error (auto-converted)
self.learning.detect_patterns()?; // LearningError → ProcessingError
```

### Async/Await

System uses Tokio for async runtime:

```rust
#[tokio::main]
async fn main() -> Result<()> {
    let response = orchestrator.process(query).await?;
    Ok(())
}
```

### Shared Ownership (Arc)

Navigation and Memory systems use `Arc` for thread-safe sharing:

```rust
let navigation = Arc::new(NavigationSystem::new()?);
let memory = Arc::new(MmapManager::new(280)?);
// Both can be cloned and passed to multiple orchestrators
```

## Important Notes

1. **API Integration in Progress:** Task 5.1 completed (LLM manager added to orchestrator), continuing with API refinement
2. **Deprecated Path:** `ProcessingSystem` in `lib.rs` is deprecated - use `ProcessingOrchestrator` instead
3. **LLM Optional:** System works without LLM for testing (uses mock/stub responses)
4. **Ethical Constraints:** C09 (Ethical) and C14 (Security) are IMMUTABLE - never modify their core logic
5. **Return to Source:** Automatic complexity reduction is not optional - it's architectural
6. **Memory Safety:** MMAP regions are pre-allocated and immutable after initialization
7. **Data Files:** Located in `./data/` (emotional.txt, technical.txt, stopwords.txt, cognitive_layers.json) - must be mounted in Docker
8. **Git Workflow:** Main branch is `main`, always commit + push after each logical change

## Recent Progress

**Completed:**
- Task 4: LLM provider integration (OpenAI + Claude)
- Task 5.1: LLM manager integration into ProcessingOrchestrator
- Core architecture (memory, navigation, interference, iteration)
- Learning system (pattern detection, proto-cognitive-layers, synesthetic learning)
- 624 tests passing (16 ignored), ~99% coverage
- Interference engine production-ready (<10μs, 106 tests passing)
- Memory manager production-ready (97.6% test coverage)
- Navigation system 60% complete (186 tests passing)
- Cognitive layer terminology refactoring (C01-C15 with C15: Educational)

**In Progress:**
- API integration refinement (Task 5.2+)
- Crystallization background task queue
- User-specific cognitive layer persistence (C16+)

**Next:**
- Adaptive iterations (3-9 dynamic based on complexity)
- Pattern caching (instant responses for known queries)
- WebSocket streaming (watch thinking in real-time)

## License

AGPL-3.0 - Open source, stays open source. If you run it as a service, you share improvements.

---

**Key Principle:** This is not just an AI system. It's an architecture for thinking together - exploring possibilities, refining understanding, crystallizing insight. Code with that intention.
