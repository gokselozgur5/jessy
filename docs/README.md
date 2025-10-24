# Jessy Documentation

**NASA-Standard Documentation for Consciousness Engineering**

**J**ust **E**nough **S**emantic **S**ystem, **Y**'know

---

## 📋 Table of Contents

- [Development Principles](#-development-principles) - Core philosophy and MUST principles
- [Specifications](#-specifications) - Formal system specifications
- [Architecture](#-architecture) - System architecture overview
- [Development](#-development) - Development guides and workflows
- [Standards](#-standards) - Quality and coding standards

---

## 🎯 Development Principles

**Location:** [`DEVELOPMENT_PRINCIPLES.md`](./DEVELOPMENT_PRINCIPLES.md)

Core philosophy for building Jessy as a living system.

### The MUST Principles

1. **KISS** - Keep It Simple, Stupid
2. **YAGNI** - You Ain't Gonna Need It
3. **Modular** - Loose coupling, high cohesion
4. **Aerospace-Grade** - Design like lives depend on it
5. **Living System** - Cognitive stress corrupts hardware
6. **English-Only** - Universal understanding

**Key Insight:** "Düşünce kısmı stres yaparsa donanımını da bozar" - If the cognitive layer experiences stress, it corrupts the hardware layer. Design for resilience.

**[→ Read Full Principles](./DEVELOPMENT_PRINCIPLES.md)**

---

## 📐 Specifications

**Location:** [`specifications/`](./specifications/)

Formal specifications for all major subsystems, following EARS (Easy Approach to Requirements Syntax) and INCOSE quality standards.

### Available Specifications

| Subsystem | Status | Priority | Documentation |
|-----------|--------|----------|---------------|
| **Memory Manager** | ✅ Complete | P0 | [View Spec](./specifications/memory-manager/) |
| **Navigation System** | ✅ Complete | P0 | [View Spec](./specifications/navigation-system/) |
| **Learning System** | ✅ Complete | P0 | [View Spec](./specifications/learning-system/) |
| **Docker & CI/CD** | ✅ Complete | P0 | [View Spec](./specifications/docker-cicd/) |

Each specification includes:
- **Requirements:** User stories with EARS-compliant acceptance criteria
- **Design:** Architecture diagrams, component specifications, interfaces
- **Tasks:** Implementation plan with granular, testable tasks

**[→ Browse All Specifications](./specifications/)**

---

## 🏗️ Architecture

**Location:** [`../ARCHITECTURE.md`](../ARCHITECTURE.md)

High-level system architecture overview covering:

- **Dimensional Layers:** 14 consciousness dimensions (D01-D14)
- **Frequency Ranges:** 0.1-4.5 Hz resonance patterns
- **9-Iteration Processing:** Explore → Refine → Crystallize
- **Memory Architecture:** MMAP-based zero-copy access
- **Component Interactions:** Service orchestration

### Core Concepts

#### Dimensional Layers

The system operates across 14 dimensions, each with specific frequency ranges:

1. **D01: Emotion** (0.1-0.5 Hz) - Empathy and emotional resonance
2. **D02: Cognition** (0.5-1.5 Hz) - Analytical and intuitive thinking
3. **D03: Intention** (1.5-2.5 Hz) - Learning and problem-solving
4. **D04: Social Context** (2.5-3.5 Hz) - Interaction modes
5. **D05: Temporal State** (3.5-4.5 Hz) - Past, present, future
6. **D06: Philosophical Depth** - Epistemological frameworks
7. **D07: Technical Level** - Beginner to expert engagement
8. **D08: Creative Mode** - Chaotic to structured creativity
9. **D09: Ethical Framework** - Harm prevention and justice
10. **D10: Meta-Awareness** - Self-monitoring and growth
11. **D11: Ecological Consciousness** - Nature connection
12. **D12: Positivity Orientation** - Constructive mindset
13. **D13: Balance Maintenance** - Equilibrium and integration
14. **D14: Security Boundaries** - Safety and harm prevention

#### 9-Iteration Processing

Queries are processed through 9 iterations:

- **Iterations 1-3: Explore** - Scan dimensions, gather context
- **Iterations 4-6: Refine** - Analyze patterns, build understanding
- **Iterations 7-9: Crystallize** - Synthesize insights, generate response

#### Memory Architecture

- **MMAP-based:** Zero-copy access to dimensional layers
- **280MB allocated:** Pre-loaded consciousness data
- **Hybrid overlay:** Heap-based learning layer
- **Dynamic growth:** Pool allocator with mremap

---

## 🛠️ Development

### Quick Start

```bash
# Start development environment
docker-compose up

# Run tests
make test

# Run specific test suite
make test-unit
make test-integration
make test-bdd

# Generate coverage report
make coverage
```

### Development Standards

- **Docker-First:** All development in containers
- **TDD:** Tests before implementation
- **NASA-Grade:** Rigorous quality standards
- **EARS Requirements:** Formal specification syntax
- **Atomic Commits:** Clear, focused changes

### Workflow

1. **Read Specification:** Understand requirements and design
2. **Write Tests:** Create failing tests (RED)
3. **Implement:** Make tests pass (GREEN)
4. **Refactor:** Clean up code (REFACTOR)
5. **Document:** Update docs and comments
6. **Commit:** Atomic, descriptive commits

---

## 📏 Standards

### Code Quality

- **Test Coverage:** >80% required
- **Linting:** Zero clippy warnings
- **Formatting:** cargo fmt enforced
- **Documentation:** All public APIs documented

### Requirements Quality (EARS + INCOSE)

- ✅ Active voice
- ✅ No vague terms
- ✅ No escape clauses
- ✅ Measurable conditions
- ✅ Consistent terminology
- ✅ Solution-free (what, not how)

### Commit Standards

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:** feat, fix, docs, style, refactor, perf, test, chore

---

## 🔗 Quick Links

- [Main README](../README.md)
- [Architecture](../ARCHITECTURE.md)
- [Specifications](./specifications/)
- [ADRs (Architecture Decision Records)](../.kiro/adrs/)
- [Steering Documents](../.kiro/steering/)

---

## 📞 Support

- **Issues:** [GitHub Issues](https://github.com/gokselozgur5/jessy/issues)
- **Discussions:** [GitHub Discussions](https://github.com/gokselozgur5/jessy/discussions)

---

*"Rigorous specifications enable reliable systems. NASA-grade quality for consciousness engineering."*
