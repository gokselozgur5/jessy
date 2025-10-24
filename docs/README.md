# Jessy Documentation

**J**ust **E**nough **S**emantic **S**ystem, **Y**'know

This directory contains public documentation for the Jessy consciousness system.

## 📚 Contents

### Architecture Decision Records (ADRs)
- [ADR-001: Use MMAP for Memory Management](adrs/001-use-mmap-for-memory-management.md)

### Specifications
- [Memory Manager](specs/memory-manager/)
  - [Requirements](specs/memory-manager/requirements.md)
  - [Design](specs/memory-manager/design.md)

## 🎯 Documentation Philosophy

- **Design First**: Complete design before implementation
- **Decision Rationale**: Every major decision documented with WHY
- **Requirements Traceability**: Every requirement maps to design
- **Living Documents**: Updated as system evolves

## 🔄 9-Iteration Method

All specifications follow the 9-iteration deep thinking process:
- **Iterations 1-3**: Explore the problem space
- **Iterations 4-6**: Refine understanding and solutions
- **Iterations 7-9**: Crystallize final design

## 📖 Reading Guide

**For Developers**:
1. Start with ADRs to understand key decisions
2. Read requirements to understand what we're building
3. Study design to understand how we're building it
4. Refer back during implementation

**For Architects**:
1. Review ADRs for decision rationale
2. Validate design against requirements
3. Ensure consistency across components
4. Guide future architectural decisions

**For Stakeholders**:
1. Read requirements for feature overview
2. Check ADRs for strategic decisions
3. Review design for technical approach

## 🏗️ Structure

```
docs/
├── README.md           # This file
├── adrs/              # Architecture Decision Records
│   └── 001-*.md
└── specs/             # Feature Specifications
    └── memory-manager/
        ├── requirements.md
        └── design.md
```

## 🔗 Related Resources

- **Main README**: [../README.md](../README.md)
- **Architecture Overview**: [../ARCHITECTURE.md](../ARCHITECTURE.md)
- **Source Code**: [../src/](../src/)

---

*"Documentation is a love letter that you write to your future self." - Damian Conway*
