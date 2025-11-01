# .kiro Directory

> **Note**: your reading is so important to develop jesse. read always with max line per chunks until to be sue end of file!!!
>
This directory contains Kiro IDE-specific configuration and development artifacts. It is **intentionally excluded from Git** to keep your development environment private.

## Contents

- **`specs/`** - Active feature specifications (requirements, design, tasks)
  - Working copies of specs that sync with Kiro's spec workflow
  - Public versions are maintained in `docs/specs/`
  
- **`steering/`** - AI assistant steering rules and guidelines
  - Custom instructions for Kiro's behavior
  - Project-specific development principles
  - Cognitive patterns and best practices

- **`settings/`** - IDE configuration
  - MCP (Model Context Protocol) server configurations
  - Tool preferences and settings

## Why is this excluded from Git?

1. **Privacy**: Your development workflow and AI interactions are personal
2. **Flexibility**: Experiment with specs and steering rules without affecting the team
3. **Clean Repository**: Keep the public repo focused on production code and documentation

## Public Documentation

All finalized specifications are published in `docs/specs/` for team visibility:
- Requirements documents
- Architecture designs
- Implementation plans

## Syncing Specs

When a spec is ready for team review:
1. Kiro automatically copies it to `docs/specs/`
2. You commit the public version to Git
3. The `.kiro/specs/` version remains your working copy
