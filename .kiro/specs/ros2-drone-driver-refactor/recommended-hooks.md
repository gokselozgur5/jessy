# Recommended Agent Hooks for ROS2 Drone Driver Project

## Overview

Agent hooks automate repetitive tasks and enforce quality standards. Here are recommended hooks for this project.

## Recommended Hooks

### 1. Test-First Enforcement Hook

**Trigger**: On file save (*.rs files)
**Purpose**: Ensure TDD is followed - tests exist before implementation

**Hook Configuration**:
```yaml
name: "Enforce TDD - Check Tests Exist"
trigger: on_save
file_pattern: "src/**/*.rs"
exclude_pattern: "tests/**/*.rs"
prompt: |
  Check if corresponding test file exists for this implementation file.
  
  Rules:
  - For src/foo.rs, tests/foo_tests.rs must exist
  - Test file must have at least one #[test] function
  - If test file missing or empty, warn user and suggest creating tests first
  
  If tests don't exist:
  1. Show warning: "TDD violation: No tests found for this file"
  2. Ask: "Would you like me to create test scaffolding?"
  3. If yes, create test file with basic test structure
```

### 2. Benchmark on Performance-Critical Changes

**Trigger**: On file save (HAL and driver files)
**Purpose**: Automatically run benchmarks when performance-critical code changes

**Hook Configuration**:
```yaml
name: "Run Benchmarks on Performance Code"
trigger: on_save
file_pattern: "hal/**/*.rs,drivers/**/*.rs"
prompt: |
  This file is performance-critical. Run benchmarks to check for regressions.
  
  Steps:
  1. Run: cargo bench --bench {module_name}
  2. Compare with baseline (if exists)
  3. If >5% regression, show warning with details
  4. If >10% regression, ask if user wants to revert changes
  5. Update baseline if performance improved
```

### 3. Requirement Traceability Check

**Trigger**: On commit
**Purpose**: Ensure all code changes reference requirements

**Hook Configuration**:
```yaml
name: "Check Requirement Traceability"
trigger: on_commit
prompt: |
  Verify commit message includes requirement references.
  
  Rules:
  - Commit message must contain "REQ-X.Y" reference
  - Check if referenced requirement exists in requirements.md
  - Suggest relevant requirements if none provided
  
  If no requirement reference:
  1. Show error: "Missing requirement reference in commit message"
  2. List potentially relevant requirements based on changed files
  3. Ask user to update commit message
```

### 4. Auto-Format and Lint on Save

**Trigger**: On file save (Rust files)
**Purpose**: Maintain code quality automatically

**Hook Configuration**:
```yaml
name: "Auto-Format Rust Code"
trigger: on_save
file_pattern: "**/*.rs"
prompt: |
  Format and lint Rust code automatically.
  
  Steps:
  1. Run: cargo fmt
  2. Run: cargo clippy --all-targets --all-features
  3. If clippy warnings, show them inline
  4. If clippy errors, prevent save and show errors
```

### 5. Update Tests When Implementation Changes

**Trigger**: On file save (implementation files)
**Purpose**: Remind to update tests when implementation changes

**Hook Configuration**:
```yaml
name: "Remind to Update Tests"
trigger: on_save
file_pattern: "src/**/*.rs"
exclude_pattern: "tests/**/*.rs"
prompt: |
  Implementation file changed. Check if tests need updating.
  
  Steps:
  1. Find corresponding test file
  2. Check git diff to see what changed
  3. If function signatures changed, warn: "Function signature changed - update tests"
  4. If new public functions added, suggest: "Add tests for new functions"
  5. Offer to run tests: cargo test {module_name}
```

### 6. Compatibility Check on Driver Changes

**Trigger**: On file save (driver files)
**Purpose**: Ensure backward compatibility maintained

**Hook Configuration**:
```yaml
name: "Check Backward Compatibility"
trigger: on_save
file_pattern: "drivers/**/*.rs"
prompt: |
  Driver code changed. Verify backward compatibility.
  
  Steps:
  1. Check if topic names match compatibility.md
  2. Check if message types match Python implementation
  3. Check if parameter names match Python implementation
  4. If mismatch found, show error with expected vs actual
  5. Suggest running compatibility tests: cargo test compatibility
```

### 7. Documentation Update Reminder

**Trigger**: On file save (public API changes)
**Purpose**: Keep documentation in sync with code

**Hook Configuration**:
```yaml
name: "Update Documentation"
trigger: on_save
file_pattern: "src/**/*.rs"
prompt: |
  Check if public API documentation needs updating.
  
  Steps:
  1. Detect if public functions/structs changed
  2. Check if doc comments exist (///)
  3. If missing doc comments, warn and offer to generate skeleton
  4. If ADR might be affected, suggest reviewing ADRs
  5. Offer to run: cargo doc --open
```

### 8. Run Tests Before Commit

**Trigger**: On commit attempt
**Purpose**: Prevent committing broken code

**Hook Configuration**:
```yaml
name: "Pre-Commit Test Run"
trigger: on_commit
prompt: |
  Run tests before committing.
  
  Steps:
  1. Run: cargo test --all
  2. If tests fail, block commit and show failures
  3. If tests pass, allow commit
  4. Show test coverage summary
  5. If coverage dropped, warn user
```

### 9. Traceability Matrix Update

**Trigger**: Manual or on demand
**Purpose**: Generate updated traceability matrix

**Hook Configuration**:
```yaml
name: "Generate Traceability Matrix"
trigger: manual
prompt: |
  Generate traceability matrix linking requirements to code.
  
  Steps:
  1. Scan all .rs files for REQ-X.Y references in comments
  2. Parse requirements.md for all requirements
  3. Generate matrix showing which code implements which requirements
  4. Identify requirements with no implementation
  5. Identify code with no requirement reference
  6. Save to docs/traceability-matrix.md
```

### 10. Performance Report Generator

**Trigger**: Manual or weekly
**Purpose**: Generate performance report

**Hook Configuration**:
```yaml
name: "Generate Performance Report"
trigger: manual
prompt: |
  Generate comprehensive performance report.
  
  Steps:
  1. Run all benchmarks: cargo bench
  2. Collect latency metrics (p50, p95, p99)
  3. Check against requirements (<10ms for critical paths)
  4. Generate flamegraph for hotspots
  5. Compare with previous report (if exists)
  6. Save report to docs/performance-report-{date}.md
```

## How to Set Up Hooks

### Option 1: Using Kiro UI
1. Open Command Palette (Cmd/Ctrl + Shift + P)
2. Search for "Open Kiro Hook UI"
3. Click "Create New Hook"
4. Copy configuration from above
5. Save and enable

### Option 2: Using Explorer View
1. Open Explorer panel
2. Find "Agent Hooks" section
3. Click "+" to create new hook
4. Configure using examples above

### Option 3: Manual Configuration
Create files in `.kiro/hooks/` directory:
```
.kiro/hooks/
  ├── tdd-enforcement.yaml
  ├── benchmark-on-save.yaml
  ├── requirement-traceability.yaml
  └── ...
```

## Priority Hooks to Set Up First

For your perfectionist, performance-focused approach, set up these first:

1. **Test-First Enforcement** - Ensures TDD is followed
2. **Benchmark on Performance-Critical Changes** - Catches regressions immediately
3. **Auto-Format and Lint** - Maintains code quality
4. **Run Tests Before Commit** - Prevents broken commits

## Hook Best Practices

### Do's:
- ✅ Keep hooks fast (<5 seconds)
- ✅ Provide clear, actionable feedback
- ✅ Allow user to skip/override when needed
- ✅ Log hook execution for debugging

### Don'ts:
- ❌ Don't block workflow unnecessarily
- ❌ Don't run expensive operations on every save
- ❌ Don't make hooks too complex
- ❌ Don't forget to test hooks themselves

## Example Hook Workflow

**Typical Development Flow with Hooks:**

1. **Start Task**: Open task from tasks.md
2. **Create Test File**: Write tests first (TDD)
3. **Save Test**: Hook checks test syntax
4. **Create Implementation**: Write code to pass tests
5. **Save Implementation**: 
   - Auto-format hook runs
   - Lint hook runs
   - Benchmark hook runs (if performance-critical)
   - Test reminder hook suggests running tests
6. **Run Tests**: `cargo test`
7. **Commit**:
   - Pre-commit test hook runs
   - Requirement traceability hook checks commit message
   - Commit succeeds if all pass

## Monitoring Hook Effectiveness

Track these metrics:
- Number of TDD violations caught
- Number of performance regressions caught
- Number of commits blocked by failing tests
- Time saved by auto-formatting
- Test coverage trend over time

## Customization

Adjust hook sensitivity based on your workflow:
- **Strict Mode**: Block on any violation
- **Warning Mode**: Show warnings but allow proceeding
- **Learning Mode**: Just log, don't interrupt

Start with Warning Mode, then move to Strict Mode once team is comfortable.
