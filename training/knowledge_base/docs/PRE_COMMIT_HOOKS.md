# Pre-Commit Hooks Guide

## Overview

Jessy uses [pre-commit](https://pre-commit.com/) to automatically run code quality checks before commits. This ensures consistent code quality and catches issues early.

## Installation

### Quick Setup

```bash
# Install and configure all hooks
make setup-hooks
```

### Manual Setup

```bash
# Install pre-commit
pip install pre-commit

# Install hooks
pre-commit install

# Install commit-msg hook
pre-commit install --hook-type commit-msg

# Test installation
pre-commit run --all-files
```

## Hooks

### File Quality Checks

**Trailing Whitespace**
- Removes trailing whitespace from all files
- Keeps code clean and consistent

**End of File Fixer**
- Ensures files end with a newline
- Required by POSIX standards

**YAML/TOML/JSON Syntax**
- Validates configuration file syntax
- Catches syntax errors before commit

**Large Files**
- Prevents committing files >1MB
- Keeps repository size manageable

**Merge Conflicts**
- Detects unresolved merge conflict markers
- Prevents accidental commits of conflicts

**Private Keys**
- Detects accidentally committed private keys
- Prevents security breaches

### Rust Checks

**cargo fmt** (Format)
- Formats Rust code according to style guide
- Runs: `cargo fmt --all`
- Auto-fixes: Yes

**cargo clippy** (Lint)
- Runs Rust linter with strict warnings
- Runs: `cargo clippy --all-features --all-targets -- -D warnings`
- Auto-fixes: No (manual fixes required)

**cargo check** (Compile)
- Verifies code compiles without errors
- Runs: `cargo check --all-features`
- Auto-fixes: No

### Go Checks

**go fmt** (Format)
- Formats Go code according to style guide
- Runs: `gofmt -w`
- Auto-fixes: Yes

**go vet** (Lint)
- Runs Go static analysis
- Catches common mistakes
- Auto-fixes: No

**go imports** (Imports)
- Organizes and formats imports
- Removes unused imports
- Auto-fixes: Yes

**go mod tidy** (Dependencies)
- Cleans up go.mod and go.sum
- Removes unused dependencies
- Auto-fixes: Yes

### Documentation Checks

**markdownlint** (Markdown)
- Lints Markdown files
- Enforces consistent formatting
- Auto-fixes: Yes (with --fix)

### Shell Script Checks

**shellcheck** (Shell)
- Lints shell scripts
- Catches common shell scripting errors
- Auto-fixes: No

### Docker Checks

**hadolint** (Dockerfile)
- Lints Dockerfiles
- Enforces best practices
- Auto-fixes: No

### Security Checks

**detect-secrets** (Secrets)
- Scans for accidentally committed secrets
- Uses baseline file to track known false positives
- Auto-fixes: No

## Usage

### Automatic Execution

Hooks run automatically on:

```bash
# Before commit
git commit -m "feat: add new feature"

# Before push (if configured)
git push origin main
```

### Manual Execution

```bash
# Run all hooks on all files
pre-commit run --all-files

# Run all hooks on staged files
pre-commit run

# Run specific hook
pre-commit run cargo-fmt
pre-commit run cargo-clippy

# Run on specific files
pre-commit run --files src/main.rs
```

### Skipping Hooks

**Not recommended**, but sometimes necessary:

```bash
# Skip all hooks
git commit --no-verify -m "emergency fix"

# Skip specific hook
SKIP=cargo-clippy git commit -m "WIP: work in progress"

# Skip multiple hooks
SKIP=cargo-clippy,cargo-check git commit -m "WIP"
```

## Configuration

### Hook Configuration

Edit `.pre-commit-config.yaml`:

```yaml
repos:
  - repo: local
    hooks:
      - id: cargo-fmt
        name: Cargo format
        entry: cargo fmt --all --
        language: system
        types: [rust]
        pass_filenames: false
```

### Excluding Files

Add patterns to exclude in `.pre-commit-config.yaml`:

```yaml
exclude: |
  (?x)^(
    target/.*|
    .git/.*|
    node_modules/.*
  )$
```

### Updating Hooks

```bash
# Update all hooks to latest versions
pre-commit autoupdate

# Update specific hook
pre-commit autoupdate --repo https://github.com/pre-commit/pre-commit-hooks
```

## Troubleshooting

### Hook Fails on Commit

**Problem**: Commit is blocked by failing hook

**Solutions**:

1. **Fix the issue** (recommended):
   ```bash
   # See what failed
   pre-commit run --all-files
   
   # Fix the issues
   cargo fmt
   cargo clippy --fix
   
   # Try commit again
   git commit -m "fix: resolve issues"
   ```

2. **Skip temporarily** (not recommended):
   ```bash
   git commit --no-verify -m "WIP"
   ```

### Slow Hook Execution

**Problem**: Hooks take too long to run

**Solutions**:

1. **Run only on changed files**:
   ```bash
   # Default behavior - only staged files
   git commit
   ```

2. **Disable slow hooks for WIP commits**:
   ```bash
   SKIP=cargo-check git commit -m "WIP"
   ```

3. **Use faster alternatives**:
   ```yaml
   # Use cargo check instead of cargo build
   - id: cargo-check
     entry: cargo check --all-features
   ```

### Hook Installation Fails

**Problem**: `pre-commit install` fails

**Solutions**:

1. **Check Python installation**:
   ```bash
   python3 --version
   pip3 --version
   ```

2. **Reinstall pre-commit**:
   ```bash
   pip3 install --upgrade pre-commit
   ```

3. **Check git hooks directory**:
   ```bash
   ls -la .git/hooks/
   ```

### False Positive in Secret Detection

**Problem**: detect-secrets flags non-secret as secret

**Solutions**:

1. **Update baseline**:
   ```bash
   detect-secrets scan > .secrets.baseline
   ```

2. **Audit baseline**:
   ```bash
   detect-secrets audit .secrets.baseline
   ```

3. **Add inline exception**:
   ```rust
   let api_key = "not-a-real-key"; // pragma: allowlist secret
   ```

## Best Practices

### Development Workflow

1. **Install hooks early**:
   ```bash
   # First thing after cloning
   make setup-hooks
   ```

2. **Run hooks before committing**:
   ```bash
   # Check before staging
   pre-commit run --all-files
   
   # Stage files
   git add .
   
   # Commit (hooks run automatically)
   git commit -m "feat: add feature"
   ```

3. **Fix issues immediately**:
   - Don't accumulate formatting issues
   - Address clippy warnings promptly
   - Keep code clean continuously

### Team Collaboration

1. **Consistent configuration**:
   - Keep `.pre-commit-config.yaml` in git
   - Update hooks together as team
   - Document any custom hooks

2. **Share baseline files**:
   - Commit `.secrets.baseline` to git
   - Update when adding known false positives
   - Review changes in PRs

3. **CI enforcement**:
   - Run same checks in CI
   - Fail CI if hooks would fail
   - Ensure consistency

### Performance Optimization

1. **Use `pass_filenames: false` for slow hooks**:
   ```yaml
   - id: cargo-check
     pass_filenames: false  # Run once, not per file
   ```

2. **Skip expensive checks for WIP**:
   ```bash
   SKIP=cargo-check git commit -m "WIP"
   ```

3. **Run full checks before PR**:
   ```bash
   pre-commit run --all-files
   ```

## Integration with CI/CD

### GitHub Actions

Pre-commit can run in CI:

```yaml
- name: Run pre-commit
  uses: pre-commit/action@v3.0.0
```

### Local CI Simulation

```bash
# Run same checks as CI
make ci

# Includes:
# - cargo fmt --check
# - cargo clippy
# - cargo test
```

## Custom Hooks

### Adding Custom Hook

Edit `.pre-commit-config.yaml`:

```yaml
repos:
  - repo: local
    hooks:
      - id: custom-check
        name: Custom check
        entry: ./scripts/custom-check.sh
        language: system
        pass_filenames: false
```

### Example Custom Hook

```bash
#!/bin/bash
# scripts/custom-check.sh

# Check for TODO comments in production code
if git diff --cached --name-only | grep -E '\.(rs|go)$' | xargs grep -n 'TODO:'; then
    echo "❌ Found TODO comments in production code"
    exit 1
fi

echo "✅ No TODO comments found"
exit 0
```

## Related Documentation

- [Development Workflow](DEVELOPMENT_PRINCIPLES.md)
- [CI/CD Pipeline](CI_CD.md)
- [Code Style Guide](STYLE_GUIDE.md)

---

*"Catch issues early. Commit with confidence. 🎣"*
