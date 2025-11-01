# 🚨 CRITICAL AGENT RULES - READ FIRST EVERY SESSION 🚨

## ⚠️ MANDATORY WORKFLOW - NEVER SKIP AND READ MAKEFILE

### 1. COMMIT AFTER EVERY MINIMAL CHANGE
```bash
# After EVERY code change, no matter how small:
git add <files>
git commit -m "<type>(<scope>): <description> (task X.Y)

- What changed
- Why it changed

Requirements: X.Y
Task: X.Y (RED/GREEN phase)"
git push origin main
```

**NEVER proceed to next change without committing!**

### 2. RUN FULL TEST SUITE AFTER EVERY TEST
```bash
# After running specific tests, ALWAYS run full suite:
make cargo ARGS="test --lib"

# This ensures:
# - No regressions in other modules
# - All tests still pass
# - Integration is maintained
```

**NEVER skip full test suite!**

### 3. WORKFLOW CHECKLIST FOR EACH SUB-TASK

#### RED Phase (Writing Tests):
- [ ] Write comprehensive tests
- [ ] Run specific tests to verify they fail (if new) or pass (if verifying)
- [ ] **RUN FULL TEST SUITE**: `make cargo ARGS="test --lib"`
- [ ] **COMMIT & PUSH**: Include all test files
- [ ] Mark task as completed in tasks.md

#### GREEN Phase (Implementation):
- [ ] Write minimal implementation to pass tests
- [ ] Run specific tests to verify they pass
- [ ] **RUN FULL TEST SUITE**: `make cargo ARGS="test --lib"`
- [ ] **COMMIT & PUSH**: Include implementation files
- [ ] Mark task as completed in tasks.md

#### After Any Code Change:
- [ ] **COMMIT IMMEDIATELY**
- [ ] **PUSH IMMEDIATELY**
- [ ] **RUN FULL TESTS**

## 🎯 QUICK REFERENCE

### Docker Service Name
Current Rust service: **unit-tests** (defined in Makefile as RUST_SERVICE)

### Common Commands
```bash
# Run cargo in Docker
make cargo ARGS="test --lib"
make cargo ARGS="test --lib path_selector"
make cargo ARGS="build"

# List all services
make services

# Run full test suite
make test-unit
```

### Commit Message Format
```
<type>(<scope>): <description> (task X.Y)

- Bullet points of changes
- What was added/modified/fixed

Requirements: X.Y-X.Z
Task: X.Y (RED/GREEN phase)
```

Types: `test`, `feat`, `refactor`, `fix`, `chore`, `docs`

## 🔴 FAILURE MODES TO AVOID

1. ❌ Writing multiple changes before committing
2. ❌ Running only specific tests without full suite
3. ❌ Forgetting to push after commit
4. ❌ Moving to next task without completing current one
5. ❌ Not marking tasks as completed in tasks.md

## ✅ SUCCESS PATTERN

```
1. Read task requirements
2. Write tests (RED)
3. Run specific tests
4. RUN FULL TEST SUITE ← CRITICAL
5. COMMIT & PUSH ← CRITICAL
6. Mark task completed
7. Write implementation (GREEN)
8. Run specific tests
9. RUN FULL TEST SUITE ← CRITICAL
10. COMMIT & PUSH ← CRITICAL
11. Mark task completed
12. Move to next task
```

---

**READ THIS FILE AT THE START OF EVERY SESSION!**
**REFER TO IT BEFORE EVERY COMMIT!**
**CHECK IT AFTER EVERY TEST RUN!**
