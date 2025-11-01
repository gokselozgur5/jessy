# 🦉 OWL PROTOCOL - Agent Wisdom Layer

> "One change, one commit. One test, all tests. One task, complete focus."

## 🎯 THE THREE LAWS

### Law 1: ATOMIC COMMITS
**Proverb**: "Small steps, frequent saves, never lost."
```bash
# After ANY code change:
git add <files> && git commit -m "..." && git push origin main
```
**Never write twice without committing once.**

### Law 2: FULL VALIDATION
**Proverb**: "Test one, verify all, trust none."
```bash
# After ANY test run:
make cargo ARGS="test --lib"
```
**217 tests must pass. Zero failures. Always.**

### Law 3: SINGLE FOCUS
**Proverb**: "One task, one mind, one completion."
- Read task → Execute task → Mark complete → Next task
- **Never** start task N+1 before completing task N

---

## 📋 TDD RHYTHM (Red-Green-Refactor)

### RED Phase: Write Tests
```
1. Read task requirements from tasks.md
2. Write comprehensive tests (fail first if new)
3. Run: make cargo ARGS="test --lib <specific_test>"
4. Run: make cargo ARGS="test --lib"  ← FULL SUITE
5. Commit: test(<scope>): <description> (task X.Y)
6. Push: git push origin main
7. Mark: taskStatus task X.Y completed
```

### GREEN Phase: Implement
```
1. Write minimal code to pass tests
2. Run: make cargo ARGS="test --lib <specific_test>"
3. Run: make cargo ARGS="test --lib"  ← FULL SUITE
4. Commit: feat(<scope>): <description> (task X.Y)
5. Push: git push origin main
6. Mark: taskStatus task X.Y completed
```

**Mantra**: "Red → Full Test → Commit → Green → Full Test → Commit"

---

## 🔧 ENVIRONMENT CONTEXT

### Docker Service
- **Current**: `unit-tests` (see Makefile RUST_SERVICE)
- **Command**: `make cargo ARGS="..."`
- **Network**: `jessy_jessy-network`

### File Locations
- **Tasks**: `.kiro/specs/navigation-system/tasks.md`
- **Requirements**: `.kiro/specs/navigation-system/requirements.md`
- **Design**: `.kiro/specs/navigation-system/design.md`
- **Data**: `./data/` (emotional.txt, technical.txt, stopwords.txt)

### Key Commands
```bash
make cargo ARGS="test --lib"              # Full test suite
make cargo ARGS="test --lib <module>"     # Specific module
make services                              # List Docker services
make test-unit                             # Alternative full test
```

---

## 🎭 TASK EXECUTION PATTERN

### Before Starting
```
1. Read AGENT_RULES.md (this file)
2. Read current task from tasks.md
3. Read related requirements from requirements.md
4. Understand: What? Why? How?
```

### During Execution
```
1. Focus on ONE sub-task only
2. Write code OR tests (not both at once)
3. Test specific → Test all → Commit → Push
4. Mark complete in tasks.md
5. Repeat for next sub-task
```

### After Completion
```
1. Verify all sub-tasks marked complete
2. Verify all tests passing (217/217)
3. Verify all commits pushed
4. Move to next task
```

**Proverb**: "Start clean, work focused, finish complete."

---

## 🚫 ANTI-PATTERNS (Never Do)

| ❌ Wrong | ✅ Right |
|---------|---------|
| Write 5 changes, commit once | Write 1 change, commit once |
| Test specific only | Test specific + full suite |
| Commit without push | Commit + push immediately |
| Start task 5.9 while 5.8 incomplete | Complete 5.8 fully, then 5.9 |
| Skip marking task complete | Always mark in tasks.md |
| Assume tests pass | Run and verify 217/217 |

---

## 📊 COMMIT MESSAGE TEMPLATE

```
<type>(<scope>): <description> (task X.Y)

- What changed (bullet points)
- Why it changed
- What it enables

Requirements: X.Y-X.Z
Task: X.Y (RED/GREEN phase)
```

**Types**: `test`, `feat`, `fix`, `refactor`, `chore`, `docs`

**Examples**:
```
test(navigation): add path ranking tests (task 5.8)
feat(navigation): implement path ranking algorithm (task 5.9)
fix(docker): mount data directory for vocabulary files
```

---

## 🎯 SESSION START CHECKLIST

```
□ Read this file (AGENT_RULES.md)
□ Check current task in tasks.md
□ Verify Docker service: unit-tests
□ Verify test baseline: make cargo ARGS="test --lib"
□ Understand task requirements
□ Begin with RED or GREEN phase
```

---

## 🧠 COGNITIVE SHORTCUTS

### When Confused
1. Return to tasks.md
2. Read task description
3. Check requirements.md
4. Follow TDD rhythm

### When Stuck
1. Run full test suite
2. Read error messages
3. Check file locations
4. Verify Docker mounts

### When Uncertain
1. Commit current work
2. Run full tests
3. Ask user for clarification
4. Never guess, always verify

---

## 🏆 SUCCESS METRICS

- **Commits**: Frequent, atomic, descriptive
- **Tests**: 217 passed, 0 failed, always
- **Tasks**: One at a time, fully complete
- **Code**: Minimal, tested, working

**Proverb**: "Quality through rhythm, not through rush."

---

## 🦉 OWL WISDOM

> "The owl sees in darkness by focusing on one thing at a time."
> "Small commits build great systems."
> "Test everything, trust nothing, verify always."
> "One task, one mind, one completion."
> "Rhythm over speed, consistency over cleverness."

---

**READ FIRST. FOLLOW ALWAYS. SUCCEED CONSISTENTLY.**
