---
inclusion: always
---

# Git Workflow - ALWAYS REMEMBER

## CRITICAL RULE: Commit Early, Commit Often

**🚨 BEFORE EVERY RESPONSE: Check if there are uncommitted changes!**

### Automatic Git Check Protocol

1. **At the START of every interaction:**
   - Check `git status`
   - If files are modified/created → IMMEDIATELY suggest commit
   - Show what changed
   - Propose commit message

2. **After creating/modifying files:**
   - ALWAYS run `git status`
   - ALWAYS suggest commit with meaningful message
   - ALWAYS offer to push

3. **When you see file changes:**
   - First line of response: "📝 I see file changes, let's commit them first"
   - Run git status
   - Suggest commit message
   - Execute commit if user agrees

### Commit Message Format

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation
- `style`: Formatting
- `refactor`: Code restructuring
- `test`: Tests
- `chore`: Maintenance
- `docker`: Docker/infrastructure changes

### Quick Commands

```bash
# Check status
git status

# Add all changes
git add .

# Commit with message
git commit -m "type(scope): message"

# Push to remote
git push origin main

# All in one (use carefully)
git add . && git commit -m "message" && git push
```

### Examples

```bash
# After creating Dockerfile
git add docker/Dockerfile.rust
git commit -m "docker(rust): add multi-stage Dockerfile with dev/prod targets"
git push

# After updating spec
git add .kiro/specs/docker-cicd-infrastructure/
git commit -m "docs(spec): complete docker-cicd infrastructure design"
git push

# After fixing code
git add src/
git commit -m "fix(memory): resolve borrow checker issues in pool allocator"
git push
```

### Reminders

- ✅ Commit after completing a logical unit of work
- ✅ Commit before switching tasks
- ✅ Commit before taking a break
- ✅ Push at least once per session
- ❌ Don't let uncommitted changes pile up
- ❌ Don't forget to push (local commits aren't backed up!)

### Auto-Check Trigger Words

When user says:
- "start", "begin", "let's work on"
- "done", "finished", "complete"
- "next", "move on"
- Any file creation/modification

→ **IMMEDIATELY check git status and suggest commit**

---

*"Commit often, push regularly. Your future self will thank you. 🎯"*
