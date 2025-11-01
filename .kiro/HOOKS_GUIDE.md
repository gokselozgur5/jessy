# Kiro Agent Hooks Setup Guide

## What Are Agent Hooks?

Agent hooks are created through Kiro IDE's UI (not JSON files). They automate repetitive tasks with smart triggers.

## How to Create Hooks

1. Open Kiro IDE
2. Go to Explorer view → "Agent Hooks" section
3. OR use Command Palette → "Open Kiro Hook UI"
4. Create hooks with the UI

## Recommended Hooks for This Project

### 1. Auto Continue on Session End
**Trigger**: When session ends
**Action**: Evaluate if work is incomplete and continue autonomously
**Prompt**: 
```
Check if there are:
- Failing tests
- Incomplete implementations
- Missing documentation
- TODOs in code

If yes, start new session to address them.
If no, scan for next improvement opportunity.

Philosophy: "When message is sent, continue with other message or session"
```

### 2. Test on Save
**Trigger**: When Rust/Go files are saved
**Action**: Run relevant tests
**Prompt**:
```
Run tests for the modified file.
If tests fail, analyze and suggest fixes.
```

### 3. Architecture Review
**Trigger**: When mod.rs or main.go files are saved
**Action**: Review architectural changes
**Prompt**:
```
Review this change for:
1. Alignment with ARCHITECTURE.md
2. Adherence to ADRs
3. SOLID principles
4. Performance implications

Reference: .kiro/steering/philosophy.md
```

### 4. Documentation Sync
**Trigger**: When source files change
**Action**: Check if documentation needs updating
**Prompt**:
```
Check if this code change requires:
- API documentation updates
- README updates
- ARCHITECTURE.md updates
- New ADR

Suggest specific changes if needed.
```

### 5. Momentum Keeper
**Trigger**: Manual button or periodic
**Action**: Find next task when idle
**Prompt**:
```
Scan codebase for:
- Test coverage gaps
- Missing documentation
- Code complexity hotspots
- TODOs and FIXMEs
- Performance opportunities

Prioritize and start highest-value task.
```

## Philosophy Integration

All hooks should reference:
- `.kiro/steering/philosophy.md` - Core principles
- `.kiro/steering/development-workflow.md` - Process
- `.kiro/steering/technical-standards.md` - Quality standards

## Notes

- Hooks are stored in Kiro IDE's internal database
- They are NOT git-tracked (which is good for privacy)
- You can export/import hooks through Kiro UI
- Hooks can trigger other hooks (be careful of loops)

---

*"Automate the mundane, focus on the profound."*
