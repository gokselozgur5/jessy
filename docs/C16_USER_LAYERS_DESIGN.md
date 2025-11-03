# C16+ User-Specific Cognitive Layers - Design Document

**Status:** Design Phase (Updated to 3-Tier Model)
**Created:** 2025-11-02
**Last Updated:** 2025-11-02 (3-Tier revision)
**Authors:** gokselozgur5 + Claude Code

## Overview

The 3-Tier Cognitive Layer System enables both collective and personal intelligence:
- **Tier 1 (C01-C15):** Universal base model - shared by all users
- **Tier 2 (C16-C30):** Collective learned layers - wisdom from all users
- **Tier 3 (C31+):** Personal learned layers - individual user expertise

## Problem Statement

**Current:** All users share the same 15 core cognitive layers (C01-C15)
- Everyone gets identical responses for similar queries
- No personalization or user-specific memory
- Cannot learn from collective patterns
- Cannot learn individual preferences, style, or domain expertise

**Goal (3-Tier Solution):**
- **Tier 1 (C01-C15):** Base cognitive model (emotion, cognition, ethics, etc.)
- **Tier 2 (C16-C30):** Shared learned patterns (e.g., "Rust best practices" learned from 1000+ users)
- **Tier 3 (C31+):** User-specific patterns (e.g., User A prefers functional style, User B prefers OOP)

**Benefits:**
- Collective intelligence: All users benefit from shared learnings
- Personal intelligence: Each user gets personalized responses
- Knowledge accumulation: System gets smarter over time for everyone
- Privacy: User-specific layers isolated per user

## Architecture

### Memory Layout (3-Tier Model)

```
MMAP Region (280MB total):
├── TIER 1: Core Layers (C01-C15)        168MB  [0x0000_0000 - 0x0A00_0000]
│   └─ Base cognitive model (immutable in production)
│
├── TIER 2: Shared Learned (C16-C30)      92MB  [0x0A00_0000 - 0x1000_0000]
│   ├─ C16-C30: 15 collective cognitive layers
│   ├─ Each layer ~6MB average
│   ├─ Learned from ALL users' interactions
│   ├─ Examples: "Rust expertise", "Python patterns", "Math reasoning"
│   └─ Benefits everyone, crystallizes from collective wisdom
│
└── TIER 3: User-Specific (C31+)          32MB  [0x1000_0000 - 0x1200_0000]
    ├─ Max 32 users × 1MB each (OR)
    ├─ Max 8 users × 4MB each (configurable)
    ├─ Each user: up to 5 personal layers (C31-C35)
    ├─ Examples: "Prefers functional style", "Spanish language", "Vim shortcuts"
    └─ Privacy: isolated per user, never shared
```

**Key Differences:**
- **C16-C30 (Tier 2):** NO user_id tag → shared by all
- **C31+ (Tier 3):** WITH user_id tag → personal only

### Data Structures (3-Tier Model)

```rust
/// Tier 2: Shared learned cognitive layer (C16-C30)
pub struct SharedLayer {
    dimension_id: DimensionId, // 16-30
    layer_id: LayerId,         // Full layer identifier
    content: Vec<u8>,          // Layer content (keywords, patterns)
    keywords: Vec<String>,     // Matched keywords
    frequency: Frequency,      // Layer frequency (Hz)
    confidence: f32,           // Pattern confidence (0.0-1.0)
    created_at: SystemTime,    // When crystallized
    last_accessed: SystemTime, // Access tracking
    access_count: u64,         // Popularity tracking
    contributor_count: u64,    // How many users contributed to this layer
}

/// Tier 3: User-specific cognitive layer (C31+)
pub struct UserLayer {
    user_id: String,           // "user_123"
    dimension_id: DimensionId, // 31, 32, 33...
    layer_id: LayerId,         // Full layer identifier
    content: Vec<u8>,          // Layer content (keywords, patterns)
    keywords: Vec<String>,     // Matched keywords
    frequency: Frequency,      // Layer frequency (Hz)
    confidence: f32,           // Pattern confidence (0.0-1.0)
    created_at: SystemTime,    // When crystallized
    last_accessed: SystemTime, // LRU eviction
    access_count: u64,         // Popularity tracking
}

/// Manages Tier 2: Shared learned layers (C16-C30)
pub struct SharedLayerManager {
    // dimension_id (16-30) → shared layer
    shared_layers: HashMap<DimensionId, SharedLayer>,

    // Memory manager for MMAP
    memory_manager: Arc<MmapManager>,

    // Max shared layers (15: C16-C30)
    max_shared_layers: usize,

    // Reserve pool region offset (0x0A00_0000)
    reserve_pool_base: usize,

    // Reserve pool size (92MB)
    reserve_pool_size: usize,
}

/// Manages Tier 3: User-specific layers (C31+)
pub struct UserLayerManager {
    // user_id → layers
    user_layers: HashMap<String, Vec<UserLayer>>,

    // Memory manager for MMAP
    memory_manager: Arc<MmapManager>,

    // Max layers per user (default: 5)
    max_layers_per_user: usize,

    // User-specific region offset (0x1000_0000)
    user_region_base: usize,

    // User region size (32MB)
    user_region_size: usize,
}
```

### Layer Lifecycle (3-Tier Model)

```
User Query (with optional user_id)
     ↓
Navigation System Scan
     ├─ Tier 1: C01-C15 (always)
     ├─ Tier 2: C16-C30 (if available)
     └─ Tier 3: C31+ (if user_id provided)
     ↓
Learning System Observation
     ├─ Record query + response + context
     └─ Tag with user_id (if provided)
     ↓
Pattern Detection (every 100 queries)
     ├─ Detect global patterns (all users)
     └─ Detect user-specific patterns (per user)
     ↓
Proto-Dimension Creation
     ├─ Global proto (no user_id) → C16-C30 candidate
     └─ User proto (with user_id) → C31+ candidate
     ↓
Crystallization Queue (background, priority-based)
     ├─ High confidence (>90%) → High priority
     └─ Medium confidence (85-90%) → Normal priority
     ↓
Crystallization Decision
     ├─ If global pattern → SharedLayerManager (C16-C30)
     └─ If user pattern → UserLayerManager (C31+)
     ↓
MMAP Allocation
     ├─ C16-C30: Reserve Pool (92MB)
     └─ C31+: User-Specific Region (32MB)
     ↓
Future Queries
     ├─ All users benefit from C16-C30 (collective wisdom)
     └─ Each user benefits from own C31+ (personal expertise)
```

## Implementation Plan (3-Tier Model)

### Phase 1: Data Structures (45 min)
- [ ] Create `src/learning/shared_layer.rs`
  - [ ] Define `SharedLayer` struct (C16-C30)
  - [ ] Define `SharedLayerManager` struct
- [ ] Create `src/learning/user_layer.rs`
  - [ ] Define `UserLayer` struct (C31+)
  - [ ] Define `UserLayerManager` struct
- [ ] Add both to `src/learning/mod.rs`

### Phase 2: Shared Layer Manager (60 min)
- [ ] Implement `create_shared_layer()` for C16-C30
- [ ] Implement `get_shared_layer(dimension_id)`
- [ ] Implement `get_all_shared_layers()`
- [ ] Implement `update_access_count()` for popularity tracking
- [ ] LRU eviction when 15 layers filled (C16-C30 max)

### Phase 3: User Layer Manager (45 min)
- [ ] Implement `create_user_layer()` for C31+
- [ ] Implement `get_user_layers(user_id)`
- [ ] Implement `remove_user_layer()`
- [ ] LRU eviction per user (max 5 layers each)

### Phase 4: MMAP Integration (45 min)
- [ ] Allocate C16-C30 in Reserve Pool (92MB)
  - [ ] Map dimension_id (16-30) → MMAP offset
  - [ ] Implement `save_shared_layer_to_mmap()`
  - [ ] Implement `load_shared_layer_from_mmap()`
- [ ] Allocate C31+ in User-Specific region (32MB)
  - [ ] Map (user_id, dimension_id) → MMAP offset
  - [ ] Implement `save_user_layer_to_mmap()`
  - [ ] Implement `load_user_layer_from_mmap()`

### Phase 5: Learning System Integration (45 min)
- [ ] Tag observations with optional user_id
- [ ] Detect global patterns (all observations) → C16-C30 candidates
- [ ] Detect user-specific patterns (per user) → C31+ candidates
- [ ] Crystallize global protos → SharedLayerManager
- [ ] Crystallize user protos → UserLayerManager
- [ ] Update LearningSystem to handle both layer types

### Phase 6: Navigation Integration (60 min)
- [ ] Scan shared layers (C16-C30) in NavigationSystem
- [ ] Scan user layers (C31+) if user_id provided
- [ ] Merge results: C01-C15 + C16-C30 + C31+ (if user)
- [ ] Priority: User layers (C31+) > Shared learned (C16-C30) > Core (C01-C15)
- [ ] Update path selection logic for 3-tier system

### Phase 7: Persistence (45 min)
- [ ] Save shared layers to disk on shutdown
  - [ ] JSON format: `data/shared_layers/c{16-30}.json`
- [ ] Save user layers to disk on shutdown
  - [ ] JSON format: `data/user_layers/{user_id}/c{31+}.json`
- [ ] Load both layer types on startup
- [ ] Verify integrity (checksums)

### Phase 8: Testing (60 min)
- [ ] Unit tests for SharedLayer
- [ ] Unit tests for SharedLayerManager
- [ ] Unit tests for UserLayer
- [ ] Unit tests for UserLayerManager
- [ ] Integration test: collective wisdom (all users benefit from C16-C30)
- [ ] Integration test: personal expertise (user A vs user B different C31+)
- [ ] Performance test: 15 shared + (32 users × 5 layers) = 175 total layers
- [ ] Memory test: Verify 92MB + 32MB allocation

**Total Estimated Time:** ~6 hours (increased due to dual-manager system)

## Key Features (3-Tier Model)

### 1. Collective Intelligence (Tier 2)
- **C16-C30:** Learned from ALL users' interactions
- **Benefit:** Everyone learns from everyone
- **Examples:** "Rust best practices", "Math reasoning", "Spanish language"
- **Max:** 15 shared cognitive layers (92MB)
- **Eviction:** LRU (least popular shared layer replaced)

### 2. Personal Intelligence (Tier 3)
- **C31+:** Learned from ONE user's interactions
- **Benefit:** Personalized responses per user
- **Examples:** "Prefers functional style", "Vim shortcuts", "Gardening interest"
- **Max:** 5 layers per user (32MB total for all users)
- **Eviction:** LRU per user (least recently used personal layer)

### 3. Privacy & Isolation
- **Tier 2 (C16-C30):** NO user_id → shared, anonymous aggregation
- **Tier 3 (C31+):** WITH user_id → isolated, never shared
- User A cannot access User B's C31+ layers
- user_id hashed for storage (no PII)

### 4. Priority System (3-Level)
- **Highest:** Tier 3 (C31+) - Personal trumps all
- **Medium:** Tier 2 (C16-C30) - Collective knowledge
- **Base:** Tier 1 (C01-C15) - Core cognitive model
- Example: User's C31 "Functional style" overrides C16 "JavaScript" overrides C07 "Technical"

### 5. Pattern Confidence Thresholds
- **Tier 2 (Shared):** >90% confidence required (high bar for global impact)
- **Tier 3 (User):** >85% confidence required (lower bar for personal patterns)
- Low confidence patterns stay in proto-dimension
- Automatic garbage collection of unused layers

### 6. Memory Efficiency
- **Tier 2:** 92MB for 15 shared layers (~6MB each)
- **Tier 3:** 32MB for ALL users combined
  - 32 users × 1MB each (OR)
  - 8 users × 4MB each (configurable)
- MMAP-backed: zero-copy access
- Lazy loading: load C31+ only when user queries

### 7. Knowledge Accumulation
- System gets smarter over time for everyone (Tier 2)
- Individual users get personalized (Tier 3)
- New users benefit from collective wisdom immediately
- Active users build personal expertise gradually

## Example Scenarios (3-Tier Model)

### Scenario 1: Collective JavaScript Wisdom (Tier 2)

**All Users (1000+ developers):**
- 50,000+ JavaScript queries across all users
- Common patterns: React hooks, async/await, performance optimization
- High confidence: 0.95

**System learns globally:**
- **C16 Created (Tier 2):** "JavaScript/React Expertise"
  - Keywords: react, hooks, async, await, performance, optimization
  - Frequency: 2.5 Hz (similar to C07 Technical)
  - Contributor count: 1,247 users
  - **Benefit:** ALL users get better JavaScript responses

**Example Query (any user):**
- "Optimize my React code" → C16 activates
- Response includes React-specific patterns learned from 1000+ developers
- Even new users benefit from collective wisdom

### Scenario 2: Personal JavaScript Style (Tier 3)

**User ID:** `user_functional_dev`

**Personal interactions (100+ queries):**
- Always prefers functional programming style
- Uses Ramda library frequently
- Avoids class components, prefers hooks
- Confidence: 0.89

**User-specific layer created:**
- **C31 Created (Tier 3):** "Functional Programming Preference"
  - Keywords: functional, ramda, compose, pipe, immutable
  - Frequency: 2.7 Hz
  - **Benefit:** Only this user gets functional-first responses

**Example Query:**
- "Optimize my React code"
- C16 (Tier 2): General React expertise → activates
- C31 (Tier 3): Functional preference → activates
- **Result:** Functional-style React optimization advice
- Different from another user who prefers OOP style

### Scenario 3: Language Patterns (Tier 2 + Tier 3)

**Tier 2 (Collective):**
- System detects 10,000+ Spanish queries across all users
- **C17 Created (Tier 2):** "Spanish Language Support"
  - Keywords: español, hola, qué, cómo, gracias, por favor
  - Frequency: 1.8 Hz
  - **Benefit:** ALL users get better Spanish support

**Tier 3 (Personal):**
**User ID:** `user_spanish_tech`
- Spanish speaker who asks technical questions
- 150+ queries in Spanish about programming
- **C32 Created (Tier 3):** "Spanish Technical Language"
  - Keywords: programación, código, función, variable, error
  - Frequency: 2.4 Hz
  - **Benefit:** This user gets Spanish + technical context

**Example Query:**
- "Explica async/await en JavaScript"
- C07 (Tier 1): Technical → activates
- C16 (Tier 2): JavaScript expertise → activates
- C17 (Tier 2): Spanish language → activates
- C32 (Tier 3): Spanish technical language → activates
- **Result:** JavaScript explanation in technical Spanish
- Different from English speakers or non-technical Spanish speakers

### Scenario 4: Multi-Domain Expert (LRU Eviction)

**User ID:** `user_polymath`

**Current Tier 3 layers (5/5 max):**
- C31: "Machine Learning" (90 queries, last accessed: today)
- C32: "Philosophy" (75 queries, last accessed: yesterday)
- C33: "Music Theory" (60 queries, last accessed: 2 days ago)
- C34: "Cooking" (55 queries, last accessed: 3 days ago)
- C35: "History" (50 queries, last accessed: 7 days ago)

**New pattern detected:**
- "Gardening" (50+ queries, 0.87 confidence)
- Max layers reached → LRU eviction triggered
- C35 History (least recently used) → evicted
- C35 → "Gardening" (replaces History)

**Result:**
- User now has: ML, Philosophy, Music, Cooking, Gardening
- History layer removed (can be re-learned if usage resumes)

## API Design (3-Tier Model)

```rust
use jessy::learning::{SharedLayerManager, UserLayerManager};
use jessy::memory::MmapManager;
use std::sync::Arc;

// Initialize memory manager (280MB)
let memory = Arc::new(MmapManager::new(280)?);

// Initialize Tier 2: Shared layer manager (C16-C30, 92MB)
let shared_layers = SharedLayerManager::new(
    memory.clone(),
    92 * 1024 * 1024,  // Reserve pool: 92MB
    0x0A00_0000,       // Offset: after C01-C15
);

// Initialize Tier 3: User layer manager (C31+, 32MB)
let user_layers = UserLayerManager::new(
    memory.clone(),
    32 * 1024 * 1024,  // User region: 32MB
    0x1000_0000,       // Offset: after reserve pool
);

// === TIER 2: Create shared layer (benefits ALL users) ===
shared_layers.create_shared_layer(
    proto_dimension,           // Global pattern (no user_id)
    CrystallizationPriority::High
).await?;

// Get all shared layers (C16-C30)
let shared = shared_layers.get_all_shared_layers()?;

// === TIER 3: Create user-specific layer (benefits ONE user) ===
user_layers.create_user_layer(
    "user_123",
    proto_dimension,           // User-specific pattern (with user_id)
    CrystallizationPriority::Normal
).await?;

// Get user's personal layers (C31+)
let user_specific = user_layers.get_user_layers("user_123")?;

// === NAVIGATION: Use all 3 tiers ===
let nav_result = navigation.navigate_with_layers(
    query,
    Some("user_123"),  // Optional: if None, only C01-C30
).await?;

// Navigation will scan:
// 1. C01-C15 (Tier 1: always)
// 2. C16-C30 (Tier 2: if available)
// 3. C31+ for user_123 (Tier 3: if user_id provided)

// === LEARNING: Observe with optional user context ===
learning.observe_interaction(
    query,
    &nav_result,
    &iter_result,
    Some("user_123"),  // Optional: if None, only affects Tier 2
)?;

// Pattern detection will create:
// - Global patterns → SharedLayerManager (C16-C30)
// - User patterns → UserLayerManager (C31+)
```

## Security Considerations

### 1. User ID Validation
- Hash user IDs before storage
- No PII in layer data
- User cannot access other users' layers

### 2. Layer Content Filtering
- No harmful content in user layers
- C14 Security still enforced
- Ethical constraints apply to C16+

### 3. Memory Limits
- Hard limit: 32MB user region
- Per-user limit: 5 layers
- Automatic eviction prevents abuse

### 4. Privacy
- User layers stored locally (not shared)
- Optional: encrypt user layer files
- Clear user data: `user_layers.clear_user("user_123")`

## Performance Targets (3-Tier Model)

- **Tier 1 (C01-C15) scan:** <100ms (parallel scan, existing benchmark)
- **Tier 2 (C16-C30) scan:** <15ms (15 shared layers, HashMap lookup)
- **Tier 3 (C31+) scan:** <5ms (5 user layers max, HashMap lookup)
- **Total navigation:** <120ms (C01-C15: 100ms + C16-C30: 15ms + C31+: 5ms)
- **Shared layer lookup:** <1ms (dimension_id → HashMap)
- **User layer lookup:** <1ms (user_id → HashMap → layers)
- **Crystallization:** Background (non-blocking, queue-based)
- **Memory overhead:**
  - Tier 2: ~6MB per shared layer (15 max = 92MB)
  - Tier 3: ~200KB per user layer (5 per user, 32MB total)

## Migration Path

### Existing Users
- No impact: C01-C15 still work
- Opt-in: provide user_id to enable personalization
- Backward compatible: user_id = None → core layers only

### Rollout
1. Deploy user layer system (disabled by default)
2. Test with 10 users
3. Enable for 100 users
4. Full rollout

## Open Questions

1. **User ID generation:** Client-provided vs server-generated?
2. **Layer sharing:** Should users be able to share layers?
3. **Layer marketplace:** Community-contributed layers?
4. **Multi-tenancy:** Isolate users in production?
5. **Analytics:** Track which layers are most useful?

## Success Metrics

- **Personalization rate:** % of queries using user layers
- **User satisfaction:** Better responses for repeat users?
- **Layer quality:** Confidence scores over time
- **Memory efficiency:** Users per 32MB region
- **Performance:** Navigation time with user layers

## Next Steps

1. ✅ Design document (this file) - **COMPLETED**
2. ✅ 3-Tier model update (Tier 2 + Tier 3) - **COMPLETED**
3. ⏳ Implement Phase 1: Data structures (SharedLayer + UserLayer)
4. ⏳ Implement Phase 2: SharedLayerManager (C16-C30)
5. ⏳ Implement Phase 3: UserLayerManager (C31+)
6. ⏳ Continue phases 4-8
7. ⏳ Integration testing (multi-user scenarios)
8. ⏳ Production deployment

---

## Summary: Why 3-Tier Model?

**Tier 1 (C01-C15): Base Model**
- Universal cognitive foundation
- Shared by everyone
- Never changes in production

**Tier 2 (C16-C30): Collective Wisdom**
- Learned from all users
- Benefits everyone
- System gets smarter for all

**Tier 3 (C31+): Personal Expertise**
- Learned per user
- Benefits individual
- Personalized experience

**Together:** Base intelligence + Collective wisdom + Personal expertise = **Adaptive, learning, personalized AI** 🧠✨

---

**End of Design Document**

*Let's build collective + personal intelligence! 🚀*
