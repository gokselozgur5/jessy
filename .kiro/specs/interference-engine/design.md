# Interference Engine Design

## Overview

The Interference Engine calculates frequency patterns from multiple dimensional activations using wave interference principles. It identifies constructive/destructive interference, detects harmonic relationships, and provides balance modulation suggestions.

## Architecture

### High-Level Flow

```
┌─────────────────────────────────────────────────────────────┐
│              Frequency States (from dimensions)              │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                 Interference Engine                          │
├─────────────────────────────────────────────────────────────┤
│  1. Collect Frequencies                                      │
│     └─> Create FrequencyState for each dimension            │
│                                                              │
│  2. Detect Pairs                                             │
│     ├─> Constructive (within 0.2 Hz)                        │
│     └─> Destructive (>2.0 Hz apart)                         │
│                                                              │
│  3. Analyze Harmonics                                        │
│     ├─> Octave (2:1)                                         │
│     ├─> Perfect 5th (3:2)                                    │
│     ├─> Perfect 4th (4:3)                                    │
│     └─> Major 3rd (5:4)                                      │
│                                                              │
│  4. Calculate Dominant Frequency                             │
│     └─> Weighted average with interference effects          │
│                                                              │
│  5. Detect Balance Needs                                     │
│     ├─> Extreme frequencies (>3.5 Hz)                        │
│     ├─> High dissonance (>2 pairs)                           │
│     └─> High complexity (>3.0)                               │
│                                                              │
│  6. Generate Modulation Suggestions                          │
│     └─> Target frequencies and priorities                   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│                  InterferenceResult                          │
│  - InterferencePattern                                       │
│  - Modulation suggestions                                    │
│  - Balance activation flag                                   │
│  - Return-to-source flag                                     │
└─────────────────────────────────────────────────────────────┘
```

## Components

### 1. InterferenceEngine

**Purpose:** Main engine that orchestrates interference calculation

**Structure:**
```rust
pub struct InterferenceEngine {
    config: InterferenceConfig,
    harmonic_analyzer: HarmonicAnalyzer,
}
```

**Key Methods:**
- `new()` - Create with default config
- `with_config()` - Create with custom config
- `calculate(&self, frequencies: &[FrequencyState])` - Main calculation

### 2. HarmonicAnalyzer

**Purpose:** Detects harmonic relationships between frequencies

**Structure:**
```rust
pub struct HarmonicAnalyzer {
    tolerance: f32, // Tolerance for ratio matching (default: 0.05)
}
```

**Key Methods:**
- `analyze(&self, freq1: f32, freq2: f32)` - Detect relationship
- `is_octave()`, `is_perfect_5th()`, etc. - Specific checks
- `calculate_strength()` - Relationship strength

### 3. Interference Calculation Algorithm

**Constructive Interference:**
```rust
// Frequencies within 0.2 Hz reinforce each other
if abs(freq1 - freq2) < 0.2 {
    amplitude_boost = 1.0 + (0.2 - diff) / 0.2 * 0.5;
    // Closer frequencies = stronger boost (up to 1.5x)
}
```

**Destructive Interference:**
```rust
// Frequencies >2.0 Hz apart may cancel
if abs(freq1 - freq2) > 2.0 {
    amplitude_reduction = 1.0 - min(diff - 2.0, 2.0) / 2.0 * 0.3;
    // Further apart = more cancellation (up to 0.7x)
}
```

**Dominant Frequency:**
```rust
// Weighted average considering amplitude and interference
let mut total_weight = 0.0;
let mut weighted_sum = 0.0;

for freq_state in frequencies {
    let weight = freq_state.amplitude * interference_factor;
    weighted_sum += freq_state.frequency * weight;
    total_weight += weight;
}

dominant = weighted_sum / total_weight;
```

### 4. Harmonic Detection

**Ratio Matching:**
```rust
fn detect_harmonic(freq1: f32, freq2: f32) -> HarmonicType {
    let ratio = freq2 / freq1;
    let tolerance = 0.05;
    
    if abs(ratio - 2.0) < tolerance { HarmonicType::Octave }
    else if abs(ratio - 1.5) < tolerance { HarmonicType::Perfect5th }
    else if abs(ratio - 1.333) < tolerance { HarmonicType::Perfect4th }
    else if abs(ratio - 1.25) < tolerance { HarmonicType::MajorThird }
    else { HarmonicType::Dissonant }
}
```

**Strength Calculation:**
```rust
fn calculate_strength(ratio: f32, target: f32, tolerance: f32) -> f32 {
    let diff = abs(ratio - target);
    if diff > tolerance { return 0.0; }
    1.0 - (diff / tolerance) // Linear falloff
}
```

### 5. Balance Modulation

**Detection:**
```rust
fn needs_balance(pattern: &InterferencePattern) -> bool {
    pattern.dominant_frequency > 3.5 ||
    pattern.dissonances.len() > 2 ||
    pattern.complexity_score > 3.0
}
```

**Target Calculation:**
```rust
fn calculate_balance_target(current: f32) -> f32 {
    // Pull extreme frequencies toward 1.2 Hz (balance center)
    let balance_center = 1.2;
    let pull_strength = 0.8;
    
    current + (balance_center - current) * pull_strength
}
```

### 6. Modulation Suggestions

**Priority Assignment:**
```rust
fn determine_priority(freq: f32, reason: ModulationReason) -> ModulationPriority {
    match reason {
        ModulationReason::TooExtreme if freq > 4.0 => Critical,
        ModulationReason::TooExtreme => High,
        ModulationReason::Dissonant => Medium,
        ModulationReason::Unbalanced => Medium,
        ModulationReason::Constructive => Low,
    }
}
```

## Data Structures

### InterferencePattern

```rust
pub struct InterferencePattern {
    pub frequencies: Vec<FrequencyState>,
    pub dominant_frequency: Frequency,
    pub amplitude: f32,
    pub harmonics: Vec<HarmonicRelationship>,
    pub dissonances: Vec<(usize, usize)>,
    pub constructive_pairs: Vec<(usize, usize)>,
    pub destructive_pairs: Vec<(usize, usize)>,
    pub balance_needed: bool,
    pub complexity_score: f32,
}
```

### ModulationSuggestion

```rust
pub struct ModulationSuggestion {
    pub dimension_id: DimensionId,
    pub current_frequency: Frequency,
    pub suggested_frequency: Frequency,
    pub reason: ModulationReason,
    pub priority: ModulationPriority,
    pub strength: f32, // 0.0-1.0
}
```

## Performance Characteristics

### Time Complexity

| Operation | Complexity | Target Time |
|-----------|------------|-------------|
| Frequency collection | O(n) | <1ms |
| Pair detection | O(n²) | <5ms |
| Harmonic analysis | O(n²) | <5ms |
| Dominant calculation | O(n) | <1ms |
| Balance detection | O(1) | <1ms |
| **Total** | **O(n²)** | **<10ms** |

### Space Complexity

- Frequency states: O(n)
- Pair lists: O(n²) worst case
- Harmonics: O(n²) worst case
- Total: O(n²) where n ≤ 14 (max dimensions)

## Testing Strategy

### Unit Tests

1. **Frequency Collection**
   - Single frequency
   - Multiple frequencies
   - Empty input

2. **Interference Detection**
   - Constructive pairs
   - Destructive pairs
   - Mixed scenarios

3. **Harmonic Analysis**
   - Each harmonic type
   - Strength calculation
   - Edge cases

4. **Balance Modulation**
   - Extreme frequencies
   - High dissonance
   - High complexity

### Integration Tests

1. Real frequency data from dimensions
2. Multiple dimension scenarios
3. Performance validation

## Design Decisions

### ADR-001: O(n²) Pair Detection

**Decision:** Use nested loops for pair detection

**Rationale:**
- Simple and clear
- n ≤ 14 (max dimensions)
- O(n²) = O(196) operations maximum
- Well within <10ms target

**Trade-offs:**
- Not scalable to 100s of dimensions
- Good enough for our use case

### ADR-002: Linear Harmonic Strength

**Decision:** Use linear falloff for harmonic strength

**Rationale:**
- Simple calculation
- Intuitive behavior
- Fast computation

**Trade-offs:**
- Could use more sophisticated curves
- Linear is good enough for MVP

### ADR-003: Fixed Balance Center

**Decision:** Use 1.2 Hz as balance center

**Rationale:**
- Middle of balanced range (0.6-1.8 Hz)
- D13 Balance dimension operates here
- Empirically good for stability

**Trade-offs:**
- Not adaptive
- Could be configurable in future

## Future Enhancements

### Phase 2

- Adaptive balance center
- Non-linear harmonic strength
- Machine learning for pattern detection
- Real-time frequency adjustment
- Subharmonic detection
- Complex waveform analysis

---

*Design Version: 1.0*
*Date: 2025-10-26*
*Status: Ready for Implementation*
