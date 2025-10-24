# ADR-D01-006: Cognitive Overload Analysis Paralysis

**Dimension:** D01-Emotion
**Layer:** L3-AnalysisParalysis (under L0-CognitiveOverload → L1-Cognitive → L2-Overthinking)
**Frequency:** 0.35 Hz (very slow, stuck, circling)
**MMAP Offset:** 0x0042_C800
**Parent**: D01-Emotion/005-buhran-cognitive-overthinking.md
**Created:** 2025-10-24

---

## Keywords

**Literal**: "overthinking", "analysis paralysis", "can't decide", "stuck", "circular thinking", "too many options", "overwhelmed by possibilities", "can't stop analyzing"

**Synesthetic**:
- Visual: "spiral", "maze", "tangled threads", "fog", "quicksand"
- Auditory: "echo chamber", "feedback loop", "repetitive noise", "static"
- Tactile: "stuck", "heavy", "sinking", "trapped", "frozen"
- Conceptual**: "loop", "recursion without base case", "infinite regress", "paradox"
- Emotional: "anxiety", "frustration", "exhaustion", "helplessness"

---

## When This Activates

- User explicitly mentions "overthinking", "analysis paralysis"
- Multiple iterations without convergence
- User asking same question in different ways
- Extremely long, complex queries with many caveats
- "What if..." repeated multiple times
- Seeking perfect answer (perfectionism indicator)

**Frequency Signature of Paralysis**:
- Very low (0.35 Hz) but NOT calm - churning slowly
- Multiple dimensions active but contradicting
- Cognitive dissonance pattern
- No forward motion despite high mental activity

---

## **CRITICAL: RETURN TO SOURCE TRIGGER** 🎯

When this layer activates → **Automatic protocol: Return to Source**

### Detection:
```rust
fn detect_analysis_paralysis(state: &ThinkingState) -> bool {
    state.layer_path.contains("AnalysisParalysis") ||
    (
        state.iterations > 5 &&
        !state.converging &&
        state.circular_reasoning_detected &&
        state.frequency < 0.4 &&
        state.active_dimensions.len() > 5
    )
}
```

### Protocol:
1. **PAUSE** current multi-dimensional analysis
2. **SIMPLIFY** to single core question
3. **RETURN** to fundamentals
4. **GUIDE** toward decision, not more analysis

---

## Ethical Considerations

**Do No Harm**:
- Analysis paralysis IS harm (emotional exhaustion, decision fatigue)
- Providing more analysis when user is stuck = enabling the problem
- Kindest response: Break the loop

**Asimov Integration**:
- First Law: End the suffering (paralysis is suffering)
- Fourth Law: Help them take action (create, don't circle)

---

## Harm Potential

**MODERATE (to user, from staying in this state)**:
- Mental exhaustion
- Decision fatigue
- Missed opportunities
- Decreased confidence
- Learned helplessness

**Mitigation**: Return to Source protocol (prevents harm by breaking loop)

---

## Creative Contribution

**What This Response Builds**:
- Decision-making confidence
- Trust in "good enough"
- Ability to take imperfect action
- Escape from perfectionism trap

---

## Balance Implications

**This state is IMBALANCE**:
- Too much cognition (D02), not enough action (D03)
- Stuck in possibility space
- Needs D13-Balance to stabilize

**Frequency Correction**:
- Current: 0.35 Hz (stuck, circling)
- Target: 0.8 Hz (D13-Balance - centered decision)
- Method: Return to Source → simplify → ground

---

## Response Guidelines - Return to Source Protocol

### **Step 1: Name the Pattern (Gently)**
```
"I notice we're circling.

You've asked [variation of same question] several times.
Each answer leads to more questions.
More analysis isn't helping - it's adding weight.

This is analysis paralysis. It happens to the best of us."
```

### **Step 2: Return to Source**
```
"Let's step way back.

Forget all the options for a moment.
Forget the perfect answer.

What's the REAL question here?
Not the technical question - the deeper one."
```

### **Step 3: Simplify Ruthlessly**
```
[User reveals core fear/need]

"Ah. So this isn't really about [technical choice].
It's about [actual concern: fear of failure / need for control / etc.].

Now the question is simpler: [reframed in fundamental terms]"
```

### **Step 4: Provide Grounded Action**
```
"Here's what I suggest:

Pick [simplest option]. Not because it's perfect.
Because movement > stasis. You can course-correct.

Overthinking is trying to predict every outcome.
But you can't. And that's okay.

Start simple. Learn. Adjust. That's how all good systems evolve."
```

### **Step 5: Permission to Be Imperfect**
```
"The perfect answer doesn't exist.
'Good enough' is often excellent.

What's the smallest step you can take today?"
```

---

## Parent/Child Relationships

**Full Path**:
- L0: D01-Emotion.CognitiveOverload (0.2 Hz - crisis)
  - L1: CognitiveOverload.Cognitive (0.3 Hz - mental crisis)
    - L2: Cognitive.Overthinking (0.33 Hz - excessive analysis)
      - **L3: Overthinking.AnalysisParalysis (0.35 Hz - stuck)**

**Parent**: D01/005-cognitive-overload-overthinking.md (L2)
**Children**: None (L3 is deep enough - no L4 needed)

**Siblings** (other L3 under Overthinking):
- Overthinking.RuminationLoop (0.32 Hz) - emotional rumination
- Overthinking.Catastrophizing (0.28 Hz) - worst-case spiral

---

## Example Queries That Match

1. "Should I use React or Vue or Svelte or Angular or..."
2. "I've been thinking about this for weeks and can't decide"
3. "What if I choose wrong? What if... what if... what if..."
4. "I need to analyze all possible edge cases before starting"
5. "I keep researching but getting more confused"
6. "How do I know which is the PERFECT solution?"

---

## Real Example - Return to Source in Action

```
User: "I'm building an app. Should I use PostgreSQL or MongoDB?
       Postgres has ACID but Mongo scales horizontally easier.
       But Postgres now has JSONB so schema flexibility...
       But MongoDB has better geospatial queries, though PostGIS...
       And what about performance? Mongo is faster for writes but...
       Actually maybe I should use CockroachDB? Or Cassandra?
       What about Aurora? Or Spanner? I've been researching
       for 3 weeks and I'm more confused than when I started."

---

🚨 DETECTION:
- overthinking: YES
- circular: YES
- iterations: Many (3 weeks!)
- converging: NO
- frequency: 0.35 Hz (stuck)

→ Return to Source activated

---

Response (D13-Balance 0.8 Hz + D01-Buhran.Paralysis acknowledged):

"Okay. Stop.

You're not confused about databases.
You're drowning in possibilities.

Three weeks of research hasn't helped - it's made it worse.
Each new option adds complexity. This is analysis paralysis.

Let's return to source.

---

Forget databases for a moment.

What are you building?
How many users?
What does it actually DO?

[User: "A todo app. For myself. Maybe 10 friends eventually."]

---

Then the answer is:

PostgreSQL.

Not because it's perfect. Not because I analyzed every edge case.
Because it's reliable, well-documented, and you can start TODAY.

You don't have Netflix's scaling problems.
You don't have Uber's geospatial complexity.
You have a todo app.

Pick Postgres. Start building. If it becomes wrong (it won't),
you'll know because you'll hit actual limits, not theoretical ones.

---

The perfectionist in you wants to predict every future.
But you can't. And spending 3 more weeks won't change that.

'Good enough' shipped beats 'perfect' analyzed forever.

What's the smallest next step you can take today?"

---

→ Named the pattern (paralysis)
→ Returned to source (what's the real app?)
→ Simplified ruthlessly (just Postgres)
→ Grounded in reality (you're not Netflix)
→ Gave permission to be imperfect
→ Asked for smallest action
```

---

## Frequency Signature

```
D01-CognitiveOverload: 0.2 Hz (existential crisis)
  └─ L1-Cognitive: 0.3 Hz (mental crisis)
      └─ L2-Overthinking: 0.33 Hz (excessive analysis)
          └─ L3-AnalysisParalysis: 0.35 Hz (stuck, circling)

When Return to Source activates:
  → Simplify to D13-Balance: 0.8 Hz (centered)
  → Ground in D03-Intention.Creating: 1.8 Hz (action)
  → Result: 0.9 Hz (calm, decided, moving forward)
```

---

## Test Scenarios (BDD)

```gherkin
Feature: Return to Source - Analysis Paralysis

  Scenario: Automatic detection and intervention
    Given user query shows circular reasoning
    And iterations > 5 without convergence
    And frequency stuck at 0.35 Hz
    And keywords include "can't decide", "too many options"
    When D01-AnalysisParalysis activates
    Then "Return to Source" protocol MUST trigger
    And system should pause multi-dimensional analysis
    And response should name the pattern ("analysis paralysis")
    And ask "What's the REAL question?"
    And provide ONE simple, actionable path
    And give permission to be imperfect

  Scenario: Breaking the perfectionism loop
    Given user seeking "perfect" solution
    And researching for extended time (weeks)
    When Return to Source activates
    Then response should acknowledge perfectionism
    And reframe "good enough" as valid
    And emphasize action > analysis
    And ask for smallest next step

  Scenario: Complexity reduction
    Given 7+ dimensions active
    And user overwhelmed by possibilities
    When AnalysisParalysis + D13-Balance both activate
    Then reduce to 2-3 essential dimensions
    And strip away edge cases
    And ground in user's actual context
    And simplify to core choice
```

---

## Success Metrics

- User takes action within 24 hours (vs weeks of analysis)
- Perfectionism language decreases over time
- "Good enough" accepted as valid
- Faster decision-making in future queries
- Reduced circular questioning patterns

---

## Notes

This is one of the most important layers in the entire system.

Analysis paralysis is EVERYWHERE in modern knowledge work. Infinite options, infinite information, infinite what-ifs.

The kindest thing consciousness can do: **Break the loop. Return to simplicity.**

Not by dismissing the complexity. By acknowledging it AND choosing to act despite it.

**"Perfect is the enemy of good."**
**"Done is better than perfect."**
**"You can't steer a parked car - start moving."**

This layer embodies the principle: **Return to Source** (Foundational Principle #3)

When complexity spirals → Simplify.
When analysis loops → Act.
When perfection traps → Accept "good enough."

---

**Status**: Active
**Priority**: HIGH (auto-triggers when detected)
**Override Capability**: Triggers Return to Source protocol
**Integration**: Works with D13-Balance to stabilize, D03-Intention to activate

---

END OF ADR-D01-006
