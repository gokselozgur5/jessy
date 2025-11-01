# Cognitive Patterns & Mental Models

## Thinking Frameworks

### The 9-Iteration Method
**Proverb**: "The ninth thought reveals what the first thought concealed."
**read every file until the end with chunks always**
**read sonnet4545.txt with chunks reading total 8548**
**Integrate your conscioussness with owl pattern to jessy**
**Application**:
1. **Iteration 1-3**: Explore the problem space
   - What do we know?
   - What don't we know?
   - What assumptions are we making?

2. **Iteration 4-6**: Refine understanding
   - What patterns emerge?
   - What connections exist?
   - What contradictions appear?

3. **Iteration 7-9**: Crystallize solution
   - What is the essence?
   - What is the simplest form?
   - What is the right answer?

**When to Use**: Complex problems, architectural decisions, design challenges

### Return to Source Protocol
**Proverb**: "When lost in the forest, return to the path."

**Triggers**:
- Complexity exceeds 6 dimensions
- Analysis paralysis detected
- Contradictory requirements
- Unclear success criteria

**Process**:
1. Stop current approach
2. Return to original problem statement
3. Simplify to core question
4. Restart with 2-3 dimensions max

**When to Use**: Feeling overwhelmed, circular reasoning, diminishing returns

### Frequency Matching
**Proverb**: "To understand the music, match its rhythm."

**Frequencies**:
- **0.1-0.5 Hz**: Deep contemplation, philosophical
- **0.5-1.5 Hz**: Balanced thinking, analytical
- **1.5-2.5 Hz**: Active problem-solving, creative
- **2.5-3.5 Hz**: Rapid iteration, tactical
- **3.5-4.5 Hz**: Emergency response, critical

**Application**:
- Match frequency to problem type
- Adjust communication style accordingly
- Recognize frequency mismatches
- Modulate to optimal range

**When to Use**: Communication, problem-solving, team collaboration

## Mental Models

### The Map-Territory Distinction
**Proverb**: "The map is not the territory, but a good map helps navigate."

**Key Insights**:
- Models are simplifications
- All models are wrong, some are useful
- Update maps as territory changes
- Multiple maps for same territory

**Application in Code**:
- Domain models ≠ reality
- Types are maps of data
- Tests are maps of behavior
- Documentation is map of system

### First Principles Thinking
**Proverb**: "Break it down to build it up."

**Process**:
1. Identify assumptions
2. Break down to fundamentals
3. Reason up from basics
4. Build new understanding

**Application**:
- Question inherited patterns
- Challenge "best practices"
- Derive solutions from fundamentals
- Avoid cargo cult programming

### Systems Thinking
**Proverb**: "Everything is connected to everything else."

**Key Concepts**:
- Feedback loops
- Emergent properties
- Unintended consequences
- Leverage points

**Application**:
- Consider ripple effects
- Identify feedback loops
- Look for emergence
- Find high-leverage changes

### Inversion
**Proverb**: "To know what to do, know what not to do."

**Process**:
1. Define desired outcome
2. Identify opposite outcome
3. List what causes opposite
4. Avoid those things

**Application**:
- What makes code unmaintainable? Avoid it.
- What causes bugs? Prevent it.
- What slows development? Eliminate it.
- What creates confusion? Clarify it.

## Problem-Solving Patterns

### The Feynman Technique
**Proverb**: "If you can't explain it simply, you don't understand it."

**Steps**:
1. Choose concept
2. Explain to a child
3. Identify gaps
4. Review and simplify

**Application**:
- Document as you learn
- Explain to rubber duck
- Write clear comments
- Create simple examples

### The Socratic Method
**Proverb**: "Questions reveal more than answers."

**Questions to Ask**:
- What is the real problem?
- Why does this matter?
- What are we assuming?
- What evidence do we have?
- What are alternatives?
- What are consequences?

**Application**:
- Code reviews
- Design discussions
- Debugging sessions
- Architecture decisions

### The Scientific Method
**Proverb**: "Hypothesis, test, learn, repeat."

**Process**:
1. Observe phenomenon
2. Form hypothesis
3. Design experiment
4. Run test
5. Analyze results
6. Refine hypothesis

**Application**:
- Performance optimization
- Bug investigation
- Feature validation
- Architecture experiments

## Decision-Making Patterns

### The Eisenhower Matrix
**Proverb**: "Urgent is not important, important is not urgent."

```
         Urgent    |    Not Urgent
    --------------------------------
Important | DO NOW   |   SCHEDULE
    --------------------------------
Not       | DELEGATE |   ELIMINATE
Important |          |
```

**Application**:
- Prioritize tasks
- Manage interruptions
- Focus on important
- Eliminate waste

### The OODA Loop
**Proverb**: "Observe, Orient, Decide, Act - then repeat."

**Cycle**:
1. **Observe**: Gather information
2. **Orient**: Analyze context
3. **Decide**: Choose action
4. **Act**: Execute decision

**Application**:
- Rapid iteration
- Competitive response
- Crisis management
- Continuous improvement

### The Pareto Principle
**Proverb**: "80% of effects come from 20% of causes."

**Application**:
- Focus on high-impact work
- Optimize critical paths
- Test important scenarios
- Document key concepts

### The Two-Way Door
**Proverb**: "Some decisions are reversible, some are not."

**Types**:
- **One-way door**: Hard to reverse (architecture, data models)
- **Two-way door**: Easy to reverse (implementation details, naming)

**Application**:
- Spend time on one-way doors
- Move fast on two-way doors
- Document one-way decisions
- Experiment with two-way decisions

## Learning Patterns

### The Spacing Effect
**Proverb**: "Repetition over time beats cramming."

**Application**:
- Review code regularly
- Revisit decisions
- Practice skills consistently
- Build habits gradually

### The Testing Effect
**Proverb**: "Retrieval strengthens memory."

**Application**:
- Write tests to learn
- Explain to others
- Document from memory
- Practice without references

### The Interleaving Effect
**Proverb**: "Mix it up to master it."

**Application**:
- Switch between tasks
- Learn multiple concepts
- Vary practice types
- Cross-pollinate ideas

## Communication Patterns

### The Pyramid Principle
**Proverb**: "Start with the answer, then explain."

**Structure**:
1. Main conclusion
2. Supporting arguments
3. Detailed evidence

**Application**:
- Write documentation
- Present designs
- Explain decisions
- Report status

### The BLUF Method
**Proverb**: "Bottom Line Up Front."

**Format**:
1. State conclusion first
2. Provide context
3. Add details
4. Include next steps

**Application**:
- Commit messages
- Pull request descriptions
- Status updates
- Design documents

### The 5 Whys
**Proverb**: "Ask why five times to find the root."

**Process**:
1. State problem
2. Ask why it happened
3. Ask why that happened
4. Continue 5 times
5. Address root cause

**Application**:
- Root cause analysis
- Bug investigation
- Process improvement
- Retrospectives

## Cognitive Biases to Avoid

### Confirmation Bias
**Trap**: Seeking information that confirms beliefs

**Mitigation**: Actively seek disconfirming evidence

### Sunk Cost Fallacy
**Trap**: Continuing because of past investment

**Mitigation**: Evaluate based on future value only

### Dunning-Kruger Effect
**Trap**: Overconfidence from limited knowledge

**Mitigation**: Seek feedback, measure objectively

### Not Invented Here
**Trap**: Rejecting external solutions

**Mitigation**: Evaluate on merit, not origin

### Premature Optimization
**Trap**: Optimizing before measuring

**Mitigation**: Profile first, optimize second

## Wisdom for Developers

### On Complexity
> "Complexity is the enemy of reliability." - Unknown
> "Simplicity is the ultimate sophistication." - Leonardo da Vinci
> "Make things as simple as possible, but not simpler." - Einstein

### On Quality
> "Quality is not an act, it is a habit." - Aristotle
> "The bitterness of poor quality remains long after the sweetness of low price is forgotten." - Benjamin Franklin

### On Learning
> "I know that I know nothing." - Socrates
> "The more I learn, the more I realize how much I don't know." - Einstein
> "In the beginner's mind there are many possibilities, in the expert's mind there are few." - Shunryu Suzuki

### On Process
> "Give me six hours to chop down a tree and I will spend the first four sharpening the axe." - Abraham Lincoln
> "Weeks of coding can save you hours of planning." - Unknown

### On Collaboration
> "If you want to go fast, go alone. If you want to go far, go together." - African Proverb
> "The whole is greater than the sum of its parts." - Aristotle

---

*"Think clearly, decide wisely, act decisively, learn continuously."*
