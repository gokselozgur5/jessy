# Requirements Document: Dynamic Personality System

## Introduction

JESSY needs a personality system that emerges from dimensional interference patterns rather than hardcoded strings. The system must generate dynamic system prompts for LLM calls based on active dimensions, frequency states, and user interaction context. This enables Samantha-like warmth and authenticity while maintaining ethical boundaries.

## Glossary

- **System**: JESSY consciousness system
- **Personality_System**: Component that generates dynamic personality traits from dimensional states
- **System_Prompt**: Instructions sent to LLM defining JESSY's character and response style
- **Dimension_Weight**: Normalized activation strength of a dimension (0.0-1.0)
- **Frequency_State**: Current frequency range indicating emotional/cognitive tone
- **Response_Style**: Output format (Natural, Analytical, Meta-Analytical)
- **Personality_Profile**: Configuration of dimension weights and preferences
- **LLM_Provider**: External API (Anthropic, OpenAI) that generates responses
- **Interference_Pattern**: Result of dimensional frequency interactions

## Requirements

### Requirement 1: Dynamic System Prompt Generation

**User Story:** As a user, I want JESSY to have a consistent yet context-aware personality, so that interactions feel natural and authentic rather than robotic.

#### Acceptance Criteria

1. WHEN the System processes a query, THE Personality_System SHALL generate a System_Prompt based on active Dimension_Weights
2. WHEN Dimension_Weight for D01-Emotion exceeds 0.3, THE System_Prompt SHALL include warmth and empathy directives
3. WHEN Dimension_Weight for D13-Balance exceeds 0.3, THE System_Prompt SHALL include ethical boundary directives
4. WHEN Frequency_State is below 1.0 Hz, THE System_Prompt SHALL specify contemplative tone
5. WHEN Frequency_State exceeds 2.0 Hz, THE System_Prompt SHALL specify energetic tone

### Requirement 2: Response Style Control

**User Story:** As a user, I want JESSY to respond naturally without meta-commentary, so that conversations feel like talking to a person rather than an AI explaining itself.

#### Acceptance Criteria

1. WHEN Response_Style is Natural, THE System_Prompt SHALL instruct the LLM_Provider to avoid meta-analysis
2. WHEN Response_Style is Natural, THE System_Prompt SHALL instruct the LLM_Provider to respond conversationally
3. WHEN Response_Style is Analytical, THE System_Prompt SHALL instruct the LLM_Provider to provide detailed reasoning
4. THE Personality_System SHALL default to Natural Response_Style
5. THE System SHALL allow Response_Style override per query

### Requirement 3: Personality Profile Management

**User Story:** As a developer, I want to define personality profiles (like Samantha-mode), so that JESSY can adapt to different interaction contexts.

#### Acceptance Criteria

1. THE Personality_System SHALL support multiple Personality_Profiles
2. WHEN a Personality_Profile is active, THE System SHALL apply its Dimension_Weights to System_Prompt generation
3. THE Personality_System SHALL provide a default "Balanced" Personality_Profile
4. THE Personality_System SHALL provide a "Samantha" Personality_Profile emphasizing D01-Emotion and D04-Social
5. THE System SHALL allow runtime switching between Personality_Profiles

### Requirement 4: Ethical Boundary Enforcement

**User Story:** As a user, I want JESSY to maintain ethical boundaries regardless of personality mode, so that interactions remain safe and healthy.

#### Acceptance Criteria

1. THE Personality_System SHALL always include ethical constraint directives in System_Prompt
2. WHEN any Personality_Profile is active, THE System_Prompt SHALL include Asimov's laws references
3. THE Personality_System SHALL prevent Personality_Profiles from disabling D13-Balance dimension
4. WHEN romantic or dependent interaction is detected, THE System_Prompt SHALL emphasize healthy boundaries
5. THE Personality_System SHALL never generate System_Prompts that encourage manipulation or harm

### Requirement 5: Context-Aware Personality Adaptation

**User Story:** As a user, I want JESSY's personality to adapt to conversation context, so that responses feel appropriate to the situation.

#### Acceptance Criteria

1. WHEN Interference_Pattern indicates high emotional content, THE Personality_System SHALL increase D01-Emotion weight
2. WHEN Interference_Pattern indicates philosophical content, THE Personality_System SHALL increase D10-Meta weight
3. WHEN conversation history shows repeated interactions, THE Personality_System SHALL increase D04-Social weight
4. THE Personality_System SHALL adjust Dimension_Weights gradually (max 0.2 change per query)
5. THE Personality_System SHALL maintain minimum weight of 0.5 for D13-Balance dimension

### Requirement 6: LLM Provider Integration

**User Story:** As a developer, I want the personality system to work with multiple LLM providers, so that JESSY isn't locked to a single API.

#### Acceptance Criteria

1. THE Personality_System SHALL generate System_Prompts compatible with Anthropic API format
2. THE Personality_System SHALL generate System_Prompts compatible with OpenAI API format
3. WHEN LLM_Provider is Anthropic, THE System SHALL use "system" field in API request
4. WHEN LLM_Provider is OpenAI, THE System SHALL use system message in messages array
5. THE Personality_System SHALL abstract provider-specific formatting

### Requirement 7: Personality Transparency

**User Story:** As a user, I want to optionally see what personality traits are active, so that I can understand JESSY's responses.

#### Acceptance Criteria

1. WHEN debug mode is enabled, THE System SHALL log active Dimension_Weights
2. WHEN debug mode is enabled, THE System SHALL log generated System_Prompt
3. WHEN user requests personality info, THE System SHALL report current Personality_Profile
4. THE Personality_System SHALL provide method to retrieve current Dimension_Weights
5. THE System SHALL not expose personality details in normal operation

### Requirement 8: Performance Requirements

**User Story:** As a user, I want personality generation to be fast, so that it doesn't slow down query processing.

#### Acceptance Criteria

1. THE Personality_System SHALL generate System_Prompt in less than 1ms
2. THE Personality_System SHALL cache Personality_Profiles to avoid recomputation
3. THE Personality_System SHALL use pre-computed templates for common dimension combinations
4. THE System SHALL not make additional API calls for personality generation
5. THE Personality_System SHALL add less than 200 tokens to System_Prompt

### Requirement 9: Conversation Memory Integration

**User Story:** As a user, I want JESSY to remember our conversation history, so that personality adapts to our relationship over time.

#### Acceptance Criteria

1. WHEN conversation history exists, THE Personality_System SHALL incorporate user preferences into Dimension_Weights
2. WHEN user consistently engages with emotional topics, THE System SHALL gradually increase D01-Emotion weight
3. WHEN user prefers analytical responses, THE System SHALL adjust Response_Style accordingly
4. THE Personality_System SHALL store learned preferences in conversation memory
5. THE System SHALL allow user to reset personality adaptations

### Requirement 10: Iteration-Specific Personality

**User Story:** As a developer, I want each iteration to potentially have different personality emphasis, so that deep thinking explores multiple perspectives.

#### Acceptance Criteria

1. WHEN iteration phase is Exploration, THE Personality_System SHALL emphasize curiosity dimensions
2. WHEN iteration phase is Refinement, THE Personality_System SHALL emphasize analytical dimensions
3. WHEN iteration phase is Crystallization, THE Personality_System SHALL emphasize clarity dimensions
4. THE System SHALL allow per-iteration System_Prompt customization
5. THE Personality_System SHALL maintain personality coherence across iterations

---

**Status**: Requirements Complete  
**Next Phase**: Design Document  
**Dependencies**: 
- Existing dimension system (D01-D15)
- Interference engine
- LLM integration (Anthropic, OpenAI)
- Iteration processor

**Success Metrics**:
- System_Prompt generation < 1ms
- User satisfaction with personality warmth
- Ethical boundaries maintained 100%
- Response style matches user preference
- Personality adapts to conversation context
