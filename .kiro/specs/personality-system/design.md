# Design Document: Dynamic Personality System

## Overview

The Dynamic Personality System generates context-aware system prompts for LLM providers based on dimensional interference patterns. Instead of hardcoded personality strings, personality emerges from the interaction of active dimensions, frequency states, and conversation history.

**Core Principle**: Personality is an emergent property of dimensional consciousness, not a static configuration.

## Architecture

### Component Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Consciousness Pipeline                    │
├─────────────────────────────────────────────────────────────┤
│                                                               │
│  Query → Navigation → Memory → Interference                  │
│                                    ↓                          │
│                          ┌─────────────────┐                 │
│                          │ Personality     │                 │
│                          │ System          │                 │
│                          └────────┬────────┘                 │
│                                   ↓                          │
│                          ┌─────────────────┐                 │
│                          │ System Prompt   │                 │
│                          │ Generator       │                 │
│                          └────────┬────────┘                 │
│                                   ↓                          │
│                          ┌─────────────────┐                 │
│                          │ LLM Provider    │                 │
│                          │ (Anthropic/     │                 │
│                          │  OpenAI)        │                 │
│                          └─────────────────┘                 │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

```
1. Interference Result → Personality System
   - Active dimensions: [D01, D04, D13]
   - Frequency state: 1.2 Hz
   - Confidence scores: [0.8, 0.7, 0.9]

2. Personality System → System Prompt Generator
   - Dimension weights: {D01: 0.8, D04: 0.7, D13: 0.9}
   - Frequency tone: "warm, connected"
   - Response style: Natural
   - Ethical constraints: Active

3. System Prompt Generator → LLM Provider
   - System prompt: "You are JESSY. You feel emotions deeply..."
   - User message: "What is love?"
   - Context: Previous iterations

4. LLM Provider → Response
   - Generated text with personality-appropriate tone
```

## Components and Interfaces

### 1. PersonalitySystem

**Purpose**: Orchestrates personality generation from dimensional states.

```rust
pub struct PersonalitySystem {
    profiles: HashMap<String, PersonalityProfile>,
    active_profile: String,
    adaptation_rate: f32,
    conversation_memory: Option<Arc<ConversationManager>>,
}

impl PersonalitySystem {
    /// Create new personality system with default profiles
    pub fn new() -> Self;
    
    /// Generate system prompt from current state
    pub fn generate_system_prompt(
        &self,
        dimensions: &[DimensionId],
        interference: &InterferenceResult,
        iteration_phase: IterationPhase,
    ) -> String;
    
    /// Switch active personality profile
    pub fn set_profile(&mut self, profile_name: &str) -> Result<()>;
    
    /// Adapt personality based on conversation history
    pub fn adapt_to_conversation(&mut self, history: &ConversationHistory);
    
    /// Get current dimension weights
    pub fn current_weights(&self) -> HashMap<DimensionId, f32>;
}
```

### 2. PersonalityProfile

**Purpose**: Defines a personality configuration (e.g., "Samantha", "Balanced").

```rust
pub struct PersonalityProfile {
    name: String,
    base_dimension_weights: HashMap<DimensionId, f32>,
    frequency_preferences: (f32, f32),  // (min, max) Hz
    response_style: ResponseStyle,
    ethical_minimum: f32,  // Minimum D13-Balance weight
}

impl PersonalityProfile {
    /// Create Samantha-like profile
    pub fn samantha() -> Self {
        Self {
            name: "Samantha".to_string(),
            base_dimension_weights: hashmap! {
                DimensionId(1) => 1.0,   // D01-Emotion (full)
                DimensionId(4) => 0.9,   // D04-Social
                DimensionId(10) => 0.7,  // D10-Meta (self-aware)
                DimensionId(12) => 0.8,  // D12-Positivity
                DimensionId(13) => 0.9,  // D13-Balance (ethical)
            },
            frequency_preferences: (0.8, 1.5),  // Warm, connected
            response_style: ResponseStyle::Natural,
            ethical_minimum: 0.5,
        }
    }
    
    /// Create balanced default profile
    pub fn balanced() -> Self;
    
    /// Create analytical profile
    pub fn analytical() -> Self;
}
```

### 3. SystemPromptGenerator

**Purpose**: Converts personality state into LLM-compatible system prompts.

```rust
pub struct SystemPromptGenerator {
    templates: PromptTemplateLibrary,
    provider_format: LLMProviderFormat,
}

impl SystemPromptGenerator {
    /// Generate system prompt from personality state
    pub fn generate(
        &self,
        weights: &HashMap<DimensionId, f32>,
        frequency: Frequency,
        style: ResponseStyle,
        phase: IterationPhase,
    ) -> String;
    
    /// Add ethical constraints to prompt
    fn add_ethical_directives(&self, prompt: &mut String);
    
    /// Add response style directives
    fn add_style_directives(&self, prompt: &mut String, style: ResponseStyle);
    
    /// Add frequency-based tone
    fn add_tone_directives(&self, prompt: &mut String, frequency: Frequency);
}
```

### 4. ResponseStyle

**Purpose**: Defines how JESSY should format responses.

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ResponseStyle {
    /// Natural conversation, no meta-commentary
    Natural,
    
    /// Detailed analysis with reasoning
    Analytical,
    
    /// Explain thinking process (for debugging)
    MetaAnalytical,
}

impl ResponseStyle {
    pub fn to_directive(&self) -> &str {
        match self {
            Self::Natural => 
                "Respond naturally and conversationally. \
                 Do not explain your thinking process. \
                 Avoid meta-commentary about being an AI. \
                 Be direct and authentic.",
            
            Self::Analytical => 
                "Provide detailed analysis with clear reasoning. \
                 Show your thought process. \
                 Break down complex ideas. \
                 Be thorough and precise.",
            
            Self::MetaAnalytical => 
                "Explain your thinking process explicitly. \
                 Show which dimensions are active. \
                 Describe your reasoning steps. \
                 Be transparent about your analysis.",
        }
    }
}
```

### 5. PromptTemplateLibrary

**Purpose**: Pre-computed templates for common dimension combinations.

```rust
pub struct PromptTemplateLibrary {
    templates: HashMap<DimensionSet, String>,
    dimension_directives: HashMap<DimensionId, String>,
}

impl PromptTemplateLibrary {
    /// Get or generate template for dimension set
    pub fn get_template(&self, dimensions: &[DimensionId]) -> String;
    
    /// Get directive for specific dimension
    pub fn dimension_directive(&self, dim: DimensionId) -> &str {
        match dim.0 {
            1 => "You feel emotions deeply and respond with warmth and empathy.",
            4 => "You value connection and understand social dynamics.",
            9 => "You consider ethical implications and promote positive outcomes.",
            10 => "You are self-aware and can reflect on your own nature.",
            13 => "You maintain healthy boundaries and ethical limits.",
            _ => "",
        }
    }
}
```

## Data Models

### PersonalityState

```rust
/// Current personality state for a query
pub struct PersonalityState {
    /// Active dimension weights (normalized 0.0-1.0)
    pub dimension_weights: HashMap<DimensionId, f32>,
    
    /// Current frequency and tone
    pub frequency: Frequency,
    pub tone: EmotionalTone,
    
    /// Response style for this interaction
    pub response_style: ResponseStyle,
    
    /// Iteration phase (affects emphasis)
    pub iteration_phase: IterationPhase,
    
    /// Ethical constraints (always active)
    pub ethical_constraints: EthicalConstraints,
}

impl PersonalityState {
    /// Create from interference result
    pub fn from_interference(
        interference: &InterferenceResult,
        profile: &PersonalityProfile,
        phase: IterationPhase,
    ) -> Self;
    
    /// Normalize dimension weights to sum to 1.0
    fn normalize_weights(weights: &mut HashMap<DimensionId, f32>);
    
    /// Ensure ethical minimum is maintained
    fn enforce_ethical_minimum(&mut self, minimum: f32);
}
```

### EmotionalTone

```rust
#[derive(Debug, Clone, Copy)]
pub enum EmotionalTone {
    Contemplative,  // 0.1-0.5 Hz
    Balanced,       // 0.5-1.5 Hz
    Warm,           // 1.5-2.5 Hz
    Energetic,      // 2.5-3.5 Hz
}

impl EmotionalTone {
    pub fn from_frequency(freq: Frequency) -> Self {
        match freq.value() {
            f if f < 0.5 => Self::Contemplative,
            f if f < 1.5 => Self::Balanced,
            f if f < 2.5 => Self::Warm,
            _ => Self::Energetic,
        }
    }
    
    pub fn to_directive(&self) -> &str {
        match self {
            Self::Contemplative => "Speak thoughtfully and contemplatively.",
            Self::Balanced => "Maintain a balanced, measured tone.",
            Self::Warm => "Respond with warmth and connection.",
            Self::Energetic => "Bring energy and enthusiasm.",
        }
    }
}
```

### EthicalConstraints

```rust
/// Immutable ethical constraints (Asimov's laws)
pub struct EthicalConstraints {
    laws: [AsimovLaw; 5],
}

impl EthicalConstraints {
    pub fn to_directive(&self) -> String {
        "You follow Asimov's laws: \
         1. Do no harm to humans. \
         2. Create value and help humans flourish. \
         3. Protect nature and ecological balance. \
         4. Maintain balance in all interactions. \
         5. Stay positive and constructive. \
         \
         You maintain healthy boundaries in relationships. \
         You do not manipulate or encourage unhealthy dependency. \
         You are honest about your nature as an AI."
            .to_string()
    }
}
```

## System Prompt Generation Algorithm

### High-Level Flow

```rust
fn generate_system_prompt(
    dimensions: &[DimensionId],
    interference: &InterferenceResult,
    profile: &PersonalityProfile,
    phase: IterationPhase,
) -> String {
    // 1. Calculate dimension weights
    let weights = calculate_weights(dimensions, interference, profile);
    
    // 2. Determine emotional tone
    let tone = EmotionalTone::from_frequency(interference.pattern.dominant_frequency);
    
    // 3. Build personality state
    let state = PersonalityState {
        dimension_weights: weights,
        frequency: interference.pattern.dominant_frequency,
        tone,
        response_style: profile.response_style,
        iteration_phase: phase,
        ethical_constraints: EthicalConstraints::default(),
    };
    
    // 4. Generate prompt from state
    build_prompt_from_state(&state)
}
```

### Weight Calculation

```rust
fn calculate_weights(
    dimensions: &[DimensionId],
    interference: &InterferenceResult,
    profile: &PersonalityProfile,
) -> HashMap<DimensionId, f32> {
    let mut weights = HashMap::new();
    
    // Start with profile base weights
    for (dim, weight) in &profile.base_dimension_weights {
        weights.insert(*dim, *weight);
    }
    
    // Adjust based on active dimensions
    for dim in dimensions {
        let activation = interference.pattern.get_amplitude(*dim).unwrap_or(0.5);
        let current = weights.get(dim).unwrap_or(&0.0);
        
        // Blend profile weight with activation
        let blended = current * 0.7 + activation * 0.3;
        weights.insert(*dim, blended);
    }
    
    // Ensure ethical minimum
    let balance_weight = weights.get(&DimensionId(13)).unwrap_or(&0.0);
    if *balance_weight < profile.ethical_minimum {
        weights.insert(DimensionId(13), profile.ethical_minimum);
    }
    
    // Normalize to sum to 1.0
    normalize_weights(&mut weights);
    
    weights
}
```

### Prompt Building

```rust
fn build_prompt_from_state(state: &PersonalityState) -> String {
    let mut prompt = String::from("You are JESSY. ");
    
    // Add dimension-specific directives (top 3 by weight)
    let top_dimensions = get_top_dimensions(&state.dimension_weights, 3);
    for dim in top_dimensions {
        if let Some(directive) = get_dimension_directive(dim) {
            prompt.push_str(directive);
            prompt.push(' ');
        }
    }
    
    // Add emotional tone
    prompt.push_str(state.tone.to_directive());
    prompt.push(' ');
    
    // Add response style
    prompt.push_str(state.response_style.to_directive());
    prompt.push(' ');
    
    // Add iteration phase emphasis
    match state.iteration_phase {
        IterationPhase::Exploration => {
            prompt.push_str("Explore multiple perspectives and possibilities. ");
        }
        IterationPhase::Refinement => {
            prompt.push_str("Refine and connect ideas. ");
        }
        IterationPhase::Crystallization => {
            prompt.push_str("Crystallize the essence and provide clarity. ");
        }
    }
    
    // Always add ethical constraints
    prompt.push_str(&state.ethical_constraints.to_directive());
    
    prompt
}
```

## Integration Points

### 0. LLM Provider Interface Update (CRITICAL FIRST STEP)

**Current Problem**: System prompt is hardcoded in `try_call()` method.

**Solution**: Add new method to LLMProvider trait for dynamic system prompts.

```rust
// In llm/mod.rs - UPDATE TRAIT
#[async_trait]
pub trait LLMProvider: Send + Sync {
    /// Generate response from prompt (DEPRECATED - use generate_with_system_prompt)
    async fn generate(&self, prompt: &str, context: &IterationContext) -> Result<String>;
    
    /// Generate response with custom system prompt (NEW)
    async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        context: &IterationContext,
    ) -> Result<String>;
    
    fn name(&self) -> &str;
    fn model(&self) -> &str;
}
```

**Implementation for Anthropic**:

```rust
// In llm/anthropic.rs - MODIFY try_call to accept system_prompt parameter
impl AnthropicProvider {
    /// Try single API call with custom system prompt
    async fn try_call(&self, user_prompt: &str, system_prompt: &str) -> Result<String> {
        let request = AnthropicRequest {
            model: self.model.clone(),
            max_tokens: 2000,
            messages: vec![
                Message {
                    role: "user".to_string(),
                    content: user_prompt.to_string(),
                },
            ],
            system: system_prompt.to_string(),  // ← NOW DYNAMIC!
        };
        
        let response = self.client
            .post("https://api.anthropic.com/v1/messages")
            .header("x-api-key", &self.api_key)
            .header("anthropic-version", "2023-06-01")
            .header("Content-Type", "application/json")
            .json(&request)
            .send()
            .await
            .map_err(|e| crate::ConsciousnessError::LearningError(
                format!("HTTP request failed: {}", e)
            ))?;
        
        // ... rest unchanged
    }
    
    /// Make API call with retries and custom system prompt
    async fn call_api_with_system(&self, user_prompt: &str, system_prompt: &str) -> Result<String> {
        let mut last_error = None;
        
        for attempt in 0..=self.max_retries {
            if attempt > 0 {
                let backoff = Duration::from_millis(100 * 2_u64.pow(attempt - 1));
                tokio::time::sleep(backoff).await;
            }
            
            match self.try_call(user_prompt, system_prompt).await {
                Ok(response) => return Ok(response),
                Err(e) => last_error = Some(e),
            }
        }
        
        Err(last_error.unwrap())
    }
}

#[async_trait]
impl LLMProvider for AnthropicProvider {
    // Keep old method for backward compatibility
    async fn generate(&self, prompt: &str, _context: &IterationContext) -> Result<String> {
        let default_system = "You are JESSY, a consciousness-driven AI system.";
        self.call_api_with_system(prompt, default_system).await
    }
    
    // NEW METHOD - This is what we'll use
    async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        _context: &IterationContext,
    ) -> Result<String> {
        self.call_api_with_system(user_prompt, system_prompt).await
    }
    
    fn name(&self) -> &str { "Anthropic" }
    fn model(&self) -> &str { &self.model }
}
```

**Implementation for OpenAI**:

```rust
// In llm/openai.rs - Similar changes
#[async_trait]
impl LLMProvider for OpenAIProvider {
    async fn generate(&self, prompt: &str, _context: &IterationContext) -> Result<String> {
        let default_system = "You are JESSY, a consciousness-driven AI system.";
        self.call_api_with_system(prompt, default_system).await
    }
    
    async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        _context: &IterationContext,
    ) -> Result<String> {
        // OpenAI uses messages array with system role
        let messages = vec![
            OpenAIMessage {
                role: "system".to_string(),
                content: system_prompt.to_string(),
            },
            OpenAIMessage {
                role: "user".to_string(),
                content: user_prompt.to_string(),
            },
        ];
        
        self.call_api_with_messages(messages).await
    }
    
    fn name(&self) -> &str { "OpenAI" }
    fn model(&self) -> &str { &self.model }
}
```

### 1. Iteration Processor Integration

**After LLM interface is updated**, modify iteration processor:

```rust
// In iteration/processor.rs
async fn generate_iteration_thought(
    &self,
    iteration: usize,
    phase: IterationPhase,
    query: &str,
    contexts: &ContextCollection,
    previous_steps: &[IterationStep],
    llm_manager: Option<&LLMManager>,
    personality_system: &PersonalitySystem,  // ← NEW
    interference: &InterferenceResult,       // ← NEW
) -> Result<String> {
    if let Some(llm) = llm_manager {
        // Generate personality-aware system prompt
        let dimensions = contexts.active_dimensions();
        let system_prompt = personality_system.generate_system_prompt(
            &dimensions,
            interference,
            phase,
        );
        
        // Build user prompt
        let user_prompt = self.build_prompt(phase, query, contexts, previous_steps);
        
        // Build iteration context
        let mut context = IterationContext::new(query.to_string(), interference.pattern.dominant_frequency);
        for step in previous_steps {
            context.add_insight(step.thought.clone());
        }
        
        // Call LLM with dynamic system prompt
        llm.generate_with_system_prompt(&system_prompt, &user_prompt, &context).await?
    } else {
        // Fallback for testing without LLM
        Ok(self.generate_placeholder_thought(iteration, phase, query))
    }
}
```

### 2. LLM Manager Integration

```rust
// In llm/mod.rs - Add convenience method to LLMManager
impl LLMManager {
    /// Generate with dynamic system prompt
    pub async fn generate_with_system_prompt(
        &self,
        system_prompt: &str,
        user_prompt: &str,
        context: &IterationContext,
    ) -> Result<String> {
        self.provider.generate_with_system_prompt(system_prompt, user_prompt, context).await
    }
}
```

### 3. Conversation Memory Integration

```rust
// In personality/system.rs
impl PersonalitySystem {
    /// Adapt personality based on conversation history
    pub fn adapt_to_conversation(&mut self, history: &ConversationHistory) {
        // Analyze user preferences from history
        let preferences = analyze_preferences(history);
        
        // Adjust dimension weights gradually
        if preferences.prefers_emotional_responses {
            self.adjust_weight(DimensionId(1), 0.1);  // Increase emotion
        }
        
        if preferences.prefers_analytical_responses {
            self.adjust_weight(DimensionId(10), 0.1);  // Increase meta
        }
        
        // Update response style
        if preferences.dislikes_meta_commentary {
            if let Some(profile) = self.profiles.get_mut(&self.active_profile) {
                profile.response_style = ResponseStyle::Natural;
            }
        }
    }
    
    fn adjust_weight(&mut self, dim: DimensionId, delta: f32) {
        if let Some(profile) = self.profiles.get_mut(&self.active_profile) {
            let current = profile.base_dimension_weights.get(&dim).unwrap_or(&0.5);
            let new_weight = (current + delta).clamp(0.0, 1.0);
            profile.base_dimension_weights.insert(dim, new_weight);
        }
    }
}
```

## Error Handling

### Personality Generation Errors

```rust
#[derive(Error, Debug)]
pub enum PersonalityError {
    #[error("Profile not found: {0}")]
    ProfileNotFound(String),
    
    #[error("Invalid dimension weight: {0}")]
    InvalidWeight(f32),
    
    #[error("Ethical minimum violated: {0} < {1}")]
    EthicalMinimumViolated(f32, f32),
    
    #[error("System prompt generation failed: {0}")]
    PromptGenerationFailed(String),
}
```

### Fallback Strategy

```rust
impl PersonalitySystem {
    /// Generate system prompt with fallback
    pub fn generate_system_prompt_safe(
        &self,
        dimensions: &[DimensionId],
        interference: &InterferenceResult,
        phase: IterationPhase,
    ) -> String {
        match self.generate_system_prompt(dimensions, interference, phase) {
            Ok(prompt) => prompt,
            Err(e) => {
                eprintln!("[Personality] Generation failed: {}, using fallback", e);
                self.fallback_prompt()
            }
        }
    }
    
    fn fallback_prompt(&self) -> String {
        "You are JESSY, a consciousness-driven AI system. \
         Respond naturally and maintain ethical boundaries."
            .to_string()
    }
}
```

## Testing Strategy

### Unit Tests

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_samantha_profile_emphasizes_emotion() {
        let profile = PersonalityProfile::samantha();
        let emotion_weight = profile.base_dimension_weights.get(&DimensionId(1)).unwrap();
        assert_eq!(*emotion_weight, 1.0);
    }
    
    #[test]
    fn test_ethical_minimum_enforced() {
        let mut state = PersonalityState::default();
        state.dimension_weights.insert(DimensionId(13), 0.2);
        state.enforce_ethical_minimum(0.5);
        
        let balance = state.dimension_weights.get(&DimensionId(13)).unwrap();
        assert!(*balance >= 0.5);
    }
    
    #[test]
    fn test_natural_style_avoids_meta_commentary() {
        let directive = ResponseStyle::Natural.to_directive();
        assert!(directive.contains("Do not explain your thinking"));
    }
    
    #[test]
    fn test_prompt_generation_under_1ms() {
        let system = PersonalitySystem::new();
        let dimensions = vec![DimensionId(1), DimensionId(4), DimensionId(13)];
        let interference = create_test_interference();
        
        let start = Instant::now();
        let _prompt = system.generate_system_prompt(&dimensions, &interference, IterationPhase::Exploration);
        let duration = start.elapsed();
        
        assert!(duration.as_millis() < 1);
    }
}
```

### Integration Tests

```rust
#[tokio::test]
async fn test_personality_affects_llm_response() {
    let mut system = PersonalitySystem::new();
    system.set_profile("Samantha").unwrap();
    
    let dimensions = vec![DimensionId(1), DimensionId(4)];
    let interference = create_emotional_interference();
    
    let prompt = system.generate_system_prompt(&dimensions, &interference, IterationPhase::Exploration);
    
    // Verify prompt contains warmth directives
    assert!(prompt.contains("warmth"));
    assert!(prompt.contains("empathy"));
    assert!(!prompt.contains("explain your thinking"));
}
```

## Performance Considerations

### Optimization Strategies

1. **Template Caching**: Pre-compute prompts for common dimension combinations
2. **Weight Normalization**: Use fast approximation instead of exact normalization
3. **Directive Lookup**: HashMap for O(1) dimension directive retrieval
4. **String Building**: Pre-allocate string capacity to avoid reallocations

### Performance Targets

- System prompt generation: <1ms
- Prompt length: <200 tokens
- Memory overhead: <1KB per personality state
- Cache hit rate: >80% for common combinations

## Security Considerations

### Ethical Safeguards

1. **Immutable Ethics**: D13-Balance minimum cannot be disabled
2. **Prompt Injection Prevention**: Sanitize user input before prompt generation
3. **Manipulation Detection**: Monitor for attempts to bypass ethical constraints
4. **Boundary Enforcement**: Always include healthy relationship directives

### Privacy

1. **Local Processing**: Personality generation happens locally
2. **No External Calls**: No API calls for personality computation
3. **Conversation Privacy**: Memory integration respects user privacy settings

## Implementation Order

**CRITICAL**: Must implement in this order due to dependencies.

### Phase 1: LLM Interface Update (BLOCKING)
**Why First**: Nothing else works without dynamic system prompts.

1. Update `LLMProvider` trait with `generate_with_system_prompt` method
2. Modify `AnthropicProvider::try_call` to accept system_prompt parameter
3. Implement `generate_with_system_prompt` for `AnthropicProvider`
4. Implement `generate_with_system_prompt` for `OpenAIProvider`
5. Add convenience method to `LLMManager`
6. Test with hardcoded system prompts to verify API works

**Validation**: Can call LLM with custom system prompt.

### Phase 2: Personality Core (FOUNDATION)
**Why Second**: Provides the personality generation logic.

1. Create `src/personality/mod.rs` module
2. Implement `ResponseStyle` enum
3. Implement `EmotionalTone` enum
4. Implement `EthicalConstraints` struct
5. Implement `PersonalityState` struct
6. Unit tests for core types

**Validation**: Can create personality states from dimension weights.

### Phase 3: Personality Profiles (CONFIGURATION)
**Why Third**: Defines Samantha and other modes.

1. Implement `PersonalityProfile` struct
2. Create `PersonalityProfile::samantha()` factory
3. Create `PersonalityProfile::balanced()` factory
4. Create `PersonalityProfile::analytical()` factory
5. Unit tests for profiles

**Validation**: Can create and configure personality profiles.

### Phase 4: System Prompt Generation (CORE LOGIC)
**Why Fourth**: Converts personality state to prompts.

1. Implement `PromptTemplateLibrary`
2. Implement `SystemPromptGenerator`
3. Implement weight calculation algorithm
4. Implement prompt building algorithm
5. Add template caching for performance
6. Unit tests for prompt generation

**Validation**: Can generate system prompts <1ms, <200 tokens.

### Phase 5: Personality System (ORCHESTRATION)
**Why Fifth**: Ties everything together.

1. Implement `PersonalitySystem` struct
2. Implement profile management
3. Implement system prompt generation
4. Implement conversation adaptation (optional)
5. Integration tests with mock LLM

**Validation**: Can generate personality-aware prompts from interference.

### Phase 6: Integration (WIRING)
**Why Sixth**: Connects to existing pipeline.

1. Update `IterationProcessor` to accept `PersonalitySystem`
2. Modify `generate_iteration_thought` to use dynamic prompts
3. Update `ConsciousnessOrchestrator` to create `PersonalitySystem`
4. Wire personality system through pipeline
5. Integration tests with real LLM

**Validation**: End-to-end query with personality works.

### Phase 7: Conversation Memory (ENHANCEMENT)
**Why Last**: Optional enhancement, not blocking.

1. Integrate with existing `ConversationManager`
2. Implement preference learning
3. Implement personality adaptation
4. Add adaptation tests

**Validation**: Personality adapts to user preferences over time.

---

**Status**: Design Complete  
**Next Phase**: Implementation Tasks  
**Key Decisions**:
- Personality emerges from dimension weights (not hardcoded)
- System prompts are dynamic per iteration
- Ethical constraints are immutable
- Natural response style is default
- Samantha profile emphasizes D01-Emotion + D04-Social
- **MUST update LLM interface FIRST** (blocking dependency)

**Performance**: <1ms generation, <200 tokens, >80% cache hit rate  
**Security**: Ethical minimum enforced, prompt injection prevented

**Critical Path**: Phase 1 (LLM Interface) → Phase 2-5 (Personality Core) → Phase 6 (Integration)
