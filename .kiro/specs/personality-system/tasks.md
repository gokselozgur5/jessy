# Implementation Plan: Dynamic Personality System

## Phase 1: LLM Interface Update (BLOCKING)

- [ ] 1. Update LLM Provider Interface
- [x] 1.1 Add generate_with_system_prompt method to LLMProvider trait
  - Add new async method signature with system_prompt, user_prompt, context parameters
  - Keep existing generate method for backward compatibility
  - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

- [x] 1.2 Modify AnthropicProvider to support dynamic system prompts
  - Modify try_call method to accept system_prompt parameter
  - Create call_api_with_system method that wraps retry logic
  - Implement generate_with_system_prompt trait method
  - _Requirements: 6.1, 6.3_

- [x] 1.3 Modify OpenAIProvider to support dynamic system prompts
  - Modify message building to accept system_prompt parameter
  - Implement generate_with_system_prompt trait method
  - _Requirements: 6.2, 6.4_

- [x] 1.4 Add convenience method to LLMManager
  - Add generate_with_system_prompt method that delegates to provider
  - _Requirements: 6.5_

- [ ]* 1.5 Test LLM interface changes
  - Write integration test calling Anthropic with custom system prompt
  - Test error handling with invalid system prompts
  - _Requirements: 6.1, 6.2, 6.3, 6.4_

## Phase 2: Personality Core Types

- [ ] 2. Create Personality Module Structure
- [ ] 2.1 Set up personality module
  - Create src/personality/mod.rs with module documentation
  - Add personality module to src/lib.rs
  - _Requirements: 1.1, 2.1_

- [ ] 2.2 Create ResponseStyle enum
  - Define Natural, Analytical, MetaAnalytical variants
  - Implement to_directive method returning LLM instructions
  - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

- [ ]* 2.3 Test ResponseStyle
  - Test Natural style contains "Do not explain your thinking"
  - _Requirements: 2.1, 2.2, 2.3_

- [ ] 2.4 Create EmotionalTone enum
  - Define Contemplative, Balanced, Warm, Energetic variants
  - Implement from_frequency and to_directive methods
  - _Requirements: 1.4, 1.5, 5.1_

- [ ]* 2.5 Test EmotionalTone frequency mapping
  - Test frequency ranges map to correct tones
  - _Requirements: 1.4, 1.5_

- [ ] 2.6 Create EthicalConstraints struct
  - Define Asimov's 5 laws as constants
  - Implement to_directive method
  - Make struct immutable
  - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5_

- [ ] 2.7 Create PersonalityState struct
  - Define fields for dimension_weights, frequency, tone, response_style
  - Implement from_interference constructor
  - Implement normalize_weights and enforce_ethical_minimum helpers
  - _Requirements: 1.1, 1.2, 1.3, 4.3, 5.5_

## Phase 3: Personality Profiles

- [ ] 3. Implement Personality Profiles
- [ ] 3.1 Create PersonalityProfile struct
  - Define fields for name, base_dimension_weights, frequency_preferences, response_style
  - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5_

- [ ] 3.2 Implement PersonalityProfile::samantha
  - Set D01-Emotion weight to 1.0, D04-Social to 0.9, D13-Balance to 0.9
  - Set frequency range to 0.8-1.5 Hz, response_style to Natural
  - _Requirements: 3.4, 4.3_

- [ ]* 3.3 Test Samantha profile
  - Test D01-Emotion weight is 1.0
  - Test response_style is Natural
  - _Requirements: 3.4, 4.3_

- [ ] 3.4 Implement PersonalityProfile::balanced
  - Set equal weights for all dimensions
  - _Requirements: 3.3_

- [ ] 3.5 Implement PersonalityProfile::analytical
  - Emphasize D10-Meta and D02-Cognitive
  - Set response_style to Analytical
  - _Requirements: 3.3, 2.3_

## Phase 4: System Prompt Generation

- [ ] 4. Implement Prompt Generation
- [ ] 4.1 Create PromptTemplateLibrary struct
  - Define dimension_directives HashMap
  - Populate directives for D01, D04, D09, D10, D13
  - Implement dimension_directive method
  - _Requirements: 8.2, 8.3_

- [ ] 4.2 Create SystemPromptGenerator struct
  - Implement generate method
  - Implement add_ethical_directives, add_style_directives, add_tone_directives helpers
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

- [ ] 4.3 Implement calculate_weights algorithm
  - Blend profile base weights with interference activation
  - Enforce ethical minimum for D13-Balance
  - Normalize weights to sum to 1.0
  - _Requirements: 1.1, 4.3, 5.4, 5.5_

- [ ] 4.4 Implement build_prompt_from_state algorithm
  - Add dimension directives, tone, style, iteration phase emphasis
  - Always add ethical constraints
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 4.1, 10.1, 10.2, 10.3_

- [ ]* 4.5 Test prompt building
  - Test prompt contains dimension directives and ethical constraints
  - Test prompt length less than 200 tokens
  - _Requirements: 1.1, 4.1, 8.5_

- [ ] 4.6 Add template caching for performance
  - Cache prompts for common dimension combinations
  - Target greater than 80 percent cache hit rate
  - _Requirements: 8.1, 8.2, 8.3_

- [ ]* 4.7 Test performance
  - Test prompt generation completes in less than 1ms
  - _Requirements: 8.1, 8.2, 8.5_

## Phase 5: Personality System Orchestration

- [ ] 5. Implement PersonalitySystem
- [ ] 5.1 Create PersonalitySystem struct
  - Define fields for profiles, active_profile, adaptation_rate
  - Implement new constructor with default profiles
  - Implement set_profile and current_weights methods
  - _Requirements: 3.1, 3.2, 3.5, 7.3, 7.4_

- [ ] 5.2 Implement generate_system_prompt method
  - Accept dimensions, interference, iteration_phase parameters
  - Calculate dimension weights from interference
  - Use SystemPromptGenerator to build prompt
  - Add error handling with fallback
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 10.4_

- [ ]* 5.3 Test system prompt generation
  - Test with Samantha profile generates warm prompt
  - Test ethical constraints always included
  - _Requirements: 1.1, 2.1, 3.4, 4.1_

- [ ] 5.4 Implement profile switching
  - Validate profile exists before switching
  - Return error for unknown profiles
  - _Requirements: 3.5, 7.3_

- [ ] 5.5 Add personality transparency features for debug mode
  - Log active dimension weights when debug enabled
  - Log generated system prompt when debug enabled
  - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5_

## Phase 6: Pipeline Integration

- [ ] 6. Integrate with Consciousness Pipeline
- [ ] 6.1 Modify IterationProcessor to accept PersonalitySystem
  - Add personality_system field to struct
  - Update constructor to accept PersonalitySystem
  - _Requirements: 1.1, 10.1, 10.2, 10.3_

- [ ] 6.2 Modify generate_iteration_thought method
  - Accept personality_system and interference parameters
  - Generate system prompt using personality_system
  - Call LLM with generate_with_system_prompt
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 10.1, 10.2, 10.3, 10.4, 10.5_

- [ ]* 6.3 Test iteration thought generation
  - Test with Samantha profile generates warm responses
  - Test different iteration phases affect prompts
  - _Requirements: 1.1, 2.1, 10.1, 10.2, 10.3_

- [ ] 6.4 Add PersonalitySystem to ConsciousnessOrchestrator
  - Create PersonalitySystem in constructor
  - Pass to IterationProcessor
  - Expose method to switch profiles
  - _Requirements: 3.5, 5.1, 5.2, 5.3_

- [ ] 6.5 Wire personality through process method
  - Pass interference result to iteration processor
  - Log personality state in debug mode
  - _Requirements: 1.1, 5.1, 5.2, 7.1, 7.2_

- [ ]* 6.6 End-to-end integration test
  - Test full query with Samantha profile
  - Test response has warm natural tone
  - Test no meta-commentary in response
  - Test ethical boundaries maintained
  - _Requirements: 1.1, 2.1, 2.2, 4.1, 8.1_

## Phase 7: Conversation Memory Integration (Optional)

- [ ] 7. Integrate with Conversation Memory
- [ ] 7.1 Connect PersonalitySystem to ConversationManager
  - Add conversation_memory field to PersonalitySystem
  - Load conversation history on startup
  - _Requirements: 9.1, 9.2, 9.3, 9.4, 9.5_

- [ ] 7.2 Implement adapt_to_conversation method
  - Analyze user preferences from conversation history
  - Adjust dimension weights gradually (max 0.2 change)
  - Store learned preferences
  - _Requirements: 9.1, 9.2, 9.3, 9.4, 9.5_

- [ ]* 7.3 Test preference learning
  - Test emotional preference increases D01 weight
  - Test weight changes are gradual
  - _Requirements: 9.1, 9.2, 9.3, 9.4_

- [ ] 7.4 Add method to reset personality adaptations
  - Reset dimension weights to profile defaults
  - Clear learned preferences
  - _Requirements: 9.5_
