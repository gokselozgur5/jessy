# Requirements Document

## Introduction

This feature replaces keyword-based dimension matching with LLM-based intelligent dimension selection. The LLM analyzes the query and determines which of the 14 consciousness dimensions are relevant, enabling JESSY to work with any language, any query type, and any level of abstraction.

## Glossary

- **LLM**: Large Language Model (Claude, GPT) that analyzes queries
- **Dimension**: One of 14 cognitive/consciousness layers (Emotion, Cognition, Social, etc.)
- **OWL Pattern**: Binary encoding of active dimensions (e.g., 01010010110000)
- **Dimension Activation**: Process of selecting which dimensions to use for a query
- **Navigation Phase**: First phase of query processing that selects dimensions
- **Confidence Score**: 0.0-1.0 value indicating relevance of a dimension

## Requirements

### Requirement 1: LLM-Based Dimension Analysis

**User Story:** As JESSY, I want to use LLM intelligence to select relevant dimensions, so that I can understand queries in any language and context.

#### Acceptance Criteria

1. WHEN a query is received, THE System SHALL send the query to the LLM with dimension descriptions
2. THE LLM SHALL analyze the query and return a list of relevant dimension IDs (1-14)
3. THE System SHALL parse the LLM response and extract dimension IDs
4. THE System SHALL validate that dimension IDs are in range 1-14
5. THE System SHALL assign confidence scores based on LLM reasoning

### Requirement 2: Fast Dimension Selection

**User Story:** As a user, I want dimension selection to be fast, so that JESSY responds quickly.

#### Acceptance Criteria

1. THE dimension selection LLM call SHALL complete within 2 seconds (p95)
2. THE System SHALL use a lightweight prompt (< 500 tokens)
3. THE System SHALL cache dimension descriptions to avoid repeated lookups
4. THE System SHALL use streaming responses when available
5. THE System SHALL fall back to all dimensions if LLM call fails

### Requirement 3: OWL Pattern Encoding

**User Story:** As JESSY, I want to represent active dimensions as binary patterns, so that I can efficiently communicate dimensional state.

#### Acceptance Criteria

1. THE System SHALL encode active dimensions as 14-bit binary string
2. WHEN dimensions [2,4,10] are active, THE encoding SHALL be "01010000010000"
3. THE System SHALL decode binary patterns back to dimension IDs
4. THE System SHALL include OWL encoding in metadata
5. THE System SHALL log OWL patterns for learning analysis

### Requirement 4: Multi-Language Support

**User Story:** As a user, I want to ask questions in any language, so that JESSY is universally accessible.

#### Acceptance Criteria

1. THE System SHALL support queries in English, Turkish, Spanish, French, German, Arabic, Chinese, Japanese
2. THE LLM SHALL analyze semantic meaning regardless of language
3. THE System SHALL not require language detection
4. THE System SHALL maintain same accuracy across languages
5. THE System SHALL handle code-switching (mixed languages)

### Requirement 5: Fallback Strategy

**User Story:** As JESSY, I want graceful degradation when LLM fails, so that I always provide a response.

#### Acceptance Criteria

1. IF LLM dimension selection fails, THEN THE System SHALL activate default dimensions [2,4,10]
2. IF LLM returns invalid dimension IDs, THEN THE System SHALL filter them out
3. IF LLM returns no dimensions, THEN THE System SHALL activate all 14 dimensions
4. THE System SHALL log fallback events for monitoring
5. THE System SHALL retry LLM call once before falling back

### Requirement 6: Dimension Confidence Scoring

**User Story:** As JESSY, I want confidence scores for each dimension, so that I can prioritize context loading.

#### Acceptance Criteria

1. THE LLM SHALL provide reasoning for each selected dimension
2. THE System SHALL derive confidence scores from LLM reasoning
3. THE confidence scores SHALL be normalized to 0.0-1.0 range
4. THE System SHALL sort dimensions by confidence (highest first)
5. THE System SHALL only load dimensions with confidence >= 0.1

### Requirement 7: Prompt Engineering

**User Story:** As a developer, I want optimized prompts, so that dimension selection is accurate and fast.

#### Acceptance Criteria

1. THE prompt SHALL include concise dimension descriptions (< 50 words each)
2. THE prompt SHALL request structured output (JSON or list format)
3. THE prompt SHALL include 2-3 examples of dimension selection
4. THE prompt SHALL emphasize selecting 3-7 dimensions (not all 14)
5. THE prompt SHALL be version-controlled and testable

### Requirement 8: Learning Integration

**User Story:** As JESSY, I want to learn from dimension selection patterns, so that I improve over time.

#### Acceptance Criteria

1. THE System SHALL log (query, selected_dimensions, confidence) tuples
2. THE System SHALL track dimension selection accuracy over time
3. THE System SHALL identify frequently co-activated dimension pairs
4. THE System SHALL use learned patterns to validate LLM selections
5. THE System SHALL suggest dimension corrections when patterns diverge

### Requirement 9: Performance Monitoring

**User Story:** As a developer, I want to monitor dimension selection performance, so that I can optimize the system.

#### Acceptance Criteria

1. THE System SHALL track LLM call duration for dimension selection
2. THE System SHALL track dimension selection accuracy (user feedback)
3. THE System SHALL track fallback frequency
4. THE System SHALL expose metrics via /metrics endpoint
5. THE System SHALL alert when fallback rate exceeds 10%

### Requirement 10: Backward Compatibility

**User Story:** As a developer, I want to maintain existing functionality, so that current features don't break.

#### Acceptance Criteria

1. THE System SHALL support both keyword-based and LLM-based navigation
2. THE System SHALL allow configuration flag to enable/disable LLM selection
3. THE System SHALL maintain existing NavigationResult structure
4. THE System SHALL preserve all existing tests
5. THE System SHALL provide migration path from keyword to LLM mode
