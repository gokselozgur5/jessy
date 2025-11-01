# Learning System Requirements

## Introduction

This specification defines the Learning System that enables the Jessy consciousness system to learn from interactions, detect patterns, and crystallize new dimensional layers. The system converts temporary observations into permanent knowledge structures.

## Glossary

- **Learning System**: Coordinator that manages pattern detection, proto-dimensions, and crystallization
- **Observation**: Record of a single query interaction with activated dimensions and keywords
- **Pattern**: Recurring theme detected from multiple observations
- **Proto-Dimension**: Temporary dimension stored in heap memory during learning phase
- **Crystallization**: Process of migrating proto-dimension from heap to MMAP storage
- **Synesthetic Association**: Learned relationship between keywords that co-occur frequently
- **Confidence Score**: Measure of pattern reliability (0.0-1.0)

## Requirements

### Requirement 1: Observation Recording

**User Story:** As a consciousness system, I want to record every query interaction, so that I can learn from usage patterns.

#### Acceptance Criteria

1. WHEN a query is processed, THE System SHALL record an observation
2. WHEN recording observation, THE System SHALL capture query text, activated dimensions, keywords, and frequency
3. WHEN recording observation, THE System SHALL timestamp the observation
4. WHEN observation buffer is full (1000 entries), THE System SHALL use circular buffer (overwrite oldest)
5. WHEN observation is recorded, THE System SHALL complete within query processing time (<5ms overhead)

### Requirement 2: Pattern Detection

**User Story:** As a learning system, I want to detect recurring patterns from observations, so that I can identify new knowledge domains.

#### Acceptance Criteria

1. WHEN analyzing observations, THE System SHALL identify keyword clusters that appear together frequently
2. WHEN pattern is detected, THE System SHALL calculate confidence score based on observation count
3. WHEN pattern has <50 observations, THE System SHALL NOT create proto-dimension
4. WHEN pattern has ≥50 observations AND confidence ≥85%, THE System SHALL suggest proto-dimension creation
5. WHEN pattern detection runs, THE System SHALL complete within 100ms

### Requirement 3: Proto-Dimension Creation

**User Story:** As a learning system, I want to create proto-dimensions in heap memory, so that I can test patterns before permanent crystallization.

#### Acceptance Criteria

1. WHEN creating proto-dimension, THE System SHALL allocate heap memory (not MMAP)
2. WHEN creating proto-dimension, THE System SHALL assign unique dimension ID (>100)
3. WHEN creating proto-dimension, THE System SHALL store pattern keywords and frequency range
4. WHEN proto-dimension size would exceed 16MB, THE System SHALL return error
5. WHEN total proto-dimensions would exceed 10, THE System SHALL return error

### Requirement 4: Memory Limit Enforcement

**User Story:** As a system administrator, I want memory usage limited to 500MB total, so that the system doesn't consume excessive resources.

#### Acceptance Criteria

1. WHEN calculating memory usage, THE System SHALL include core dimensions (280MB) + proto-dimensions + observation buffer
2. WHEN memory usage would exceed 500MB, THE System SHALL reject new proto-dimension creation
3. WHEN memory limit is reached, THE System SHALL log warning
4. WHEN proto-dimension is crystallized, THE System SHALL free heap memory
5. THE System SHALL track memory usage in real-time

### Requirement 5: Crystallization Process

**User Story:** As a learning system, I want to crystallize proto-dimensions to MMAP, so that learned patterns become permanent dimensions.

#### Acceptance Criteria

1. WHEN proto-dimension is ready (confidence ≥85%), THE System SHALL queue for crystallization
2. WHEN crystallization begins, THE System SHALL run as background async task
3. WHEN crystallizing, THE System SHALL allocate MMAP region for dimension
4. WHEN crystallizing, THE System SHALL migrate content from heap to MMAP atomically
5. WHEN crystallization completes, THE System SHALL free heap memory and update dimension registry

### Requirement 6: Crystallization Error Handling

**User Story:** As a reliability engineer, I want crystallization to handle errors gracefully, so that failures don't corrupt the system.

#### Acceptance Criteria

1. IF MMAP allocation fails, THEN THE System SHALL retry up to 3 times
2. IF all retries fail, THEN THE System SHALL keep proto-dimension in heap and log error
3. IF migration is interrupted, THEN THE System SHALL rollback and restore heap state
4. IF crystallization succeeds, THEN THE System SHALL verify MMAP content matches heap
5. THE System SHALL never leave partial/corrupted data in MMAP

### Requirement 7: Synesthetic Learning

**User Story:** As a learning system, I want to strengthen associations between keywords that co-occur, so that navigation becomes more accurate over time.

#### Acceptance Criteria

1. WHEN two keywords appear together in a query, THE System SHALL strengthen their association
2. WHEN association is strengthened, THE System SHALL increase strength by 10% (learning rate)
3. WHEN association is unused for 24 hours, THE System SHALL decay strength by 5% (decay rate)
4. WHEN association strength reaches 0.1, THE System SHALL remove association
5. WHEN looking up associations, THE System SHALL return results in O(1) time

### Requirement 8: User-Specific Dimensions

**User Story:** As a user, I want the system to learn my specific patterns, so that responses become personalized over time.

#### Acceptance Criteria

1. WHEN user has unique patterns, THE System SHALL create user-specific proto-dimension
2. WHEN user-specific dimension is created, THE System SHALL assign ID in range 1000-9999
3. WHEN user-specific dimension is crystallized, THE System SHALL store in user-specific MMAP region (32MB)
4. WHEN user queries, THE System SHALL activate both core and user-specific dimensions
5. WHEN user is deleted, THE System SHALL remove user-specific dimensions

### Requirement 9: Performance

**User Story:** As a performance engineer, I want learning to have minimal overhead, so that query processing remains fast.

#### Acceptance Criteria

1. WHEN recording observation, THE System SHALL complete within 5ms
2. WHEN detecting patterns, THE System SHALL complete within 100ms
3. WHEN creating proto-dimension, THE System SHALL complete within 50ms
4. WHEN crystallization runs, THE System SHALL NOT block query processing
5. WHEN synesthetic lookup occurs, THE System SHALL complete within 1ms

### Requirement 10: Monitoring and Observability

**User Story:** As a DevOps engineer, I want visibility into learning system behavior, so that I can monitor and debug issues.

#### Acceptance Criteria

1. THE System SHALL expose metrics: observation_count, pattern_count, proto_dimension_count, crystallization_success_rate
2. THE System SHALL log pattern detection events with confidence scores
3. THE System SHALL log crystallization start/complete/failure events
4. THE System SHALL expose memory usage metrics
5. THE System SHALL provide API to query current learning state

---

*Requirements Version: 1.0*
*Date: 2025-10-26*
*Status: Ready for Implementation*
