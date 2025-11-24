# Requirements Document

## Introduction

Jessy backend'de LLM API çağrıları sırasında response gelmediğinde sonsuz bekleme sorunu var. Kullanıcılar "waiting" durumunda takılıyor ve hiç response alamıyorlar. Bu sorun Anthropic API'nin bazen connection açık tutup data göndermemesinden kaynaklanıyor.

## Glossary

- **Jessy**: Rust ile yazılmış AI consciousness engine
- **LLM**: Large Language Model (Claude/Anthropic API)
- **Observer Chain**: Jessy'nin 2-stage thinking pipeline'ı
- **Timeout**: Maksimum bekleme süresi
- **Reqwest**: Rust HTTP client library
- **Tokio**: Async runtime for Rust

## Requirements

### Requirement 1

**User Story:** As a user, I want to receive an error message within 45 seconds if Jessy cannot generate a response, so that I don't wait indefinitely.

#### Acceptance Criteria

1. WHEN a user sends a message to Jessy, THE System SHALL respond within 45 seconds maximum
2. IF the LLM API call exceeds 45 seconds, THEN THE System SHALL return a timeout error to the user
3. WHEN a timeout occurs, THE System SHALL log the timeout event with context (query, stage, duration)
4. THE System SHALL use tokio::time::timeout wrapper for all LLM API calls
5. WHERE the timeout is configurable, THE System SHALL allow timeout duration to be set via environment variable

### Requirement 2

**User Story:** As a developer, I want detailed timeout logging, so that I can debug and monitor API performance issues.

#### Acceptance Criteria

1. WHEN a timeout occurs, THE System SHALL log the query that timed out
2. WHEN a timeout occurs, THE System SHALL log which observer chain stage failed
3. WHEN a timeout occurs, THE System SHALL log the elapsed time before timeout
4. THE System SHALL log successful API call durations for performance monitoring
5. THE System SHALL include timeout information in error responses returned to users

### Requirement 3

**User Story:** As a system administrator, I want graceful degradation when timeouts occur, so that the system remains stable and responsive.

#### Acceptance Criteria

1. WHEN a timeout occurs in observer chain stage 1, THE System SHALL return a fallback response
2. WHEN a timeout occurs in observer chain stage 2, THE System SHALL return the stage 1 observation
3. THE System SHALL NOT crash or hang when timeouts occur
4. THE System SHALL release all locks and resources when a timeout occurs
5. THE System SHALL continue accepting new requests after a timeout occurs
