# Centralized Logging Guide

## Overview

The Jessy system uses structured logging across all services with centralized log aggregation through Docker. This enables easy debugging, monitoring, and analysis of system behavior.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Docker Compose                         │
│                  (Log Aggregation)                       │
└─────────────────────────────────────────────────────────┘
           ↑                              ↑
           │                              │
    ┌──────┴──────┐              ┌───────┴────────┐
    │ Rust Core   │              │   Go API       │
    │  (tracing)  │              │  (zerolog)     │
    └─────────────┘              └────────────────┘
```

## Logging Configuration

### Rust Service (jessy-core)

**Library**: `tracing` + `tracing-subscriber`

**Features**:
- Structured logging with key-value pairs
- JSON format in production
- Pretty console format in development
- Automatic log level filtering via `RUST_LOG`
- Thread IDs, file names, and line numbers in production

**Environment Variables**:
```bash
RUST_LOG=jessy=debug,actix_web=info  # Log level control
RUST_ENV=development                  # development or production
RUST_BACKTRACE=1                      # Enable backtraces
```

**Log Levels**:
- `error`: Critical errors that need immediate attention
- `warn`: Warning conditions
- `info`: Informational messages (default in production)
- `debug`: Detailed debugging information (default in development)
- `trace`: Very detailed tracing information

**Example Structured Log**:
```rust
info!(
    service = "jessy-core",
    endpoint = "/health",
    status = "healthy",
    "Health check completed"
);
```

**JSON Output (Production)**:
```json
{
  "timestamp": "2024-10-24T10:30:45.123Z",
  "level": "INFO",
  "target": "jessy::bin::jessy",
  "fields": {
    "service": "jessy-core",
    "endpoint": "/health",
    "status": "healthy",
    "message": "Health check completed"
  },
  "span": {
    "name": "request"
  },
  "thread_id": "ThreadId(2)",
  "file": "src/bin/jessy.rs",
  "line": 42
}
```

### Go Service (jessy-api)

**Library**: `zerolog`

**Features**:
- Zero-allocation structured logging
- JSON format in production
- Pretty console format in development
- Automatic log level filtering via `LOG_LEVEL`
- Caller information and timestamps

**Environment Variables**:
```bash
LOG_LEVEL=debug    # debug, info, warn, error
GO_ENV=development # development or production
```

**Log Levels**:
- `error`: Critical errors
- `warn`: Warning conditions
- `info`: Informational messages (default)
- `debug`: Detailed debugging information

**Example Structured Log**:
```go
log.Info().
    Str("service", "jessy-api").
    Str("endpoint", "/api/health").
    Str("status", "healthy").
    Msg("Health check completed")
```

**JSON Output (Production)**:
```json
{
  "level": "info",
  "service": "jessy-api",
  "endpoint": "/api/health",
  "status": "healthy",
  "time": "2024-10-24T10:30:45Z",
  "message": "Health check completed"
}
```

## Docker Logging Configuration

### Log Driver

All services use the `json-file` driver with rotation:

```yaml
logging:
  driver: "json-file"
  options:
    max-size: "10m"      # Maximum size per log file
    max-file: "3"        # Keep 3 rotated files
    labels: "service,environment"
```

**Total log storage per service**: 30MB (3 files × 10MB)

### Service Labels

Each service is labeled for easy filtering:

```yaml
labels:
  service: "jessy-core"
  environment: "development"
```

## Viewing Logs

### All Services

```bash
# Follow all logs
make logs

# Or directly
docker-compose logs -f
```

### Specific Service

```bash
# Rust service
make logs-rust
docker-compose logs -f jessy-core

# Go service
make logs-go
docker-compose logs -f jessy-api
```

### Filtered Logs

```bash
# Last 100 lines
docker-compose logs --tail=100

# Since timestamp
docker-compose logs --since 2024-10-24T10:00:00

# Specific service with tail
docker-compose logs -f --tail=50 jessy-core
```

### JSON Logs Only

```bash
# Extract only JSON-formatted logs
make logs-json

# Or manually
docker-compose logs --no-color | grep -E '^\{.*\}$'
```

### Test Log Aggregation

```bash
# Run automated log aggregation test
make logs-test
```

This command will:
1. Start all services
2. Generate test traffic (health checks, status checks)
3. Display aggregated logs from all services
4. Verify log collection is working

## Log Analysis

### Search Logs

```bash
# Search for specific text
docker-compose logs | grep "error"

# Search in specific service
docker-compose logs jessy-core | grep "health"

# Case-insensitive search
docker-compose logs | grep -i "shutdown"
```

### Parse JSON Logs

```bash
# Extract all error-level logs
docker-compose logs --no-color | \
  grep -E '^\{.*\}$' | \
  jq 'select(.level == "error" or .level == "ERROR")'

# Count logs by level
docker-compose logs --no-color | \
  grep -E '^\{.*\}$' | \
  jq -r '.level' | \
  sort | uniq -c

# Extract specific fields
docker-compose logs jessy-core --no-color | \
  grep -E '^\{.*\}$' | \
  jq '{time: .timestamp, level: .level, message: .fields.message}'
```

### Export Logs

```bash
# Export to file
docker-compose logs > logs_$(date +%Y%m%d_%H%M%S).txt

# Export JSON logs only
docker-compose logs --no-color | \
  grep -E '^\{.*\}$' > logs_json_$(date +%Y%m%d_%H%M%S).jsonl

# Export specific service
docker-compose logs jessy-core > jessy_core_logs.txt
```

## Log Rotation

Docker automatically rotates logs based on the configuration:

- **Max size**: 10MB per file
- **Max files**: 3 files kept
- **Total storage**: 30MB per service

Old logs are automatically deleted when limits are reached.

### Manual Log Cleanup

```bash
# Remove all logs (stops containers first)
docker-compose down
docker-compose up -d

# Or use Docker prune (removes all unused data)
docker system prune -f
```

## Monitoring and Alerting

### Real-time Monitoring

```bash
# Watch for errors in real-time
docker-compose logs -f | grep -i error

# Watch specific service
docker-compose logs -f jessy-core | grep -E "(error|warn)"

# Count errors per minute
docker-compose logs -f | grep -i error | pv -l -i 60 > /dev/null
```

### Log Metrics

```bash
# Count total log lines
docker-compose logs --no-color | wc -l

# Count by service
docker-compose logs jessy-core --no-color | wc -l
docker-compose logs jessy-api --no-color | wc -l

# Count by log level (JSON logs)
docker-compose logs --no-color | \
  grep -E '^\{.*\}$' | \
  jq -r '.level' | \
  sort | uniq -c
```

## Best Practices

### 1. Use Structured Logging

**✅ Good**:
```rust
info!(
    user_id = %user_id,
    action = "login",
    duration_ms = duration,
    "User login successful"
);
```

**❌ Bad**:
```rust
info!("User {} logged in, took {}ms", user_id, duration);
```

### 2. Include Context

Always include:
- Service name
- Request ID (for tracing)
- User ID (if applicable)
- Action/operation
- Duration (for performance tracking)

### 3. Choose Appropriate Log Levels

- **ERROR**: System failures, exceptions
- **WARN**: Degraded performance, retries
- **INFO**: Important business events
- **DEBUG**: Detailed diagnostic information

### 4. Avoid Logging Sensitive Data

**Never log**:
- Passwords
- API keys
- Personal information (PII)
- Credit card numbers
- Session tokens

### 5. Use Correlation IDs

For distributed tracing across services:

```rust
info!(
    correlation_id = %correlation_id,
    service = "jessy-core",
    "Processing request"
);
```

```go
log.Info().
    Str("correlation_id", correlationID).
    Str("service", "jessy-api").
    Msg("Processing request")
```

## Troubleshooting

### No Logs Appearing

1. Check if services are running:
   ```bash
   docker-compose ps
   ```

2. Check log driver configuration:
   ```bash
   docker inspect jessy-core | jq '.[0].HostConfig.LogConfig'
   ```

3. Verify log level settings:
   ```bash
   docker-compose exec jessy-core env | grep RUST_LOG
   docker-compose exec jessy-api env | grep LOG_LEVEL
   ```

### Logs Not Structured

1. Check environment variables:
   ```bash
   # Rust should have RUST_ENV set
   docker-compose exec jessy-core env | grep RUST_ENV
   
   # Go should have GO_ENV set
   docker-compose exec jessy-api env | grep GO_ENV
   ```

2. Restart services to apply changes:
   ```bash
   docker-compose restart
   ```

### Log Files Too Large

1. Check current log sizes:
   ```bash
   docker inspect jessy-core | jq '.[0].LogPath' | xargs ls -lh
   ```

2. Adjust rotation settings in `docker-compose.yml`:
   ```yaml
   logging:
     options:
       max-size: "5m"   # Reduce to 5MB
       max-file: "2"    # Keep only 2 files
   ```

3. Restart services:
   ```bash
   docker-compose down
   docker-compose up -d
   ```

## Integration with External Systems

### Sending to Elasticsearch

```bash
# Use Filebeat or Logstash to ship logs
docker-compose logs --no-color | \
  grep -E '^\{.*\}$' | \
  curl -X POST "http://elasticsearch:9200/logs/_bulk" \
    -H "Content-Type: application/x-ndjson" \
    --data-binary @-
```

### Sending to Splunk

```bash
# Configure Splunk HTTP Event Collector
docker-compose logs --no-color | \
  grep -E '^\{.*\}$' | \
  curl -X POST "https://splunk:8088/services/collector" \
    -H "Authorization: Splunk <token>" \
    -d @-
```

### Sending to CloudWatch

Use the `awslogs` driver instead of `json-file`:

```yaml
logging:
  driver: "awslogs"
  options:
    awslogs-region: "us-east-1"
    awslogs-group: "jessy-logs"
    awslogs-stream: "jessy-core"
```

## References

- [Tracing Documentation](https://docs.rs/tracing/)
- [Zerolog Documentation](https://github.com/rs/zerolog)
- [Docker Logging](https://docs.docker.com/config/containers/logging/)
- [JSON Lines Format](https://jsonlines.org/)

---

*"Logs tell the story of your system. Make them readable, searchable, and actionable."*
