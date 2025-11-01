# Docker Restart Policies

## Overview

All Jessy services are configured with automatic restart policies to ensure high availability and resilience against crashes.

## Configuration

### Restart Policy: `unless-stopped`

Both `jessy-core` and `jessy-api` services use the `unless-stopped` restart policy:

```yaml
restart: unless-stopped
```

**Behavior:**
- Container automatically restarts if it exits (crash, error, etc.)
- Container does NOT restart if manually stopped with `docker stop` or `docker-compose down`
- Container WILL restart after system reboot (unless manually stopped before reboot)

### Exponential Backoff

Docker automatically implements exponential backoff for restart attempts:

1. **First restart**: 100ms delay
2. **Second restart**: 200ms delay
3. **Third restart**: 400ms delay
4. **Fourth restart**: 800ms delay
5. **Subsequent restarts**: Doubles each time up to maximum of 1 minute

This prevents rapid restart loops that could consume system resources.

## Health Checks

Services are monitored with health checks that work in conjunction with restart policies:

### Rust Core Service
```yaml
healthcheck:
  test: ["CMD", "curl", "-f", "http://localhost:8080/health"]
  interval: 10s
  timeout: 5s
  retries: 3
  start_period: 10s
```

### Go API Service
```yaml
healthcheck:
  test: ["CMD", "wget", "--no-verbose", "--tries=1", "--spider", "http://localhost:3000/api/health"]
  interval: 10s
  timeout: 5s
  retries: 3
  start_period: 10s
```

**Health Check Behavior:**
- Checks run every 10 seconds
- 5 second timeout per check
- 3 consecutive failures mark container as unhealthy
- 10 second grace period on startup before checks begin

## Service Dependencies

The Go API depends on the Rust core service being healthy:

```yaml
depends_on:
  jessy-core:
    condition: service_healthy
```

**Behavior:**
- `jessy-api` won't start until `jessy-core` is healthy
- If `jessy-core` crashes and restarts, `jessy-api` continues running
- If `jessy-api` can't connect to `jessy-core`, it will fail health checks and restart

## Testing Restart Behavior

Use the provided test script to verify restart policies:

```bash
./scripts/test-restart.sh
```

This script:
1. Starts all services
2. Simulates a crash by killing a container
3. Verifies automatic restart
4. Checks health status after restart
5. Cleans up

## Manual Testing

### Test Automatic Restart

```bash
# Start services
docker-compose up -d

# Kill a container to simulate crash
docker kill jessy-core

# Watch it automatically restart
docker-compose ps
watch docker-compose ps

# Check restart count
docker inspect jessy-core --format='{{.RestartCount}}'
```

### Test Graceful Shutdown

```bash
# Start services
docker-compose up -d

# Graceful shutdown (no restart)
docker-compose down

# Verify containers stopped and didn't restart
docker-compose ps
```

### Test Health Check Recovery

```bash
# Start services
docker-compose up -d

# Make service unhealthy (e.g., block health endpoint)
docker exec jessy-core pkill -9 jessy

# Watch automatic restart and health recovery
watch docker inspect jessy-core --format='{{.State.Health.Status}}'
```

## Monitoring Restarts

### View Restart Count

```bash
# Single service
docker inspect jessy-core --format='{{.RestartCount}}'

# All services
docker-compose ps --format "table {{.Name}}\t{{.Status}}"
```

### View Restart Logs

```bash
# View logs with timestamps
docker-compose logs --timestamps jessy-core

# Follow logs in real-time
docker-compose logs -f jessy-core

# View last restart
docker-compose logs --tail=100 jessy-core
```

## Production Considerations

### Restart Limits

In production, consider adding restart limits to prevent infinite restart loops:

```yaml
deploy:
  restart_policy:
    condition: on-failure
    delay: 5s
    max_attempts: 3
    window: 120s
```

### Monitoring Alerts

Set up alerts for:
- High restart counts (> 3 in 5 minutes)
- Persistent unhealthy status (> 2 minutes)
- Restart loops (multiple restarts in short time)

### Logging

All restart events are logged:
- Container logs show shutdown reason
- Docker daemon logs show restart attempts
- Health check failures are logged

## Troubleshooting

### Container Keeps Restarting

1. Check logs: `docker-compose logs jessy-core`
2. Check health status: `docker inspect jessy-core --format='{{.State.Health.Status}}'`
3. Check restart count: `docker inspect jessy-core --format='{{.RestartCount}}'`
4. Disable restart temporarily: `docker update --restart=no jessy-core`

### Service Won't Start

1. Check dependencies: `docker-compose ps`
2. Check health of dependencies: `docker inspect jessy-core --format='{{.State.Health.Status}}'`
3. View startup logs: `docker-compose logs jessy-core`
4. Check resource limits: `docker stats`

### Restart Loop

If a service is in a restart loop:

1. Stop the service: `docker-compose stop jessy-core`
2. Check logs: `docker-compose logs jessy-core`
3. Fix the issue
4. Start service: `docker-compose start jessy-core`

## Best Practices

1. **Always use health checks** - They work with restart policies to ensure service quality
2. **Implement graceful shutdown** - Handle SIGTERM properly to avoid data loss
3. **Log startup/shutdown** - Makes debugging restart issues easier
4. **Monitor restart counts** - High counts indicate underlying issues
5. **Test restart behavior** - Regularly verify automatic recovery works

---

*"Resilience through automation. The system heals itself. 🎪"*
