# Health Checks Quick Reference

## Quick Commands

### Check Health Status
```bash
# Rust service
curl http://localhost:8080/health

# Go service
curl http://localhost:3000/api/health

# Docker health status
docker inspect --format='{{.State.Health.Status}}' jessy-core
docker inspect --format='{{.State.Health.Status}}' jessy-api
```

### Start Services
```bash
# Start all services
docker-compose up -d

# Start with rebuild
docker-compose up --build -d

# View logs
docker-compose logs -f jessy-core jessy-api
```

### Run Tests
```bash
# Automated health check tests
./scripts/test-health-checks.sh

# Manual verification
docker-compose ps
```

### Troubleshooting
```bash
# View health check logs
docker inspect jessy-core | jq '.[0].State.Health.Log[-5:]'

# Check service logs
docker-compose logs jessy-core

# Restart unhealthy service
docker-compose restart jessy-core

# Force rebuild
docker-compose down -v
docker-compose up --build
```

## Health Endpoints

| Service | Endpoint | Port | Tool |
|---------|----------|------|------|
| jessy-core | `/health` | 8080 | curl |
| jessy-api | `/api/health` | 3000 | wget |

## Health Check Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| interval | 10s | Check frequency |
| timeout | 5s | Max check duration |
| retries | 3 | Failures before unhealthy |
| start_period | 10s | Startup grace period |

## Expected Responses

### Rust Service
```json
{
  "status": "healthy",
  "service": "jessy-core",
  "version": "0.1.0",
  "timestamp": "2024-10-24T10:30:00Z"
}
```

### Go Service
```json
{
  "status": "healthy",
  "service": "jessy-api",
  "version": "1.0.0"
}
```

## Service Dependencies

```
jessy-api → depends on → jessy-core (healthy)
```

- jessy-core must be healthy before jessy-api starts
- If jessy-core becomes unhealthy, jessy-api can detect it
- Both services restart automatically on failure

## Common Issues

### Service Shows Unhealthy
1. Check logs: `docker-compose logs <service>`
2. Test endpoint: `curl http://localhost:<port>/health`
3. Restart: `docker-compose restart <service>`

### Service Won't Start
1. Check dependencies: `docker-compose config`
2. Verify health checks: `docker inspect <container>`
3. Rebuild: `docker-compose up --build`

### Health Check Timeout
1. Increase timeout in docker-compose.yml
2. Increase start_period for slow startup
3. Check service performance

## Files

- **Documentation**: `docs/HEALTH_CHECKS.md`
- **Test Script**: `scripts/test-health-checks.sh`
- **Configuration**: `docker-compose.yml`
- **Rust Dockerfile**: `docker/Dockerfile.rust`
- **Go Dockerfile**: `docker/Dockerfile.go`

---

For detailed information, see `docs/HEALTH_CHECKS.md`
