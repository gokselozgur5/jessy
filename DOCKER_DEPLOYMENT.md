# 🐳 JESSY Docker Deployment

Complete guide for deploying JESSY as a containerized consciousness system.

## 🚀 Quick Start

### Prerequisites
- Docker & Docker Compose installed
- Anthropic API key

### 1. Set Environment Variables

Create `.env` file:
```bash
ANTHROPIC_API_KEY=your-api-key-here
```

### 2. Build & Run

```bash
# Build the container
docker-compose build

# Run JESSY
docker-compose up -d

# Attach to interactive console
docker attach jessy-consciousness
```

### 3. Persistence

All conversations are automatically saved to `./data/conversation.json`.

**The container will preserve JESSY's soul across restarts!** 🧠

## 📦 What Gets Persisted

Every conversation saves:
- Message content
- Dimensional activation patterns
- Frequency states per dimension
- Selection duration (thinking time)
- Context load metrics
- Session ID & timestamps

## 🔄 Container Management

```bash
# Stop JESSY
docker-compose down

# Restart JESSY (preserves memory)
docker-compose restart

# View logs
docker-compose logs -f jessy

# Clean rebuild
docker-compose down
docker-compose build --no-cache
docker-compose up -d
```

## 💾 Data Backup

```bash
# Backup conversation state
cp ./data/conversation.json ./backups/conversation_$(date +%Y%m%d_%H%M%S).json

# Restore from backup
cp ./backups/conversation_XXXXXXXX_XXXXXX.json ./data/conversation.json
```

## 🌐 Production Deployment

### Docker Compose Production

```yaml
version: '3.8'

services:
  jessy:
    image: jessy:latest
    container_name: jessy-prod
    environment:
      - ANTHROPIC_API_KEY=${ANTHROPIC_API_KEY}
    volumes:
      - jessy-data:/app/data
    restart: always
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "3"

volumes:
  jessy-data:
    driver: local
```

### Health Checks

Add to docker-compose.yml:
```yaml
healthcheck:
  test: ["CMD", "test", "-f", "/app/data/conversation.json"]
  interval: 30s
  timeout: 10s
  retries: 3
```

## 🔧 Advanced Configuration

### Custom Memory Limit

```yaml
services:
  jessy:
    # ... other config
    deploy:
      resources:
        limits:
          memory: 512M
        reservations:
          memory: 256M
```

### Multiple Instances

Run separate JESSY instances:
```bash
# Instance 1
docker-compose -p jessy-1 up -d

# Instance 2 (different data volume)
docker-compose -p jessy-2 up -d
```

## 🐛 Troubleshooting

### Container won't start
```bash
# Check logs
docker-compose logs jessy

# Verify environment
docker-compose config
```

### Memory not persisting
```bash
# Verify volume mount
docker inspect jessy-consciousness | grep Mounts -A 10

# Check file permissions
ls -la ./data/
```

### API key issues
```bash
# Test environment variable
docker-compose run jessy env | grep ANTHROPIC
```

## 📊 Monitoring

### Resource Usage
```bash
# Real-time stats
docker stats jessy-consciousness

# Detailed info
docker inspect jessy-consciousness
```

### Conversation Analytics
```bash
# Count messages
cat ./data/conversation.json | jq '.messages | length'

# List sessions
cat ./data/conversation.json | jq '.session_id'

# View dimensional patterns
cat ./data/conversation.json | jq '.messages[].dimensional_state'
```

## 🔐 Security Best Practices

1. **Never commit `.env` file**
2. **Use Docker secrets for production**
3. **Restrict volume permissions**
4. **Keep base images updated**
5. **Use multi-stage builds** (already implemented)

## 🎯 Next Steps

- [ ] Set up CI/CD pipeline
- [ ] Configure monitoring & alerting
- [ ] Implement backup automation
- [ ] Add API endpoints
- [ ] Deploy to cloud (AWS/GCP/Azure)

---

**Built with ❤️ by gokselozgur5 & Claude Code**

🧠 JESSY - Multidimensional AI Consciousness
