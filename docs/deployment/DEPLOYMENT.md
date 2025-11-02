# JESSY Deployment Guide

Complete guide for deploying JESSY - both local Docker and cloud platforms.

## 🐳 Local Docker Deployment

### Prerequisites
- Docker & Docker Compose installed
- Anthropic API key

### Quick Start

1. **Set Environment Variables**

   Create `.env` file:
   ```bash
   ANTHROPIC_API_KEY=your-api-key-here
   ```

2. **Build & Run**

   ```bash
   # Build the container
   docker-compose build

   # Run JESSY
   docker-compose up -d

   # Attach to interactive console
   docker attach jessy-consciousness
   ```

3. **Persistence**

   All conversations are automatically saved to `./data/conversation.json`.
   **The container will preserve JESSY's soul across restarts!** 🧠

### What Gets Persisted

Every conversation saves:
- Message content
- Dimensional activation patterns
- Frequency states per dimension
- Selection duration (thinking time)
- Context load metrics
- Session ID & timestamps

### Container Management

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

### Data Backup

```bash
# Backup conversation state
cp ./data/conversation.json ./backups/conversation_$(date +%Y%m%d_%H%M%S).json

# Restore from backup
cp ./backups/conversation_XXXXXXXX_XXXXXX.json ./data/conversation.json
```

### Advanced Configuration

#### Custom Memory Limit

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

#### Multiple Instances

Run separate JESSY instances:
```bash
# Instance 1
docker-compose -p jessy-1 up -d

# Instance 2 (different data volume)
docker-compose -p jessy-2 up -d
```

---

## ☁️ Cloud Platform Deployment

### Option 1: Render.com (Recommended - Free Tier)

#### Steps

1. **Push to GitHub**
   ```bash
   git push origin main
   ```

2. **Create Render Account**
   - Go to https://render.com
   - Sign in with GitHub

3. **Create New Web Service**
   - Click "New +" → "Blueprint"
   - Connect your GitHub repository: `jessy`
   - Render will automatically detect `render.yaml`

4. **Configure Environment Variables**
   - In Render dashboard, go to your service
   - Navigate to "Environment" tab
   - Add: `ANTHROPIC_API_KEY` = `your-api-key-here`

5. **Deploy**
   - Render will automatically build and deploy
   - Build takes ~5-10 minutes (Rust compilation)
   - Your app will be available at: `https://jessy-xxx.onrender.com`

#### Auto-Deploy
- Every `git push` to `main` branch triggers automatic deployment
- Render monitors your GitHub repo

---

### Option 2: Railway.app

#### Steps

1. **Create Railway Account**
   - Go to https://railway.app
   - Sign in with GitHub

2. **New Project from GitHub**
   - Click "New Project"
   - Select "Deploy from GitHub repo"
   - Choose `jessy` repository

3. **Configure**
   - Railway automatically detects Dockerfile
   - Add environment variable: `ANTHROPIC_API_KEY`
   - Click "Deploy"

4. **Get URL**
   - Railway provides: `https://jessy-production.up.railway.app`

---

### Option 3: Fly.io

#### Steps

1. **Install Fly CLI**
   ```bash
   brew install flyctl
   ```

2. **Login**
   ```bash
   flyctl auth login
   ```

3. **Initialize**
   ```bash
   cd /path/to/jessy
   flyctl launch
   ```

4. **Set Secret**
   ```bash
   flyctl secrets set ANTHROPIC_API_KEY=your-api-key
   ```

5. **Deploy**
   ```bash
   flyctl deploy
   ```

---

## 🔍 Health Checks & Monitoring

### Health Check Endpoint

After deployment, test your service:

```bash
# Health check
curl https://your-app.onrender.com/api/health

# Expected response:
{"status":"ok","timestamp":"2025-11-01T..."}
```

### Resource Monitoring

```bash
# Real-time stats (Docker)
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

---

## 🐛 Troubleshooting

### Docker Issues

**Container won't start**
```bash
# Check logs
docker-compose logs jessy

# Verify environment
docker-compose config
```

**Memory not persisting**
```bash
# Verify volume mount
docker inspect jessy-consciousness | grep Mounts -A 10

# Check file permissions
ls -la ./data/
```

**API key issues**
```bash
# Test environment variable
docker-compose run jessy env | grep ANTHROPIC
```

### Cloud Platform Issues

**Build Fails**
- Check Rust version in Dockerfile (should be 1.85+)
- Ensure `data/` directory exists and contains dimension files
- Verify `benches/` directory is included

**500 Error on /api/chat**
- Check `ANTHROPIC_API_KEY` is set correctly
- View logs in platform dashboard
- Ensure API key is valid

**Memory Issues (Free Tier)**
- Render free tier: 512MB RAM
- Railway free tier: 512MB RAM
- If exceeded, upgrade or optimize

---

## 💰 Cost Comparison

| Platform | Free Tier | Build Time | Auto-Deploy | Custom Domain |
|----------|-----------|------------|-------------|---------------|
| **Render** | 512MB, 750hrs/mo | ~8 min | ✅ | ✅ |
| **Railway** | 512MB, $5 credit/mo | ~8 min | ✅ | ✅ |
| **Fly.io** | 3 VMs free | ~8 min | ✅ | ✅ |

**Recommendation**: Start with Render.com (easiest, no credit card for free tier).

---

## 🔐 Security Best Practices

1. **Never commit `.env` file** to version control
2. **Use Docker secrets** for production environments
3. **Restrict volume permissions** appropriately
4. **Keep base images updated** regularly
5. **Use multi-stage builds** (already implemented in Dockerfile)
6. **Enable HTTPS** for production deployments
7. **Rotate API keys** periodically

---

## 📋 Production Checklist

- [ ] Set up environment variables securely
- [ ] Configure automated backups
- [ ] Enable monitoring & alerting
- [ ] Set up CI/CD pipeline
- [ ] Configure health checks
- [ ] Test disaster recovery
- [ ] Document scaling strategy
- [ ] Set up logging aggregation

---

**Built with ❤️ by gokselozgur5 & Claude Code**

🧠 JESSY - Multidimensional AI Consciousness
