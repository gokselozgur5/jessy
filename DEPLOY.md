# JESSY Deployment Guide

## Deploy to Render.com (Recommended - Free Tier)

### Prerequisites
- GitHub account
- Anthropic API key

### Steps

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

### Auto-Deploy
- Every `git push` to `main` branch triggers automatic deployment
- Render monitors your GitHub repo

---

## Alternative: Railway.app

### Steps

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

## Alternative: Fly.io

### Steps

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

## Health Check

After deployment, test your service:

```bash
# Health check
curl https://your-app.onrender.com/api/health

# Expected response:
{"status":"ok","timestamp":"2025-11-01T..."}
```

---

## Troubleshooting

### Build Fails
- Check Rust version in Dockerfile (should be 1.83+)
- Ensure `data/` directory exists and contains dimension files
- Verify `benches/` directory is included

### 500 Error on /api/chat
- Check `ANTHROPIC_API_KEY` is set correctly
- View logs in Render dashboard
- Ensure API key is valid

### Memory Issues (Free Tier)
- Render free tier: 512MB RAM
- Railway free tier: 512MB RAM
- If exceeded, upgrade or optimize

---

## Cost Comparison

| Platform | Free Tier | Build Time | Auto-Deploy | Custom Domain |
|----------|-----------|------------|-------------|---------------|
| **Render** | 512MB, 750hrs/mo | ~8 min | ✅ | ✅ |
| **Railway** | 512MB, $5 credit/mo | ~8 min | ✅ | ✅ |
| **Fly.io** | 3 VMs free | ~8 min | ✅ | ✅ |

**Recommendation**: Start with Render.com (easiest, no credit card for free tier).
