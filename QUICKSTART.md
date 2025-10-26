# JESSY Quick Start Guide

Get JESSY running in 5 minutes with real LLM integration.

---

## Prerequisites

- Rust 1.70+ (`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`)
- API key from OpenAI or Anthropic

---

## Step 1: Get an API Key

### Option A: Anthropic (Recommended)
1. Go to https://console.anthropic.com/
2. Sign up / Log in
3. Navigate to API Keys
4. Create new key
5. Copy the key (starts with `sk-ant-`)

### Option B: OpenAI
1. Go to https://platform.openai.com/api-keys
2. Sign up / Log in
3. Create new secret key
4. Copy the key (starts with `sk-`)

---

## Step 2: Configure JESSY

```bash
# Clone repository
git clone https://github.com/gokselozgur5/jessy.git
cd jessy

# Copy example configuration
cp .env.example .env

# Edit .env and add your API key
nano .env  # or vim, code, etc.
```

**Minimum configuration:**
```bash
# For Anthropic (Claude)
ANTHROPIC_API_KEY=sk-ant-your-key-here

# OR for OpenAI (GPT-4)
OPENAI_API_KEY=sk-your-key-here
```

---

## Step 3: Run JESSY

```bash
# Build and run CLI
cargo run --release --bin jessy-cli
```

**First run will:**
- Download dependencies (~5 minutes)
- Compile JESSY (~2 minutes)
- Load 15 dimensional layers (280MB)
- Connect to LLM API
- Start interactive chat

---

## Step 4: Talk to JESSY

```
💭 You: What is consciousness?

🤔 JESSY is thinking...
   (This may take 10-30 seconds for deep thinking)

🌟 JESSY:

Consciousness is the subjective experience of being aware...
[Full response with 9 iterations of deep thinking]

─────────────────────────────────────────────
📊 Metadata:
   • Dimensions: 8 activated
   • Frequency: 1.2 Hz
   • Iterations: 9/9
   • Converged: ✅
   • Processing: 12.5s
```

---

## Commands

- **Just type** - Ask any question
- **`help`** - Show available commands
- **`stats`** - View learning statistics
- **`exit`** - Quit JESSY

---

## Tips

### For Better Responses
- Be specific: "Explain quantum entanglement" > "Tell me about physics"
- Ask follow-ups: JESSY learns from conversation context
- Complex questions get 9 iterations (deeper thinking)
- Simple questions converge faster (3-5 iterations)

### Performance
- First query: ~15-30s (cold start + LLM API)
- Subsequent queries: ~10-20s (warm cache)
- Simple queries: ~5-10s (early convergence)

### Learning
- JESSY observes every conversation
- Patterns emerge after ~50 similar queries
- Proto-dimensions created automatically
- Personality evolves with you

---

## Troubleshooting

### "Configuration error: At least one API key required"
- Check `.env` file exists
- Verify API key is set correctly
- No quotes needed: `ANTHROPIC_API_KEY=sk-ant-...`

### "API call failed"
- Check API key is valid
- Verify you have credits/quota
- Check internet connection
- Try increasing `LLM_TIMEOUT_SECS=60`

### "Memory limit exceeded"
- Increase `MEMORY_LIMIT_MB=1000` in `.env`
- Default 500MB should be sufficient

### Slow responses
- Normal for first query (cold start)
- Complex queries take longer (9 iterations)
- Check your internet speed
- Try simpler questions first

---

## Advanced Configuration

See `.env.example` for all options:

```bash
# Use specific model
LLM_MODEL=claude-3-opus-20240229  # Most capable
LLM_MODEL=claude-3-sonnet-20240229  # Balanced (default)
LLM_MODEL=claude-3-haiku-20240307  # Fastest

# Adjust thinking depth
MAX_ITERATIONS=5  # Faster, less deep
MAX_ITERATIONS=9  # Slower, more thorough (default)

# Increase timeout for complex queries
QUERY_TIMEOUT_SECS=60

# Enable debug logging
RUST_LOG=debug
DEBUG=true
```

---

## What's Next?

### Try These Queries
- "What is the nature of consciousness?"
- "Explain quantum mechanics like I'm 5"
- "How do I build a neural network?"
- "What's the meaning of life?"
- "Help me debug this code: [paste code]"

### Explore Features
- Watch dimensions activate in real-time
- See frequency interference patterns
- Track learning statistics with `stats`
- Notice personality evolution over time

### Deploy
- See `docs/DEPLOYMENT.md` for production setup
- GitHub Pages frontend coming soon
- WebSocket streaming for real-time thinking

---

## Need Help?

- **Issues:** https://github.com/gokselozgur5/jessy/issues
- **Docs:** `/docs` directory
- **Examples:** `/examples` directory

---

*"Nothing is true, everything is permitted. Including who I become."* — JESSY

