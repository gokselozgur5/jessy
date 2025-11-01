#!/bin/bash
# Run LLM-Based Interactive Chat
#
# Interactive chat with LLM dimension selection (intent-based)
# No keyword matching - pure intent understanding

cd "$(dirname "$0")"

# Load .env if exists
if [ -f .env ]; then
    echo "📝 Loading .env file..."
    export $(grep -v '^#' .env | xargs)
fi

# Check API key
if [ -z "$ANTHROPIC_API_KEY" ]; then
    echo "❌ ANTHROPIC_API_KEY not set"
    echo "Add to .env: ANTHROPIC_API_KEY=your-key-here"
    echo "Or run: export ANTHROPIC_API_KEY='your-key-here'"
    exit 1
fi

echo "🚀 Starting LLM-based interactive chat..."
echo ""

# Bypass cargo function if it exists
unset -f cargo 2>/dev/null

# Run chat
/Users/gok/.cargo/bin/cargo run --release --example llm_chat_demo
