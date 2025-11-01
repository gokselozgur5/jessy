#!/bin/bash
# Run LLM-Based Interactive Chat with Full Integration
#
# Full integration:
# - LLM dimension selection (claude-haiku-4-5)
# - Memory context loading
# - Contextual response generation

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

echo "🚀 Starting LLM-based interactive chat (full integration)..."
echo ""
echo "⚡ Dimension selection: claude-haiku-4-5 (~1 second)"
echo "🧠 Response generation: claude-sonnet-4 (~10 seconds)"
echo ""

# Bypass cargo function if it exists
unset -f cargo 2>/dev/null

# Run interactive chat
/Users/gok/.cargo/bin/cargo run --release --example llm_interactive_full
