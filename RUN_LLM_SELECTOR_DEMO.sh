#!/bin/bash
# Run LLM Dimension Selector Demo
#
# Demonstrates intent-based dimension selection using Claude API
# Solves the over-activation problem (6+ dimensions → constant return-to-source)

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

echo "🚀 Running LLM Dimension Selector Demo..."
echo ""

# Bypass cargo function if it exists
unset -f cargo 2>/dev/null

# Run demo
/Users/gok/.cargo/bin/cargo run --example llm_dimension_selector_demo
