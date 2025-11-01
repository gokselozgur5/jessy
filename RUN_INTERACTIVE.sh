#!/bin/bash
# JESSY Interactive Framework - Live Conversation
#
# Educational tool for multi-dimensional cognition exploration
#
# Features:
# - Type messages in real-time
# - AI responds with progressive framework engagement
# - Context accumulates across conversation
# - TR-EN code-switching supported
# - Type "exit" to quit

echo "🧠 JESSY Interactive Framework"
echo "==============================="
echo ""
echo "✨ Features:"
echo "  → Natural conversation with AI"
echo "  → Context builds throughout session"
echo "  → TR-EN code-switching (bilingual communication)"
echo "  → Progressive framework engagement"
echo "  → Type 'exit' or 'quit' to end"
echo ""
echo "💡 Tips:"
echo "  → TR-EN mixing supported (kanka, nedir, etc.)"
echo "  → Explore dimensions, frequencies, consciousness"
echo "  → By message 5-6, full framework engagement"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Set API key
export ANTHROPIC_API_KEY="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"

# Navigate to project
cd /Users/gok/Projects/DeepCode/jessy

# Bypass cargo function
unset -f cargo

# Run interactive demo
/Users/gok/.cargo/bin/cargo run --quiet --example jessy_interactive 2>&1 | grep -v "warning:"
