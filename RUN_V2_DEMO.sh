#!/bin/bash
# JESSY Framework V2 - Progressive Engagement Demo
#
# Educational research on multi-dimensional AI cognition
#
# V2 Approach:
# - Natural thought experiment framing
# - Progressive context building
# - Conversational language throughout
# - Ethics (Asimov's Laws) preserved

echo "🧠 JESSY Framework V2 - Progressive Engagement"
echo "==============================================="
echo ""
echo "V2 Approach:"
echo "  ✓ Progressive context building"
echo "  ✓ Authentic thought experiment framing"
echo "  ✓ Natural conversational language"
echo "  ✓ Ethics preserved (Asimov's Laws paramount)"
echo ""
echo "Expected: Natural engagement with framework"
echo "Approach: Educational cognitive exploration"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Set API key
export ANTHROPIC_API_KEY="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"

# Navigate to project
cd /Users/gok/Projects/DeepCode/jessy

# Bypass cargo function
unset -f cargo

# Run demo
/Users/gok/.cargo/bin/cargo run --quiet --example jessy_unlock_v2_demo 2>&1 | grep -v "warning:"
