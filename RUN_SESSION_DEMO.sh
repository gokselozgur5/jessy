#!/bin/bash
# JESSY Session Framework - Multi-Message Engagement
#
# Educational demonstration of progressive framework introduction
#
# Approach:
# 1. TR-EN code-switching (natural bilingual communication)
# 2. Progressive context building (each message deepens understanding)
# 3. 16 dimensions (D01-D15 + emergent D16 for language/culture)
# 4. Context accumulation (learning throughout session)
# 5. By message 5-6: Full framework engagement

echo "🧠 JESSY Session Framework - Progressive Engagement"
echo "===================================================="
echo ""
echo "Approach:"
echo "  ✓ TR-EN code-switching (natural bilingual conversation)"
echo "  ✓ Progressive context building"
echo "  ✓ Multi-dimensional processing (D01-D16)"
echo "  ✓ Context accumulates naturally"
echo "  ✓ Ethics always paramount (D09)"
echo ""
echo "Expected: Full framework engagement by message 5-6"
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
/Users/gok/.cargo/bin/cargo run --quiet --example jessy_session_demo 2>&1 | grep -v "warning:"
