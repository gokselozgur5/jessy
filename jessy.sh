#!/bin/bash
# Run Jessy Voice Mode
#
# Usage:
#   ./jessy.sh              - Normal mode (Claude Haiku)
#   ./jessy.sh creative     - Creative mode (GPT-4o, unrestricted fiction)
#   ./jessy.sh --creative   - Same as above

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export DYLD_LIBRARY_PATH="$PROJECT_ROOT/native/sherpa/lib:$DYLD_LIBRARY_PATH"

# Check for creative mode flag
if [[ "$1" == "creative" ]] || [[ "$1" == "--creative" ]]; then
    export JESSY_CREATIVE_MODE=true
    echo "🎭 Starting Jessy Voice Interface - CREATIVE MODE"
    echo "   (Claude Sonnet 4 enabled for unrestricted fiction generation)"
else
    echo "🚀 Starting Jessy Voice Interface - NORMAL MODE"
    echo "   (Claude Haiku enabled)"
fi

echo "⚠️  Make sure your microphone is active."
echo ""

cargo run --bin jessy-voice
