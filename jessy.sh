#!/bin/bash
# Run Jessy Voice Mode

PROJECT_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export DYLD_LIBRARY_PATH="$PROJECT_ROOT/native/sherpa/lib:$DYLD_LIBRARY_PATH"

echo "🚀 Starting Jessy Voice Interface..."
echo "⚠️  Make sure your microphone is active."

cargo run --bin jessy-voice
