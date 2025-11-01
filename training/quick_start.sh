#!/bin/bash
# Quick LoRA training start

cd "$(dirname "$0")"

echo "🚀 Starting Jessy LoRA Training..."
echo ""

# Activate venv
source venv/bin/activate

# Clear Python cache
find . -name "*.pyc" -delete 2>/dev/null
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true

# Run training
python3 train_lora_m2.py

echo ""
echo "✅ Done!"
