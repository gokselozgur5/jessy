#!/bin/bash
# Quick start script for Jessy LoRA training on M2

set -e

echo "🚀 Jessy LoRA Training - M2 Optimized"
echo "======================================"
echo ""

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "❌ Virtual environment not found!"
    echo "Run: python3 -m venv venv"
    exit 1
fi

# Activate venv
echo "📦 Activating virtual environment..."
source venv/bin/activate

# Check datasets
if [ ! -f "datasets/jessy_maximum_train.jsonl" ]; then
    echo "❌ Training dataset not found!"
    echo "Expected: datasets/jessy_maximum_train.jsonl"
    exit 1
fi

if [ ! -f "datasets/jessy_maximum_val.jsonl" ]; then
    echo "❌ Validation dataset not found!"
    echo "Expected: datasets/jessy_maximum_val.jsonl"
    exit 1
fi

echo "✅ Datasets found"
echo ""

# Check if model is already downloaded
echo "📥 Checking model..."
python3 -c "from transformers import AutoTokenizer; AutoTokenizer.from_pretrained('meta-llama/Llama-3.2-3B-Instruct')" 2>/dev/null && echo "✅ Model ready" || echo "⏳ Model will be downloaded (first time only)"
echo ""

# Estimate time
TRAIN_SAMPLES=$(wc -l < datasets/jessy_maximum_train.jsonl)
echo "📊 Training Info:"
echo "   Samples: $TRAIN_SAMPLES"
echo "   Epochs: 3"
echo "   Estimated time: 6-8 hours on M2"
echo ""

# Confirm
read -p "Start training? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Cancelled."
    exit 0
fi

# Create output directories
mkdir -p models/jessy-lora-m2
mkdir -p checkpoints/jessy-lora

# Start training
echo ""
echo "🎯 Starting training..."
echo "======================================"
echo ""

python3 train_lora_m2.py 2>&1 | tee training_log.txt

echo ""
echo "✅ Training complete!"
echo "📁 Model saved to: models/jessy-lora-m2"
echo "📝 Log saved to: training_log.txt"
