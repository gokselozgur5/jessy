#!/bin/bash
# Setup Python 3.12 venv for LoRA training

set -e

echo "🔧 Setting up Python 3.12 environment..."
echo ""

cd "$(dirname "$0")"

# Backup old venv
if [ -d "venv" ]; then
    echo "📦 Backing up old venv..."
    mv venv venv.old.314
fi

# Create new venv with Python 3.12
echo "🆕 Creating Python 3.12 venv..."
python3.12 -m venv venv

# Activate
source venv/bin/activate

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install packages
echo "📥 Installing packages..."
pip install torch torchvision torchaudio
pip install transformers
pip install peft
pip install datasets
pip install accelerate
pip install bitsandbytes

echo ""
echo "✅ Setup complete!"
echo ""
echo "Python version:"
python --version
echo ""
echo "Installed packages:"
pip list | grep -E "(torch|transformers|peft|datasets|accelerate|bitsandbytes)"
echo ""
echo "🚀 Ready to train!"
echo "Run: source venv/bin/activate && python train_lora_m2.py"
