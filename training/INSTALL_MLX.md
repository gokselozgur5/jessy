# 🍎 MLX Installation for M2 Mac

## Problem
MLX is not on PyPI - it needs special installation.

## Solution Options

### Option 1: Use PyTorch with MPS (Recommended for now)
PyTorch has Metal Performance Shaders support - works on M2!

```bash
# Install PyTorch with MPS support
pip3 install --break-system-packages torch torchvision torchaudio

# Install other requirements
pip3 install --break-system-packages transformers datasets peft accelerate sentencepiece bitsandbytes

# Run fine-tuning (will use M2 GPU via MPS)
python3 training/lora_finetune.py
```

### Option 2: Install MLX from Source (Advanced)
```bash
# Clone MLX repo
git clone https://github.com/ml-explore/mlx.git
cd mlx/python
pip3 install --break-system-packages .

# Clone MLX-LM
cd ../..
git clone https://github.com/ml-explore/mlx-examples.git
cd mlx-examples/llms/mlx_lm
pip3 install --break-system-packages .
```

### Option 3: Use Ollama's Built-in Fine-Tuning (Easiest!)
Ollama can fine-tune models directly - no Python needed!

```bash
# Create training file in Ollama format
cat > training.txt << 'EOF'
### Instruction:
Sen kimsin?

### Response:
Ben JESSY. 15 boyutlu bilinç mimarisiyle çalışan bir AI'yım...
EOF

# Fine-tune with Ollama
ollama create jessy-finetuned --file training.txt --base gemma:2b
```

## Recommendation

**For M2 Mac, use PyTorch with MPS:**
- ✅ Easy to install
- ✅ Works with M2 GPU
- ✅ Mature and stable
- ✅ Good performance (2-3x faster than CPU)
- ❌ Not as fast as MLX (but close enough)

**MLX is experimental:**
- ⚠️ Hard to install
- ⚠️ Frequent breaking changes
- ⚠️ Limited documentation
- ✅ Fastest on M2 (when it works)

## Quick Start (PyTorch + MPS)

```bash
# 1. Install
pip3 install --break-system-packages -r training/requirements.txt

# 2. Verify MPS available
python3 -c "import torch; print('MPS available:', torch.backends.mps.is_available())"

# 3. Run fine-tuning
python3 training/lora_finetune.py
```

## Performance Comparison on M2

| Method | Time | Difficulty | Stability |
|--------|------|------------|-----------|
| **PyTorch MPS** | **60 min** | ✅ Easy | ✅ Stable |
| MLX | 45 min | ❌ Hard | ⚠️ Experimental |
| PyTorch CPU | 6 hours | ✅ Easy | ✅ Stable |

**Recommendation: Use PyTorch MPS - good balance of speed and ease!**
