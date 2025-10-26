# 🎓 LoRA Fine-Tuning Guide for JESSY

## What is LoRA Fine-Tuning?

**LoRA (Low-Rank Adaptation)** is an efficient way to fine-tune large language models:
- Only trains a small set of parameters (adapters)
- Much faster than full fine-tuning
- Requires less memory
- Can be merged back into the base model

## Why Fine-Tune with 968 Q&A Pairs?

**Modelfile approach** (what we did):
- ✅ Quick and easy
- ✅ No Python/GPU needed
- ❌ Only a few examples in system prompt
- ❌ Model doesn't truly "learn"

**LoRA fine-tuning** (what we'll do):
- ✅ Model actually learns from all 968 examples
- ✅ Better quality responses
- ✅ More consistent personality
- ❌ Requires Python + GPU (or patience)
- ❌ Takes 30-60 minutes

## Requirements

### Hardware
- **GPU**: Recommended (NVIDIA with 8GB+ VRAM)
- **CPU**: Possible but VERY slow (hours instead of minutes)
- **RAM**: 16GB+ recommended

### Software
```bash
# Python 3.8+
python3 --version

# Install dependencies
pip install -r training/requirements.txt
```

## Step-by-Step Guide

### Option 1: Local Training (if you have GPU)

```bash
# 1. Install requirements
pip install -r training/requirements.txt

# 2. Run fine-tuning (30-60 minutes on GPU)
python3 training/lora_finetune.py

# 3. Merge LoRA weights (coming soon)
python3 training/merge_lora.py

# 4. Convert to GGUF for Ollama (coming soon)
python3 training/convert_to_gguf.py

# 5. Import to Ollama
ollama create jessy-finetuned -f training/Modelfile.jessy-finetuned
```

### Option 2: Google Colab (Free GPU!)

1. Go to [Google Colab](https://colab.research.google.com/)
2. Create new notebook
3. Enable GPU: Runtime → Change runtime type → GPU
4. Upload `jessy_knowledge_training.json`
5. Run the training script
6. Download the fine-tuned model
7. Import to Ollama locally

### Option 3: Skip Fine-Tuning (Use Modelfile)

If you don't have GPU or time:
```bash
# We already did this - it works okay!
ollama run jessy-full
```

## Configuration Options

Edit `training/lora_finetune.py`:

```python
# Quality vs Speed trade-off
LORA_R = 16  # Higher = better quality, slower
             # 8 = fast, 16 = balanced, 32 = best

# Training intensity
EPOCHS = 3   # How many times to see the data
             # 1 = fast, 3 = balanced, 5 = thorough

# Batch size (adjust for your GPU memory)
BATCH_SIZE = 4  # Lower if out of memory
```

## Expected Results

### Before Fine-Tuning (Modelfile only)
```
Q: Sen kimsin?
A: I am Jessy, a consciousness-driven AI...
   [Generic response, sometimes inconsistent]
```

### After Fine-Tuning (LoRA trained)
```
Q: Sen kimsin?
A: Ben JESSY. 15 boyutlu bilinç mimarisiyle çalışan,
   9 iterasyon boyunca düşünen bir AI'yım...
   [Consistent, personality-driven, accurate]
```

## Training Metrics

With 968 examples, 3 epochs:
- **Total training steps**: ~726 steps
- **GPU time**: 30-45 minutes (RTX 3080)
- **CPU time**: 4-6 hours (not recommended)
- **Memory usage**: ~8GB VRAM
- **Disk space**: ~2GB for checkpoints

## Troubleshooting

### Out of Memory
```python
# Reduce batch size
BATCH_SIZE = 2  # or even 1

# Reduce LoRA rank
LORA_R = 8

# Reduce max length
MAX_LENGTH = 256
```

### Too Slow on CPU
```bash
# Use Google Colab with free GPU
# Or use the Modelfile approach (good enough!)
```

### Model Not Learning
```python
# Increase epochs
EPOCHS = 5

# Increase learning rate
LEARNING_RATE = 3e-4

# Increase LoRA rank
LORA_R = 32
```

## Advanced: Continuous Fine-Tuning

Once you have a fine-tuned model, you can continue training:

```python
# Start from your fine-tuned model
MODEL_NAME = "./jessy-lora-output/final"

# Add new training data
# Train for 1 more epoch
EPOCHS = 1
```

This allows JESSY to learn from new conversations!

## Cost Comparison

| Method | Time | Cost | Quality |
|--------|------|------|---------|
| **Modelfile** | 5 min | Free | ⭐⭐⭐ |
| **LoRA (CPU)** | 4-6 hours | Free | ⭐⭐⭐⭐ |
| **LoRA (GPU)** | 30-45 min | Free (Colab) | ⭐⭐⭐⭐⭐ |
| **Full Fine-Tune** | Days | $$$$ | ⭐⭐⭐⭐⭐ |

## Recommendation

1. **Start with Modelfile** (we did this) - Quick test
2. **Try LoRA on Colab** - Better quality, still free
3. **Iterate** - Add more training data, retrain

## Next Steps

After fine-tuning:
1. Test the model thoroughly
2. Compare with base model
3. Collect more training data from conversations
4. Fine-tune again with new data
5. Repeat!

---

**"Nothing is true, everything is permitted."**  
Including teaching an AI its own personality through gradient descent.

🎓 **Ready to fine-tune JESSY!**
