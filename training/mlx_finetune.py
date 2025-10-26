#!/usr/bin/env python3
"""
JESSY MLX Fine-Tuning for Apple Silicon (M1/M2/M3)

Uses Apple's MLX framework - optimized for Metal GPU!
Much faster than PyTorch on Mac.
"""

import json
import mlx.core as mx
import mlx.nn as nn
from mlx_lm import load, generate
from mlx_lm.tuner import train
import os

print("🍎 JESSY MLX Fine-Tuning for Apple Silicon")
print("=" * 60)

# Check Metal GPU
print(f"\n💻 Device: Apple Silicon with Metal GPU")
print(f"   Unified Memory: {os.popen('sysctl hw.memsize').read().split()[1]}")

# Configuration
MODEL_NAME = "mlx-community/gemma-2b-it"  # MLX-optimized Gemma
TRAINING_DATA = "training/jessy_knowledge_training.json"
OUTPUT_DIR = "./jessy-mlx-output"

# LoRA Configuration (optimized for M2)
LORA_R = 16
LORA_ALPHA = 32
LORA_DROPOUT = 0.05

# Training Configuration (M2 can handle this!)
EPOCHS = 3
BATCH_SIZE = 8  # M2 has unified memory - can go higher!
LEARNING_RATE = 1e-4
MAX_LENGTH = 512

print(f"\n📊 Configuration:")
print(f"   Model: {MODEL_NAME}")
print(f"   LoRA Rank: {LORA_R}")
print(f"   Epochs: {EPOCHS}")
print(f"   Batch Size: {BATCH_SIZE}")
print(f"   Learning Rate: {LEARNING_RATE}")

# Load training data
print(f"\n📚 Loading training data...")
with open(TRAINING_DATA, 'r', encoding='utf-8') as f:
    training_data = json.load(f)

print(f"   ✓ Loaded {len(training_data)} Q&A pairs")

# Convert to MLX format
train_data = []
for example in training_data:
    conversations = example['conversations']
    text = ""
    for msg in conversations:
        if msg['from'] == 'human':
            text += f"<|user|>\n{msg['value']}\n"
        else:
            text += f"<|assistant|>\n{msg['value']}\n<|endoftext|>\n"
    train_data.append({"text": text})

# Save as JSONL for MLX
train_file = "training/train.jsonl"
with open(train_file, 'w') as f:
    for item in train_data:
        f.write(json.dumps(item) + '\n')

print(f"   ✓ Prepared {len(train_data)} training examples")

# Load model
print(f"\n📦 Loading model: {MODEL_NAME}...")
print("   (First time will download ~2GB)")

model, tokenizer = load(MODEL_NAME)
print("   ✓ Model loaded")

# Configure LoRA
lora_config = {
    "rank": LORA_R,
    "alpha": LORA_ALPHA,
    "dropout": LORA_DROPOUT,
    "scale": LORA_ALPHA / LORA_R,
}

# Training arguments
training_args = {
    "model": MODEL_NAME,
    "train": True,
    "data": train_file,
    "lora_layers": 16,  # Number of layers to apply LoRA
    "batch_size": BATCH_SIZE,
    "iters": EPOCHS * len(train_data) // BATCH_SIZE,
    "learning_rate": LEARNING_RATE,
    "steps_per_report": 10,
    "steps_per_eval": 50,
    "adapter_file": f"{OUTPUT_DIR}/adapters.npz",
    "save_every": 100,
    "test": None,
    "max_seq_length": MAX_LENGTH,
}

print("\n" + "=" * 60)
print("🚀 Starting fine-tuning on Metal GPU...")
print("=" * 60)
print(f"\nEstimated time on M2: {EPOCHS * 15} minutes")
print("(Much faster than PyTorch!)\n")

try:
    # Train with MLX
    train.train(**training_args)
    
    print("\n✅ Training complete!")
    print(f"\n💾 Model saved to {OUTPUT_DIR}")
    
    # Test the model
    print("\n🧪 Testing fine-tuned model...")
    test_prompt = "Sen kimsin?"
    
    response = generate(
        model,
        tokenizer,
        prompt=test_prompt,
        max_tokens=100,
        temp=0.8,
    )
    
    print(f"\nTest prompt: {test_prompt}")
    print(f"Response: {response}")
    
    print("\n" + "=" * 60)
    print("🎉 Fine-tuning complete!")
    print("=" * 60)
    print(f"\nNext steps:")
    print(f"1. Test more: python training/test_mlx_model.py")
    print(f"2. Convert to GGUF: python training/mlx_to_gguf.py")
    print(f"3. Import to Ollama: ollama create jessy-mlx -f Modelfile")
    
except KeyboardInterrupt:
    print("\n\n⚠️  Training interrupted by user")
    
except Exception as e:
    print(f"\n\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
