#!/usr/bin/env python3
"""
JESSY Turkish Fine-Tuning with PyTorch MPS (M2 Mac)

Trains JESSY on Turkish conversational data to fix overthinking.
Uses LoRA for efficient training on M2 GPU.
"""

import json
import torch
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    TrainingArguments,
    Trainer,
    DataCollatorForLanguageModeling,
)
from peft import LoraConfig, get_peft_model
from datasets import Dataset
import time

print("🇹🇷 JESSY Turkish Fine-Tuning (M2 Mac)")
print("=" * 60)

# Configuration
# Using TinyLlama - fully open, no authentication needed
MODEL_NAME = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
OUTPUT_DIR = "./jessy-turkish-lora"
TRAINING_DATA = "training/jessy_train.json"
VAL_DATA = "training/jessy_val.json"

# LoRA Configuration (optimized for M2)
LORA_R = 16  # Rank
LORA_ALPHA = 32  # Scaling
LORA_DROPOUT = 0.05
TARGET_MODULES = ["q_proj", "v_proj"]  # Fewer modules = faster on M2

# Training Configuration (optimized for M2 with 10GB memory)
EPOCHS = 3
BATCH_SIZE = 2  # Small batch for M2
GRADIENT_ACCUMULATION = 4  # Effective batch = 8
LEARNING_RATE = 1e-4
MAX_LENGTH = 256  # Shorter for conversational data

print(f"\n📊 Configuration:")
print(f"   Model: {MODEL_NAME}")
print(f"   Device: {'MPS (M2 GPU)' if torch.backends.mps.is_available() else 'CPU'}")
print(f"   LoRA Rank: {LORA_R}")
print(f"   Epochs: {EPOCHS}")
print(f"   Batch Size: {BATCH_SIZE} (effective: {BATCH_SIZE * GRADIENT_ACCUMULATION})")
print(f"   Learning Rate: {LEARNING_RATE}")

# Load data
print(f"\n📂 Loading training data...")
with open(TRAINING_DATA, 'r', encoding='utf-8') as f:
    train_data = json.load(f)

with open(VAL_DATA, 'r', encoding='utf-8') as f:
    val_data = json.load(f)

print(f"   Train: {len(train_data)} examples")
print(f"   Validation: {len(val_data)} examples")

# Load tokenizer
print(f"\n🔤 Loading tokenizer...")
tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
tokenizer.pad_token = tokenizer.eos_token
tokenizer.padding_side = "right"

# Prepare datasets
def format_instruction(example):
    """Format as instruction-following"""
    text = f"### Instruction:\n{example['instruction']}\n\n### Response:\n{example['output']}"
    return {"text": text}

def tokenize_function(examples):
    """Tokenize examples"""
    return tokenizer(
        examples["text"],
        truncation=True,
        max_length=MAX_LENGTH,
        padding="max_length",
    )

print(f"\n🔄 Preparing datasets...")
train_dataset = Dataset.from_list(train_data)
val_dataset = Dataset.from_list(val_data)

train_dataset = train_dataset.map(format_instruction)
val_dataset = val_dataset.map(format_instruction)

train_dataset = train_dataset.map(
    tokenize_function,
    batched=True,
    remove_columns=train_dataset.column_names
)

val_dataset = val_dataset.map(
    tokenize_function,
    batched=True,
    remove_columns=val_dataset.column_names
)

print(f"   ✅ Datasets prepared")

# Load model
print(f"\n🤖 Loading model...")
device = "mps" if torch.backends.mps.is_available() else "cpu"

model = AutoModelForCausalLM.from_pretrained(
    MODEL_NAME,
    torch_dtype=torch.float16,
    device_map=device,
)

print(f"   ✅ Model loaded on {device}")

# Configure LoRA
print(f"\n🔧 Configuring LoRA...")
lora_config = LoraConfig(
    r=LORA_R,
    lora_alpha=LORA_ALPHA,
    target_modules=TARGET_MODULES,
    lora_dropout=LORA_DROPOUT,
    bias="none",
    task_type="CAUSAL_LM",
)

model = get_peft_model(model, lora_config)
model.print_trainable_parameters()

# Training arguments
training_args = TrainingArguments(
    output_dir=OUTPUT_DIR,
    num_train_epochs=EPOCHS,
    per_device_train_batch_size=BATCH_SIZE,
    per_device_eval_batch_size=BATCH_SIZE,
    gradient_accumulation_steps=GRADIENT_ACCUMULATION,
    learning_rate=LEARNING_RATE,
    logging_steps=10,
    eval_strategy="steps",
    eval_steps=50,
    save_steps=100,
    save_total_limit=2,
    warmup_steps=10,
    fp16=False,  # MPS doesn't support fp16 yet
    report_to="none",
    remove_unused_columns=False,
)

# Data collator
data_collator = DataCollatorForLanguageModeling(
    tokenizer=tokenizer,
    mlm=False,
)

# Trainer
trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=train_dataset,
    eval_dataset=val_dataset,
    data_collator=data_collator,
)

# Train!
print(f"\n🚀 Starting training...")
print(f"   This will take ~20-30 minutes on M2 Mac")
print(f"   Watch for loss decreasing (target: <1.0)")
print()

start_time = time.time()

try:
    trainer.train()
    
    elapsed = time.time() - start_time
    print(f"\n✅ Training complete!")
    print(f"   Time: {elapsed/60:.1f} minutes")
    
    # Save model
    print(f"\n💾 Saving model...")
    model.save_pretrained(OUTPUT_DIR)
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    print(f"   ✅ Saved to {OUTPUT_DIR}")
    
    # Evaluate
    print(f"\n📊 Final evaluation...")
    eval_results = trainer.evaluate()
    print(f"   Loss: {eval_results['eval_loss']:.4f}")
    
    print(f"\n" + "=" * 60)
    print(f"🎉 JESSY Turkish fine-tuning complete!")
    print(f"\nNext steps:")
    print(f"1. Test with: python3 training/test_turkish_model.py")
    print(f"2. Convert to GGUF for Ollama")
    print(f"3. Import to Ollama as jessy-v2")
    
except KeyboardInterrupt:
    print(f"\n⚠️  Training interrupted by user")
    print(f"   Partial model saved to {OUTPUT_DIR}")
    
except Exception as e:
    print(f"\n❌ Training failed: {e}")
    import traceback
    traceback.print_exc()
