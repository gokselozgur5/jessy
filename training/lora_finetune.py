#!/usr/bin/env python3
"""
JESSY LoRA Fine-Tuning Script

Uses the 968 Q&A pairs to fine-tune gemma:2b with LoRA.
Requires: transformers, peft, datasets, torch, bitsandbytes
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
from peft import LoraConfig, get_peft_model, prepare_model_for_kbit_training
from datasets import Dataset
import os

print("🔥 JESSY LoRA Fine-Tuning")
print("=" * 60)

# Configuration
MODEL_NAME = "google/gemma-2b"
OUTPUT_DIR = "./jessy-lora-output"
TRAINING_DATA = "training/jessy_knowledge_training.json"

# LoRA Configuration
LORA_R = 16  # Rank (higher = more parameters, better quality, slower)
LORA_ALPHA = 32  # Scaling factor
LORA_DROPOUT = 0.05
TARGET_MODULES = ["q_proj", "v_proj", "k_proj", "o_proj"]

# Training Configuration
EPOCHS = 3  # Number of times to go through the data
BATCH_SIZE = 4
GRADIENT_ACCUMULATION = 4
LEARNING_RATE = 2e-4
MAX_LENGTH = 512

print(f"\n📊 Configuration:")
print(f"   Model: {MODEL_NAME}")
print(f"   LoRA Rank: {LORA_R}")
print(f"   Epochs: {EPOCHS}")
print(f"   Batch Size: {BATCH_SIZE}")
print(f"   Learning Rate: {LEARNING_RATE}")

# Check if CUDA available
device = "cuda" if torch.cuda.is_available() else "cpu"
print(f"\n💻 Device: {device}")
if device == "cpu":
    print("   ⚠️  Warning: Training on CPU will be VERY slow!")
    print("   Consider using Google Colab with GPU for faster training")

# Load training data
print(f"\n📚 Loading training data from {TRAINING_DATA}...")
with open(TRAINING_DATA, 'r', encoding='utf-8') as f:
    training_data = json.load(f)

print(f"   ✓ Loaded {len(training_data)} Q&A pairs")

# Convert to training format
def format_conversation(example):
    """Convert Q&A to training text"""
    conversations = example['conversations']
    text = ""
    for msg in conversations:
        if msg['from'] == 'human':
            text += f"<|user|>\n{msg['value']}\n"
        else:
            text += f"<|assistant|>\n{msg['value']}\n<|endoftext|>\n"
    return {"text": text}

# Create dataset
print("\n🔄 Preparing dataset...")
dataset = Dataset.from_list(training_data)
dataset = dataset.map(format_conversation)
print(f"   ✓ Dataset prepared: {len(dataset)} examples")

# Load model and tokenizer
print(f"\n📦 Loading model: {MODEL_NAME}...")
print("   (This may take a few minutes...)")

tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
tokenizer.pad_token = tokenizer.eos_token

model = AutoModelForCausalLM.from_pretrained(
    MODEL_NAME,
    load_in_8bit=True,  # 8-bit quantization to save memory
    device_map="auto",
    torch_dtype=torch.float16,
)

print("   ✓ Model loaded")

# Prepare model for training
print("\n🔧 Preparing model for LoRA training...")
model = prepare_model_for_kbit_training(model)

# Configure LoRA
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

# Tokenize dataset
def tokenize_function(examples):
    return tokenizer(
        examples["text"],
        truncation=True,
        max_length=MAX_LENGTH,
        padding="max_length",
    )

print("\n🔤 Tokenizing dataset...")
tokenized_dataset = dataset.map(
    tokenize_function,
    batched=True,
    remove_columns=dataset.column_names,
)
print("   ✓ Tokenization complete")

# Training arguments
training_args = TrainingArguments(
    output_dir=OUTPUT_DIR,
    num_train_epochs=EPOCHS,
    per_device_train_batch_size=BATCH_SIZE,
    gradient_accumulation_steps=GRADIENT_ACCUMULATION,
    learning_rate=LEARNING_RATE,
    fp16=True,
    logging_steps=10,
    save_steps=100,
    save_total_limit=3,
    warmup_steps=50,
    report_to="none",  # Disable wandb
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
    train_dataset=tokenized_dataset,
    data_collator=data_collator,
)

# Train!
print("\n" + "=" * 60)
print("🚀 Starting fine-tuning...")
print("=" * 60)
print(f"\nThis will take approximately:")
print(f"   - GPU: {EPOCHS * 10} minutes")
print(f"   - CPU: {EPOCHS * 60} minutes (not recommended!)")
print("\nPress Ctrl+C to cancel\n")

try:
    trainer.train()
    print("\n✅ Training complete!")
    
    # Save model
    print(f"\n💾 Saving model to {OUTPUT_DIR}...")
    model.save_pretrained(f"{OUTPUT_DIR}/final")
    tokenizer.save_pretrained(f"{OUTPUT_DIR}/final")
    print("   ✓ Model saved")
    
    print("\n" + "=" * 60)
    print("🎉 Fine-tuning complete!")
    print("=" * 60)
    print(f"\nNext steps:")
    print(f"1. Merge LoRA weights: python training/merge_lora.py")
    print(f"2. Convert to GGUF: python training/convert_to_gguf.py")
    print(f"3. Import to Ollama: ollama create jessy-finetuned -f Modelfile")
    
except KeyboardInterrupt:
    print("\n\n⚠️  Training interrupted by user")
    print("   Partial model saved in:", OUTPUT_DIR)

except Exception as e:
    print(f"\n\n❌ Error during training: {e}")
    import traceback
    traceback.print_exc()
