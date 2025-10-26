#!/usr/bin/env python3
"""
Simple JESSY Turkish Fine-Tuning

Minimal training script that works with Python 3.14 and M2 Mac.
"""

import json
import torch
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    Trainer,
    TrainingArguments,
    DataCollatorForLanguageModeling,
)
from peft import LoraConfig, get_peft_model
import time

print("🇹🇷 JESSY Turkish Fine-Tuning (Simple)")
print("=" * 60)

# Config
MODEL_NAME = "TinyLlama/TinyLlama-1.1B-Chat-v1.0"
OUTPUT_DIR = "./jessy-turkish-simple"

print(f"\n📊 Configuration:")
print(f"   Model: {MODEL_NAME}")
print(f"   Device: {'MPS' if torch.backends.mps.is_available() else 'CPU'}")

# Load data
print(f"\n📂 Loading data...")
with open('training/jessy_train.json', 'r') as f:
    train_data = json.load(f)

print(f"   Examples: {len(train_data)}")

# Load tokenizer and model
print(f"\n🔤 Loading tokenizer and model...")
tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME)
tokenizer.pad_token = tokenizer.eos_token

device = "mps" if torch.backends.mps.is_available() else "cpu"
model = AutoModelForCausalLM.from_pretrained(
    MODEL_NAME,
    torch_dtype=torch.float16,
    device_map=device,
)

print(f"   ✅ Loaded on {device}")

# Prepare training texts
print(f"\n🔄 Preparing training data...")
texts = []
for ex in train_data:
    text = f"### Instruction:\n{ex['instruction']}\n\n### Response:\n{ex['output']}"
    texts.append(text)

# Tokenize
encodings = tokenizer(
    texts,
    truncation=True,
    max_length=256,
    padding="max_length",
    return_tensors="pt"
)

# Create simple dataset
class SimpleDataset(torch.utils.data.Dataset):
    def __init__(self, encodings):
        self.encodings = encodings
    
    def __len__(self):
        return len(self.encodings['input_ids'])
    
    def __getitem__(self, idx):
        return {
            'input_ids': self.encodings['input_ids'][idx],
            'attention_mask': self.encodings['attention_mask'][idx],
            'labels': self.encodings['input_ids'][idx],
        }

train_dataset = SimpleDataset(encodings)
print(f"   ✅ Dataset ready: {len(train_dataset)} examples")

# Configure LoRA
print(f"\n🔧 Configuring LoRA...")
lora_config = LoraConfig(
    r=8,  # Smaller rank for faster training
    lora_alpha=16,
    target_modules=["q_proj", "v_proj"],
    lora_dropout=0.05,
    bias="none",
    task_type="CAUSAL_LM",
)

model = get_peft_model(model, lora_config)
model.print_trainable_parameters()

# Training args
training_args = TrainingArguments(
    output_dir=OUTPUT_DIR,
    num_train_epochs=2,  # Fewer epochs for quick test
    per_device_train_batch_size=2,
    gradient_accumulation_steps=2,
    learning_rate=1e-4,
    logging_steps=10,
    save_steps=50,
    fp16=False,  # MPS doesn't support fp16
    report_to="none",
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
    data_collator=data_collator,
)

# Train!
print(f"\n🚀 Starting training...")
print(f"   This will take ~15-20 minutes")
print()

start_time = time.time()

try:
    trainer.train()
    
    elapsed = time.time() - start_time
    print(f"\n✅ Training complete!")
    print(f"   Time: {elapsed/60:.1f} minutes")
    
    # Save
    print(f"\n💾 Saving model...")
    model.save_pretrained(OUTPUT_DIR)
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    print(f"   ✅ Saved to {OUTPUT_DIR}")
    print(f"\n🎉 Done! Model ready for testing.")
    
except KeyboardInterrupt:
    print(f"\n⚠️  Interrupted")
except Exception as e:
    print(f"\n❌ Error: {e}")
    import traceback
    traceback.print_exc()
