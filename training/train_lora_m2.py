#!/usr/bin/env python3
"""
LoRA Fine-tuning for Jessy on M2 Mac
Optimized for Apple Silicon with MPS acceleration
"""

import os
import json
import torch
from transformers import (
    AutoModelForCausalLM,
    AutoTokenizer,
    TrainingArguments,
    Trainer,
    DataCollatorForLanguageModeling
)
from peft import LoraConfig, get_peft_model, prepare_model_for_kbit_training
from datasets import load_dataset
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configuration
# Using Qwen2.5 - open model, great for Turkish, no auth needed
MODEL_NAME = "Qwen/Qwen2.5-3B-Instruct"  # 3B params, fully open, Turkish support
TRAIN_FILE = "training/datasets/jessy_maximum_train.jsonl"
VAL_FILE = "training/datasets/jessy_maximum_val.jsonl"
OUTPUT_DIR = "training/models/jessy-lora-m2"
CHECKPOINT_DIR = "training/checkpoints/jessy-lora"

# LoRA Configuration - Optimized for M2
LORA_CONFIG = {
    "r": 16,  # Rank - balance between quality and speed
    "lora_alpha": 32,  # Scaling factor
    "target_modules": ["q_proj", "k_proj", "v_proj", "o_proj"],  # Attention layers
    "lora_dropout": 0.05,
    "bias": "none",
    "task_type": "CAUSAL_LM"
}

# Training Configuration - M2 Optimized
TRAINING_CONFIG = {
    "output_dir": OUTPUT_DIR,
    "num_train_epochs": 3,
    "per_device_train_batch_size": 2,  # Small batch for M2
    "per_device_eval_batch_size": 2,
    "gradient_accumulation_steps": 8,  # Effective batch size = 16
    "learning_rate": 2e-4,
    "warmup_steps": 100,
    "logging_steps": 10,
    "save_steps": 100,
    "eval_steps": 100,
    "save_total_limit": 3,
    "fp16": False,  # M2 doesn't support fp16
    "bf16": False,  # Use full precision on M2
    "optim": "adamw_torch",
    "lr_scheduler_type": "cosine",
    "max_grad_norm": 1.0,
    "report_to": "none",  # Disable wandb
    "load_best_model_at_end": True,
    "metric_for_best_model": "eval_loss",
    "greater_is_better": False,
}

def check_device():
    """Check available device"""
    if torch.backends.mps.is_available():
        device = torch.device("mps")
        logger.info("✅ Using Apple Silicon MPS")
    elif torch.cuda.is_available():
        device = torch.device("cuda")
        logger.info("✅ Using CUDA")
    else:
        device = torch.device("cpu")
        logger.info("⚠️  Using CPU (will be slow)")
    return device

def load_and_prepare_model(model_name):
    """Load model and prepare for LoRA training"""
    logger.info(f"Loading model: {model_name}")
    
    # Load tokenizer
    tokenizer = AutoTokenizer.from_pretrained(model_name)
    tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    # Load model
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        torch_dtype=torch.float32,  # Full precision for M2
        device_map="auto",
        trust_remote_code=True
    )
    
    # Prepare for training
    model.config.use_cache = False
    model.config.pretraining_tp = 1
    
    # Apply LoRA
    logger.info("Applying LoRA configuration...")
    lora_config = LoraConfig(**LORA_CONFIG)
    model = get_peft_model(model, lora_config)
    
    # Print trainable parameters
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total_params = sum(p.numel() for p in model.parameters())
    logger.info(f"Trainable params: {trainable_params:,} ({100 * trainable_params / total_params:.2f}%)")
    logger.info(f"Total params: {total_params:,}")
    
    return model, tokenizer

def prepare_dataset(tokenizer, train_file, val_file, max_length=512):
    """Prepare training and validation datasets"""
    logger.info("Loading datasets...")
    
    # Load datasets - use absolute paths to avoid Python 3.14 pickle issues
    import os
    train_file = os.path.abspath(train_file)
    val_file = os.path.abspath(val_file)
    
    dataset = load_dataset(
        "json",
        data_files={"train": train_file, "validation": val_file},
        keep_in_memory=True  # Avoid pickle issues
    )
    
    def format_prompt(example):
        """Format example into prompt"""
        # Handle both formats: messages and input/output
        if "messages" in example:
            messages = example["messages"]
            text = ""
            for msg in messages:
                role = msg.get("role", "")
                content = msg.get("content", "")
                if role == "system":
                    text += f"<|system|>\n{content}\n"
                elif role == "user":
                    text += f"<|user|>\n{content}\n"
                elif role == "assistant":
                    text += f"<|assistant|>\n{content}\n"
            return {"text": text}
        
        elif "input" in example and "output" in example:
            # Simple input/output format
            inp = example["input"]
            out = example["output"]
            text = f"<|user|>\n{inp}\n<|assistant|>\n{out}\n"
            return {"text": text}
        
        else:
            return {"text": ""}
    
    # Format datasets
    dataset = dataset.map(format_prompt, remove_columns=dataset["train"].column_names)
    
    # Tokenize
    def tokenize_function(examples):
        return tokenizer(
            examples["text"],
            truncation=True,
            max_length=max_length,
            padding="max_length",
            return_tensors="pt"
        )
    
    logger.info("Tokenizing datasets...")
    tokenized_dataset = dataset.map(
        tokenize_function,
        batched=True,
        remove_columns=["text"],
        desc="Tokenizing"
    )
    
    logger.info(f"Train samples: {len(tokenized_dataset['train'])}")
    logger.info(f"Validation samples: {len(tokenized_dataset['validation'])}")
    
    return tokenized_dataset

def train():
    """Main training function"""
    logger.info("🚀 Starting Jessy LoRA Training on M2")
    logger.info("=" * 60)
    
    # Check device
    device = check_device()
    
    # Create output directories
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    
    # Load model and tokenizer
    model, tokenizer = load_and_prepare_model(MODEL_NAME)
    
    # Prepare datasets
    tokenized_dataset = prepare_dataset(tokenizer, TRAIN_FILE, VAL_FILE)
    
    # Data collator
    data_collator = DataCollatorForLanguageModeling(
        tokenizer=tokenizer,
        mlm=False
    )
    
    # Training arguments
    training_args = TrainingArguments(**TRAINING_CONFIG)
    
    # Trainer
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=tokenized_dataset["train"],
        eval_dataset=tokenized_dataset["validation"],
        data_collator=data_collator,
    )
    
    # Train
    logger.info("🎯 Starting training...")
    logger.info(f"Epochs: {TRAINING_CONFIG['num_train_epochs']}")
    logger.info(f"Batch size: {TRAINING_CONFIG['per_device_train_batch_size']}")
    logger.info(f"Gradient accumulation: {TRAINING_CONFIG['gradient_accumulation_steps']}")
    logger.info(f"Effective batch size: {TRAINING_CONFIG['per_device_train_batch_size'] * TRAINING_CONFIG['gradient_accumulation_steps']}")
    logger.info("=" * 60)
    
    trainer.train()
    
    # Save final model
    logger.info("💾 Saving final model...")
    trainer.save_model(OUTPUT_DIR)
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    # Save LoRA adapters separately
    model.save_pretrained(f"{OUTPUT_DIR}/lora_adapters")
    
    logger.info("✅ Training complete!")
    logger.info(f"Model saved to: {OUTPUT_DIR}")
    logger.info(f"LoRA adapters saved to: {OUTPUT_DIR}/lora_adapters")
    
    # Print final stats
    logger.info("=" * 60)
    logger.info("📊 Training Statistics:")
    logger.info(f"Total steps: {trainer.state.global_step}")
    logger.info(f"Best eval loss: {trainer.state.best_metric:.4f}")
    logger.info("=" * 60)

if __name__ == "__main__":
    train()
