#!/usr/bin/env python3
"""
ATAK AI Training on Thor
Fine-tune a model specifically for ATAK (Android Team Awareness Kit) assistance
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
from peft import LoraConfig, get_peft_model, TaskType
from datasets import load_dataset
import logging
from datetime import datetime

# Logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ============================================================================
# CONFIGURATION
# ============================================================================

# Model - Using smaller model for faster training
MODEL_NAME = "Qwen/Qwen2.5-1.5B-Instruct"  # Smaller, faster for ATAK domain
MAX_SEQ_LENGTH = 1024  # ATAK responses are typically shorter

# Data - Using comprehensive dataset
TRAIN_FILE = "datasets/atak_comprehensive_train.jsonl"
VAL_FILE = "datasets/atak_comprehensive_val.jsonl"

# Output
OUTPUT_DIR = "models/atak-assistant"
CHECKPOINT_DIR = "checkpoints/atak-assistant"
LOG_DIR = "logs"

# LoRA Configuration - Aggressive for small dataset
LORA_CONFIG = {
    "r": 64,  # High rank for better learning on small dataset
    "lora_alpha": 128,
    "target_modules": [
        "q_proj", "k_proj", "v_proj", "o_proj",
        "gate_proj", "up_proj", "down_proj"
    ],
    "lora_dropout": 0.05,
    "bias": "none",
    "task_type": TaskType.CAUSAL_LM
}

# Training Configuration - Optimized for small dataset
TRAINING_CONFIG = {
    "output_dir": OUTPUT_DIR,
    "num_train_epochs": 10,  # More epochs for small dataset
    "per_device_train_batch_size": 4,
    "per_device_eval_batch_size": 4,
    "gradient_accumulation_steps": 8,  # Effective batch = 32
    "learning_rate": 3e-4,  # Higher LR for small dataset
    "weight_decay": 0.01,
    "warmup_ratio": 0.1,
    "lr_scheduler_type": "cosine",
    "logging_steps": 5,
    "save_steps": 50,
    "eval_steps": 50,
    "save_total_limit": 5,
    "fp16": True,
    "optim": "adamw_torch",
    "max_grad_norm": 1.0,
    "report_to": "none",
    "dataloader_num_workers": 4,
}

# ============================================================================
# FUNCTIONS
# ============================================================================

def check_environment():
    """Check CUDA availability"""
    logger.info("=" * 70)
    logger.info("🔍 Environment Check")
    logger.info("=" * 70)
    
    logger.info(f"PyTorch: {torch.__version__}")
    
    if torch.cuda.is_available():
        logger.info(f"✅ CUDA: {torch.version.cuda}")
        logger.info(f"✅ GPU Count: {torch.cuda.device_count()}")
        
        for i in range(torch.cuda.device_count()):
            name = torch.cuda.get_device_name(i)
            memory = torch.cuda.get_device_properties(i).total_memory / 1e9
            logger.info(f"✅ GPU {i}: {name} ({memory:.1f} GB)")
        
        return torch.device("cuda:0")
    else:
        logger.error("❌ CUDA not available!")
        raise RuntimeError("CUDA required")

def load_model_and_tokenizer(model_name):
    """Load model and tokenizer"""
    logger.info("=" * 70)
    logger.info(f"📦 Loading Model: {model_name}")
    logger.info("=" * 70)
    
    # Tokenizer
    logger.info("Loading tokenizer...")
    tokenizer = AutoTokenizer.from_pretrained(
        model_name,
        trust_remote_code=True
    )
    
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    logger.info(f"✅ Tokenizer loaded (vocab: {len(tokenizer)})")
    
    # Model
    logger.info("Loading model...")
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        torch_dtype=torch.float16,
        device_map="auto",
        trust_remote_code=True
    )
    
    model.config.use_cache = False
    model.config.pretraining_tp = 1
    
    logger.info("✅ Model loaded")
    
    return model, tokenizer

def apply_lora(model):
    """Apply LoRA"""
    logger.info("=" * 70)
    logger.info("🔧 Applying LoRA")
    logger.info("=" * 70)
    
    # Enable gradient checkpointing before applying LoRA
    model.gradient_checkpointing_enable()
    
    lora_config = LoraConfig(**LORA_CONFIG)
    model = get_peft_model(model, lora_config)
    
    # Ensure model is in training mode
    model.train()
    
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total = sum(p.numel() for p in model.parameters())
    
    logger.info(f"📊 Trainable: {trainable:,} ({100*trainable/total:.2f}%)")
    logger.info(f"📊 Total: {total:,}")
    logger.info(f"📊 LoRA Rank: {LORA_CONFIG['r']}")
    
    return model

def prepare_datasets(tokenizer, train_file, val_file):
    """Prepare datasets"""
    logger.info("=" * 70)
    logger.info("📚 Preparing Datasets")
    logger.info("=" * 70)
    
    if not os.path.exists(train_file):
        raise FileNotFoundError(f"Training file not found: {train_file}")
    if not os.path.exists(val_file):
        raise FileNotFoundError(f"Validation file not found: {val_file}")
    
    logger.info(f"Train: {train_file}")
    logger.info(f"Val: {val_file}")
    
    # Load
    dataset = load_dataset(
        "json",
        data_files={"train": train_file, "validation": val_file}
    )
    
    logger.info(f"✅ Train: {len(dataset['train'])} examples")
    logger.info(f"✅ Val: {len(dataset['validation'])} examples")
    
    def format_example(example):
        """Format as chat"""
        # ATAK-specific system prompt
        system_prompt = """You are an expert ATAK (Android Team Awareness Kit) assistant. You help users with:
- ATAK basics and features
- Marker management and navigation
- Tools and measurements
- Team communication and tracking
- Map management and offline maps
- Settings and configuration
- Plugins and extensions
- Tactical operations
- Troubleshooting

Provide clear, concise, step-by-step instructions. Be helpful and professional."""
        
        inp = example.get("input", "")
        out = example.get("output", "")
        
        text = f"<|system|>\n{system_prompt}\n<|user|>\n{inp}\n<|assistant|>\n{out}\n"
        return {"text": text}
    
    # Format
    logger.info("Formatting...")
    dataset = dataset.map(
        format_example,
        remove_columns=dataset["train"].column_names,
        desc="Formatting"
    )
    
    def tokenize_function(examples):
        """Tokenize"""
        return tokenizer(
            examples["text"],
            truncation=True,
            max_length=MAX_SEQ_LENGTH,
            padding="max_length",
            return_tensors="pt"
        )
    
    # Tokenize
    logger.info("Tokenizing...")
    tokenized = dataset.map(
        tokenize_function,
        batched=True,
        remove_columns=["text"],
        desc="Tokenizing"
    )
    
    logger.info("✅ Datasets ready")
    
    return tokenized

def train_model(model, tokenizer, dataset):
    """Train"""
    logger.info("=" * 70)
    logger.info("🚀 Training")
    logger.info("=" * 70)
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    
    data_collator = DataCollatorForLanguageModeling(
        tokenizer=tokenizer,
        mlm=False
    )
    
    training_args = TrainingArguments(**TRAINING_CONFIG)
    
    logger.info(f"📊 Epochs: {TRAINING_CONFIG['num_train_epochs']}")
    logger.info(f"📊 Batch Size: {TRAINING_CONFIG['per_device_train_batch_size']}")
    logger.info(f"📊 Effective Batch: {TRAINING_CONFIG['per_device_train_batch_size'] * TRAINING_CONFIG['gradient_accumulation_steps']}")
    logger.info(f"📊 Learning Rate: {TRAINING_CONFIG['learning_rate']}")
    
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=dataset["train"],
        eval_dataset=dataset["validation"],
        data_collator=data_collator,
    )
    
    logger.info("=" * 70)
    logger.info("🎯 Training Started!")
    logger.info("=" * 70)
    
    start = datetime.now()
    trainer.train()
    end = datetime.now()
    
    duration = (end - start).total_seconds()
    
    logger.info("=" * 70)
    logger.info("✅ Training Complete!")
    logger.info("=" * 70)
    logger.info(f"⏱️  Time: {duration/60:.1f} minutes")
    logger.info(f"📊 Steps: {trainer.state.global_step}")
    logger.info(f"📊 Best Loss: {trainer.state.best_metric:.4f}")
    
    return trainer

def save_model(trainer, model, tokenizer):
    """Save model"""
    logger.info("=" * 70)
    logger.info("💾 Saving")
    logger.info("=" * 70)
    
    logger.info(f"Saving to: {OUTPUT_DIR}")
    trainer.save_model(OUTPUT_DIR)
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    lora_dir = f"{OUTPUT_DIR}/lora_adapters"
    logger.info(f"LoRA adapters: {lora_dir}")
    model.save_pretrained(lora_dir)
    
    logger.info("✅ Saved!")
    
    # Training info
    info = {
        "model_name": MODEL_NAME,
        "lora_config": LORA_CONFIG,
        "training_config": TRAINING_CONFIG,
        "timestamp": datetime.now().isoformat(),
        "purpose": "ATAK Assistant - Android Team Awareness Kit"
    }
    
    with open(f"{OUTPUT_DIR}/training_info.json", "w") as f:
        json.dump(info, f, indent=2)
    
    logger.info(f"✅ Info saved")

def main():
    """Main"""
    logger.info("=" * 70)
    logger.info("⚡ ATAK AI Training on Thor")
    logger.info("=" * 70)
    logger.info(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logger.info("=" * 70)
    
    try:
        device = check_environment()
        model, tokenizer = load_model_and_tokenizer(MODEL_NAME)
        model = apply_lora(model)
        dataset = prepare_datasets(tokenizer, TRAIN_FILE, VAL_FILE)
        trainer = train_model(model, tokenizer, dataset)
        save_model(trainer, model, tokenizer)
        
        logger.info("=" * 70)
        logger.info("🎉 COMPLETE!")
        logger.info("=" * 70)
        logger.info(f"Model: {OUTPUT_DIR}")
        logger.info(f"LoRA: {OUTPUT_DIR}/lora_adapters")
        logger.info("=" * 70)
        logger.info("Next steps:")
        logger.info("1. Test the model")
        logger.info("2. Convert to GGUF")
        logger.info("3. Deploy to Ollama")
        logger.info("4. Use for ATAK assistance!")
        logger.info("=" * 70)
        
    except Exception as e:
        logger.error("=" * 70)
        logger.error(f"❌ Failed: {e}")
        logger.error("=" * 70)
        raise

if __name__ == "__main__":
    main()
