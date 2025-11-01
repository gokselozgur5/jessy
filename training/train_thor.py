#!/usr/bin/env python3
"""
JESSY LoRA Fine-tuning on Thor (NVIDIA GPU)
Sıfırdan yazılmış, temiz ve optimize edilmiş training script
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

# Logging setup
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# ============================================================================
# CONFIGURATION
# ============================================================================

# Model Configuration
MODEL_NAME = "Qwen/Qwen2.5-3B-Instruct"  # Open, Turkish support, 3B params
MAX_SEQ_LENGTH = 2048

# Data Configuration
TRAIN_FILE = "training/datasets/jessy_maximum_train.jsonl"
VAL_FILE = "training/datasets/jessy_maximum_val.jsonl"

# Output Configuration
OUTPUT_DIR = "training/models/jessy-thor"
CHECKPOINT_DIR = "training/checkpoints/jessy-thor"
LOG_DIR = "training/logs"

# LoRA Configuration - Optimized for Thor GPU
LORA_CONFIG = {
    "r": 32,  # Rank - higher for better quality on powerful GPU
    "lora_alpha": 64,  # Scaling factor
    "target_modules": [
        "q_proj", "k_proj", "v_proj", "o_proj",  # Attention
        "gate_proj", "up_proj", "down_proj"  # MLP
    ],
    "lora_dropout": 0.05,
    "bias": "none",
    "task_type": TaskType.CAUSAL_LM
}

# Training Configuration - Thor GPU Optimized
TRAINING_CONFIG = {
    "output_dir": OUTPUT_DIR,
    "num_train_epochs": 3,
    "per_device_train_batch_size": 8,  # Larger batch for Thor
    "per_device_eval_batch_size": 8,
    "gradient_accumulation_steps": 4,  # Effective batch = 32
    "learning_rate": 2e-4,
    "weight_decay": 0.01,
    "warmup_ratio": 0.1,
    "lr_scheduler_type": "cosine",
    "logging_steps": 10,
    "save_steps": 100,
    "eval_steps": 100,
    "save_total_limit": 3,
    "fp16": True,  # Use FP16 for speed
    "bf16": False,
    "optim": "adamw_torch",
    "max_grad_norm": 1.0,
    "report_to": "none",
    "load_best_model_at_end": True,
    "metric_for_best_model": "eval_loss",
    "greater_is_better": False,
    "gradient_checkpointing": True,  # Save memory
    "dataloader_num_workers": 4,  # Parallel data loading
}

# ============================================================================
# FUNCTIONS
# ============================================================================

def check_environment():
    """Check CUDA and GPU availability"""
    logger.info("=" * 70)
    logger.info("🔍 Environment Check")
    logger.info("=" * 70)
    
    # PyTorch version
    logger.info(f"PyTorch Version: {torch.__version__}")
    
    # CUDA availability
    if torch.cuda.is_available():
        logger.info(f"✅ CUDA Available: {torch.version.cuda}")
        logger.info(f"✅ GPU Count: {torch.cuda.device_count()}")
        
        for i in range(torch.cuda.device_count()):
            gpu_name = torch.cuda.get_device_name(i)
            gpu_memory = torch.cuda.get_device_properties(i).total_memory / 1e9
            logger.info(f"✅ GPU {i}: {gpu_name} ({gpu_memory:.1f} GB)")
        
        # Set device
        device = torch.device("cuda:0")
        logger.info(f"🎯 Using Device: {device}")
        return device
    else:
        logger.error("❌ CUDA not available! This script requires GPU.")
        raise RuntimeError("CUDA not available")

def load_model_and_tokenizer(model_name):
    """Load base model and tokenizer"""
    logger.info("=" * 70)
    logger.info(f"📦 Loading Model: {model_name}")
    logger.info("=" * 70)
    
    # Load tokenizer
    logger.info("Loading tokenizer...")
    tokenizer = AutoTokenizer.from_pretrained(
        model_name,
        trust_remote_code=True
    )
    
    # Set padding token
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    logger.info(f"✅ Tokenizer loaded (vocab size: {len(tokenizer)})")
    
    # Load model
    logger.info("Loading base model...")
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        torch_dtype=torch.float16,  # FP16 for speed
        device_map="auto",
        trust_remote_code=True
    )
    
    # Model configuration
    model.config.use_cache = False  # Required for gradient checkpointing
    model.config.pretraining_tp = 1
    
    logger.info(f"✅ Base model loaded")
    
    return model, tokenizer

def apply_lora(model):
    """Apply LoRA adapters to model"""
    logger.info("=" * 70)
    logger.info("🔧 Applying LoRA Configuration")
    logger.info("=" * 70)
    
    # Create LoRA config
    lora_config = LoraConfig(**LORA_CONFIG)
    
    # Apply LoRA
    model = get_peft_model(model, lora_config)
    
    # Print trainable parameters
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total_params = sum(p.numel() for p in model.parameters())
    trainable_percent = 100 * trainable_params / total_params
    
    logger.info(f"📊 Trainable Parameters: {trainable_params:,} ({trainable_percent:.2f}%)")
    logger.info(f"📊 Total Parameters: {total_params:,}")
    logger.info(f"📊 LoRA Rank: {LORA_CONFIG['r']}")
    logger.info(f"📊 LoRA Alpha: {LORA_CONFIG['lora_alpha']}")
    
    return model

def prepare_datasets(tokenizer, train_file, val_file):
    """Load and prepare training datasets"""
    logger.info("=" * 70)
    logger.info("📚 Preparing Datasets")
    logger.info("=" * 70)
    
    # Check files exist
    if not os.path.exists(train_file):
        raise FileNotFoundError(f"Training file not found: {train_file}")
    if not os.path.exists(val_file):
        raise FileNotFoundError(f"Validation file not found: {val_file}")
    
    logger.info(f"Train file: {train_file}")
    logger.info(f"Val file: {val_file}")
    
    # Load datasets
    dataset = load_dataset(
        "json",
        data_files={"train": train_file, "validation": val_file}
    )
    
    logger.info(f"✅ Loaded {len(dataset['train'])} training examples")
    logger.info(f"✅ Loaded {len(dataset['validation'])} validation examples")
    
    def format_example(example):
        """Format example into chat format"""
        if "messages" in example:
            # Chat format
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
            # Simple format
            return {"text": f"<|user|>\n{example['input']}\n<|assistant|>\n{example['output']}\n"}
        else:
            return {"text": ""}
    
    # Format datasets
    logger.info("Formatting examples...")
    dataset = dataset.map(
        format_example,
        remove_columns=dataset["train"].column_names,
        desc="Formatting"
    )
    
    def tokenize_function(examples):
        """Tokenize examples"""
        return tokenizer(
            examples["text"],
            truncation=True,
            max_length=MAX_SEQ_LENGTH,
            padding="max_length",
            return_tensors="pt"
        )
    
    # Tokenize
    logger.info("Tokenizing...")
    tokenized_dataset = dataset.map(
        tokenize_function,
        batched=True,
        remove_columns=["text"],
        desc="Tokenizing"
    )
    
    logger.info(f"✅ Datasets prepared")
    
    return tokenized_dataset

def train_model(model, tokenizer, dataset):
    """Train the model"""
    logger.info("=" * 70)
    logger.info("🚀 Starting Training")
    logger.info("=" * 70)
    
    # Create output directories
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    os.makedirs(LOG_DIR, exist_ok=True)
    
    # Data collator
    data_collator = DataCollatorForLanguageModeling(
        tokenizer=tokenizer,
        mlm=False
    )
    
    # Training arguments
    training_args = TrainingArguments(**TRAINING_CONFIG)
    
    # Print training info
    logger.info(f"📊 Epochs: {TRAINING_CONFIG['num_train_epochs']}")
    logger.info(f"📊 Batch Size: {TRAINING_CONFIG['per_device_train_batch_size']}")
    logger.info(f"📊 Gradient Accumulation: {TRAINING_CONFIG['gradient_accumulation_steps']}")
    logger.info(f"📊 Effective Batch Size: {TRAINING_CONFIG['per_device_train_batch_size'] * TRAINING_CONFIG['gradient_accumulation_steps']}")
    logger.info(f"📊 Learning Rate: {TRAINING_CONFIG['learning_rate']}")
    logger.info(f"📊 FP16: {TRAINING_CONFIG['fp16']}")
    
    # Create trainer
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=dataset["train"],
        eval_dataset=dataset["validation"],
        data_collator=data_collator,
    )
    
    # Train
    logger.info("=" * 70)
    logger.info("🎯 Training Started!")
    logger.info("=" * 70)
    
    start_time = datetime.now()
    trainer.train()
    end_time = datetime.now()
    
    training_time = (end_time - start_time).total_seconds()
    
    logger.info("=" * 70)
    logger.info("✅ Training Complete!")
    logger.info("=" * 70)
    logger.info(f"⏱️  Training Time: {training_time/60:.1f} minutes")
    logger.info(f"📊 Total Steps: {trainer.state.global_step}")
    logger.info(f"📊 Best Eval Loss: {trainer.state.best_metric:.4f}")
    
    return trainer

def save_model(trainer, model, tokenizer):
    """Save the trained model"""
    logger.info("=" * 70)
    logger.info("💾 Saving Model")
    logger.info("=" * 70)
    
    # Save full model
    logger.info(f"Saving to: {OUTPUT_DIR}")
    trainer.save_model(OUTPUT_DIR)
    tokenizer.save_pretrained(OUTPUT_DIR)
    
    # Save LoRA adapters separately
    lora_dir = f"{OUTPUT_DIR}/lora_adapters"
    logger.info(f"Saving LoRA adapters to: {lora_dir}")
    model.save_pretrained(lora_dir)
    
    logger.info("✅ Model saved successfully!")
    
    # Save training info
    info_file = f"{OUTPUT_DIR}/training_info.json"
    training_info = {
        "model_name": MODEL_NAME,
        "lora_config": LORA_CONFIG,
        "training_config": TRAINING_CONFIG,
        "timestamp": datetime.now().isoformat()
    }
    
    with open(info_file, "w") as f:
        json.dump(training_info, f, indent=2)
    
    logger.info(f"✅ Training info saved to: {info_file}")

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main training pipeline"""
    logger.info("=" * 70)
    logger.info("⚡ JESSY LoRA Fine-tuning on Thor")
    logger.info("=" * 70)
    logger.info(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logger.info("=" * 70)
    
    try:
        # 1. Check environment
        device = check_environment()
        
        # 2. Load model and tokenizer
        model, tokenizer = load_model_and_tokenizer(MODEL_NAME)
        
        # 3. Apply LoRA
        model = apply_lora(model)
        
        # 4. Prepare datasets
        dataset = prepare_datasets(tokenizer, TRAIN_FILE, VAL_FILE)
        
        # 5. Train
        trainer = train_model(model, tokenizer, dataset)
        
        # 6. Save
        save_model(trainer, model, tokenizer)
        
        logger.info("=" * 70)
        logger.info("🎉 ALL DONE!")
        logger.info("=" * 70)
        logger.info(f"Model saved to: {OUTPUT_DIR}")
        logger.info(f"LoRA adapters: {OUTPUT_DIR}/lora_adapters")
        logger.info("=" * 70)
        logger.info("Next steps:")
        logger.info("1. Test the model")
        logger.info("2. Convert to GGUF for Ollama")
        logger.info("3. Deploy and enjoy!")
        logger.info("=" * 70)
        
    except Exception as e:
        logger.error("=" * 70)
        logger.error(f"❌ Training failed: {e}")
        logger.error("=" * 70)
        raise

if __name__ == "__main__":
    main()
