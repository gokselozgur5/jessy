#!/usr/bin/env python3
"""
JESSY Fine-Tuning on Thor (NVIDIA GPU)
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
MODEL_NAME = "Qwen/Qwen2.5-3B-Instruct"  # 3B model, Turkish support, open source
MAX_SEQ_LENGTH = 2048

# LoRA Configuration
LORA_CONFIG = {
    "r": 32,                    # Rank - higher = better quality
    "lora_alpha": 64,           # Scaling factor
    "target_modules": [         # Which layers to adapt
        "q_proj", "k_proj", "v_proj", "o_proj",
        "gate_proj", "up_proj", "down_proj"
    ],
    "lora_dropout": 0.05,
    "bias": "none",
    "task_type": TaskType.CAUSAL_LM
}

# Training Configuration
TRAINING_CONFIG = {
    "output_dir": "./jessy-thor-output",
    "num_train_epochs": 3,
    "per_device_train_batch_size": 8,      # Thor can handle this
    "per_device_eval_batch_size": 8,
    "gradient_accumulation_steps": 4,       # Effective batch = 32
    "learning_rate": 2e-4,
    "weight_decay": 0.01,
    "warmup_steps": 100,
    "logging_steps": 10,
    "save_steps": 200,
    "eval_steps": 200,
    "save_total_limit": 3,
    "fp16": True,                           # Use mixed precision
    "optim": "adamw_torch",
    "lr_scheduler_type": "cosine",
    "max_grad_norm": 1.0,
    "report_to": "none",
    "load_best_model_at_end": True,
    "metric_for_best_model": "eval_loss",
    "greater_is_better": False,
    "dataloader_num_workers": 4,            # Parallel data loading
    "remove_unused_columns": False,
}

# Data paths
TRAIN_FILE = "jessy_maximum_train.jsonl"
VAL_FILE = "jessy_maximum_val.jsonl"

# ============================================================================
# FUNCTIONS
# ============================================================================

def check_environment():
    """Check GPU and environment"""
    logger.info("=" * 70)
    logger.info("🔥 JESSY Training on Thor")
    logger.info("=" * 70)
    
    # Check CUDA
    if not torch.cuda.is_available():
        logger.error("❌ CUDA not available!")
        exit(1)
    
    logger.info(f"✅ PyTorch: {torch.__version__}")
    logger.info(f"✅ CUDA: {torch.version.cuda}")
    logger.info(f"✅ GPU Count: {torch.cuda.device_count()}")
    
    for i in range(torch.cuda.device_count()):
        props = torch.cuda.get_device_properties(i)
        logger.info(f"✅ GPU {i}: {torch.cuda.get_device_name(i)}")
        logger.info(f"   Memory: {props.total_memory / 1024**3:.1f} GB")
    
    logger.info("=" * 70)

def load_model_and_tokenizer(model_name):
    """Load base model and tokenizer"""
    logger.info(f"📥 Loading model: {model_name}")
    
    # Load tokenizer
    tokenizer = AutoTokenizer.from_pretrained(
        model_name,
        trust_remote_code=True
    )
    
    # Set padding token
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    # Load model
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        torch_dtype=torch.float16,
        device_map="auto",
        trust_remote_code=True
    )
    
    # Disable caching for training
    model.config.use_cache = False
    model.config.pretraining_tp = 1
    
    logger.info("✅ Model loaded")
    return model, tokenizer

def apply_lora(model):
    """Apply LoRA adapters to model"""
    logger.info("🔧 Applying LoRA configuration...")
    
    lora_config = LoraConfig(**LORA_CONFIG)
    model = get_peft_model(model, lora_config)
    
    # Print trainable parameters
    trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total_params = sum(p.numel() for p in model.parameters())
    trainable_pct = 100 * trainable_params / total_params
    
    logger.info(f"📊 Trainable params: {trainable_params:,} ({trainable_pct:.2f}%)")
    logger.info(f"📊 Total params: {total_params:,}")
    logger.info("✅ LoRA applied")
    
    return model

def load_and_prepare_data(tokenizer, train_file, val_file):
    """Load and tokenize datasets"""
    logger.info("📚 Loading datasets...")
    
    # Check if files exist
    if not os.path.exists(train_file):
        logger.error(f"❌ Training file not found: {train_file}")
        exit(1)
    if not os.path.exists(val_file):
        logger.error(f"❌ Validation file not found: {val_file}")
        exit(1)
    
    # Load datasets
    dataset = load_dataset(
        "json",
        data_files={
            "train": train_file,
            "validation": val_file
        }
    )
    
    logger.info(f"📊 Train samples: {len(dataset['train'])}")
    logger.info(f"📊 Validation samples: {len(dataset['validation'])}")
    
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
            inp = example["input"]
            out = example["output"]
            text = f"<|user|>\n{inp}\n<|assistant|>\n{out}\n"
            return {"text": text}
        else:
            return {"text": ""}
    
    # Format datasets
    logger.info("🔄 Formatting datasets...")
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
    logger.info("🔄 Tokenizing datasets...")
    tokenized_dataset = dataset.map(
        tokenize_function,
        batched=True,
        remove_columns=["text"],
        desc="Tokenizing"
    )
    
    logger.info("✅ Datasets ready")
    return tokenized_dataset

def train_model(model, tokenizer, dataset):
    """Train the model"""
    logger.info("=" * 70)
    logger.info("🚀 Starting Training")
    logger.info("=" * 70)
    
    # Training arguments
    training_args = TrainingArguments(**TRAINING_CONFIG)
    
    # Data collator
    data_collator = DataCollatorForLanguageModeling(
        tokenizer=tokenizer,
        mlm=False
    )
    
    # Trainer
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=dataset["train"],
        eval_dataset=dataset["validation"],
        data_collator=data_collator,
    )
    
    # Log training info
    logger.info(f"📊 Epochs: {TRAINING_CONFIG['num_train_epochs']}")
    logger.info(f"📊 Batch size: {TRAINING_CONFIG['per_device_train_batch_size']}")
    logger.info(f"📊 Gradient accumulation: {TRAINING_CONFIG['gradient_accumulation_steps']}")
    effective_batch = TRAINING_CONFIG['per_device_train_batch_size'] * TRAINING_CONFIG['gradient_accumulation_steps']
    logger.info(f"📊 Effective batch size: {effective_batch}")
    logger.info(f"📊 Learning rate: {TRAINING_CONFIG['learning_rate']}")
    logger.info("=" * 70)
    
    # Train
    start_time = datetime.now()
    trainer.train()
    end_time = datetime.now()
    
    training_time = (end_time - start_time).total_seconds()
    logger.info("=" * 70)
    logger.info(f"✅ Training complete in {training_time/60:.1f} minutes")
    logger.info("=" * 70)
    
    return trainer

def save_model(trainer, model, tokenizer, output_dir):
    """Save the trained model"""
    logger.info("💾 Saving model...")
    
    # Save full model
    trainer.save_model(output_dir)
    tokenizer.save_pretrained(output_dir)
    
    # Save LoRA adapters separately
    lora_dir = f"{output_dir}/lora_adapters"
    model.save_pretrained(lora_dir)
    
    logger.info(f"✅ Model saved to: {output_dir}")
    logger.info(f"✅ LoRA adapters saved to: {lora_dir}")
    
    # Print final stats
    logger.info("=" * 70)
    logger.info("📊 Final Statistics:")
    logger.info(f"   Total steps: {trainer.state.global_step}")
    logger.info(f"   Best eval loss: {trainer.state.best_metric:.4f}")
    logger.info("=" * 70)

# ============================================================================
# MAIN
# ============================================================================

def main():
    """Main training function"""
    try:
        # Check environment
        check_environment()
        
        # Load model and tokenizer
        model, tokenizer = load_model_and_tokenizer(MODEL_NAME)
        
        # Apply LoRA
        model = apply_lora(model)
        
        # Load and prepare data
        dataset = load_and_prepare_data(tokenizer, TRAIN_FILE, VAL_FILE)
        
        # Train
        trainer = train_model(model, tokenizer, dataset)
        
        # Save
        save_model(trainer, model, tokenizer, TRAINING_CONFIG["output_dir"])
        
        logger.info("=" * 70)
        logger.info("🎉 JESSY Training Complete!")
        logger.info("=" * 70)
        
    except Exception as e:
        logger.error(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        exit(1)

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
🔨 JESSY Fine-Tuning on Thor
Sıfırdan yazılmış, temiz, güçlü training script
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
from datasets import Dataset
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

CONFIG = {
    # Model
    "model_name": "Qwen/Qwen2.5-3B-Instruct",  # 3B, açık, Türkçe destekli
    "max_length": 1024,
    
    # LoRA
    "lora_r": 32,  # Rank - Thor güçlü, max quality
    "lora_alpha": 64,
    "lora_dropout": 0.05,
    "lora_target_modules": ["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"],
    
    # Training
    "num_epochs": 3,
    "batch_size": 8,  # Thor güçlü, büyük batch
    "gradient_accumulation_steps": 4,  # Effective batch = 32
    "learning_rate": 2e-4,
    "warmup_ratio": 0.1,
    "weight_decay": 0.01,
    "max_grad_norm": 1.0,
    
    # Optimization
    "fp16": True,  # Thor NVIDIA, fp16 destekler
    "optim": "adamw_torch",
    "lr_scheduler_type": "cosine",
    
    # Paths
    "train_file": "datasets/jessy_maximum_train.jsonl",
    "val_file": "datasets/jessy_maximum_val.jsonl",
    "output_dir": "models/jessy-thor",
    "logging_dir": "logs/jessy-thor",
}

# ============================================================================
# FUNCTIONS
# ============================================================================

def print_banner():
    """Thor banner"""
    banner = """
    ╔══════════════════════════════════════════════════════════╗
    ║                                                          ║
    ║              🔨 JESSY TRAINING ON THOR 🔨               ║
    ║                                                          ║
    ║          "Nothing is true, everything is permitted"      ║
    ║                                                          ║
    ╚══════════════════════════════════════════════════════════╝
    """
    print(banner)
    logger.info(f"Training started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    logger.info("=" * 60)

def check_gpu():
    """GPU kontrolü"""
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        gpu_memory = torch.cuda.get_device_properties(0).total_memory / 1e9
        logger.info(f"✅ GPU: {gpu_name}")
        logger.info(f"✅ VRAM: {gpu_memory:.1f} GB")
        logger.info(f"✅ CUDA Version: {torch.version.cuda}")
        return True
    else:
        logger.warning("⚠️  No GPU found! Training will be SLOW on CPU")
        return False

def load_dataset(train_file, val_file, tokenizer, max_length):
    """Dataset yükleme ve hazırlama"""
    logger.info("📚 Loading datasets...")
    
    # JSONL dosyalarını oku
    def read_jsonl(file_path):
        data = []
        with open(file_path, 'r', encoding='utf-8') as f:
            for line in f:
                data.append(json.loads(line))
        return data
    
    train_data = read_jsonl(train_file)
    val_data = read_jsonl(val_file)
    
    logger.info(f"  Train samples: {len(train_data)}")
    logger.info(f"  Val samples: {len(val_data)}")
    
    # Format ve tokenize
    def format_and_tokenize(examples):
        texts = []
        for ex in examples:
            # Messages formatını text'e çevir
            if "messages" in ex:
                text = ""
                for msg in ex["messages"]:
                    role = msg.get("role", "")
                    content = msg.get("content", "")
                    if role == "system":
                        text += f"<|system|>\n{content}\n"
                    elif role == "user":
                        text += f"<|user|>\n{content}\n"
                    elif role == "assistant":
                        text += f"<|assistant|>\n{content}\n"
                texts.append(text)
            elif "input" in ex and "output" in ex:
                text = f"<|user|>\n{ex['input']}\n<|assistant|>\n{ex['output']}\n"
                texts.append(text)
        
        # Tokenize
        tokenized = tokenizer(
            texts,
            truncation=True,
            max_length=max_length,
            padding="max_length",
            return_tensors="pt"
        )
        
        return {
            "input_ids": tokenized["input_ids"],
            "attention_mask": tokenized["attention_mask"],
            "labels": tokenized["input_ids"].clone()
        }
    
    # Dataset oluştur
    train_dataset = Dataset.from_list(train_data)
    val_dataset = Dataset.from_list(val_data)
    
    # Tokenize
    logger.info("🔤 Tokenizing...")
    train_dataset = train_dataset.map(
        lambda x: format_and_tokenize([x]),
        batched=False,
        remove_columns=train_dataset.column_names
    )
    val_dataset = val_dataset.map(
        lambda x: format_and_tokenize([x]),
        batched=False,
        remove_columns=val_dataset.column_names
    )
    
    logger.info("✅ Datasets ready!")
    return train_dataset, val_dataset

def load_model_and_tokenizer(model_name):
    """Model ve tokenizer yükleme"""
    logger.info(f"🤖 Loading model: {model_name}")
    
    # Tokenizer
    tokenizer = AutoTokenizer.from_pretrained(model_name, trust_remote_code=True)
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token
    tokenizer.padding_side = "right"
    
    # Model
    model = AutoModelForCausalLM.from_pretrained(
        model_name,
        torch_dtype=torch.float16 if CONFIG["fp16"] else torch.float32,
        device_map="auto",
        trust_remote_code=True
    )
    
    model.config.use_cache = False
    model.config.pretraining_tp = 1
    
    logger.info("✅ Model loaded!")
    return model, tokenizer

def apply_lora(model):
    """LoRA uygulama"""
    logger.info("🎯 Applying LoRA...")
    
    lora_config = LoraConfig(
        r=CONFIG["lora_r"],
        lora_alpha=CONFIG["lora_alpha"],
        target_modules=CONFIG["lora_target_modules"],
        lora_dropout=CONFIG["lora_dropout"],
        bias="none",
        task_type=TaskType.CAUSAL_LM
    )
    
    model = get_peft_model(model, lora_config)
    
    # Trainable params
    trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    total = sum(p.numel() for p in model.parameters())
    percentage = 100 * trainable / total
    
    logger.info(f"  Trainable params: {trainable:,} ({percentage:.2f}%)")
    logger.info(f"  Total params: {total:,}")
    logger.info("✅ LoRA applied!")
    
    return model

def train():
    """Ana training fonksiyonu"""
    print_banner()
    
    # GPU check
    has_gpu = check_gpu()
    
    # Directories
    os.makedirs(CONFIG["output_dir"], exist_ok=True)
    os.makedirs(CONFIG["logging_dir"], exist_ok=True)
    
    # Model ve tokenizer
    model, tokenizer = load_model_and_tokenizer(CONFIG["model_name"])
    
    # LoRA
    model = apply_lora(model)
    
    # Dataset
    train_dataset, val_dataset = load_dataset(
        CONFIG["train_file"],
        CONFIG["val_file"],
        tokenizer,
        CONFIG["max_length"]
    )
    
    # Training arguments
    training_args = TrainingArguments(
        output_dir=CONFIG["output_dir"],
        num_train_epochs=CONFIG["num_epochs"],
        per_device_train_batch_size=CONFIG["batch_size"],
        per_device_eval_batch_size=CONFIG["batch_size"],
        gradient_accumulation_steps=CONFIG["gradient_accumulation_steps"],
        learning_rate=CONFIG["learning_rate"],
        warmup_ratio=CONFIG["warmup_ratio"],
        weight_decay=CONFIG["weight_decay"],
        max_grad_norm=CONFIG["max_grad_norm"],
        fp16=CONFIG["fp16"] and has_gpu,
        optim=CONFIG["optim"],
        lr_scheduler_type=CONFIG["lr_scheduler_type"],
        logging_dir=CONFIG["logging_dir"],
        logging_steps=10,
        save_steps=100,
        eval_steps=100,
        save_total_limit=3,
        load_best_model_at_end=True,
        metric_for_best_model="eval_loss",
        greater_is_better=False,
        report_to="none",
        remove_unused_columns=False,
    )
    
    # Data collator
    data_collator = DataCollatorForLanguageModeling(
        tokenizer=tokenizer,
        mlm=False
    )
    
    # Trainer
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=train_dataset,
        eval_dataset=val_dataset,
        data_collator=data_collator,
    )
    
    # Training info
    logger.info("=" * 60)
    logger.info("🚀 TRAINING CONFIGURATION")
    logger.info("=" * 60)
    logger.info(f"  Model: {CONFIG['model_name']}")
    logger.info(f"  Epochs: {CONFIG['num_epochs']}")
    logger.info(f"  Batch size: {CONFIG['batch_size']}")
    logger.info(f"  Gradient accumulation: {CONFIG['gradient_accumulation_steps']}")
    logger.info(f"  Effective batch size: {CONFIG['batch_size'] * CONFIG['gradient_accumulation_steps']}")
    logger.info(f"  Learning rate: {CONFIG['learning_rate']}")
    logger.info(f"  LoRA rank: {CONFIG['lora_r']}")
    logger.info(f"  FP16: {CONFIG['fp16'] and has_gpu}")
    logger.info("=" * 60)
    logger.info("🔥 Starting training...")
    logger.info("=" * 60)
    
    # TRAIN!
    trainer.train()
    
    # Save
    logger.info("💾 Saving model...")
    trainer.save_model(CONFIG["output_dir"])
    tokenizer.save_pretrained(CONFIG["output_dir"])
    model.save_pretrained(f"{CONFIG['output_dir']}/lora_adapters")
    
    # Stats
    logger.info("=" * 60)
    logger.info("✅ TRAINING COMPLETE!")
    logger.info("=" * 60)
    logger.info(f"  Total steps: {trainer.state.global_step}")
    logger.info(f"  Best eval loss: {trainer.state.best_metric:.4f}")
    logger.info(f"  Model saved to: {CONFIG['output_dir']}")
    logger.info(f"  LoRA adapters: {CONFIG['output_dir']}/lora_adapters")
    logger.info("=" * 60)
    logger.info(f"Training finished at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    print("""
    ╔══════════════════════════════════════════════════════════╗
    ║                                                          ║
    ║                  🎉 TRAINING COMPLETE! 🎉               ║
    ║                                                          ║
    ║              Thor has forged JESSY's mind!               ║
    ║                                                          ║
    ╚══════════════════════════════════════════════════════════╝
    """)

if __name__ == "__main__":
    train()
