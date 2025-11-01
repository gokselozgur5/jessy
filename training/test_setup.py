#!/usr/bin/env python3
"""
Test setup before training
"""

import sys
import json
import torch
from pathlib import Path

def check_pytorch():
    """Check PyTorch and MPS"""
    print("🔍 Checking PyTorch...")
    print(f"   Version: {torch.__version__}")
    
    if torch.backends.mps.is_available():
        print("   ✅ MPS (Apple Silicon) available")
        return True
    elif torch.cuda.is_available():
        print("   ✅ CUDA available")
        return True
    else:
        print("   ⚠️  Only CPU available (will be slow)")
        return False

def check_packages():
    """Check required packages"""
    print("\n🔍 Checking packages...")
    required = ["transformers", "peft", "datasets", "accelerate"]
    
    for pkg in required:
        try:
            __import__(pkg)
            print(f"   ✅ {pkg}")
        except ImportError:
            print(f"   ❌ {pkg} not installed")
            return False
    return True

def check_datasets():
    """Check training datasets"""
    print("\n🔍 Checking datasets...")
    
    # Try both paths (from root and from training/)
    train_file = Path("datasets/jessy_maximum_train.jsonl")
    if not train_file.exists():
        train_file = Path("training/datasets/jessy_maximum_train.jsonl")
    
    val_file = Path("datasets/jessy_maximum_val.jsonl")
    if not val_file.exists():
        val_file = Path("training/datasets/jessy_maximum_val.jsonl")
    
    if not train_file.exists():
        print(f"   ❌ Training dataset not found: {train_file}")
        return False
    
    if not val_file.exists():
        print(f"   ❌ Validation dataset not found: {val_file}")
        return False
    
    # Count samples
    with open(train_file) as f:
        train_count = sum(1 for _ in f)
    
    with open(val_file) as f:
        val_count = sum(1 for _ in f)
    
    print(f"   ✅ Training samples: {train_count:,}")
    print(f"   ✅ Validation samples: {val_count:,}")
    
    # Check format
    with open(train_file) as f:
        sample = json.loads(f.readline())
        if "messages" in sample:
            print(f"   ✅ Format: Chat (messages)")
        elif "text" in sample:
            print(f"   ✅ Format: Text")
        else:
            print(f"   ⚠️  Unknown format: {list(sample.keys())}")
    
    return True

def estimate_time(train_samples, epochs=3, batch_size=2, grad_accum=8):
    """Estimate training time"""
    print("\n⏱️  Time Estimation:")
    
    steps_per_epoch = train_samples // (batch_size * grad_accum)
    total_steps = steps_per_epoch * epochs
    
    # M2 estimate: ~2 seconds per step
    seconds_per_step = 2.0
    total_seconds = total_steps * seconds_per_step
    hours = total_seconds / 3600
    
    print(f"   Steps per epoch: {steps_per_epoch}")
    print(f"   Total steps: {total_steps}")
    print(f"   Estimated time: {hours:.1f} hours")
    
    if hours > 10:
        print(f"   ⚠️  This will take a while!")
    elif hours > 5:
        print(f"   ⏳ Good time for overnight training")
    else:
        print(f"   ✅ Reasonable training time")

def check_disk_space():
    """Check available disk space"""
    print("\n💾 Disk Space:")
    import shutil
    stat = shutil.disk_usage(".")
    
    free_gb = stat.free / (1024**3)
    print(f"   Free space: {free_gb:.1f} GB")
    
    # LoRA adapters are small, ~5GB should be enough
    if free_gb < 3:
        print(f"   ❌ Insufficient disk space! Need at least 3GB")
        return False
    elif free_gb < 5:
        print(f"   ⚠️  Low disk space, but should work")
        return True
    else:
        print(f"   ✅ Sufficient space")
        return True

def main():
    print("=" * 60)
    print("Jessy LoRA Training - Setup Check")
    print("=" * 60)
    
    checks = []
    
    # Run checks
    checks.append(("PyTorch", check_pytorch()))
    checks.append(("Packages", check_packages()))
    checks.append(("Datasets", check_datasets()))
    checks.append(("Disk Space", check_disk_space()))
    
    # Estimate time
    try:
        train_file = Path("datasets/jessy_maximum_train.jsonl")
        if not train_file.exists():
            train_file = Path("training/datasets/jessy_maximum_train.jsonl")
        
        with open(train_file) as f:
            train_count = sum(1 for _ in f)
        estimate_time(train_count)
    except Exception as e:
        print(f"\n⚠️  Could not estimate time: {e}")
    
    # Summary
    print("\n" + "=" * 60)
    print("Summary:")
    print("=" * 60)
    
    all_passed = all(result for _, result in checks)
    
    for name, result in checks:
        status = "✅" if result else "❌"
        print(f"{status} {name}")
    
    print("=" * 60)
    
    if all_passed:
        print("\n✅ All checks passed! Ready to train.")
        print("\nRun: ./start_training.sh")
        return 0
    else:
        print("\n❌ Some checks failed. Fix issues before training.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
