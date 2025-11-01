# 🔨 Thor Training Guide

Thor'da JESSY'yi fine-tune etmek için sıfırdan yazılmış, temiz ve kolay kullanımlı guide.

## 🎯 Quick Start

### 1. Deploy to Thor (İlk Kez)
```bash
./training/deploy_to_thor.sh
```

Bu komut:
- Training script'ini Thor'a kopyalar
- Dataset'leri transfer eder
- Python environment'ı hazırlar
- Her şeyi otomatik kurar

### 2. Start Training
```bash
./training/start_training_on_thor.sh
```

Bu komut:
- Training'i Thor'da background'da başlatır
- Output'u real-time gösterir
- Ctrl+C ile monitoring'i durdurabilirsin (training devam eder)

### 3. Check Status
```bash
./training/check_thor_status.sh
```

Bu komut:
- Training'in çalışıp çalışmadığını kontrol eder
- GPU kullanımını gösterir
- Son log'ları gösterir
- Disk kullanımını gösterir

## 📊 Training Details

### Configuration
- **Model**: Qwen2.5-3B-Instruct (3B parameters)
- **Method**: LoRA (Low-Rank Adaptation)
- **LoRA Rank**: 32 (high quality)
- **Batch Size**: 8 per device
- **Gradient Accumulation**: 4 (effective batch = 32)
- **Epochs**: 3
- **Learning Rate**: 2e-4
- **FP16**: Enabled (faster training)

### Expected Performance
- **Training Time**: ~30-45 minutes (depends on GPU)
- **GPU Memory**: ~8-12 GB
- **Dataset**: ~3,600 examples
- **Output Size**: ~2-3 GB

## 🔧 Manual Commands

### SSH to Thor
```bash
sshpass -p jensen ssh jensen@192.168.88.55
```

### Start Training Manually
```bash
ssh jensen@192.168.88.55
cd /home/jensen/jessy-training
./run_training.sh
```

### Monitor Training
```bash
# Real-time monitoring
sshpass -p jensen ssh jensen@192.168.88.55 "tail -f /home/jensen/jessy-training/training.out"

# Or check logs
sshpass -p jensen ssh jensen@192.168.88.55 "tail -f /home/jensen/jessy-training/logs/training_*.log"
```

### Check GPU
```bash
sshpass -p jensen ssh jensen@192.168.88.55 "nvidia-smi"
```

### Stop Training
```bash
sshpass -p jensen ssh jensen@192.168.88.55 "pkill -f train_thor.py"
```

## 📁 File Structure on Thor

```
/home/jensen/jessy-training/
├── train_thor.py              # Main training script
├── run_training.sh            # Wrapper script
├── requirements.txt           # Python dependencies
├── training.out               # Training output
├── datasets/
│   ├── jessy_maximum_train.jsonl
│   └── jessy_maximum_val.jsonl
├── models/
│   └── jessy-thor/           # Trained model (after training)
├── checkpoints/
│   └── jessy-thor/           # Training checkpoints
├── logs/
│   └── training_*.log        # Detailed logs
└── venv/                     # Python virtual environment
```

## 🎯 Training Progress

Training'i takip etmek için şu satırları ara:

```
🔍 Environment Check
✅ CUDA Available: 13.0
✅ GPU 0: NVIDIA Thor

📦 Loading Model: Qwen/Qwen2.5-3B-Instruct
✅ Tokenizer loaded
✅ Base model loaded

🔧 Applying LoRA Configuration
📊 Trainable Parameters: 41,943,040 (1.32%)
📊 Total Parameters: 3,174,748,160

📚 Preparing Datasets
✅ Loaded 3,243 training examples
✅ Loaded 360 validation examples

🚀 Starting Training
📊 Epochs: 3
📊 Batch Size: 8
📊 Effective Batch Size: 32

🎯 Training Started!
[Training progress will show here...]

✅ Training Complete!
⏱️  Training Time: 35.2 minutes
📊 Best Eval Loss: 0.4523
```

## 🎉 After Training

Training bitince model şurada olacak:
```
/home/jensen/jessy-training/models/jessy-thor/
```

### Download Model
```bash
# Download trained model
sshpass -p jensen scp -r jensen@192.168.88.55:/home/jensen/jessy-training/models/jessy-thor ./training/models/

# Download LoRA adapters
sshpass -p jensen scp -r jensen@192.168.88.55:/home/jensen/jessy-training/models/jessy-thor/lora_adapters ./training/models/
```

### Convert to GGUF (for Ollama)
```bash
# TODO: Add conversion script
# This will convert the model to GGUF format for Ollama
```

## 🐛 Troubleshooting

### Training Not Starting
```bash
# Check if Python packages are installed
sshpass -p jensen ssh jensen@192.168.88.55 "cd /home/jensen/jessy-training && source venv/bin/activate && pip list | grep torch"

# Reinstall if needed
sshpass -p jensen ssh jensen@192.168.88.55 "cd /home/jensen/jessy-training && source venv/bin/activate && pip install -r requirements.txt"
```

### Out of Memory
```bash
# Edit training script to reduce batch size
# Change per_device_train_batch_size from 8 to 4
```

### CUDA Not Found
```bash
# Check CUDA installation
sshpass -p jensen ssh jensen@192.168.88.55 "nvcc --version"
sshpass -p jensen ssh jensen@192.168.88.55 "python3 -c 'import torch; print(torch.cuda.is_available())'"
```

## 📊 Monitoring Tips

### Watch GPU Usage
```bash
watch -n 1 'sshpass -p jensen ssh jensen@192.168.88.55 "nvidia-smi"'
```

### Watch Training Progress
```bash
sshpass -p jensen ssh jensen@192.168.88.55 "tail -f /home/jensen/jessy-training/training.out | grep -E '(loss|epoch|step)'"
```

### Check Disk Space
```bash
sshpass -p jensen ssh jensen@192.168.88.55 "df -h /home/jensen/jessy-training"
```

## 🎯 Next Steps

1. ✅ Deploy to Thor
2. ✅ Start training
3. ⏳ Wait 30-45 minutes
4. ✅ Download trained model
5. 🔄 Convert to GGUF
6. 🚀 Deploy to Ollama
7. 🎉 Test JESSY!

---

**"Nothing is true, everything is permitted."**  
Including training AI on a machine named after a Norse god. ⚡🔨

