# 🔥 Thor Training Setup Guide

## Quick Start (5 dakika)

### 1. Thor'a Bağlan
```bash
ssh jensen@192.168.88.55
# Password: jensen
```

### 2. Dizin Oluştur
```bash
mkdir -p ~/jessy-training
cd ~/jessy-training
```

### 3. Dosyaları Kopyala (Mac'ten)

**Yeni terminal aç (Mac'te):**
```bash
# Training script'i kopyala
scp training/train_on_thor.py jensen@192.168.88.55:~/jessy-training/

# Training data'yı kopyala
scp training/datasets/jessy_maximum_train.jsonl jensen@192.168.88.55:~/jessy-training/
scp training/datasets/jessy_maximum_val.jsonl jensen@192.168.88.55:~/jessy-training/
```

### 4. Dependencies Kur (Thor'da)
```bash
# Thor'da çalıştır
python3 -m pip install --user torch transformers peft datasets accelerate bitsandbytes
```

### 5. Training Başlat
```bash
# Foreground (ekranda görmek için)
python3 train_on_thor.py

# VEYA Background (kapanmaz)
nohup python3 train_on_thor.py > training.log 2>&1 &

# Log'u takip et
tail -f training.log
```

## Beklenen Çıktı

```
======================================================================
🔥 JESSY Training on Thor
======================================================================
✅ PyTorch: 2.9.0
✅ CUDA: 13.0
✅ GPU Count: 1
✅ GPU 0: NVIDIA Thor
   Memory: XX.X GB
======================================================================
📥 Loading model: Qwen/Qwen2.5-3B-Instruct
✅ Model loaded
🔧 Applying LoRA configuration...
📊 Trainable params: 41,943,040 (1.32%)
📊 Total params: 3,174,400,000
✅ LoRA applied
📚 Loading datasets...
📊 Train samples: XXXX
📊 Validation samples: XXX
🔄 Formatting datasets...
🔄 Tokenizing datasets...
✅ Datasets ready
======================================================================
🚀 Starting Training
======================================================================
📊 Epochs: 3
📊 Batch size: 8
📊 Gradient accumulation: 4
📊 Effective batch size: 32
📊 Learning rate: 0.0002
======================================================================
[Training progress...]
```

## Training Süresi

- **GPU**: NVIDIA Thor
- **Model**: Qwen2.5-3B (3 billion parameters)
- **LoRA Rank**: 32
- **Epochs**: 3
- **Batch Size**: 32 (effective)

**Tahmini Süre**: 30-60 dakika

## Training Sırasında

### GPU Kullanımını İzle
```bash
# Yeni terminal aç
watch -n 1 nvidia-smi
```

### Log'u İzle
```bash
tail -f training.log
```

### Training'i Durdur
```bash
# Process ID bul
ps aux | grep train_on_thor

# Durdur
kill <PID>
```

## Training Bittikten Sonra

### Model Dosyalarını Kontrol Et
```bash
ls -lh ~/jessy-training/jessy-thor-output/
```

### Model'i Mac'e Kopyala
```bash
# Mac'te çalıştır
scp -r jensen@192.168.88.55:~/jessy-training/jessy-thor-output ./training/models/
```

### Ollama'ya Import Et
```bash
# Mac'te
cd training/models/jessy-thor-output
# GGUF'a çevir ve Ollama'ya import et (ayrı script gerekli)
```

## Troubleshooting

### Out of Memory
```python
# train_on_thor.py içinde değiştir:
"per_device_train_batch_size": 4,  # 8'den 4'e düşür
"gradient_accumulation_steps": 8,  # 4'ten 8'e çıkar
```

### CUDA Error
```bash
# CUDA path'i ekle
export PATH=/usr/local/cuda-13.0/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-13.0/lib64:$LD_LIBRARY_PATH
```

### Slow Training
```bash
# GPU kullanımını kontrol et
nvidia-smi

# CPU'da çalışıyorsa:
python3 -c "import torch; print(torch.cuda.is_available())"
```

## Hızlı Komutlar

```bash
# Tek komutla setup (Mac'ten)
scp training/train_on_thor.py training/datasets/jessy_maximum_*.jsonl jensen@192.168.88.55:~/jessy-training/

# Tek komutla training başlat (Thor'da)
cd ~/jessy-training && nohup python3 train_on_thor.py > training.log 2>&1 & tail -f training.log

# Model'i geri getir (Mac'ten)
scp -r jensen@192.168.88.55:~/jessy-training/jessy-thor-output ./training/models/
```

## Sonraki Adımlar

1. ✅ Training tamamlandı
2. Model'i Mac'e kopyala
3. GGUF formatına çevir
4. Ollama'ya import et
5. Test et!

---

**"Nothing is true, everything is permitted."**  
Including training AI on Thor's hammer! ⚡🔨
