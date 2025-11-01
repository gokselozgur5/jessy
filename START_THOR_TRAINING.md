# 🔨 Thor Training - Quick Start

## Hazırlık Tamamlandı! ✅

Sıfırdan temiz bir training script hazırlandı:
- `training/train_on_thor.py` - Ana training script
- Dataset'ler hazır (1.7MB train, 193KB val)
- Thor'a özel optimize edilmiş konfigürasyon

## Adım 1: Dosyaları Thor'a Kopyala

Terminal'de şu komutları çalıştır (her birinde şifre: jensen):

```bash
# Dizinleri oluştur
ssh jensen@192.168.88.55 "mkdir -p /home/jensen/jessy-training/{datasets,models,logs}"

# Training script'i kopyala
scp training/train_on_thor.py jensen@192.168.88.55:/home/jensen/jessy-training/

# Dataset'leri kopyala
scp training/datasets/jessy_maximum_train.jsonl jensen@192.168.88.55:/home/jensen/jessy-training/datasets/
scp training/datasets/jessy_maximum_val.jsonl jensen@192.168.88.55:/home/jensen/jessy-training/datasets/
```

## Adım 2: Requirements Hazırla

```bash
# Requirements dosyası oluştur
cat > /tmp/thor_requirements.txt << 'EOF'
torch>=2.0.0
transformers>=4.35.0
peft>=0.7.0
datasets>=2.14.0
accelerate>=0.24.0
bitsandbytes>=0.41.0
sentencepiece>=0.1.99
protobuf>=3.20.0
EOF

# Thor'a kopyala
scp /tmp/thor_requirements.txt jensen@192.168.88.55:/home/jensen/jessy-training/requirements.txt
```

## Adım 3: Thor'a Bağlan ve Kur

```bash
# Thor'a SSH ile bağlan
ssh jensen@192.168.88.55

# Dizine git
cd /home/jensen/jessy-training

# Paketleri kur (ilk seferinde biraz zaman alır)
python3 -m pip install --user -r requirements.txt

# GPU kontrolü
nvidia-smi
```

## Adım 4: Training'i Başlat! 🚀

```bash
# Training'i başlat
python3 train_on_thor.py

# Ya da arka planda çalıştır
nohup python3 train_on_thor.py > training.log 2>&1 &

# Log'u takip et
tail -f training.log
```

## Training Süresi

- **Dataset**: ~3,600 örnek
- **Epochs**: 3
- **Tahmini süre**: 2-4 saat (GPU'ya bağlı)
- **Çıktı**: `/home/jensen/jessy-training/models/jessy-thor/`

## Training Tamamlandıktan Sonra

```bash
# Model'i geri kopyala
scp -r jensen@192.168.88.55:/home/jensen/jessy-training/models/jessy-thor ./training/models/

# Ollama'ya import et (local'de)
# (Bu adım için ayrı bir conversion script gerekecek)
```

## Monitoring

Training sırasında göreceğin çıktılar:

```
╔══════════════════════════════════════════════════════════╗
║                                                          ║
║              🔨 JESSY TRAINING ON THOR 🔨               ║
║                                                          ║
║          "Nothing is true, everything is permitted"      ║
║                                                          ║
╚══════════════════════════════════════════════════════════╝

✅ GPU: NVIDIA Thor
✅ VRAM: XX.X GB
✅ CUDA Version: XX.X

📚 Loading datasets...
  Train samples: 3240
  Val samples: 360

🤖 Loading model: Qwen/Qwen2.5-3B-Instruct
✅ Model loaded!

🎯 Applying LoRA...
  Trainable params: 67,108,864 (2.15%)
  Total params: 3,120,000,000
✅ LoRA applied!

🚀 Starting training...
```

## Troubleshooting

### Out of Memory
Script'te batch_size'ı düşür:
```python
"batch_size": 4,  # 8'den 4'e düşür
```

### Çok Yavaş
GPU kullanıldığından emin ol:
```bash
nvidia-smi  # GPU görünmeli
```

### Paket Hataları
```bash
python3 -m pip install --user --upgrade pip
python3 -m pip install --user -r requirements.txt --force-reinstall
```

## Hızlı Başlatma (Tek Komut)

Tüm setup'ı tek seferde yapmak için:

```bash
# Local'den çalıştır
ssh jensen@192.168.88.55 "mkdir -p /home/jensen/jessy-training/{datasets,models,logs}" && \
scp training/train_on_thor.py jensen@192.168.88.55:/home/jensen/jessy-training/ && \
scp training/datasets/jessy_maximum_*.jsonl jensen@192.168.88.55:/home/jensen/jessy-training/datasets/ && \
echo "✅ Dosyalar kopyalandı! Şimdi Thor'a bağlan ve training'i başlat:"
echo "ssh jensen@192.168.88.55"
echo "cd /home/jensen/jessy-training && python3 train_on_thor.py"
```

---

**🔥 Hadi Thor'un gücüyle JESSY'yi eğitelim! 🔥**
