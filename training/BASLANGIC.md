# 🚀 Jessy LoRA Training - Hızlı Başlangıç

## ✅ Hazırlık Tamamlandı!

Her şey hazır, training'e başlayabilirsin.

## 📊 Durum

```
✅ PyTorch 2.9.0 + MPS (Apple Silicon)
✅ Tüm paketler yüklü (transformers, peft, datasets)
✅ Training data: 3,242 örnek
✅ Validation data: 361 örnek
✅ Disk alanı: 6.2 GB (yeterli)
```

## ⏱️ Süre Tahmini

```
Epochs: 3
Steps: 606
Tahmini süre: ~20 dakika (M2'de)
```

## 🎯 Training Başlatma

### Yöntem 1: Otomatik Script (Önerilen)

```bash
cd training
./start_training.sh
```

### Yöntem 2: Manuel

```bash
cd training
source venv/bin/activate
python3 train_lora_m2.py
```

## 📁 Çıktılar

Training tamamlandığında:

```
training/
├── models/jessy-lora-m2/          # Ana model
│   ├── lora_adapters/              # LoRA adaptörleri
│   ├── config.json
│   └── adapter_model.bin
├── checkpoints/jessy-lora/         # Ara checkpointler
└── training_log.txt                # Training log'u
```

## 🔍 Training Sırasında

Terminal'de göreceksin:
- Loss değerleri (azalmalı)
- Training/Validation metrikleri
- Checkpoint kaydetme
- Tahmini kalan süre

## ⚡ Optimizasyonlar

M2 için optimize edilmiş:
- ✅ MPS acceleration
- ✅ Küçük batch size (2)
- ✅ Gradient accumulation (8)
- ✅ Full precision (fp32)
- ✅ LoRA rank 16

## 🎯 Sonraki Adımlar

Training bittikten sonra:

1. **Test Et:**
   ```bash
   python3 test_model.py
   ```

2. **Ollama'ya Deploy Et:**
   ```bash
   # Model'i Ollama formatına çevir
   python3 convert_to_ollama.py
   
   # Ollama'da çalıştır
   ollama run jessy-lora
   ```

3. **Karşılaştır:**
   - Eski Jessy (Modelfile)
   - Yeni Jessy (LoRA)

## 🐛 Sorun Giderme

### Training çok yavaş
```bash
# MPS kullanıldığını kontrol et
python3 -c "import torch; print(torch.backends.mps.is_available())"
```

### Out of memory
```bash
# Batch size'ı azalt (train_lora_m2.py içinde)
per_device_train_batch_size: 1  # 2 yerine
```

### Dataset hatası
```bash
# Dataset formatını kontrol et
head -1 datasets/jessy_maximum_train.jsonl | python3 -m json.tool
```

## 📈 Beklenen Sonuçlar

```
Epoch 1: Loss ~2.5 → ~1.8
Epoch 2: Loss ~1.8 → ~1.5
Epoch 3: Loss ~1.5 → ~1.3

Final: Eval loss ~1.2-1.4
```

## 🎉 Başarı Kriterleri

Training başarılı sayılır:
- ✅ Loss azalıyor
- ✅ Eval loss < 1.5
- ✅ Model kaydedildi
- ✅ Test çıktıları mantıklı

## 💡 İpuçları

1. **İlk kez:** Model indirilecek (~6GB), biraz zaman alır
2. **Gece:** Overnight training gerekmiyor, 20 dakika yeter
3. **Checkpoint:** Her 100 step'te kaydediliyor
4. **Log:** training_log.txt'ye kaydediliyor

## 🚀 Hadi Başlayalım!

```bash
cd training
./start_training.sh
```

---

**Not:** İlk training'de model indirilecek. Sonraki training'ler daha hızlı başlar.
