#!/bin/bash
# 🔨 Thor Setup Script
# Thor'a training dosyalarını kopyala ve hazırla

set -e

THOR_USER="jensen"
THOR_IP="192.168.88.55"
THOR_DIR="/home/jensen/jessy-training"

echo "╔══════════════════════════════════════════════════════════╗"
echo "║                                                          ║"
echo "║              🔨 SETTING UP THOR FOR TRAINING 🔨         ║"
echo "║                                                          ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# 1. Thor'da dizin oluştur
echo "📁 Creating directory on Thor..."
ssh ${THOR_USER}@${THOR_IP} "mkdir -p ${THOR_DIR}/{datasets,models,logs}"

# 2. Training script'i kopyala
echo "📤 Copying training script..."
scp training/train_on_thor.py ${THOR_USER}@${THOR_IP}:${THOR_DIR}/

# 3. Dataset'leri kopyala
echo "📤 Copying datasets..."
if [ -f "training/datasets/jessy_maximum_train.jsonl" ]; then
    scp training/datasets/jessy_maximum_train.jsonl ${THOR_USER}@${THOR_IP}:${THOR_DIR}/datasets/
    scp training/datasets/jessy_maximum_val.jsonl ${THOR_USER}@${THOR_IP}:${THOR_DIR}/datasets/
else
    echo "⚠️  Dataset files not found! Creating them first..."
    # Dataset yoksa oluştur
    python3 training/combine_all_training_data.py
    scp training/datasets/jessy_maximum_train.jsonl ${THOR_USER}@${THOR_IP}:${THOR_DIR}/datasets/
    scp training/datasets/jessy_maximum_val.jsonl ${THOR_USER}@${THOR_IP}:${THOR_DIR}/datasets/
fi

# 4. Requirements dosyası oluştur ve kopyala
echo "📤 Creating and copying requirements..."
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

scp /tmp/thor_requirements.txt ${THOR_USER}@${THOR_IP}:${THOR_DIR}/requirements.txt

# 5. Thor'da Python paketlerini kur
echo "📦 Installing Python packages on Thor..."
ssh ${THOR_USER}@${THOR_IP} << 'ENDSSH'
cd /home/jensen/jessy-training
python3 -m pip install --user --upgrade pip
python3 -m pip install --user -r requirements.txt
ENDSSH

# 6. GPU kontrolü
echo "🎮 Checking GPU on Thor..."
ssh ${THOR_USER}@${THOR_IP} "nvidia-smi || echo 'GPU check failed, but training might still work'"

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║                                                          ║"
echo "║                  ✅ THOR IS READY! ✅                    ║"
echo "║                                                          ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "🚀 To start training, run:"
echo ""
echo "   ssh ${THOR_USER}@${THOR_IP}"
echo "   cd ${THOR_DIR}"
echo "   python3 train_on_thor.py"
echo ""
echo "Or run directly:"
echo ""
echo "   ssh ${THOR_USER}@${THOR_IP} 'cd ${THOR_DIR} && python3 train_on_thor.py'"
echo ""
