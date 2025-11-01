#!/bin/bash
# Deploy ATAK Training to Thor

set -e

THOR_HOST="jensen@192.168.88.55"
THOR_PASS="jensen"
THOR_DIR="/home/jensen/atak-training"

echo "========================================================================"
echo "⚡ ATAK Training Deployment to Thor"
echo "========================================================================"

# 1. Create directory
echo "📁 Creating directory on Thor..."
sshpass -p "$THOR_PASS" ssh -o StrictHostKeyChecking=no $THOR_HOST "mkdir -p $THOR_DIR/datasets $THOR_DIR/models $THOR_DIR/checkpoints $THOR_DIR/logs"

# 2. Transfer training script
echo "📤 Transferring training script..."
sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no training/train_atak_thor.py $THOR_HOST:$THOR_DIR/

# 3. Transfer datasets
echo "📤 Transferring ATAK datasets..."
sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no training/datasets/atak_train.jsonl $THOR_HOST:$THOR_DIR/datasets/
sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no training/datasets/atak_val.jsonl $THOR_HOST:$THOR_DIR/datasets/

# 4. Create requirements
echo "📝 Creating requirements..."
cat > /tmp/atak_requirements.txt << 'EOF'
torch>=2.0.0
transformers>=4.35.0
peft>=0.7.0
datasets>=2.14.0
accelerate>=0.24.0
bitsandbytes>=0.41.0
sentencepiece>=0.1.99
protobuf>=3.20.0
EOF

sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no /tmp/atak_requirements.txt $THOR_HOST:$THOR_DIR/requirements.txt

# 5. Create run script
echo "📝 Creating run script..."
sshpass -p "$THOR_PASS" ssh -o StrictHostKeyChecking=no $THOR_HOST "cat > $THOR_DIR/run_training.sh << 'EOFRUN'
#!/bin/bash
set -e

cd /home/jensen/atak-training

echo '========================================================================'
echo '🔧 Setting up environment'
echo '========================================================================'

# Create venv if needed
if [ ! -d 'venv' ]; then
    echo 'Creating virtual environment...'
    python3 -m venv venv
fi

# Activate
source venv/bin/activate

# Install requirements
echo 'Installing requirements...'
pip install --upgrade pip
pip install -r requirements.txt

echo '========================================================================'
echo '🚀 Starting ATAK Training'
echo '========================================================================'

# Run training
python3 train_atak_thor.py 2>&1 | tee logs/training_\$(date +%Y%m%d_%H%M%S).log

echo '========================================================================'
echo '✅ Training Complete!'
echo '========================================================================'
EOFRUN
"

# Make executable
sshpass -p "$THOR_PASS" ssh -o StrictHostKeyChecking=no $THOR_HOST "chmod +x $THOR_DIR/run_training.sh"

echo "========================================================================"
echo "✅ Deployment Complete!"
echo "========================================================================"
echo ""
echo "📊 Dataset Info:"
echo "   • Training examples: 79"
echo "   • Validation examples: 14"
echo "   • Total: 93 ATAK Q&A pairs"
echo ""
echo "🎯 Next Steps:"
echo ""
echo "1. SSH to Thor:"
echo "   sshpass -p 'jensen' ssh jensen@192.168.88.55"
echo ""
echo "2. Start training:"
echo "   cd $THOR_DIR"
echo "   ./run_training.sh"
echo ""
echo "3. Monitor (in another terminal):"
echo "   sshpass -p 'jensen' ssh jensen@192.168.88.55 'tail -f $THOR_DIR/logs/training_*.log'"
echo ""
echo "4. Or run in background:"
echo "   nohup ./run_training.sh > training.out 2>&1 &"
echo ""
echo "========================================================================"
echo "⚡ Ready to train ATAK AI on Thor!"
echo "========================================================================"
