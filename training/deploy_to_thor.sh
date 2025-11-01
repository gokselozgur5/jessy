#!/bin/bash
# JESSY Training Deployment to Thor
# Bu script training dosyalarını Thor'a transfer eder ve training'i başlatır

set -e  # Exit on error

# Configuration
THOR_HOST="jensen@192.168.88.55"
THOR_PASSWORD="jensen"  # Username ile aynı
THOR_DIR="/home/jensen/jessy-training"
LOCAL_DIR="$(pwd)"

# SSH/SCP with password
SSH_CMD="sshpass -p $THOR_PASSWORD ssh"
SCP_CMD="sshpass -p $THOR_PASSWORD scp"

echo "========================================================================"
echo "⚡ JESSY Training Deployment to Thor"
echo "========================================================================"
echo "Thor Host: $THOR_HOST"
echo "Thor Directory: $THOR_DIR"
echo "Local Directory: $LOCAL_DIR"
echo "========================================================================"

# 1. Create directory on Thor
echo "📁 Creating directory on Thor..."
$SSH_CMD $THOR_HOST "mkdir -p $THOR_DIR/datasets $THOR_DIR/models $THOR_DIR/checkpoints $THOR_DIR/logs"

# 2. Transfer training script
echo "📤 Transferring training script..."
$SCP_CMD training/train_thor.py $THOR_HOST:$THOR_DIR/

# 3. Transfer datasets
echo "📤 Transferring datasets..."
$SCP_CMD training/datasets/jessy_maximum_train.jsonl $THOR_HOST:$THOR_DIR/datasets/
$SCP_CMD training/datasets/jessy_maximum_val.jsonl $THOR_HOST:$THOR_DIR/datasets/

# 4. Create requirements file
echo "📝 Creating requirements.txt..."
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

$SCP_CMD /tmp/thor_requirements.txt $THOR_HOST:$THOR_DIR/requirements.txt

# 5. Create run script
echo "📝 Creating run script on Thor..."
$SSH_CMD $THOR_HOST "cat > $THOR_DIR/run_training.sh << 'EOFRUN'
#!/bin/bash
set -e

cd /home/jensen/jessy-training

echo '========================================================================'
echo '🔧 Setting up Python environment'
echo '========================================================================'

# Check if venv exists
if [ ! -d 'venv' ]; then
    echo 'Creating virtual environment...'
    python3 -m venv venv
fi

# Activate venv
source venv/bin/activate

# Install requirements
echo 'Installing requirements...'
pip install --upgrade pip
pip install -r requirements.txt

echo '========================================================================'
echo '🚀 Starting Training'
echo '========================================================================'

# Run training
python3 train_thor.py 2>&1 | tee logs/training_\$(date +%Y%m%d_%H%M%S).log

echo '========================================================================'
echo '✅ Training Complete!'
echo '========================================================================'
EOFRUN
"

# Make run script executable
$SSH_CMD $THOR_HOST "chmod +x $THOR_DIR/run_training.sh"

echo "========================================================================"
echo "✅ Deployment Complete!"
echo "========================================================================"
echo ""
echo "Next steps:"
echo ""
echo "1. SSH to Thor:"
echo "   ssh $THOR_HOST"
echo ""
echo "2. Start training:"
echo "   cd $THOR_DIR"
echo "   ./run_training.sh"
echo ""
echo "3. Monitor training:"
echo "   tail -f logs/training_*.log"
echo ""
echo "4. Or run in background:"
echo "   nohup ./run_training.sh > training.out 2>&1 &"
echo "   tail -f training.out"
echo ""
echo "========================================================================"
echo "⚡ Ready to train on Thor!"
echo "========================================================================"
