#!/bin/bash
# Start training on Thor and monitor progress

THOR_HOST="jensen@192.168.88.55"
THOR_PASSWORD="jensen"
THOR_DIR="/home/jensen/jessy-training"

echo "========================================================================"
echo "🚀 Starting JESSY Training on Thor"
echo "========================================================================"

# Start training in background
echo "Starting training in background..."
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "cd $THOR_DIR && nohup ./run_training.sh > training.out 2>&1 &"

echo "✅ Training started!"
echo ""
echo "Waiting 5 seconds for training to initialize..."
sleep 5

echo ""
echo "========================================================================"
echo "📊 Training Output (Press Ctrl+C to stop monitoring, training continues)"
echo "========================================================================"
echo ""

# Monitor training output
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "tail -f $THOR_DIR/training.out"
