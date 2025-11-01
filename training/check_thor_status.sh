#!/bin/bash
# Check training status on Thor

THOR_HOST="jensen@192.168.88.55"
THOR_PASSWORD="jensen"
THOR_DIR="/home/jensen/jessy-training"

echo "========================================================================"
echo "📊 Thor Training Status"
echo "========================================================================"

# Check if training is running
echo ""
echo "🔍 Checking if training is running..."
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "ps aux | grep train_thor.py | grep -v grep" || echo "❌ Training not running"

echo ""
echo "========================================================================"
echo "💾 Disk Usage"
echo "========================================================================"
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "du -sh $THOR_DIR/* 2>/dev/null"

echo ""
echo "========================================================================"
echo "📝 Recent Log Files"
echo "========================================================================"
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "ls -lht $THOR_DIR/logs/ 2>/dev/null | head -5" || echo "No logs yet"

echo ""
echo "========================================================================"
echo "🎯 GPU Status"
echo "========================================================================"
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "nvidia-smi --query-gpu=index,name,temperature.gpu,utilization.gpu,utilization.memory,memory.used,memory.total --format=csv,noheader,nounits"

echo ""
echo "========================================================================"
echo "📊 Last 20 lines of training output"
echo "========================================================================"
sshpass -p $THOR_PASSWORD ssh $THOR_HOST "tail -20 $THOR_DIR/training.out 2>/dev/null" || echo "No output yet"

echo ""
echo "========================================================================"
