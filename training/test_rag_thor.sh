#!/bin/bash
# Test RAG System on Thor

echo "🔧 Testing ATAK RAG System on Thor..."
echo ""

sshpass -p 'jensen' ssh jensen@192.168.88.55 << 'EOF'
cd /home/jensen/atak-rag

echo "========================================================================"
echo "🔧 Setting up environment"
echo "========================================================================"

# Activate venv
source venv/bin/activate

# Check if packages installed
echo "📦 Checking installed packages..."
pip list | grep -E "(torch|transformers|sentence|chroma)" || echo "⚠️  Packages not installed yet"

echo ""
echo "========================================================================"
echo "📥 Installing requirements (this may take a while)..."
echo "========================================================================"

pip install --upgrade pip
pip install -r requirements.txt

echo ""
echo "========================================================================"
echo "✅ Installation complete!"
echo "========================================================================"

# Check PDFs
echo ""
echo "📄 PDF files:"
ls -lh pdfs/

echo ""
echo "========================================================================"
echo "🚀 Ready to run RAG system"
echo "========================================================================"
echo ""
echo "To run interactively:"
echo "  cd /home/jensen/atak-rag"
echo "  source venv/bin/activate"
echo "  python3 scripts/atak_rag.py"
echo ""

EOF
