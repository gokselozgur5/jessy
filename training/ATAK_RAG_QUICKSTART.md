# 🚀 ATAK RAG System - Quick Start

## Copy-Paste Commands

### 1️⃣ Run Setup Script (Mac)

```bash
chmod +x training/setup_atak_rag_thor.sh
./training/setup_atak_rag_thor.sh
```

### 2️⃣ Copy ATAK PDFs (Mac)

```bash
sshpass -p 'jensen' scp training/knowledge_base/ATAK*.pdf jensen@192.168.88.55:/home/jensen/atak-rag/pdfs/
```

### 3️⃣ Connect to Thor

```bash
sshpass -p 'jensen' ssh jensen@192.168.88.55
```

### 4️⃣ Start RAG System (on Thor)

```bash
cd /home/jensen/atak-rag
./run_rag.sh
```

---

## Alternative: Step by Step

### Create Directory on Thor

```bash
sshpass -p 'jensen' ssh jensen@192.168.88.55 "mkdir -p /home/jensen/atak-rag/{pdfs,vector_db,scripts,logs}"
```

### Create and Copy Requirements

```bash
cat > /tmp/rag_requirements.txt << 'EOF'
torch>=2.0.0
transformers>=4.35.0
sentence-transformers>=2.2.0
chromadb>=0.4.0
pypdf2>=3.0.0
langchain>=0.1.0
langchain-community>=0.0.10
peft>=0.7.0
accelerate>=0.24.0
EOF

sshpass -p 'jensen' scp /tmp/rag_requirements.txt jensen@192.168.88.55:/home/jensen/atak-rag/
```

### Copy RAG Script

```bash
sshpass -p 'jensen' scp training/atak_rag.py jensen@192.168.88.55:/home/jensen/atak-rag/scripts/
```

### Copy PDFs

```bash
sshpass -p 'jensen' scp training/knowledge_base/ATAK*.pdf jensen@192.168.88.55:/home/jensen/atak-rag/pdfs/
```

### Setup Environment on Thor

```bash
sshpass -p 'jensen' ssh jensen@192.168.88.55 << 'EOF'
cd /home/jensen/atak-rag
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
EOF
```

### Run RAG System

```bash
sshpass -p 'jensen' ssh jensen@192.168.88.55 << 'EOF'
cd /home/jensen/atak-rag
source venv/bin/activate
python3 scripts/atak_rag.py
EOF
```

---

## Test Queries

Try these after RAG system starts:

```
❓ Question: How do I set up a TAK server?
❓ Question: What is a CoT message?
❓ Question: How do I configure ATAK for offline use?
❓ Question: What are the system requirements for ATAK?
```

---

## Expected Output

```
========================================================================
📚 ATAK RAG System Ready
========================================================================
Type your questions (or 'quit' to exit)

❓ Question: How do I set up a TAK server?

🔍 Query: How do I set up a TAK server?

📝 Answer:
To set up a TAK server, you need to...

📚 Citations:
• ATAK_User_Guide_5.3.pdf (v5.3) - Page 42
• ATAK_Installation_Guide_5.2.pdf (v5.2) - Page 15
• ATAK_Server_Setup_5.5.pdf (v5.5) - Page 8
```

---

## Features

✅ **Auto Version Detection**
- Extract version from filename
- Supports ATAK_5.1, ATAK-5.2, v5.3 formats

✅ **Answer with Citations**
- Shows source file
- Includes version number
- Provides page number

✅ **Vector Search**
- Semantic search for relevant chunks
- Fast search with ChromaDB
- Embeddings via sentence-transformers

✅ **Fine-tuned Model Integration**
- Use trained ATAK model
- LoRA adapter support
- Better answers

---

## Troubleshooting

### PDFs Not Found

```bash
# Check PDF location
ls -la training/knowledge_base/ATAK*.pdf

# Manual copy
scp training/knowledge_base/*.pdf jensen@192.168.88.55:/home/jensen/atak-rag/pdfs/
```

### ChromaDB Error

```bash
# On Thor
cd /home/jensen/atak-rag
source venv/bin/activate
pip install --upgrade chromadb
```

### Out of Memory

```python
# In atak_rag.py, use smaller model:
model_name = "sentence-transformers/all-MiniLM-L6-v2"
```

### Slow Indexing

```bash
# Check GPU usage
nvidia-smi

# First indexing takes time on CPU
```

---

## Advanced Usage

### Use Fine-tuned Model

```python
# Use trained model on Thor
rag = ATAKRAGSystem(
    pdf_dir="./pdfs",
    db_dir="./vector_db",
    lora_path="/home/jensen/atak-training/checkpoints/checkpoint-40"
)
```

### Version Filtering

```python
# Search only v5.3
results = rag.search(
    query="TAK server setup",
    version_filter="5.3"
)
```

### Run as API

```python
# Add Flask API
from flask import Flask, request, jsonify

app = Flask(__name__)
rag = ATAKRAGSystem()

@app.route('/query', methods=['POST'])
def query():
    data = request.json
    result = rag.answer_with_citations(data['query'])
    return jsonify(result)

app.run(host='0.0.0.0', port=5000)
```

---

## Next Steps

1. ✅ RAG system working
2. Integrate fine-tuned model
3. Add web API
4. Integrate with Jessy
5. Add multi-version comparison

---

**"Nothing is true, everything is permitted."**  
Including RAG systems with version citations! 📚⚡
