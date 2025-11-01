# ATAK RAG Setup Summary

## Quick Deploy to Thor

```bash
# 1. Run setup (Mac)
chmod +x training/setup_atak_rag_thor.sh
./training/setup_atak_rag_thor.sh

# 2. Copy PDFs (Mac)
sshpass -p 'jensen' scp training/knowledge_base/ATAK*.pdf jensen@192.168.88.55:/home/jensen/atak-rag/pdfs/

# 3. Connect and run (Thor)
sshpass -p 'jensen' ssh jensen@192.168.88.55
cd /home/jensen/atak-rag
./run_rag.sh
```

## What It Does

- Extracts text from ATAK PDFs
- Creates vector embeddings
- Stores in ChromaDB
- Answers questions with version citations

## Example Query

```
❓ Question: How do I set up a TAK server?

📝 Answer:
To set up a TAK server, you need to...

📚 Citations:
• ATAK_User_Guide_5.3.pdf (v5.3) - Page 42
• ATAK_Installation_Guide_5.2.pdf (v5.2) - Page 15
```

## Files Created

- `training/setup_atak_rag_thor.sh` - Setup script
- `training/atak_rag.py` - RAG system
- `training/ATAK_RAG_QUICKSTART.md` - Full guide

## Architecture

```
ATAK PDFs → PyPDF2 → Text Chunks
                ↓
         Sentence Transformer
                ↓
            ChromaDB
                ↓
         Vector Search
                ↓
    Answer + Citations
```

## Features

✅ Auto version detection from filename  
✅ Citation with version + page  
✅ Semantic search  
✅ Optional fine-tuned model integration  

---

**Ready to deploy!** 🚀
