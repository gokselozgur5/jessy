#!/bin/bash
# Setup ATAK RAG on Thor

set -e

THOR_HOST="jensen@192.168.88.55"
THOR_PASS="jensen"
THOR_DIR="/home/jensen/atak-rag"

echo "========================================================================"
echo "📚 ATAK RAG System Setup on Thor"
echo "========================================================================"

echo "📁 Creating directories..."
sshpass -p "$THOR_PASS" ssh -o StrictHostKeyChecking=no $THOR_HOST "mkdir -p $THOR_DIR/{pdfs,vector_db,scripts,logs}"

echo "📤 Transferring PDFs..."
sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no training/knowledge_base/ATAK*.pdf $THOR_HOST:$THOR_DIR/pdfs/ || echo "⚠️  PDF transfer failed"

echo "📝 Creating requirements..."
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

sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no /tmp/rag_requirements.txt $THOR_HOST:$THOR_DIR/requirements.txt

echo "📝 Creating RAG script..."
cat > /tmp/atak_rag.py << 'EOFPYTHON'
#!/usr/bin/env python3
"""ATAK RAG System with Version Citations"""

import os
import re
from pathlib import Path
from typing import List, Dict, Tuple
import torch
from sentence_transformers import SentenceTransformer
import chromadb
from chromadb.config import Settings
from PyPDF2 import PdfReader
from transformers import AutoTokenizer, AutoModelForCausalLM
from peft import PeftModel

class ATAKVersionExtractor:
    """Extract version from ATAK guide filename"""
    
    PATTERNS = [
        r'ATAK[_\s-]?(\d+\.\d+)',
        r'v(\d+\.\d+)',
        r'version[_\s-]?(\d+\.\d+)',
    ]
    
    @classmethod
    def extract_version(cls, filename: str) -> str:
        for pattern in cls.PATTERNS:
            match = re.search(pattern, filename, re.IGNORECASE)
            if match:
                return match.group(1)
        return "unknown"

class ATAKRAGSystem:
    """RAG system with version tracking"""
    
    def __init__(
        self,
        pdf_dir: str = "./pdfs",
        db_dir: str = "./vector_db",
        model_name: str = "sentence-transformers/all-MiniLM-L6-v2",
        lora_path: str = None
    ):
        self.pdf_dir = Path(pdf_dir)
        self.db_dir = Path(db_dir)
        
        print("🔧 Initializing RAG System...")
        
        print(f"📥 Loading embedding model: {model_name}")
        self.embedding_model = SentenceTransformer(model_name)
        
        print("🗄️  Initializing vector database...")
        self.chroma_client = chromadb.Client(Settings(
            chroma_db_impl="duckdb+parquet",
            persist_directory=str(self.db_dir)
        ))
        
        self.collection = self.chroma_client.get_or_create_collection(
            name="atak_guides",
            metadata={"description": "ATAK guides with versions"}
        )
        
        self.llm_model = None
        self.llm_tokenizer = None
        if lora_path:
            self._load_llm(lora_path)
        
        print("✅ RAG System initialized")
    
    def _load_llm(self, lora_path: str):
        """Load fine-tuned model"""
        print(f"📥 Loading model from {lora_path}")
        base_model = "Qwen/Qwen2.5-1.5B-Instruct"
        
        self.llm_tokenizer = AutoTokenizer.from_pretrained(base_model)
        base = AutoModelForCausalLM.from_pretrained(
            base_model,
            torch_dtype=torch.float16,
            device_map="auto"
        )
        self.llm_model = PeftModel.from_pretrained(base, lora_path)
        print("✅ Model loaded")
    
    def extract_text_from_pdf(self, pdf_path: Path) -> List[Dict[str, str]]:
        """Extract text and split into chunks"""
        print(f"📄 Processing: {pdf_path.name}")
        
        version = ATAKVersionExtractor.extract_version(pdf_path.name)
        print(f"   Version: {version}")
        
        reader = PdfReader(pdf_path)
        chunks = []
        
        for page_num, page in enumerate(reader.pages, 1):
            text = page.extract_text()
            
            if text.strip():
                chunks.append({
                    "text": text,
                    "version": version,
                    "filename": pdf_path.name,
                    "page": page_num
                })
        
        print(f"   Extracted {len(chunks)} pages")
        return chunks
    
    def index_pdfs(self):
        """Index all PDFs"""
        print("\n" + "="*70)
        print("📚 Indexing ATAK Guides")
        print("="*70)
        
        pdf_files = list(self.pdf_dir.glob("*.pdf"))
        if not pdf_files:
            print("⚠️  No PDF files found!")
            return
        
        print(f"Found {len(pdf_files)} PDF files")
        
        all_chunks = []
        for pdf_path in pdf_files:
            chunks = self.extract_text_from_pdf(pdf_path)
            all_chunks.extend(chunks)
        
        print(f"\n📊 Total chunks: {len(all_chunks)}")
        
        print("🔄 Creating embeddings...")
        texts = [chunk["text"] for chunk in all_chunks]
        embeddings = self.embedding_model.encode(texts, show_progress_bar=True)
        
        print("💾 Storing in vector database...")
        self.collection.add(
            embeddings=embeddings.tolist(),
            documents=texts,
            metadatas=[{
                "version": chunk["version"],
                "filename": chunk["filename"],
                "page": chunk["page"]
            } for chunk in all_chunks],
            ids=[f"{chunk['filename']}_p{chunk['page']}" for chunk in all_chunks]
        )
        
        print("✅ Indexing complete!")
        print("="*70)
    
    def search(
        self,
        query: str,
        n_results: int = 3,
        version_filter: str = None
    ) -> List[Dict]:
        """Search relevant chunks"""
        
        query_embedding = self.embedding_model.encode([query])[0]
        
        where_filter = None
        if version_filter:
            where_filter = {"version": version_filter}
        
        results = self.collection.query(
            query_embeddings=[query_embedding.tolist()],
            n_results=n_results,
            where=where_filter
        )
        
        formatted = []
        for i in range(len(results['documents'][0])):
            formatted.append({
                "text": results['documents'][0][i],
                "version": results['metadatas'][0][i]['version'],
                "filename": results['metadatas'][0][i]['filename'],
                "page": results['metadatas'][0][i]['page'],
                "distance": results['distances'][0][i] if 'distances' in results else None
            })
        
        return formatted
    
    def answer_with_citations(self, query: str, n_results: int = 3) -> Dict:
        """Answer with version citations"""
        
        print(f"\n🔍 Query: {query}")
        
        results = self.search(query, n_results=n_results)
        
        if not results:
            return {
                "answer": "No relevant information found.",
                "citations": []
            }
        
        context = "\n\n".join([
            f"[{r['filename']} - Page {r['page']}]\n{r['text']}"
            for r in results
        ])
        
        if self.llm_model:
            prompt = f"""Based on the following ATAK documentation, answer the question.

Context:
{context}

Question: {query}

Answer:"""
            
            inputs = self.llm_tokenizer(prompt, return_tensors="pt").to(self.llm_model.device)
            outputs = self.llm_model.generate(
                **inputs,
                max_new_tokens=256,
                temperature=0.7,
                do_sample=True
            )
            answer = self.llm_tokenizer.decode(outputs[0], skip_special_tokens=True)
            answer = answer.split("Answer:")[-1].strip()
        else:
            answer = f"Based on the documentation:\n\n{results[0]['text'][:500]}..."
        
        citations = [
            f"• {r['filename']} (v{r['version']}) - Page {r['page']}"
            for r in results
        ]
        
        return {
            "answer": answer,
            "citations": citations,
            "sources": results
        }

def main():
    """Interactive RAG demo"""
    
    rag = ATAKRAGSystem(
        pdf_dir="./pdfs",
        db_dir="./vector_db",
        lora_path=None
    )
    
    if not rag.collection.count():
        rag.index_pdfs()
    else:
        print(f"✅ Database indexed ({rag.collection.count()} chunks)")
    
    print("\n" + "="*70)
    print("💬 ATAK RAG System Ready")
    print("="*70)
    print("Type your questions (or 'quit' to exit)")
    print()
    
    while True:
        query = input("❓ Question: ").strip()
        
        if query.lower() in ['quit', 'exit', 'q']:
            break
        
        if not query:
            continue
        
        result = rag.answer_with_citations(query)
        
        print("\n📝 Answer:")
        print(result['answer'])
        print("\n📚 Citations:")
        for citation in result['citations']:
            print(citation)
        print()

if __name__ == "__main__":
    main()
EOFPYTHON

sshpass -p "$THOR_PASS" scp -o StrictHostKeyChecking=no /tmp/atak_rag.py $THOR_HOST:$THOR_DIR/scripts/

echo "📝 Creating run script..."
sshpass -p "$THOR_PASS" ssh -o StrictHostKeyChecking=no $THOR_HOST "cat > $THOR_DIR/run_rag.sh << 'EOFRUN'
#!/bin/bash
set -e

cd /home/jensen/atak-rag

echo '========================================================================'
echo '🔧 Setting up RAG environment'
echo '========================================================================'

# Create venv
if [ ! -d 'venv' ]; then
    echo 'Creating virtual environment...'
    python3 -m venv venv
fi

# Activate
source venv/bin/activate

# Install
echo 'Installing requirements...'
pip install --upgrade pip
pip install -r requirements.txt

echo '========================================================================'
echo '🚀 Starting ATAK RAG System'
echo '========================================================================'

# Run
python3 scripts/atak_rag.py

EOFRUN
"

sshpass -p "$THOR_PASS" ssh -o StrictHostKeyChecking=no $THOR_HOST "chmod +x $THOR_DIR/run_rag.sh"

echo ""
echo "========================================================================"
echo "✅ ATAK RAG Setup Complete!"
echo "========================================================================"
echo ""
echo "📋 Next Steps:"
echo ""
echo "1. Copy ATAK PDFs to Thor (if not already done):"
echo "   sshpass -p 'jensen' scp training/knowledge_base/ATAK*.pdf jensen@192.168.88.55:$THOR_DIR/pdfs/"
echo ""
echo "2. SSH to Thor:"
echo "   sshpass -p 'jensen' ssh jensen@192.168.88.55"
echo ""
echo "3. Run RAG system:"
echo "   cd $THOR_DIR"
echo "   ./run_rag.sh"
echo ""
echo "4. Or run in background:"
echo "   nohup ./run_rag.sh > logs/rag.log 2>&1 &"
echo ""
echo "========================================================================"
echo "📚 RAG System Features:"
echo "   • Automatic version detection from filenames"
echo "   • Citation with version + page number"
echo "   • Vector search with ChromaDB"
echo "   • Optional fine-tuned model integration"
echo "========================================================================"
