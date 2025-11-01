#!/bin/bash
# Run RAG System Test on Thor

echo "🚀 Running ATAK RAG System Test..."
echo ""

sshpass -p 'jensen' ssh jensen@192.168.88.55 << 'EOF'
cd /home/jensen/atak-rag
source venv/bin/activate

echo "========================================================================"
echo "🔧 Testing ATAK RAG System"
echo "========================================================================"
echo ""

# Run with test queries
python3 << 'PYTEST'
import sys
sys.path.insert(0, '/home/jensen/atak-rag/scripts')

from atak_rag import ATAKRAGSystem

print("🔧 Initializing RAG System...")
rag = ATAKRAGSystem(
    pdf_dir="/home/jensen/atak-rag/pdfs",
    db_dir="/home/jensen/atak-rag/vector_db"
)

# Index if needed
if rag.collection.count() == 0:
    print("\n📚 Indexing PDFs (first time)...")
    rag.index_pdfs()
else:
    print(f"\n✅ Database already indexed ({rag.collection.count()} chunks)")

# Test queries
test_queries = [
    "What is ATAK?",
    "How do I set up a TAK server?",
    "What are CoT messages?"
]

print("\n" + "="*70)
print("🧪 Running Test Queries")
print("="*70)

for query in test_queries:
    print(f"\n{'='*70}")
    print(f"❓ Query: {query}")
    print("="*70)
    
    result = rag.answer_with_citations(query, n_results=2)
    
    print("\n📝 Answer:")
    print(result['answer'][:500] + "..." if len(result['answer']) > 500 else result['answer'])
    
    print("\n📚 Citations:")
    for citation in result['citations']:
        print(f"  {citation}")
    print()

print("\n" + "="*70)
print("✅ Test Complete!")
print("="*70)

PYTEST

EOF
