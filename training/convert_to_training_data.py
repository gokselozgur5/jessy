#!/usr/bin/env python3
"""
JESSY Knowledge Base → Fine-Tuning Data Converter

Tüm MD dosyalarını okur ve Q&A formatına çevirir.
"""

import json
import os
from pathlib import Path
import re

def extract_sections(content):
    """MD dosyasından section'ları çıkar"""
    sections = []
    current_section = {"title": "", "content": ""}
    
    for line in content.split('\n'):
        if line.startswith('#'):
            if current_section["content"]:
                sections.append(current_section)
            current_section = {
                "title": line.strip('#').strip(),
                "content": ""
            }
        else:
            current_section["content"] += line + "\n"
    
    if current_section["content"]:
        sections.append(current_section)
    
    return sections

def create_qa_pairs(filepath, content):
    """Dosyadan Q&A çiftleri oluştur"""
    pairs = []
    filename = os.path.basename(filepath)
    
    # Steering files - Philosophy
    if 'steering' in filepath:
        sections = extract_sections(content)
        for section in sections:
            if len(section["content"].strip()) > 50:
                pairs.append({
                    "question": f"JESSY'nin {section['title']} prensibi nedir?",
                    "answer": f"{section['content'].strip()}\n\nBu JESSY'nin temel prensiplerinden biridir."
                })
    
    # Documentation
    elif 'docs' in filepath or 'README' in filename:
        sections = extract_sections(content)
        for section in sections:
            if len(section["content"].strip()) > 50:
                pairs.append({
                    "question": f"{section['title']} hakkında bilgi ver",
                    "answer": section['content'].strip()
                })
    
    # Specs
    elif 'specs' in filepath:
        if 'requirements' in filename:
            pairs.append({
                "question": f"{Path(filepath).parent.name} özelliğinin gereksinimleri neler?",
                "answer": content.strip()
            })
        elif 'design' in filename:
            pairs.append({
                "question": f"{Path(filepath).parent.name} nasıl tasarlandı?",
                "answer": content.strip()
            })
    
    # Progress notes
    elif 'PROGRESS' in filename or 'COMPLETE' in filename:
        pairs.append({
            "question": f"{filename.replace('.md', '')} hakkında ne biliyorsun?",
            "answer": content.strip()
        })
    
    return pairs

def convert_to_training_format(qa_pairs):
    """Q&A çiftlerini training formatına çevir"""
    training_data = []
    
    for pair in qa_pairs:
        training_data.append({
            "conversations": [
                {
                    "from": "human",
                    "value": pair["question"]
                },
                {
                    "from": "jessy",
                    "value": pair["answer"]
                }
            ]
        })
    
    return training_data

def main():
    knowledge_base = Path("training/knowledge_base")
    all_qa_pairs = []
    
    print("🔄 Converting knowledge base to training data...")
    
    # Tüm MD dosyalarını işle
    for filepath in knowledge_base.rglob("*.md"):
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
            
            qa_pairs = create_qa_pairs(str(filepath), content)
            all_qa_pairs.extend(qa_pairs)
            
            if qa_pairs:
                print(f"   ✓ {filepath.name}: {len(qa_pairs)} Q&A pairs")
        
        except Exception as e:
            print(f"   ✗ {filepath.name}: {e}")
    
    # Training formatına çevir
    training_data = convert_to_training_format(all_qa_pairs)
    
    # Kaydet
    output_file = "training/jessy_knowledge_training.json"
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(training_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n✅ Conversion complete!")
    print(f"   Total Q&A pairs: {len(all_qa_pairs)}")
    print(f"   Output: {output_file}")
    print(f"   Size: {os.path.getsize(output_file) / 1024:.1f} KB")
    
    # Örnek göster
    if training_data:
        print(f"\n📝 Example:")
        example = training_data[0]
        print(f"   Q: {example['conversations'][0]['value'][:80]}...")
        print(f"   A: {example['conversations'][1]['value'][:80]}...")

if __name__ == "__main__":
    main()
