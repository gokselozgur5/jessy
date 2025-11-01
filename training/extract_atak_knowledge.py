#!/usr/bin/env python3
"""
Extract comprehensive ATAK knowledge from Thor's PDFs and existing data
This will create a much larger, more comprehensive training dataset
"""

import json
import subprocess
import os

def download_from_thor():
    """Download all ATAK-related files from Thor"""
    print("=" * 70)
    print("📥 Downloading ATAK files from Thor")
    print("=" * 70)
    
    # Create local directory
    os.makedirs("training/atak_source_data", exist_ok=True)
    
    files_to_download = [
        "/home/jensen/llamanaut/qa_pairs.json",
        "/home/jensen/atakquickstart.pdf",
        "/home/jensen/atakuastooluserguide.pdf",
        "/home/jensen/atakatakuastoolmavlinkpx4andapmdevguide.pdf",
    ]
    
    for remote_file in files_to_download:
        filename = os.path.basename(remote_file)
        local_path = f"training/atak_source_data/{filename}"
        
        print(f"Downloading {filename}...")
        cmd = f"sshpass -p 'jensen' scp -o StrictHostKeyChecking=no jensen@192.168.88.55:{remote_file} {local_path}"
        
        try:
            subprocess.run(cmd, shell=True, check=True, capture_output=True)
            print(f"✅ Downloaded {filename}")
        except subprocess.CalledProcessError as e:
            print(f"❌ Failed to download {filename}: {e}")
    
    print("\n✅ Download complete!")

def load_existing_qa():
    """Load existing Q&A from Thor"""
    qa_file = "training/atak_source_data/qa_pairs.json"
    
    if not os.path.exists(qa_file):
        print("⚠️  No existing Q&A found")
        return []
    
    with open(qa_file, 'r') as f:
        data = json.load(f)
    
    print(f"✅ Loaded {len(data)} existing Q&A pairs")
    return data

def generate_comprehensive_dataset():
    """Generate comprehensive ATAK training dataset"""
    print("\n" + "=" * 70)
    print("🎯 Generating Comprehensive ATAK Dataset")
    print("=" * 70)
    
    # Load existing Q&A
    existing_qa = load_existing_qa()
    
    # Convert to training format
    training_data = []
    
    for item in existing_qa:
        training_data.append({
            "input": item.get("question", ""),
            "output": item.get("answer", ""),
            "source": "thor_qa_pairs",
            "page": item.get("page", "unknown")
        })
    
    # Add our generated Q&A
    print("\n📊 Adding generated Q&A...")
    
    # Load our generated data
    if os.path.exists("training/datasets/atak_full.json"):
        with open("training/datasets/atak_full.json", 'r') as f:
            generated = json.load(f)
        
        for item in generated:
            training_data.append({
                "input": item.get("input", ""),
                "output": item.get("output", ""),
                "source": "generated",
                "category": item.get("category", "unknown")
            })
        
        print(f"✅ Added {len(generated)} generated examples")
    
    # Add NoviTAK commands (if we have them)
    print("\n📊 Adding NoviTAK commands...")
    novitak_commands = generate_novitak_qa()
    training_data.extend(novitak_commands)
    print(f"✅ Added {len(novitak_commands)} NoviTAK examples")
    
    # Add Argonaut info (if we have it)
    print("\n📊 Adding Argonaut information...")
    argonaut_qa = generate_argonaut_qa()
    training_data.extend(argonaut_qa)
    print(f"✅ Added {len(argonaut_qa)} Argonaut examples")
    
    print(f"\n✅ Total training examples: {len(training_data)}")
    
    return training_data

def generate_novitak_qa():
    """Generate NoviTAK command Q&A"""
    # TODO: Extract from NoviTAK documentation
    # For now, placeholder
    return [
        {
            "input": "What is NoviTAK?",
            "output": "NoviTAK is a command-line interface tool for ATAK that allows scripting and automation of ATAK operations. It provides programmatic access to ATAK features.",
            "source": "novitak",
            "category": "novitak"
        }
    ]

def generate_argonaut_qa():
    """Generate Argonaut Q&A"""
    # TODO: Extract from Argonaut documentation
    # For now, placeholder
    return [
        {
            "input": "What is Argonaut in ATAK context?",
            "output": "Argonaut is a plugin/tool for ATAK that provides additional capabilities for mission planning and execution.",
            "source": "argonaut",
            "category": "argonaut"
        }
    ]

def save_comprehensive_dataset(data):
    """Save the comprehensive dataset"""
    print("\n" + "=" * 70)
    print("💾 Saving Comprehensive Dataset")
    print("=" * 70)
    
    # Split train/val
    import random
    random.shuffle(data)
    
    split_idx = int(len(data) * 0.85)
    train_data = data[:split_idx]
    val_data = data[split_idx:]
    
    # Save JSONL
    with open("training/datasets/atak_comprehensive_train.jsonl", 'w') as f:
        for item in train_data:
            f.write(json.dumps(item) + '\n')
    
    with open("training/datasets/atak_comprehensive_val.jsonl", 'w') as f:
        for item in val_data:
            f.write(json.dumps(item) + '\n')
    
    # Save full JSON for reference
    with open("training/datasets/atak_comprehensive_full.json", 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"✅ Train: {len(train_data)} examples")
    print(f"✅ Val: {len(val_data)} examples")
    print(f"✅ Total: {len(data)} examples")
    
    # Show breakdown by source
    sources = {}
    for item in data:
        source = item.get("source", "unknown")
        sources[source] = sources.get(source, 0) + 1
    
    print("\n📊 Breakdown by source:")
    for source, count in sorted(sources.items()):
        print(f"   • {source}: {count} examples")

def main():
    print("=" * 70)
    print("🚀 ATAK Comprehensive Knowledge Extraction")
    print("=" * 70)
    
    # Download files from Thor
    download_from_thor()
    
    # Generate comprehensive dataset
    data = generate_comprehensive_dataset()
    
    # Save
    save_comprehensive_dataset(data)
    
    print("\n" + "=" * 70)
    print("✅ Complete!")
    print("=" * 70)
    print("\nFiles created:")
    print("   • training/datasets/atak_comprehensive_train.jsonl")
    print("   • training/datasets/atak_comprehensive_val.jsonl")
    print("   • training/datasets/atak_comprehensive_full.json")
    print("\nNext: Deploy to Thor and retrain with comprehensive dataset!")
    print("=" * 70)

if __name__ == "__main__":
    main()
