#!/usr/bin/env python3
"""
Prepare Training Data for Ollama Fine-Tuning

Converts our Turkish training data to Ollama's format.
"""

import json

def main():
    print("🔄 Preparing Ollama Training Data...")
    print("=" * 60)
    
    # Load training data
    with open('training/jessy_train.json', 'r', encoding='utf-8') as f:
        train_data = json.load(f)
    
    print(f"📊 Loaded {len(train_data)} training examples")
    
    # Convert to Ollama format
    ollama_format = []
    
    for example in train_data:
        instruction = example['instruction']
        output = example['output']
        
        # Ollama format: simple Q&A pairs
        ollama_format.append(f"### Instruction:\n{instruction}\n\n### Response:\n{output}\n")
    
    # Save
    output_file = 'training/jessy_ollama_training.txt'
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(ollama_format))
    
    print(f"💾 Saved to: {output_file}")
    print(f"📊 File size: {len('\\n'.join(ollama_format)) / 1024:.1f} KB")
    
    # Show sample
    print(f"\n📝 Sample (first 3 examples):")
    for i, example in enumerate(ollama_format[:3]):
        print(f"\n--- Example {i+1} ---")
        print(example[:200] + "..." if len(example) > 200 else example)
    
    print("\n" + "=" * 60)
    print("✨ Ollama training data ready!")
    print("\nNext: Run Ollama fine-tuning:")
    print(f"  ollama create jessy-turkish -f training/Modelfile.jessy-turkish")


if __name__ == "__main__":
    main()
