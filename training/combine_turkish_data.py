#!/usr/bin/env python3
"""
Combine Turkish Training Data

Merges all Turkish training files into a single dataset
ready for fine-tuning.
"""

import json
from pathlib import Path


def load_json(filepath):
    """Load JSON file"""
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def save_json(data, filepath):
    """Save JSON file"""
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def convert_to_training_format(conversations):
    """Convert to Hugging Face training format"""
    training_data = []
    
    for item in conversations:
        convs = item['conversations']
        
        # Extract human and assistant messages
        human_msg = None
        assistant_msg = None
        
        for msg in convs:
            if msg['from'] == 'human':
                human_msg = msg['value']
            elif msg['from'] == 'jessy':
                assistant_msg = msg['value']
        
        if human_msg and assistant_msg:
            training_data.append({
                'instruction': human_msg,
                'output': assistant_msg
            })
    
    return training_data


def main():
    print("🔄 Combining Turkish Training Data...")
    print("=" * 60)
    
    # Load all Turkish data files
    files = [
        'training/jessy_turkish_conversational.json',
        'training/jessy_turkish_qa_chat.json',
        'training/jessy_iteration_control.json'
    ]
    
    all_conversations = []
    
    for filepath in files:
        data = load_json(filepath)
        count = len(data)
        all_conversations.extend(data)
        print(f"✅ Loaded {filepath}: {count} examples")
    
    print(f"\n📊 Total conversations: {len(all_conversations)}")
    
    # Convert to training format
    training_data = convert_to_training_format(all_conversations)
    
    print(f"📊 Training examples: {len(training_data)}")
    
    # Save combined data
    output_file = 'training/jessy_turkish_combined.json'
    save_json(training_data, output_file)
    
    print(f"\n💾 Saved to: {output_file}")
    
    # Show sample
    print(f"\n📝 Sample training example:")
    sample = training_data[0]
    print(f"   Instruction: {sample['instruction']}")
    print(f"   Output: {sample['output']}")
    
    # Split train/validation (90/10)
    split_idx = int(len(training_data) * 0.9)
    train_data = training_data[:split_idx]
    val_data = training_data[split_idx:]
    
    save_json(train_data, 'training/jessy_train.json')
    save_json(val_data, 'training/jessy_val.json')
    
    print(f"\n📊 Split:")
    print(f"   Train: {len(train_data)} examples (90%)")
    print(f"   Validation: {len(val_data)} examples (10%)")
    
    print("\n" + "=" * 60)
    print("✨ Data combination complete!")
    print("   Ready for fine-tuning with PyTorch MPS")


if __name__ == "__main__":
    main()
