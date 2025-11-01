#!/usr/bin/env python3
"""
Combine all existing JESSY training data into unified dataset
"""

import json
import random
from pathlib import Path
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn

console = Console()

def load_json_file(filepath):
    """Load JSON file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
            if isinstance(data, list):
                return data
            elif isinstance(data, dict) and 'conversations' in data:
                return data['conversations']
            elif isinstance(data, dict) and 'examples' in data:
                return data['examples']
            else:
                return [data]
    except Exception as e:
        console.print(f"[yellow]Warning: Could not load {filepath}: {e}[/yellow]")
        return []

def normalize_example(ex, source_file):
    """Normalize example to standard format"""
    
    # Already in correct format
    if 'input' in ex and 'output' in ex:
        return {
            'input': ex['input'],
            'output': ex['output'],
            'source': source_file,
            'category': ex.get('category', 'general'),
        }
    
    # Conversations array format (from/value)
    if 'conversations' in ex:
        convs = ex['conversations']
        if len(convs) >= 2:
            user_msg = next((c['value'] for c in convs if c.get('from') in ['human', 'user']), None)
            assistant_msg = next((c['value'] for c in convs if c.get('from') in ['jessy', 'assistant', 'gpt']), None)
            if user_msg and assistant_msg:
                return {
                    'input': user_msg,
                    'output': assistant_msg,
                    'source': source_file,
                    'category': 'conversation',
                }
    
    # Messages format
    if 'messages' in ex:
        messages = ex['messages']
        if len(messages) >= 2:
            return {
                'input': messages[0].get('content', ''),
                'output': messages[1].get('content', ''),
                'source': source_file,
                'category': 'conversation',
            }
    
    # Question/Answer format
    if 'question' in ex and 'answer' in ex:
        return {
            'input': ex['question'],
            'output': ex['answer'],
            'source': source_file,
            'category': ex.get('category', 'qa'),
        }
    
    # Prompt/Response format
    if 'prompt' in ex and 'response' in ex:
        return {
            'input': ex['prompt'],
            'output': ex['response'],
            'source': source_file,
            'category': ex.get('category', 'general'),
        }
    
    # Text/Completion format
    if 'text' in ex:
        # Try to split on common patterns
        text = ex['text']
        if '\nJESSY:' in text or '\nAssistant:' in text:
            parts = text.split('\nJESSY:') if '\nJESSY:' in text else text.split('\nAssistant:')
            if len(parts) >= 2:
                return {
                    'input': parts[0].replace('User:', '').replace('Human:', '').strip(),
                    'output': parts[1].strip(),
                    'source': source_file,
                    'category': 'text',
                }
    
    return None

def main():
    console.print("[bold green]🚀 Combining All JESSY Training Data[/bold green]")
    console.print("=" * 60)
    
    # Find all JSON training files
    training_dir = Path("training")
    json_files = [
        "jessy_knowledge_training.json",
        "jessy_turkish_qa_chat.json",
        "jessy_technical_qa.json",
        "jessy_philosophical_qa.json",
        "jessy_turkish_combined.json",
        "jessy_train.json",
        "jessy_turkish_conversational.json",
        "jessy_iteration_control.json",
        "jessy_personality_training.json",
        "jessy_val.json",
        "relationship_training.json",
    ]
    
    all_examples = []
    
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
    ) as progress:
        
        task = progress.add_task("Loading training files...", total=len(json_files))
        
        for json_file in json_files:
            filepath = training_dir / json_file
            if filepath.exists():
                data = load_json_file(filepath)
                
                # Normalize each example
                for ex in data:
                    normalized = normalize_example(ex, json_file)
                    if normalized and normalized['input'] and normalized['output']:
                        all_examples.append(normalized)
                
                console.print(f"✅ Loaded {len(data)} examples from {json_file}")
            
            progress.update(task, advance=1)
    
    console.print(f"\n[green]✅ Total examples loaded: {len(all_examples)}[/green]")
    
    # Shuffle
    random.shuffle(all_examples)
    
    # Split train/val (90/10)
    split_idx = int(len(all_examples) * 0.9)
    train_data = all_examples[:split_idx]
    val_data = all_examples[split_idx:]
    
    # Save as JSONL
    output_dir = training_dir / "datasets"
    output_dir.mkdir(exist_ok=True)
    
    train_file = output_dir / "jessy_combined_train.jsonl"
    val_file = output_dir / "jessy_combined_val.jsonl"
    
    # Write JSONL
    with open(train_file, 'w', encoding='utf-8') as f:
        for ex in train_data:
            f.write(json.dumps(ex, ensure_ascii=False) + '\n')
    
    with open(val_file, 'w', encoding='utf-8') as f:
        for ex in val_data:
            f.write(json.dumps(ex, ensure_ascii=False) + '\n')
    
    console.print("\n" + "=" * 60)
    console.print("[bold green]✅ Dataset Preparation Complete![/bold green]")
    console.print(f"\n📊 Statistics:")
    console.print(f"  Total examples: {len(all_examples)}")
    console.print(f"  Training: {len(train_data)}")
    console.print(f"  Validation: {len(val_data)}")
    console.print(f"\n📁 Output Files:")
    console.print(f"  Train: {train_file}")
    console.print(f"  Val: {val_file}")
    
    # Category breakdown
    categories = {}
    for ex in all_examples:
        cat = ex.get('category', 'unknown')
        categories[cat] = categories.get(cat, 0) + 1
    
    console.print(f"\n📚 Categories:")
    for cat, count in sorted(categories.items(), key=lambda x: x[1], reverse=True):
        console.print(f"  {cat}: {count}")
    
    console.print(f"\n[bold green]🎯 Ready for training![/bold green]")
    console.print(f"Next: python train_jessy_complete.py --train {train_file} --val {val_file}")

if __name__ == "__main__":
    main()
