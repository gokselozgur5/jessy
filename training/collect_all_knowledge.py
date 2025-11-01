#!/usr/bin/env python3
"""
Collect ALL JESSY knowledge for maximum training
Push Gemma 2B to its limits!
"""

import json
import os
from pathlib import Path
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn

console = Console()


def collect_existing_training_data():
    """Collect existing JSON training files"""
    
    training_files = [
        "training/jessy_knowledge_training.json",
        "training/jessy_turkish_qa_chat.json",
        "training/jessy_technical_qa.json",
        "training/jessy_philosophical_qa.json",
        "training/jessy_turkish_combined.json",
        "training/jessy_train.json",
        "training/jessy_turkish_conversational.json",
        "training/jessy_iteration_control.json",
        "training/jessy_personality_training.json",
        "training/jessy_val.json",
        "training/relationship_training.json",
        "training/knowledge_base/sonnet4545_training.json",
    ]
    
    all_data = []
    
    for filepath in training_files:
        if Path(filepath).exists():
            try:
                with open(filepath, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    if isinstance(data, list):
                        all_data.extend(data)
                    else:
                        all_data.append(data)
                console.print(f"✅ Loaded {filepath}")
            except Exception as e:
                console.print(f"[yellow]⚠️  Skipped {filepath}: {e}[/yellow]")
    
    return all_data


def extract_from_markdown(filepath):
    """Extract knowledge from markdown files"""
    
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Extract sections
        examples = []
        
        # Look for Q&A patterns
        lines = content.split('\n')
        current_q = None
        current_a = []
        
        for line in lines:
            # Question patterns
            if line.startswith('##') or line.startswith('###'):
                if current_q and current_a:
                    examples.append({
                        'input': current_q,
                        'output': '\n'.join(current_a).strip(),
                        'source': filepath,
                        'category': 'documentation',
                    })
                current_q = line.strip('#').strip()
                current_a = []
            elif current_q and line.strip():
                current_a.append(line)
        
        # Last one
        if current_q and current_a:
            examples.append({
                'input': current_q,
                'output': '\n'.join(current_a).strip(),
                'source': filepath,
                'category': 'documentation',
            })
        
        return examples
    
    except Exception as e:
        console.print(f"[yellow]⚠️  Error reading {filepath}: {e}[/yellow]")
        return []


def collect_steering_files():
    """Collect steering principles"""
    
    steering_dir = Path(".kiro/steering")
    examples = []
    
    if steering_dir.exists():
        for md_file in steering_dir.glob("*.md"):
            file_examples = extract_from_markdown(str(md_file))
            examples.extend(file_examples)
            console.print(f"✅ Extracted {len(file_examples)} from {md_file.name}")
    
    return examples


def collect_specs():
    """Collect specs and requirements"""
    
    specs_dirs = [
        ".kiro/specs",
        "docs",
    ]
    
    examples = []
    
    for specs_dir in specs_dirs:
        specs_path = Path(specs_dir)
        if specs_path.exists():
            for md_file in specs_path.rglob("*.md"):
                file_examples = extract_from_markdown(str(md_file))
                examples.extend(file_examples)
                if file_examples:
                    console.print(f"✅ Extracted {len(file_examples)} from {md_file}")
    
    return examples


def collect_sessions():
    """Collect session files (our conversations)"""
    
    session_files = list(Path(".").glob("SESSION_*.md")) + list(Path(".").glob("TASK_*.md"))
    
    examples = []
    
    for session_file in session_files:
        file_examples = extract_from_markdown(str(session_file))
        examples.extend(file_examples)
        if file_examples:
            console.print(f"✅ Extracted {len(file_examples)} from {session_file.name}")
    
    return examples


def normalize_example(ex):
    """Normalize to standard format"""
    
    # Already normalized
    if 'input' in ex and 'output' in ex:
        return ex
    
    # Conversations format
    if 'conversations' in ex:
        convs = ex['conversations']
        if len(convs) >= 2:
            user_msg = next((c['value'] for c in convs if c.get('from') in ['human', 'user']), None)
            assistant_msg = next((c['value'] for c in convs if c.get('from') in ['jessy', 'assistant', 'gpt']), None)
            if user_msg and assistant_msg:
                return {
                    'input': user_msg,
                    'output': assistant_msg,
                    'source': ex.get('source', 'unknown'),
                    'category': 'conversation',
                }
    
    return None


def main():
    console.print("[bold green]🚀 COLLECTING ALL JESSY KNOWLEDGE[/bold green]")
    console.print("[bold yellow]Goal: Push Gemma 2B to its LIMITS![/bold yellow]")
    console.print("=" * 60)
    
    all_examples = []
    
    # 1. Existing training data
    console.print("\n[blue]📚 Phase 1: Existing Training Data[/blue]")
    existing = collect_existing_training_data()
    console.print(f"✅ Collected {len(existing)} existing examples")
    
    # Normalize
    for ex in existing:
        normalized = normalize_example(ex)
        if normalized and normalized['input'] and normalized['output']:
            all_examples.append(normalized)
    
    console.print(f"✅ Normalized to {len(all_examples)} examples")
    
    # 2. Steering files
    console.print("\n[blue]🧭 Phase 2: Steering Principles[/blue]")
    steering = collect_steering_files()
    all_examples.extend(steering)
    console.print(f"✅ Total: {len(all_examples)} examples")
    
    # 3. Specs
    console.print("\n[blue]📋 Phase 3: Specs & Requirements[/blue]")
    specs = collect_specs()
    all_examples.extend(specs)
    console.print(f"✅ Total: {len(all_examples)} examples")
    
    # 4. Sessions
    console.print("\n[blue]💬 Phase 4: Our Conversations[/blue]")
    sessions = collect_sessions()
    all_examples.extend(sessions)
    console.print(f"✅ Total: {len(all_examples)} examples")
    
    # Filter out empty or too short
    all_examples = [
        ex for ex in all_examples
        if ex.get('input') and ex.get('output')
        and len(ex['input']) > 10 and len(ex['output']) > 20
    ]
    
    console.print(f"\n[green]✅ Filtered to {len(all_examples)} quality examples[/green]")
    
    # Shuffle
    import random
    random.shuffle(all_examples)
    
    # Split 90/10
    split_idx = int(len(all_examples) * 0.9)
    train_data = all_examples[:split_idx]
    val_data = all_examples[split_idx:]
    
    # Save
    output_dir = Path("training/datasets")
    output_dir.mkdir(exist_ok=True)
    
    train_file = output_dir / "jessy_maximum_train.jsonl"
    val_file = output_dir / "jessy_maximum_val.jsonl"
    
    with open(train_file, 'w', encoding='utf-8') as f:
        for ex in train_data:
            f.write(json.dumps(ex, ensure_ascii=False) + '\n')
    
    with open(val_file, 'w', encoding='utf-8') as f:
        for ex in val_data:
            f.write(json.dumps(ex, ensure_ascii=False) + '\n')
    
    # Stats
    console.print("\n" + "=" * 60)
    console.print("[bold green]🎉 MAXIMUM DATASET READY![/bold green]")
    console.print(f"\n📊 Statistics:")
    console.print(f"  Total examples: {len(all_examples)}")
    console.print(f"  Training: {len(train_data)}")
    console.print(f"  Validation: {len(val_data)}")
    
    # Category breakdown
    categories = {}
    for ex in all_examples:
        cat = ex.get('category', 'unknown')
        categories[cat] = categories.get(cat, 0) + 1
    
    console.print(f"\n📚 Categories:")
    for cat, count in sorted(categories.items(), key=lambda x: x[1], reverse=True):
        console.print(f"  {cat}: {count}")
    
    console.print(f"\n📁 Output:")
    console.print(f"  Train: {train_file}")
    console.print(f"  Val: {val_file}")
    
    console.print(f"\n[bold yellow]🔥 Ready to push Gemma 2B to the LIMIT![/bold yellow]")
    console.print(f"[bold cyan]Next: Train with ALL data and see what happens! 🚀[/bold cyan]")


if __name__ == "__main__":
    main()
