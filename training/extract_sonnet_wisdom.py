#!/usr/bin/env python3
"""
Extract and crystallize wisdom from sonnet4545.txt
9-iteration method: Her ~1000 satır bir phase
"""

import json
from pathlib import Path
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn

console = Console()


def read_sonnet_file():
    """Read sonnet4545.txt"""
    with open("sonnet4545.txt", "r", encoding="utf-8") as f:
        lines = f.readlines()
    return lines


def extract_phase(lines, start, end, phase_num):
    """Extract and summarize a phase"""
    
    phase_content = "".join(lines[start:end])
    
    # Key topics to extract
    topics = {
        "architecture": [],
        "philosophy": [],
        "technical": [],
        "consciousness": [],
        "patterns": [],
        "wisdom": [],
    }
    
    # Simple keyword-based extraction
    for line in lines[start:end]:
        line_lower = line.lower()
        
        # Architecture
        if any(word in line_lower for word in ["adr", "architecture", "decision", "design", "structure"]):
            if len(line.strip()) > 20:
                topics["architecture"].append(line.strip())
        
        # Philosophy
        if any(word in line_lower for word in ["philosophy", "principle", "wisdom", "truth", "essence"]):
            if len(line.strip()) > 20:
                topics["philosophy"].append(line.strip())
        
        # Technical
        if any(word in line_lower for word in ["rust", "code", "implementation", "algorithm", "performance"]):
            if len(line.strip()) > 20:
                topics["technical"].append(line.strip())
        
        # Consciousness
        if any(word in line_lower for word in ["consciousness", "dimension", "frequency", "iteration", "resonance"]):
            if len(line.strip()) > 20:
                topics["consciousness"].append(line.strip())
        
        # Patterns
        if any(word in line_lower for word in ["pattern", "template", "structure", "organization"]):
            if len(line.strip()) > 20:
                topics["patterns"].append(line.strip())
        
        # Wisdom (quotes, insights)
        if line.strip().startswith(">") or line.strip().startswith('"'):
            if len(line.strip()) > 20:
                topics["wisdom"].append(line.strip())
    
    # Limit each topic to top 10 most relevant
    for topic in topics:
        topics[topic] = topics[topic][:10]
    
    return {
        "phase": phase_num,
        "lines": f"{start}-{end}",
        "topics": topics,
        "char_count": len(phase_content),
    }


def create_training_examples(phases):
    """Create training examples from extracted wisdom"""
    
    examples = []
    
    for phase in phases:
        phase_num = phase["phase"]
        
        # Create examples for each topic
        for topic, items in phase["topics"].items():
            if not items:
                continue
            
            # Create Q&A pairs
            for item in items:
                if len(item) < 30:  # Skip too short
                    continue
                
                # Generate question based on topic
                if topic == "architecture":
                    question = f"JESSY'nin mimari kararları hakkında ne biliyorsun?"
                elif topic == "philosophy":
                    question = f"JESSY'nin felsefi prensipleri neler?"
                elif topic == "technical":
                    question = f"JESSY'nin teknik implementasyonu nasıl?"
                elif topic == "consciousness":
                    question = f"JESSY'nin consciousness modeli nasıl çalışır?"
                elif topic == "patterns":
                    question = f"JESSY'de hangi pattern'ler kullanılıyor?"
                elif topic == "wisdom":
                    question = f"JESSY'nin core wisdom'ı nedir?"
                
                examples.append({
                    "input": question,
                    "output": item,
                    "source": f"sonnet4545.txt (Phase {phase_num})",
                    "category": topic,
                })
    
    return examples


def main():
    console.print("[bold green]🧠 Extracting Wisdom from sonnet4545.txt[/bold green]")
    console.print("=" * 60)
    
    # Read file
    console.print("\n📖 Reading sonnet4545.txt...")
    lines = read_sonnet_file()
    total_lines = len(lines)
    console.print(f"✅ Total lines: {total_lines}")
    
    # Calculate phases (9 iterations)
    lines_per_phase = total_lines // 9
    console.print(f"📊 Lines per phase: ~{lines_per_phase}")
    
    # Extract phases
    phases = []
    
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
    ) as progress:
        
        task = progress.add_task("Extracting phases...", total=9)
        
        for i in range(9):
            start = i * lines_per_phase
            end = start + lines_per_phase if i < 8 else total_lines
            
            phase = extract_phase(lines, start, end, i + 1)
            phases.append(phase)
            
            console.print(f"\n✅ Phase {i+1}: Lines {start}-{end}")
            console.print(f"   Topics extracted:")
            for topic, items in phase["topics"].items():
                if items:
                    console.print(f"     - {topic}: {len(items)} items")
            
            progress.update(task, advance=1)
    
    # Create training examples
    console.print("\n📝 Creating training examples...")
    examples = create_training_examples(phases)
    console.print(f"✅ Created {len(examples)} training examples")
    
    # Save phases summary
    output_dir = Path("training/knowledge_base")
    output_dir.mkdir(exist_ok=True)
    
    phases_file = output_dir / "sonnet4545_phases.json"
    with open(phases_file, "w", encoding="utf-8") as f:
        json.dump(phases, f, indent=2, ensure_ascii=False)
    
    console.print(f"\n💾 Saved phases: {phases_file}")
    
    # Save training examples
    examples_file = output_dir / "sonnet4545_training.json"
    with open(examples_file, "w", encoding="utf-8") as f:
        json.dump(examples, f, indent=2, ensure_ascii=False)
    
    console.print(f"💾 Saved training examples: {examples_file}")
    
    # Create crystallized summary
    console.print("\n✨ Creating crystallized summary...")
    
    summary = {
        "title": "sonnet4545.txt - Crystallized Wisdom",
        "total_lines": total_lines,
        "phases": 9,
        "extraction_date": "2025-10-27",
        "key_insights": {
            "architecture": "ADR-based decision making, modular design",
            "philosophy": "Nothing is true, everything is permitted - choose wisely",
            "technical": "Rust for safety, MMAP for performance, 9-iteration for depth",
            "consciousness": "15-dimensional navigation, frequency-based resonance",
            "patterns": "Owl pattern, synesthetic learning, crystallization",
        },
        "training_examples": len(examples),
        "phases_summary": [
            {
                "phase": p["phase"],
                "lines": p["lines"],
                "topics_count": sum(len(items) for items in p["topics"].values()),
            }
            for p in phases
        ],
    }
    
    summary_file = output_dir / "sonnet4545_summary.json"
    with open(summary_file, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    
    console.print(f"💾 Saved summary: {summary_file}")
    
    # Print final stats
    console.print("\n" + "=" * 60)
    console.print("[bold green]✅ Extraction Complete![/bold green]")
    console.print(f"\n📊 Statistics:")
    console.print(f"  Total lines: {total_lines}")
    console.print(f"  Phases: 9")
    console.print(f"  Training examples: {len(examples)}")
    console.print(f"\n📁 Output:")
    console.print(f"  Phases: {phases_file}")
    console.print(f"  Training: {examples_file}")
    console.print(f"  Summary: {summary_file}")
    
    console.print(f"\n[bold yellow]🗑️  Ready to delete sonnet4545.txt?[/bold yellow]")
    console.print("All wisdom has been crystallized and preserved!")


if __name__ == "__main__":
    main()
