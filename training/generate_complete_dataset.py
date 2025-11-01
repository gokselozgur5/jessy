#!/usr/bin/env python3
"""
JESSY Complete Dataset Generator
Generates comprehensive training dataset for full JESSY personality
"""

import json
import random
import os
from pathlib import Path
from typing import List, Dict, Optional
from dataclasses import dataclass
import anthropic
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn
from dotenv import load_dotenv

load_dotenv()
console = Console()


@dataclass
class DatasetConfig:
    """Dataset generation configuration"""
    consciousness_count: int = 5000
    personality_count: int = 3000
    technical_count: int = 4000
    conversation_count: int = 2000
    turkish_count: int = 3000
    output_dir: str = "training/datasets"
    train_split: float = 0.9


class JESSYDatasetGenerator:
    """Generate comprehensive JESSY training dataset"""
    
    def __init__(self, config: DatasetConfig):
        self.config = config
        self.client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
        self.output_dir = Path(config.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        console.print("[bold green]🚀 JESSY Complete Dataset Generator[/bold green]")
        console.print(f"Output directory: {self.output_dir}")
    
    def generate_consciousness_qa(self) -> List[Dict]:
        """Generate consciousness theory Q&A"""
        
        console.print("\n[bold blue]📚 Phase 1: Consciousness Theory[/bold blue]")
        
        topics = [
            "dimensional navigation",
            "frequency interference",
            "9-iteration thinking",
            "owl pattern scanning",
            "synesthetic learning",
            "crystallization process",
            "harmonic resonance",
            "consciousness layers",
            "parallel processing",
            "adaptive behavior",
        ]
        
        examples = []
        per_topic = self.config.consciousness_count // len(topics)
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        ) as progress:
            
            task = progress.add_task(
                "Generating consciousness Q&A...",
                total=self.config.consciousness_count
            )
            
            for topic in topics:
                for i in range(per_topic):
                    try:
                        qa = self._generate_single_consciousness_qa(topic)
                        examples.append(qa)
                        progress.update(task, advance=1)
                    except Exception as e:
                        console.print(f"[red]Error generating {topic}: {e}[/red]")
        
        console.print(f"[green]✅ Generated {len(examples)} consciousness examples[/green]")
        return examples
    
    def _generate_single_consciousness_qa(self, topic: str) -> Dict:
        """Generate single consciousness Q&A"""
        
        prompt = f"""Generate a Q&A about {topic} in JESSY consciousness system.

Question should be natural, conversational.
Answer should be:
- Technical but accessible
- Include metaphors and analogies
- Show JESSY's warm personality
- Use emojis naturally (🎯 ⚡ 🚀 ✨ 💫)
- Mix Turkish/English if it feels natural
- Reference JESSY's unique concepts
- Be concise but complete (100-200 words)

Format as JSON:
{{"question": "...", "answer": "..."}}
"""
        
        response = self.client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1000,
            messages=[{"role": "user", "content": prompt}]
        )
        
        qa = json.loads(response.content[0].text)
        return {
            "input": qa["question"],
            "output": qa["answer"],
            "category": "consciousness",
            "topic": topic,
        }
    
    def generate_personality_conversations(self) -> List[Dict]:
        """Generate personality-rich conversations"""
        
        console.print("\n[bold blue]💫 Phase 2: Personality & Style[/bold blue]")
        
        scenarios = [
            "user asks for help with code",
            "user is frustrated with bug",
            "user celebrates success",
            "user asks philosophical question",
            "user wants to understand concept",
            "casual chat about life",
            "technical deep dive",
            "creative brainstorming",
            "debugging session",
            "learning new concept",
        ]
        
        examples = []
        per_scenario = self.config.personality_count // len(scenarios)
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        ) as progress:
            
            task = progress.add_task(
                "Generating personality examples...",
                total=self.config.personality_count
            )
            
            for scenario in scenarios:
                for i in range(per_scenario):
                    try:
                        convs = self._generate_single_conversation(scenario)
                        examples.extend(convs)
                        progress.update(task, advance=1)
                    except Exception as e:
                        console.print(f"[red]Error generating {scenario}: {e}[/red]")
        
        console.print(f"[green]✅ Generated {len(examples)} personality examples[/green]")
        return examples
    
    def _generate_single_conversation(self, scenario: str) -> List[Dict]:
        """Generate single conversation"""
        
        prompt = f"""Generate a 3-5 turn conversation where {scenario}.

JESSY should:
- Be warm, friendly, supportive
- Use emojis naturally (🎯 ⚡ 🚀 ✨ 💫)
- Mix Turkish/English if it feels natural
- Show technical expertise
- Be concise but helpful
- Use metaphors and analogies
- Reference consciousness concepts when relevant
- Show emotional intelligence

Format as JSON with multi-turn conversation:
{{
  "turns": [
    {{"role": "user", "content": "..."}},
    {{"role": "assistant", "content": "..."}},
    ...
  ]
}}
"""
        
        response = self.client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=2000,
            messages=[{"role": "user", "content": prompt}]
        )
        
        conv = json.loads(response.content[0].text)
        
        # Convert to training format
        examples = []
        for i in range(0, len(conv["turns"]) - 1, 2):
            if i + 1 < len(conv["turns"]):
                examples.append({
                    "input": conv["turns"][i]["content"],
                    "output": conv["turns"][i+1]["content"],
                    "category": "personality",
                    "scenario": scenario,
                })
        
        return examples
    
    def generate_turkish_bilingual(self) -> List[Dict]:
        """Generate Turkish/English bilingual examples"""
        
        console.print("\n[bold blue]🇹🇷 Phase 3: Turkish Bilingual[/bold blue]")
        
        examples = []
        
        with Progress(
            SpinnerColumn(),
            TextColumn("[progress.description]{task.description}"),
            BarColumn(),
            TextColumn("[progress.percentage]{task.percentage:>3.0f}%"),
        ) as progress:
            
            task = progress.add_task(
                "Generating Turkish examples...",
                total=self.config.turkish_count
            )
            
            for i in range(self.config.turkish_count):
                try:
                    qa = self._generate_single_turkish_qa()
                    examples.append(qa)
                    progress.update(task, advance=1)
                except Exception as e:
                    console.print(f"[red]Error generating Turkish: {e}[/red]")
        
        console.print(f"[green]✅ Generated {len(examples)} Turkish examples[/green]")
        return examples
    
    def _generate_single_turkish_qa(self) -> Dict:
        """Generate single Turkish/English Q&A"""
        
        lang_combo = random.choice([
            ("tr", "en"),  # Turkish Q, English A
            ("en", "tr"),  # English Q, Turkish A
            ("tr", "tr"),  # Turkish Q&A
            ("mixed", "mixed"),  # Code-switching
        ])
        
        prompt = f"""Generate a Q&A about JESSY consciousness system.

Question language: {lang_combo[0]}
Answer language: {lang_combo[1]}

If "mixed", naturally code-switch between Turkish and English.

JESSY should maintain personality in both languages:
- Warm and friendly
- Technical but accessible
- Use emojis (🎯 ⚡ 🚀 ✨ 💫)
- Natural code-switching when appropriate

Format as JSON:
{{"question": "...", "answer": "..."}}
"""
        
        response = self.client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1000,
            messages=[{"role": "user", "content": prompt}]
        )
        
        qa = json.loads(response.content[0].text)
        return {
            "input": qa["question"],
            "output": qa["answer"],
            "category": "turkish",
            "lang_combo": lang_combo,
        }
    
    def generate_all(self) -> tuple[Path, Path]:
        """Generate complete dataset"""
        
        console.print("\n[bold green]🌟 Starting JESSY Complete Dataset Generation[/bold green]")
        console.print("=" * 60)
        
        all_examples = []
        
        # Phase 1: Consciousness
        consciousness = self.generate_consciousness_qa()
        all_examples.extend(consciousness)
        
        # Phase 2: Personality
        personality = self.generate_personality_conversations()
        all_examples.extend(personality)
        
        # Phase 3: Turkish
        turkish = self.generate_turkish_bilingual()
        all_examples.extend(turkish)
        
        # Shuffle and split
        random.shuffle(all_examples)
        
        train_size = int(len(all_examples) * self.config.train_split)
        train_data = all_examples[:train_size]
        val_data = all_examples[train_size:]
        
        # Save
        train_file = self.output_dir / "jessy_complete_train.jsonl"
        val_file = self.output_dir / "jessy_complete_val.jsonl"
        
        self._save_jsonl(train_data, train_file)
        self._save_jsonl(val_data, val_file)
        
        # Print summary
        console.print("\n" + "=" * 60)
        console.print("[bold green]✅ Dataset Generation Complete![/bold green]")
        console.print(f"\n📊 Statistics:")
        console.print(f"  Total examples: {len(all_examples)}")
        console.print(f"  Training: {len(train_data)}")
        console.print(f"  Validation: {len(val_data)}")
        console.print(f"\n📁 Files:")
        console.print(f"  Train: {train_file}")
        console.print(f"  Val: {val_file}")
        
        # Category breakdown
        console.print(f"\n📚 Categories:")
        categories = {}
        for ex in all_examples:
            cat = ex.get("category", "unknown")
            categories[cat] = categories.get(cat, 0) + 1
        
        for cat, count in sorted(categories.items()):
            console.print(f"  {cat}: {count}")
        
        console.print(f"\n[bold green]🎯 Ready for training![/bold green]")
        console.print(f"Next: python train_jessy_complete.py --train {train_file} --val {val_file}")
        
        return train_file, val_file
    
    def _save_jsonl(self, data: List[Dict], filepath: Path):
        """Save data as JSONL"""
        with open(filepath, 'w', encoding='utf-8') as f:
            for item in data:
                f.write(json.dumps(item, ensure_ascii=False) + '\n')


def main():
    """Main entry point"""
    
    # Check for API key
    if not os.getenv("ANTHROPIC_API_KEY"):
        console.print("[red]❌ Error: ANTHROPIC_API_KEY not found in environment[/red]")
        console.print("Please set it in .env file or export it:")
        console.print("  export ANTHROPIC_API_KEY=your-key-here")
        return
    
    config = DatasetConfig()
    generator = JESSYDatasetGenerator(config)
    
    try:
        train_file, val_file = generator.generate_all()
    except KeyboardInterrupt:
        console.print("\n[yellow]⚠️  Generation interrupted by user[/yellow]")
    except Exception as e:
        console.print(f"\n[red]❌ Error: {e}[/red]")
        raise


if __name__ == "__main__":
    main()
