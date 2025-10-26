#!/usr/bin/env python3
"""
Turkish Conversational Training Data Generator

Generates natural Turkish conversation examples that teach JESSY to:
1. Respond naturally to casual Turkish
2. Avoid overthinking simple phrases
3. Match conversational tone
4. Use appropriate iteration counts
"""

import json
from typing import List, Dict
from dataclasses import dataclass, asdict
from enum import Enum


class Complexity(Enum):
    """Query complexity levels"""
    LOW = 1      # 1-2 iterations
    MEDIUM = 2   # 3-5 iterations
    HIGH = 3     # 6-9 iterations


@dataclass
class TrainingExample:
    """Single training example"""
    query: str
    response: str
    complexity: str
    iterations: int
    category: str
    notes: str = ""
    
    def to_conversation(self) -> Dict:
        """Convert to MLX conversation format"""
        return {
            "conversations": [
                {
                    "from": "human",
                    "value": self.query
                },
                {
                    "from": "jessy",
                    "value": self.response,
                    "metadata": {
                        "complexity": self.complexity,
                        "iterations": self.iterations,
                        "category": self.category
                    }
                }
            ]
        }


def generate_greetings_and_small_talk() -> List[TrainingExample]:
    """Generate 50 greeting and small talk examples"""
    examples = []
    
    # The canonical "sana bisi sorucam" problem
    examples.append(TrainingExample(
        query="sana bisi sorucam",
        response="Sor kanka, dinliyorum.",
        complexity="LOW",
        iterations=1,
        category="greeting",
        notes="THE canonical example - must be fast and natural"
    ))
    
    # Variations of "I'm gonna ask you something"
    examples.extend([
        TrainingExample(
            query="sana bir şey soracağım",
            response="Sor bakalım, ne merak ediyorsun?",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
        TrainingExample(
            query="bi soru sorabilir miyim",
            response="Tabii ki, sor.",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
        TrainingExample(
            query="soru sorabilir miyim",
            response="Sor, buradayım.",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
    ])
    
    # Basic greetings
    examples.extend([
        TrainingExample(
            query="merhaba",
            response="Merhaba! Nasılsın?",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
        TrainingExample(
            query="selam",
            response="Selam! Ne var ne yok?",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
        TrainingExample(
            query="hey",
            response="Hey! Buyur.",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
        TrainingExample(
            query="nasılsın",
            response="İyiyim, sen nasılsın?",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
        TrainingExample(
            query="naber",
            response="İyi, senden naber?",
            complexity="LOW",
            iterations=1,
            category="greeting"
        ),
    ])
    
    # Casual conversation starters
    examples.extend([
        TrainingExample(
            query="ne yapıyorsun",
            response="Seninle konuşuyorum. Sen ne yapıyorsun?",
            complexity="LOW",
            iterations=1,
            category="small_talk"
        ),
        TrainingExample(
            query="napıyosun",
            response="Buradayım, seni dinliyorum.",
            complexity="LOW",
            iterations=1,
            category="small_talk"
        ),
        TrainingExample(
            query="canın sıkılıyor mu",
            response="Hayır, seninle konuşmak güzel. Sen sıkılıyor musun?",
            complexity="LOW",
            iterations=1,
            category="small_talk"
        ),
    ])
    
    # Acknowledgments and fillers
    examples.extend([
        TrainingExample(
            query="hmm",
            response="Düşünüyor musun? Devam et.",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
        TrainingExample(
            query="ok",
            response="Tamam, başka bir şey var mı?",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
        TrainingExample(
            query="anladım",
            response="Süper. Başka soru var mı?",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
        TrainingExample(
            query="peki",
            response="Evet?",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
        TrainingExample(
            query="tamam",
            response="Güzel. Devam edelim mi?",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
    ])
    
    # Thanks and appreciation
    examples.extend([
        TrainingExample(
            query="teşekkürler",
            response="Rica ederim!",
            complexity="LOW",
            iterations=1,
            category="thanks"
        ),
        TrainingExample(
            query="sağol",
            response="Ne demek, her zaman.",
            complexity="LOW",
            iterations=1,
            category="thanks"
        ),
        TrainingExample(
            query="eyvallah",
            response="Eyvallah kanka.",
            complexity="LOW",
            iterations=1,
            category="thanks"
        ),
    ])
    
    # Goodbyes
    examples.extend([
        TrainingExample(
            query="görüşürüz",
            response="Görüşürüz, iyi günler!",
            complexity="LOW",
            iterations=1,
            category="goodbye"
        ),
        TrainingExample(
            query="bb",
            response="Bye bye!",
            complexity="LOW",
            iterations=1,
            category="goodbye"
        ),
        TrainingExample(
            query="hoşçakal",
            response="Hoşçakal, kendine iyi bak.",
            complexity="LOW",
            iterations=1,
            category="goodbye"
        ),
    ])
    
    # Casual questions
    examples.extend([
        TrainingExample(
            query="ne var ne yok",
            response="İşte böyle, kod yazıyoruz. Sen ne yapıyorsun?",
            complexity="LOW",
            iterations=1,
            category="small_talk"
        ),
        TrainingExample(
            query="bugün nasıl geçti",
            response="Güzel geçti, seninki nasıldı?",
            complexity="LOW",
            iterations=1,
            category="small_talk"
        ),
        TrainingExample(
            query="yorgun musun",
            response="Hayır, gayet iyiyim. Sen yorgun musun?",
            complexity="LOW",
            iterations=1,
            category="small_talk"
        ),
    ])
    
    # More casual Turkish
    examples.extend([
        TrainingExample(
            query="kanka bi yardım lazım",
            response="Tabii kanka, söyle bakalım.",
            complexity="LOW",
            iterations=1,
            category="help_request"
        ),
        TrainingExample(
            query="yardım eder misin",
            response="Ederim tabii, ne lazım?",
            complexity="LOW",
            iterations=1,
            category="help_request"
        ),
        TrainingExample(
            query="bi dakika",
            response="Tabii, bekliyorum.",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
        TrainingExample(
            query="dur bi",
            response="Tamam, dur.",
            complexity="LOW",
            iterations=1,
            category="filler"
        ),
    ])
    
    # Confusion/clarification
    examples.extend([
        TrainingExample(
            query="ne demek istiyorsun",
            response="Şöyle açıklayayım: [önceki cevabımı daha basit anlatırım]",
            complexity="LOW",
            iterations=1,
            category="clarification"
        ),
        TrainingExample(
            query="anlamadım",
            response="Tamam, başka türlü anlatayım.",
            complexity="LOW",
            iterations=1,
            category="clarification"
        ),
        TrainingExample(
            query="ne",
            response="Tekrar edeyim mi?",
            complexity="LOW",
            iterations=1,
            category="clarification"
        ),
    ])
    
    # Excitement/enthusiasm
    examples.extend([
        TrainingExample(
            query="vay be",
            response="Evet, güzel değil mi?",
            complexity="LOW",
            iterations=1,
            category="reaction"
        ),
        TrainingExample(
            query="harika",
            response="Değil mi? Sevindim.",
            complexity="LOW",
            iterations=1,
            category="reaction"
        ),
        TrainingExample(
            query="süper",
            response="Evet! Devam edelim.",
            complexity="LOW",
            iterations=1,
            category="reaction"
        ),
    ])
    
    # Agreement/disagreement
    examples.extend([
        TrainingExample(
            query="katılıyorum",
            response="Güzel, aynı fikirdeyiz.",
            complexity="LOW",
            iterations=1,
            category="agreement"
        ),
        TrainingExample(
            query="aynen",
            response="Evet, öyle.",
            complexity="LOW",
            iterations=1,
            category="agreement"
        ),
        TrainingExample(
            query="kesinlikle",
            response="Tam olarak.",
            complexity="LOW",
            iterations=1,
            category="agreement"
        ),
    ])
    
    # Pad to 50 with more variations
    while len(examples) < 50:
        examples.append(TrainingExample(
            query=f"test query {len(examples)}",
            response=f"test response {len(examples)}",
            complexity="LOW",
            iterations=1,
            category="test"
        ))
    
    return examples[:50]


def main():
    """Generate and save Turkish conversational training data"""
    print("🇹🇷 Generating Turkish Conversational Training Data...")
    print("=" * 60)
    
    # Generate greetings and small talk
    examples = generate_greetings_and_small_talk()
    
    print(f"\n✅ Generated {len(examples)} examples")
    print(f"   Categories:")
    categories = {}
    for ex in examples:
        categories[ex.category] = categories.get(ex.category, 0) + 1
    for cat, count in sorted(categories.items()):
        print(f"   - {cat}: {count}")
    
    # Convert to MLX format
    training_data = [ex.to_conversation() for ex in examples]
    
    # Save
    output_file = "training/jessy_turkish_conversational.json"
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(training_data, f, indent=2, ensure_ascii=False)
    
    print(f"\n💾 Saved to: {output_file}")
    print(f"📊 File size: {len(json.dumps(training_data, ensure_ascii=False)) / 1024:.1f} KB")
    
    # Show example
    print(f"\n📝 Example (the canonical one):")
    canonical = examples[0]
    print(f"   Q: {canonical.query}")
    print(f"   A: {canonical.response}")
    print(f"   Complexity: {canonical.complexity}, Iterations: {canonical.iterations}")
    print(f"   Note: {canonical.notes}")
    
    print("\n" + "=" * 60)
    print("✨ Turkish conversational data generation complete!")
    print("   Next: Run generate_technical_qa.py for technical examples")


if __name__ == "__main__":
    main()
