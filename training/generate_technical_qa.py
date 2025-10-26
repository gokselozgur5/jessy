#!/usr/bin/env python3
"""
Technical Q&A Training Data Generator

Generates 100 technical examples covering:
- Rust programming (30 examples)
- Architecture and design (30 examples)
- Performance and optimization (20 examples)
- Debugging and troubleshooting (20 examples)
"""

import json
from typing import List, Dict
from dataclasses import dataclass


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


def generate_rust_programming() -> List[TrainingExample]:
    """Generate 30 Rust programming examples"""
    examples = []
    
    # Ownership basics (10 examples)
    examples.extend([
        TrainingExample(
            query="Rust'ta ownership nedir",
            response="I think ownership Rust'ın en güçlü özelliği. Her value'nun tek bir owner'ı var. Owner scope dışına çıkınca memory otomatik temizlenir. Bu compile-time'da kontrol edilir, runtime overhead yok.",
            complexity="MEDIUM",
            iterations=3,
            category="rust_ownership"
        ),
        TrainingExample(
            query="Borrowing nasıl çalışır",
            response="Borrowing geçici erişim sağlar. İki tür var: immutable (&T) - birden fazla olabilir, mutable (&mut T) - tek bir tane. Compiler borrow checker ile data race'leri compile-time'da önler.",
            complexity="MEDIUM",
            iterations=4,
            category="rust_ownership"
        ),
        TrainingExample(
            query="Move semantics ne demek",
            response="Move ownership'i transfer eder. Value yeni owner'a geçer, eski owner artık kullanamaz. Copy trait yoksa default behavior move'dur. Bu memory safety garantisi verir.",
            complexity="MEDIUM",
            iterations=3,
            category="rust_ownership"
        ),
