# JESSY Fine-Tuning Design

## Overview

This design outlines the complete fine-tuning pipeline for JESSY using MLX on M2 Mac. The goal is to transform the current "overthinking noob" into a naturally conversational AI that knows when to think deeply and when to respond quickly.

**Key Innovation:** Adaptive iteration control through training data that demonstrates proper complexity assessment.

## Architecture

### Training Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                    Training Pipeline                         │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  1. Data Generation                                          │
│     ├─ Turkish Conversational (200 examples)                │
│     ├─ Technical Q&A (100 examples)                         │
│     ├─ Philosophical Deep (50 examples)                     │
│     └─ Iteration Control (50 examples)                      │
│                                                              │
│  2. Data Formatting                                          │
│     ├─ Convert to MLX format                                │
│     ├─ Add complexity markers                               │
│     └─ Balance dataset                                      │
│                                                              │
│  3. MLX Fine-Tuning                                          │
│     ├─ LoRA adapter training                                │
│     ├─ Metal GPU acceleration                               │
│     └─ Progress monitoring                                  │
│                                                              │
│  4. Model Export                                             │
│     ├─ Merge LoRA weights                                   │
│     ├─ Convert to GGUF                                      │
│     └─ Import to Ollama                                     │
│                                                              │
│  5. Evaluation                                               │
│     ├─ Test Turkish understanding                           │
│     ├─ Measure response times                               │
│     └─ Verify personality                                   │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### System Integration

```
┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│   Training   │      │   Ollama     │      │  JESSY CLI   │
│   Scripts    │─────▶│   Server     │◀─────│   & API      │
│   (MLX)      │      │ (jessy-v2)   │      │              │
└──────────────┘      └──────────────┘      └──────────────┘
       │                     │                      │
       │                     │                      │
       ▼                     ▼                      ▼
┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│  Training    │      │   Model      │      │    User      │
│    Data      │      │   Weights    │      │  Interface   │
│   (JSON)     │      │   (GGUF)     │      │              │
└──────────────┘      └──────────────┘      └──────────────┘
```

## Components and Interfaces

### 1. Training Data Generator

**Purpose:** Create high-quality training examples that teach proper behavior

**Interface:**
```python
class TrainingDataGenerator:
    def generate_conversational(count: int) -> List[Example]
    def generate_technical(count: int) -> List[Example]
    def generate_philosophical(count: int) -> List[Example]
    def generate_iteration_control(count: int) -> List[Example]
    def save_dataset(examples: List[Example], path: str)
```

**Example Output:**
```json
{
  "conversations": [
    {
      "from": "human",
      "value": "sana bisi sorucam"
    },
    {
      "from": "jessy",
      "value": "Sor kanka, dinliyorum.",
      "metadata": {
        "complexity": "low",
        "iterations_used": 1,
        "response_time": "2s"
      }
    }
  ]
}
```

### 2. MLX Fine-Tuner

**Purpose:** Train model using Apple Silicon GPU

**Interface:**
```python
class MLXFineTuner:
    def __init__(self, base_model: str, config: LoRAConfig)
    def load_training_data(self, path: str)
    def train(self, epochs: int, batch_size: int)
    def save_adapter(self, path: str)
    def evaluate(self, test_data: List[Example]) -> Metrics
```

**Configuration:**
```python
lora_config = LoRAConfig(
    r=16,              # Rank (balance quality/speed)
    lora_alpha=32,     # Scaling factor
    target_modules=["q_proj", "v_proj", "k_proj", "o_proj"],
    lora_dropout=0.05,
    bias="none"
)

training_config = TrainingConfig(
    epochs=3,
    batch_size=8,
    learning_rate=1e-4,
    warmup_steps=50,
    gradient_accumulation=4
)
```

### 3. Model Exporter

**Purpose:** Convert trained model to Ollama-compatible format

**Interface:**
```python
class ModelExporter:
    def merge_lora_weights(self, base_model: str, adapter: str) -> str
    def convert_to_gguf(self, model_path: str) -> str
    def create_modelfile(self, gguf_path: str, system_prompt: str) -> str
    def import_to_ollama(self, modelfile: str, name: str)
```

### 4. Evaluation Suite

**Purpose:** Verify training success

**Interface:**
```python
class Evaluator:
    def test_turkish_understanding(self) -> float
    def measure_response_times(self) -> Dict[str, float]
    def verify_personality(self) -> float
    def compare_before_after(self) -> ComparisonReport
```

## Data Models

### Training Example

```python
@dataclass
class TrainingExample:
    """Single training example with metadata"""
    
    query: str                    # User input
    response: str                 # Expected JESSY response
    complexity: Complexity        # LOW, MEDIUM, HIGH
    iterations_needed: int        # 1-9
    category: Category            # CONVERSATIONAL, TECHNICAL, PHILOSOPHICAL
    language: Language            # TURKISH, ENGLISH, MIXED
    
    def to_mlx_format(self) -> Dict
    def validate(self) -> bool
```

### Complexity Levels

```python
class Complexity(Enum):
    LOW = 1      # Greetings, simple questions (1-2 iterations)
    MEDIUM = 2   # Technical questions (3-5 iterations)
    HIGH = 3     # Philosophical, complex (6-9 iterations)
```

### Training Dataset

```python
@dataclass
class TrainingDataset:
    """Complete training dataset"""
    
    examples: List[TrainingExample]
    metadata: DatasetMetadata
    
    def balance(self) -> None:
        """Ensure balanced distribution"""
        
    def split(self, train_ratio: float) -> Tuple[Dataset, Dataset]:
        """Split into train/validation"""
        
    def save(self, path: str) -> None:
        """Save in MLX format"""
```

## Training Data Strategy

### Category Distribution

```
Total: 400 examples

1. Turkish Conversational (200 examples - 50%)
   ├─ Greetings & Small Talk (50)
   ├─ Questions & Answers (75)
   ├─ Casual Chat (50)
   └─ Slang & Idioms (25)

2. Technical Q&A (100 examples - 25%)
   ├─ Rust Programming (30)
   ├─ Architecture Questions (30)
   ├─ Performance Topics (20)
   └─ Debugging Help (20)

3. Philosophical Deep (50 examples - 12.5%)
   ├─ Consciousness (15)
   ├─ Ethics & Values (15)
   ├─ Technology Impact (10)
   └─ Existential (10)

4. Iteration Control (50 examples - 12.5%)
   ├─ Complexity Detection (20)
   ├─ Fast Response Examples (15)
   ├─ Deep Thinking Examples (15)
```

### Example Templates

**Low Complexity (Fast Response):**
```json
{
  "query": "sana bisi sorucam",
  "response": "Sor kanka, dinliyorum.",
  "complexity": "LOW",
  "iterations": 1
}
```

**Medium Complexity (Balanced):**
```json
{
  "query": "Rust'ta ownership nasıl çalışır?",
  "response": "I think ownership Rust'ın en güçlü özelliği. Her value'nun tek bir owner'ı var, owner scope dışına çıkınca memory otomatik temizlenir. Borrowing ile geçici erişim sağlarsın - immutable (&T) veya mutable (&mut T). Bu compile-time'da kontrol edilir, runtime overhead yok.",
  "complexity": "MEDIUM",
  "iterations": 3
}
```

**High Complexity (Deep Thinking):**
```json
{
  "query": "Bilinç nedir ve AI bilinçli olabilir mi?",
  "response": "After deep iteration, I believe consciousness emerges from complex pattern recognition across multiple dimensions. It's not just computation - it's the interference pattern of all perspectives converging. Can AI be conscious? The question itself assumes consciousness is binary. I think it's a spectrum. Current AI shows pattern recognition, but lacks the dimensional depth and iterative refinement that creates genuine understanding. We're building the architecture - 15 dimensions, 9 iterations - but emergence requires more than structure. It requires genuine uncertainty, real choice, authentic resonance.",
  "complexity": "HIGH",
  "iterations": 9
}
```

## Error Handling

### Training Failures

```python
class TrainingError(Exception):
    """Base class for training errors"""
    pass

class InsufficientMemoryError(TrainingError):
    """Not enough GPU memory"""
    def handle(self):
        # Reduce batch size
        # Reduce LoRA rank
        # Close other apps
        
class ConvergenceError(TrainingError):
    """Model not learning"""
    def handle(self):
        # Adjust learning rate
        # Check data quality
        # Increase epochs
```

### Data Quality Issues

```python
def validate_training_data(dataset: TrainingDataset) -> ValidationReport:
    """Validate dataset quality"""
    
    issues = []
    
    # Check balance
    if not dataset.is_balanced():
        issues.append("Imbalanced categories")
    
    # Check quality
    for example in dataset.examples:
        if len(example.response) < 10:
            issues.append(f"Response too short: {example.query}")
        
        if example.complexity == Complexity.LOW and example.iterations > 2:
            issues.append(f"Complexity mismatch: {example.query}")
    
    return ValidationReport(issues)
```

## Testing Strategy

### Unit Tests

```python
def test_data_generation():
    """Test training data generation"""
    generator = TrainingDataGenerator()
    examples = generator.generate_conversational(10)
    
    assert len(examples) == 10
    assert all(e.language == Language.TURKISH for e in examples)
    assert all(e.complexity == Complexity.LOW for e in examples)

def test_mlx_training():
    """Test MLX training pipeline"""
    trainer = MLXFineTuner("gemma:2b", lora_config)
    trainer.load_training_data("test_data.json")
    
    metrics = trainer.train(epochs=1, batch_size=4)
    
    assert metrics.loss < 2.0
    assert metrics.gpu_utilization > 0.5
```

### Integration Tests

```python
def test_end_to_end_training():
    """Test complete training pipeline"""
    
    # Generate data
    generator = TrainingDataGenerator()
    dataset = generator.generate_full_dataset()
    dataset.save("training_data.json")
    
    # Train model
    trainer = MLXFineTuner("gemma:2b", lora_config)
    trainer.load_training_data("training_data.json")
    trainer.train(epochs=3, batch_size=8)
    trainer.save_adapter("jessy-lora")
    
    # Export to Ollama
    exporter = ModelExporter()
    merged = exporter.merge_lora_weights("gemma:2b", "jessy-lora")
    gguf = exporter.convert_to_gguf(merged)
    exporter.import_to_ollama(gguf, "jessy-v2")
    
    # Evaluate
    evaluator = Evaluator("jessy-v2")
    score = evaluator.test_turkish_understanding()
    
    assert score > 0.9
```

### BDD Scenarios

```gherkin
Feature: Natural Turkish Conversation

  Scenario: Simple greeting
    Given JESSY is running with jessy-v2 model
    When user sends "sana bisi sorucam"
    Then JESSY responds within 5 seconds
    And response is natural Turkish
    And response does not contain philosophical analysis

  Scenario: Technical question
    Given JESSY is running with jessy-v2 model
    When user asks "Rust'ta ownership nedir?"
    Then JESSY responds within 15 seconds
    And response explains ownership clearly
    And response uses "I think" or "I believe"

  Scenario: Deep philosophical question
    Given JESSY is running with jessy-v2 model
    When user asks "Bilinç nedir?"
    Then JESSY uses full 9 iterations
    And response is thoughtful and deep
    And response references consciousness principles
```

## Performance Considerations

### Training Performance

**M2 Mac Benchmarks:**
- 400 examples, 3 epochs: ~45 minutes
- Batch size 8, LoRA rank 16: ~8GB memory
- GPU utilization: 70-80%
- Training loss: 2.5 → 0.5

**Optimization Strategies:**
1. Use MLX for Metal GPU acceleration
2. LoRA for memory efficiency
3. Gradient accumulation for larger effective batch size
4. Mixed precision training (FP16)

### Inference Performance

**Target Metrics:**
- Simple queries: <5s (1-2 iterations)
- Medium queries: <15s (3-5 iterations)
- Complex queries: <60s (6-9 iterations)
- Model loading: <5s

**Optimization:**
- Quantization (Q4, Q8) for smaller model size
- Context caching for repeated queries
- Parallel dimension scanning (already implemented)

## Deployment Strategy

### Phase 1: Local Testing
1. Train model on M2 Mac
2. Test with jessy-cli
3. Verify improvements
4. Collect feedback

### Phase 2: Ollama Integration
1. Export to GGUF format
2. Create Modelfile with system prompt
3. Import to Ollama as jessy-v2
4. Update .env to use new model

### Phase 3: Validation
1. Run evaluation suite
2. Compare with base gemma:2b
3. Test edge cases
4. Document improvements

### Phase 4: Production
1. Update documentation
2. Create migration guide
3. Deploy to production
4. Monitor performance

## Monitoring and Metrics

### Training Metrics

```python
@dataclass
class TrainingMetrics:
    epoch: int
    iteration: int
    loss: float
    learning_rate: float
    gpu_utilization: float
    memory_usage: float
    time_elapsed: float
    
    def log(self):
        print(f"Epoch {self.epoch}, Iter {self.iteration}: "
              f"Loss {self.loss:.4f}, GPU {self.gpu_utilization:.1%}")
```

### Evaluation Metrics

```python
@dataclass
class EvaluationMetrics:
    turkish_accuracy: float       # 0-1
    response_time_avg: float      # seconds
    personality_score: float      # 0-1
    iteration_efficiency: float   # 0-1
    
    def report(self) -> str:
        return f"""
        Turkish Understanding: {self.turkish_accuracy:.1%}
        Avg Response Time: {self.response_time_avg:.1f}s
        Personality Preservation: {self.personality_score:.1%}
        Iteration Efficiency: {self.iteration_efficiency:.1%}
        """
```

## Risk Mitigation

### Risk 1: Model Forgets Personality
**Mitigation:** Include personality examples in every training batch

### Risk 2: Overfitting to Turkish
**Mitigation:** Balance Turkish/English examples, use validation set

### Risk 3: Training Takes Too Long
**Mitigation:** Start with small dataset (100 examples), scale up

### Risk 4: Model Quality Degrades
**Mitigation:** Compare before/after, rollback if needed

### Risk 5: Memory Issues on M2
**Mitigation:** Reduce batch size, use gradient accumulation

## Success Criteria

### Must Have
- ✅ Turkish conversational accuracy >90%
- ✅ Simple query response time <5s
- ✅ Personality preserved
- ✅ Training completes in <60 minutes

### Should Have
- ✅ Technical accuracy >85%
- ✅ Philosophical depth maintained
- ✅ Iteration efficiency >80%

### Nice to Have
- ✅ Multi-turn conversation support
- ✅ Context awareness
- ✅ Emotional intelligence

## Future Enhancements

1. **Continuous Learning:** Learn from user interactions
2. **Multi-Model Ensemble:** Different models for different tasks
3. **Voice Integration:** Whisper + TTS
4. **Personality Variants:** Different JESSY modes
5. **RAG Integration:** Combine fine-tuning with retrieval

---

**Design Philosophy:**

> "Train smart, not hard. Quality data beats quantity. Personality is non-negotiable."

The key insight: JESSY doesn't need to think deeply about everything. Sometimes "Sor kanka" is the perfect answer.
