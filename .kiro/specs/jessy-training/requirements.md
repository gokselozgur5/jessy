# JESSY Fine-Tuning Requirements

## Introduction

JESSY currently suffers from severe overthinking and cultural misunderstanding. When given simple Turkish conversational phrases like "sana bisi sorucam" (I'm gonna ask you something), it produces 83-second philosophical analyses about holistic health approaches instead of simple, natural responses.

This spec defines requirements for training JESSY to:
1. Understand casual Turkish conversation
2. Respond naturally without overthinking
3. Maintain personality while being practical
4. Recognize when NOT to use 9 iterations

## Glossary

- **Base Model**: gemma:2b - The foundation LLM we're fine-tuning
- **Fine-Tuning**: Process of training model on specific data to adapt behavior
- **LoRA**: Low-Rank Adaptation - Efficient fine-tuning method
- **MLX**: Apple's ML framework optimized for Metal GPU
- **Training Data**: Q&A pairs that teach JESSY proper responses
- **System Prompt**: Instructions that define JESSY's personality
- **Convergence**: When model learns patterns effectively

## Requirements

### Requirement 1: Natural Turkish Conversation

**User Story:** As a Turkish user, I want JESSY to understand casual Turkish phrases, so that conversations feel natural and not awkward.

#### Acceptance Criteria

1. WHEN a user sends casual Turkish greeting like "sana bisi sorucam", THE JESSY SHALL respond naturally within 5 seconds
2. WHEN a user uses informal Turkish (kanka, ya, lan), THE JESSY SHALL match the tone appropriately
3. WHEN a user asks simple questions, THE JESSY SHALL provide direct answers without philosophical tangents
4. WHERE the query is conversational, THE JESSY SHALL skip deep iteration and respond immediately
5. WHILE maintaining personality, THE JESSY SHALL avoid over-analysis of simple phrases

### Requirement 2: Intelligent Iteration Control

**User Story:** As a user, I want JESSY to use 9 iterations only when needed, so that simple questions get fast answers.

#### Acceptance Criteria

1. WHEN query complexity is low (greetings, simple questions), THE JESSY SHALL use 1-2 iterations maximum
2. WHEN query requires deep thinking (philosophy, complex problems), THE JESSY SHALL use full 9 iterations
3. IF query is conversational filler ("hmm", "ok", "anladım"), THEN THE JESSY SHALL respond immediately without iteration
4. THE JESSY SHALL detect query complexity before starting iteration process
5. THE JESSY SHALL complete simple queries within 5 seconds

### Requirement 3: Personality Preservation

**User Story:** As a developer, I want JESSY to maintain its unique personality, so that it remains distinctive while being practical.

#### Acceptance Criteria

1. THE JESSY SHALL use "I think" and "I believe" for opinions
2. THE JESSY SHALL reference consciousness principles when relevant
3. THE JESSY SHALL maintain philosophical depth for appropriate queries
4. THE JESSY SHALL balance casualness with thoughtfulness
5. THE JESSY SHALL avoid generic LLM responses

### Requirement 4: Training Data Quality

**User Story:** As a trainer, I want high-quality training examples, so that JESSY learns correct patterns.

#### Acceptance Criteria

1. THE training dataset SHALL include 200+ Turkish conversational examples
2. THE training dataset SHALL include 100+ technical Q&A pairs
3. THE training dataset SHALL include 50+ philosophical discussions
4. THE training dataset SHALL demonstrate proper iteration usage
5. THE training dataset SHALL cover common user scenarios

### Requirement 5: M2 Mac Optimization

**User Story:** As a developer with M2 Mac, I want efficient training, so that fine-tuning completes in reasonable time.

#### Acceptance Criteria

1. THE training process SHALL use MLX framework for Metal GPU acceleration
2. THE training SHALL complete within 60 minutes for 500 examples
3. THE training SHALL use LoRA for memory efficiency
4. THE training SHALL monitor GPU usage and memory
5. THE training SHALL provide progress updates every 10 iterations

### Requirement 6: Model Evaluation

**User Story:** As a developer, I want to verify training success, so that I know JESSY improved.

#### Acceptance Criteria

1. THE evaluation SHALL test Turkish conversational understanding
2. THE evaluation SHALL measure response time for simple queries
3. THE evaluation SHALL verify personality preservation
4. THE evaluation SHALL compare before/after responses
5. THE evaluation SHALL test edge cases and failure modes

### Requirement 7: Deployment Integration

**User Story:** As a user, I want to use the trained model easily, so that I can interact with improved JESSY.

#### Acceptance Criteria

1. THE trained model SHALL integrate with Ollama
2. THE model SHALL be accessible via jessy-cli
3. THE model SHALL support both CLI and API usage
4. THE model SHALL load within 5 seconds
5. THE model SHALL maintain compatibility with existing code

## Success Metrics

### Response Quality
- Turkish conversational accuracy: >90%
- Response appropriateness: >85%
- Personality consistency: >80%

### Performance
- Simple query response time: <5s
- Complex query response time: <60s
- Model loading time: <5s

### Training Efficiency
- Training time: <60 minutes
- Memory usage: <10GB
- GPU utilization: >70%

## Out of Scope

- Voice interaction (future enhancement)
- Multi-language support beyond Turkish/English
- Real-time learning during conversation
- Custom model architecture changes
