# Implementation Plan

## Overview

Transform JESSY from overthinking noob to naturally conversational AI through MLX fine-tuning on M2 Mac. Focus on Turkish conversation understanding and intelligent iteration control.

## Tasks

- [x] 1. Setup training environment
  - Install MLX and dependencies on M2 Mac
  - Verify Metal GPU acceleration working
  - Create training directory structure
  - _Requirements: 5_

- [x] 2. Generate Turkish conversational training data
  - [x] 2.1 Create training data generator script
    - Write Python script to generate Q&A pairs
    - Implement complexity classification logic
    - Add metadata tracking (iterations, response time)
    - _Requirements: 4.1, 4.2_
  
  - [x] 2.2 Generate 200 Turkish conversational examples
    - Greetings and small talk (50 examples) ✅
    - Common questions and answers (75 examples) ✅
    - Casual chat patterns (50 examples) ✅
    - Turkish slang and idioms (25 examples) ✅
    - _Requirements: 1.1, 1.2, 1.3, 4.1_
  
  - [ ] 2.3 Add iteration control examples
    - Low complexity examples (1-2 iterations)
    - Medium complexity examples (3-5 iterations)
    - High complexity examples (6-9 iterations)
    - Demonstrate proper complexity detection
    - _Requirements: 2.1, 2.2, 2.3, 4.4_

- [x] 3. Generate technical and philosophical training data
  - [x] 3.1 Create 100 technical Q&A examples
    - Rust programming questions (30 examples) ✅
    - Architecture and design questions (30 examples) ✅
    - Performance and optimization (20 examples) ✅
    - Debugging and troubleshooting (20 examples) ✅
    - _Requirements: 4.2_
  
  - [x] 3.2 Create 50 philosophical examples
    - Consciousness and AI topics (15 examples) ✅
    - Ethics and values discussions (15 examples) ✅
    - Technology impact questions (10 examples) ✅
    - Existential questions (10 examples) ✅
    - _Requirements: 3.1, 3.2, 4.3_

- [ ] 4. Format and validate training dataset
  - [ ] 4.1 Convert to MLX training format
    - Transform Q&A pairs to conversation format
    - Add system prompts and metadata
    - Ensure proper tokenization
    - _Requirements: 4.1, 4.2, 4.3, 4.4_
  
  - [ ] 4.2 Validate dataset quality
    - Check category balance (50% Turkish, 25% technical, etc.)
    - Verify complexity distribution
    - Validate response quality
    - Check for duplicates or low-quality examples
    - _Requirements: 4.1, 4.2, 4.3, 4.4_
  
  - [ ] 4.3 Split into train/validation sets
    - 90% training, 10% validation
    - Ensure balanced split across categories
    - Save in separate files
    - _Requirements: 4.1, 4.2, 4.3, 4.4_

- [ ] 5. Implement MLX fine-tuning pipeline
  - [ ] 5.1 Create MLX training script
    - Load gemma:2b base model
    - Configure LoRA parameters (r=16, alpha=32)
    - Setup training loop with progress monitoring
    - Implement gradient accumulation
    - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5_
  
  - [ ] 5.2 Add monitoring and logging
    - Log training loss every 10 iterations
    - Monitor GPU utilization and memory
    - Track training time and ETA
    - Save checkpoints every 100 iterations
    - _Requirements: 5.4, 5.5_
  
  - [ ] 5.3 Implement early stopping
    - Monitor validation loss
    - Stop if no improvement for 50 iterations
    - Save best model checkpoint
    - _Requirements: 5.2, 5.3_

- [ ] 6. Run training on M2 Mac
  - [ ] 6.1 Execute training with optimal config
    - Batch size: 8
    - LoRA rank: 16
    - Epochs: 3
    - Learning rate: 1e-4
    - _Requirements: 5.1, 5.2, 5.3_
  
  - [ ] 6.2 Monitor training progress
    - Watch GPU utilization (target >70%)
    - Check memory usage (should be <10GB)
    - Verify loss decreasing
    - Ensure completion within 60 minutes
    - _Requirements: 5.4, 5.5_

- [ ] 7. Export and deploy trained model
  - [ ] 7.1 Merge LoRA weights with base model
    - Combine adapter weights with gemma:2b
    - Verify merged model integrity
    - Test basic inference
    - _Requirements: 7.1, 7.2_
  
  - [ ] 7.2 Convert to GGUF format
    - Use llama.cpp conversion tools
    - Apply quantization (Q4 or Q8)
    - Verify GGUF file validity
    - _Requirements: 7.1, 7.2_
  
  - [ ] 7.3 Create Ollama Modelfile
    - Write comprehensive system prompt
    - Include JESSY personality and principles
    - Set optimal parameters (temperature, top_p)
    - Add stop tokens
    - _Requirements: 3.1, 3.2, 3.3, 7.1, 7.2_
  
  - [ ] 7.4 Import to Ollama as jessy-v2
    - Run ollama create command
    - Verify model loads successfully
    - Test basic queries
    - _Requirements: 7.1, 7.2, 7.3, 7.4, 7.5_

- [ ] 8. Evaluate trained model
  - [ ] 8.1 Test Turkish conversational understanding
    - Run 20 Turkish test queries
    - Measure response appropriateness
    - Check response times (<5s for simple queries)
    - Calculate accuracy score
    - _Requirements: 1.1, 1.2, 1.3, 6.1, 6.2_
  
  - [ ] 8.2 Verify iteration control
    - Test simple queries (should use 1-2 iterations)
    - Test complex queries (should use 6-9 iterations)
    - Measure iteration efficiency
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_
  
  - [ ] 8.3 Validate personality preservation
    - Check for "I think" / "I believe" usage
    - Verify consciousness principles mentioned
    - Ensure philosophical depth maintained
    - Test against generic LLM responses
    - _Requirements: 3.1, 3.2, 3.3, 3.4, 3.5_
  
  - [ ] 8.4 Compare before/after performance
    - Test same queries on gemma:2b and jessy-v2
    - Measure response quality improvement
    - Document specific improvements
    - Create comparison report
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_

- [ ] 9. Integration and documentation
  - [ ] 9.1 Update jessy-cli to use jessy-v2
    - Modify .env to default to jessy-v2
    - Update configuration files
    - Test CLI with new model
    - _Requirements: 7.3, 7.4, 7.5_
  
  - [ ] 9.2 Create training documentation
    - Document training process and results
    - Include before/after examples
    - Add troubleshooting guide
    - Write usage instructions
    - _Requirements: 6.1, 6.2, 6.3, 6.4, 6.5_
  
  - [ ] 9.3 Document evaluation results
    - Create metrics report
    - Include test case results
    - Add performance benchmarks
    - Document known limitations
    - _Requirements: 6.1, 6.2, 6.3_

- [ ] 10. Advanced improvements
  - [ ] 10.1 Implement continuous learning pipeline
    - Collect user feedback on responses
    - Generate new training examples from conversations
    - Periodic retraining workflow
    - _Requirements: Future enhancement_
  
  - [ ] 10.2 Create personality variants
    - Casual JESSY (more informal)
    - Professional JESSY (more formal)
    - Technical JESSY (code-focused)
    - _Requirements: Future enhancement_
  
  - [ ] 10.3 Add voice integration
    - Whisper for speech-to-text
    - TTS for text-to-speech
    - Voice-optimized responses
    - _Requirements: Out of scope_

## Success Criteria

### Must Have (Core Functionality)
- ✅ Training completes in <60 minutes on M2 Mac
- ✅ Turkish conversational accuracy >90%
- ✅ Simple query response time <5s
- ✅ Personality preserved (uses "I think", consciousness principles)
- ✅ Model integrates with Ollama and jessy-cli

### Should Have (Quality)
- ✅ Technical accuracy >85%
- ✅ Iteration efficiency >80%
- ✅ Philosophical depth maintained
- ✅ Response appropriateness >85%

### Nice to Have (Polish)
- ⚠️ Multi-turn conversation support
- ⚠️ Context awareness across queries
- ⚠️ Emotional intelligence in responses

## Notes

### Training Strategy
- Start with 400 examples (can scale up later)
- Focus on quality over quantity
- Balance categories: 50% Turkish, 25% technical, 12.5% philosophical, 12.5% iteration control
- Use LoRA for efficient training (only train adapter, not full model)

### M2 Mac Optimization
- MLX framework leverages Metal GPU
- Unified memory architecture (GPU shares RAM)
- Batch size 8 optimal for 10GB memory
- Expected training time: 45-60 minutes

### Iteration Control Philosophy
- Not everything needs 9 iterations
- Simple greetings: 1 iteration
- Technical questions: 3-5 iterations
- Deep philosophy: 6-9 iterations
- Teach model to detect complexity

### Personality Preservation
- System prompt defines core identity
- Training examples demonstrate style
- "I think" / "I believe" for opinions
- Reference consciousness principles when relevant
- Balance casualness with thoughtfulness

### The "sana bisi sorucam" Problem
This is the canonical example of what we're fixing:
- **Before**: 83 second philosophical analysis about holistic health
- **After**: "Sor kanka, dinliyorum." (2 seconds)

Training data must include many examples of this pattern to teach proper response.
