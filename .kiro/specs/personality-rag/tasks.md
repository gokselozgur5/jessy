# Implementation Plan - Personality RAG System

## Task List

- [x] 1. Setup dependencies and project structure
  - Add Qdrant client to Cargo.toml
  - Create directory structure (models, services, utils, bin)
  - _Requirements: All_

- [x] 2. Implement PersonalityChunk model
  - [x] 2.1 Create PersonalityChunk struct with all fields
    - Define id, source, section, content, metadata
    - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_
  
  - [x] 2.2 Create PersonalitySource enum (Kiro, Goksel)
    - _Requirements: 1.1_
  
  - [x] 2.3 Create ChunkMetadata struct
    - Define category, relevance_tags, priority fields
    - _Requirements: 1.3, 1.4, 1.5_
  
  - [x] 2.4 Create ChunkCategory enum
    - Define all categories (CoreIdentity, DecisionFramework, etc.)
    - _Requirements: 1.3_

- [x] 3. Implement PersonalityChunker utility
  - [x] 3.1 Create PersonalityChunker struct with config
    - max_chunk_size (500 tokens), overlap (50 tokens)
    - _Requirements: 1.2_
  
  - [x] 3.2 Implement markdown section parsing
    - Parse by ## and ### headers using pulldown-cmark or regex
    - Extract section title and content
    - _Requirements: 1.1_
  
  - [x] 3.3 Implement category detection logic
    - Pattern match on section titles (identity, framework, heuristic, etc.)
    - Return appropriate ChunkCategory
    - _Requirements: 1.3_
  
  - [x] 3.4 Implement tag extraction logic
    - Extract keywords from content (dating, philosophy, technical, etc.)
    - Return Vec<String> of tags
    - _Requirements: 1.4_
  
  - [x] 3.5 Implement priority calculation
    - Map ChunkCategory to priority (1-10)
    - CoreIdentity=10, DecisionFramework=9, etc.
    - _Requirements: 1.5_
  
  - [x] 3.6 Implement chunk splitting with overlap
    - Split large sections (>500 tokens) into sub-chunks
    - Maintain 50-token overlap between chunks
    - _Requirements: 1.2_
  
  - [x] 3.7 Implement main chunk_markdown() method
    - Orchestrate all steps: parse → detect → extract → split
    - Return Vec<PersonalityChunk>
    - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

- [x] 4. Implement VectorStore service
  - [x] 4.1 Create VectorStore struct with Qdrant client
    - Initialize QdrantClient from URL
    - Store collection name ("jessy_personality")
    - _Requirements: 3.1_
  
  - [x] 4.2 Implement collection initialization
    - Create collection if not exists
    - Configure vector size (from orchestrator's model)
    - Set distance metric to Cosine
    - _Requirements: 3.1, 3.3, 3.4_
  
  - [x] 4.3 Implement chunk upsert method
    - Convert PersonalityChunk + embedding to PointStruct
    - Batch upsert to Qdrant
    - _Requirements: 3.2_
  
  - [x] 4.4 Implement similarity search method
    - Search Qdrant with query embedding
    - Return top-K chunks with scores
    - _Requirements: 4.1, 4.2_
  
  - [x] 4.5 Add error handling for Qdrant operations
    - Handle connection errors, timeouts
    - Log errors and return Result
    - _Requirements: 10.1, 10.4_

- [x] 5. Implement PersonalityRAG service
  - [x] 5.1 Create PersonalityRAG struct
    - Store VectorStore, CognitiveOrchestrator, PersonalityChunker, top_k
    - _Requirements: 2.1, 4.1_
  
  - [x] 5.2 Implement initialize_from_files() method
    - Read personality files (realkiro.md, gokselclaude.md)
    - Chunk both files using PersonalityChunker
    - _Requirements: 6.1, 6.2_
  
  - [x] 5.3 Implement batch embedding generation
    - For each chunk, call orchestrator.generate_embedding()
    - Process in parallel using tokio::spawn
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 6.3_
  
  - [x] 5.4 Complete initialize_from_files() - upsert to Qdrant
    - Call vector_store.upsert_chunks() with chunks + embeddings
    - Log total chunks indexed
    - _Requirements: 6.4, 6.5_
  
  - [x] 5.5 Implement retrieve_relevant_context() method
    - Generate query embedding using orchestrator
    - Search vector store for top-K chunks
    - _Requirements: 4.1, 4.2, 7.1_
  
  - [x] 5.6 Implement format_chunks_for_prompt() method
    - Format chunks as markdown with source and section
    - Return formatted string for system prompt
    - _Requirements: 4.3_
  
  - [x] 5.7 Add error handling and graceful degradation
    - Handle embedding failures, search timeouts
    - Return empty context on error (fallback to base prompt)
    - _Requirements: 10.1, 10.2, 10.4, 7.4_

- [x] 6. Integrate RAG into LLMService
  - [x] 6.1 Add PersonalityRAG field to LLMService
    - Store Arc<PersonalityRAG> in AppState struct (optional for graceful degradation)
    - _Requirements: 7.1_
  
  - [x] 6.2 Modify generate_response() to call RAG
    - Modified build_system_prompt to accept optional personality context
    - Handle errors with graceful degradation (None = fallback to base prompt)
    - _Requirements: 7.1, 7.4_
  
  - [x] 6.3 Inject personality context into system prompt
    - Append retrieved context after base Jessy prompt
    - Format as "# Relevant Personality Context:"
    - _Requirements: 5.1, 5.2, 5.3, 7.2_
  
  - [x] 6.4 Send to Claude API with enhanced prompt
    - System ready - will use enhanced prompt when RAG initialized
    - Return response to user
    - _Requirements: 7.3_

- [ ] 7. Create initialization script (bin/initialize_rag.rs)
  - [ ] 7.1 Setup environment and load config
    - Load .env file
    - Read QDRANT_URL, personality file paths
    - _Requirements: 6.1, 8.1_
  
  - [ ] 7.2 Initialize services
    - Create VectorStore, CognitiveOrchestrator, PersonalityChunker
    - Create PersonalityRAG
    - _Requirements: 6.1_
  
  - [ ] 7.3 Call initialize_from_files()
    - Pass paths to realkiro.md and gokselclaude.md
    - Wait for completion
    - _Requirements: 6.2, 6.3, 6.4_
  
  - [ ] 7.4 Log initialization results
    - Log total chunks indexed per source
    - Log success/failure status
    - _Requirements: 6.5, 9.4_

- [ ] 8. Add configuration via environment variables
  - [ ] 8.1 Add QDRANT_URL to .env
    - Default: http://localhost:6334
    - _Requirements: 8.1_
  
  - [ ] 8.2 Add RAG_TOP_K to .env
    - Default: 5
    - _Requirements: 8.2_
  
  - [ ] 8.3 Add RAG_CHUNK_SIZE to .env
    - Default: 500
    - _Requirements: 8.3_
  
  - [ ] 8.4 Add RAG_CHUNK_OVERLAP to .env
    - Default: 50
    - _Requirements: 8.4_
  
  - [ ] 8.5 Add personality file paths to .env
    - KIRO_PERSONALITY_PATH, GOKSEL_PERSONALITY_PATH
    - _Requirements: 6.1_

- [ ] 9. Add observability and logging
  - [ ] 9.1 Log chunk retrieval operations
    - Log query, top-K chunks, similarity scores
    - _Requirements: 9.1_
  
  - [ ] 9.2 Log embedding generation
    - Log chunk count, processing time
    - _Requirements: 9.2_
  
  - [ ] 9.3 Log vector search latency
    - Log search time in milliseconds
    - _Requirements: 9.3_
  
  - [ ] 9.4 Log initialization results
    - Log total chunks indexed per source
    - _Requirements: 9.4_

- [ ] 10. Setup Qdrant (deployment)
  - [ ] 10.1 Create docker-compose.yml for Qdrant
    - Expose ports 6333, 6334
    - Mount volume for persistence
    - _Requirements: 3.1_
  
  - [ ] 10.2 Document Qdrant setup in README
    - Docker command, embedded mode option
    - _Requirements: 3.1_

- [ ]* 11. Write tests
  - [ ]* 11.1 Unit tests for PersonalityChunker
    - Test markdown parsing, category detection, tag extraction
    - _Requirements: 1.1, 1.3, 1.4_
  
  - [ ]* 11.2 Unit tests for VectorStore
    - Mock Qdrant client, test upsert and search
    - _Requirements: 3.2, 4.1_
  
  - [ ]* 11.3 Integration test for RAG flow
    - Test end-to-end: chunk → embed → store → retrieve
    - _Requirements: All_

- [ ]* 12. Performance optimization
  - [ ]* 12.1 Benchmark embedding generation
    - Measure time for 100 chunks, target <5s
    - _Requirements: 2.4_
  
  - [ ]* 12.2 Benchmark vector search
    - Measure search latency, target <50ms
    - _Requirements: 4.2_
  
  - [ ]* 12.3 Profile end-to-end latency
    - Measure total RAG overhead, target <100ms
    - _Requirements: 7.1, 7.2_
