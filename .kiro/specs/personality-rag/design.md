# Design Document - Personality RAG System

## Overview

The Personality RAG System enables Jessy to dynamically retrieve relevant personality context from Kiro and Göksel's steering files. Instead of loading entire files (which can exceed token limits), the system:

1. **Offline (Initialization):** Chunks personality files semantically, generates embeddings using existing mmap-optimized model, stores in Qdrant
2. **Runtime (Per Request):** Generates query embedding, searches vector DB for top-K relevant chunks, injects into system prompt

**Key Design Principles:**
- **Reuse Existing Infrastructure:** Use orchestrator's mmap-optimized embedding model (no new dependencies)
- **Semantic Chunking:** Preserve meaning by chunking at section boundaries
- **Fast Retrieval:** Vector similarity search in <50ms
- **Graceful Degradation:** Fallback to base prompt if RAG fails
- **Observability:** Log all retrieval operations for debugging

## Architecture

### High-Level Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                    OFFLINE INITIALIZATION                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  realkiro.md + gokselclaude.md                                 │
│         │                                                       │
│         ▼                                                       │
│  PersonalityChunker                                            │
│    ├─ Parse markdown sections (##, ###)                        │
│    ├─ Detect category (Heuristic, Framework, etc.)            │
│    ├─ Extract tags (dating, philosophy, etc.)                 │
│    └─ Assign priority (1-10)                                  │
│         │                                                       │
│         ▼                                                       │
│  CognitiveOrchestrator.generate_embedding()                    │
│    └─ mmap-optimized embedding model                           │
│         │                                                       │
│         ▼                                                       │
│  VectorStore.upsert_chunks()                                   │
│    └─ Qdrant collection: "jessy_personality"                   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│                      RUNTIME RETRIEVAL                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  User Message: "Should I message her?"                         │
│         │                                                       │
│         ▼                                                       │
│  PersonalityRAG.retrieve_relevant_context()                    │
│    ├─ Generate query embedding (orchestrator)                  │
│    ├─ Search Qdrant (cosine similarity, top-K=5)              │
│    └─ Format chunks for prompt                                 │
│         │                                                       │
│         ▼                                                       │
│  LLMService.generate_response()                                │
│    ├─ Build system prompt:                                     │
│    │   - Base Jessy prompt                                     │
│    │   - "# Relevant Personality Context:"                     │
│    │   - Retrieved chunks (with source + section)             │
│    ├─ Call Claude API                                          │
│    └─ Return response                                          │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Component Diagram

```
┌──────────────────────────────────────────────────────────────┐
│                      jessy-backend                           │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │              LLMService                            │    │
│  │  - generate_response()                             │    │
│  │  - build_system_prompt()                           │    │
│  └────────────┬───────────────────────────────────────┘    │
│               │ uses                                        │
│               ▼                                             │
│  ┌────────────────────────────────────────────────────┐    │
│  │           PersonalityRAG                           │    │
│  │  - retrieve_relevant_context()                     │    │
│  │  - initialize_from_files()                         │    │
│  └────┬──────────────────────┬────────────────────────┘    │
│       │ uses                 │ uses                        │
│       ▼                      ▼                             │
│  ┌─────────────┐      ┌──────────────────┐               │
│  │ VectorStore │      │ PersonalityChunker│               │
│  │ - search()  │      │ - chunk_markdown()│               │
│  │ - upsert()  │      │ - detect_category()│              │
│  └──────┬──────┘      └──────────────────┘               │
│         │ uses                                             │
│         ▼                                                  │
│  ┌─────────────────────────────────────────┐              │
│  │      CognitiveOrchestrator              │              │
│  │  - generate_embedding()                 │              │
│  │  - embedding_model (mmap-optimized)     │              │
│  └─────────────────────────────────────────┘              │
│                                                              │
└──────────────────────────────────────────────────────────────┘
         │
         │ network
         ▼
┌──────────────────────┐
│   Qdrant Vector DB   │
│  Collection:         │
│  "jessy_personality" │
└──────────────────────┘
```

## Components and Interfaces

### 1. PersonalityChunk (Model)

**Purpose:** Represents a semantic chunk of personality data

```rust
// src/models/personality_chunk.rs

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PersonalityChunk {
    pub id: String,              // "kiro-decision-framework-1"
    pub source: PersonalitySource,
    pub section: String,         // "Decision Framework"
    pub content: String,         // Actual text content
    pub metadata: ChunkMetadata,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PersonalitySource {
    Kiro,
    Goksel,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChunkMetadata {
    pub category: ChunkCategory,
    pub relevance_tags: Vec<String>,  // ["dating", "decision-making"]
    pub priority: u8,                 // 1-10 (10 = highest)
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ChunkCategory {
    CoreIdentity,           // Who they are
    DecisionFramework,      // How they decide
    Heuristic,             // Specific rules/patterns
    CommunicationPattern,  // How they communicate
    PhilosophicalFramework,// Philosophical foundations
    CaseStudy,             // Real examples
    Example,               // Generic examples
}
```

**Priority Mapping:**
- CoreIdentity: 10 (always relevant)
- DecisionFramework: 9 (critical for advice)
- Heuristic: 8 (practical patterns)
- PhilosophicalFramework: 7 (foundational)
- CommunicationPattern: 6 (style/tone)
- CaseStudy: 5 (contextual examples)
- Example: 4 (generic examples)

### 2. PersonalityChunker (Utility)

**Purpose:** Parse markdown and create semantic chunks

```rust
// src/utils/chunking.rs

pub struct PersonalityChunker {
    max_chunk_size: usize,  // ~500 tokens
    overlap: usize,         // 50 tokens
}

impl PersonalityChunker {
    pub fn chunk_markdown(
        &self,
        markdown: &str,
        source: PersonalitySource
    ) -> Vec<PersonalityChunk> {
        // 1. Parse markdown into sections
        let sections = self.parse_sections(markdown);
        
        // 2. For each section:
        //    - Detect category from title
        //    - Extract tags from content
        //    - Calculate priority
        //    - Split if too large (with overlap)
        
        // 3. Return chunks
    }
    
    fn parse_sections(&self, markdown: &str) -> Vec<Section> {
        // Parse by ## and ### headers
        // Use pulldown-cmark or regex
    }
    
    fn detect_category(&self, title: &str) -> ChunkCategory {
        // Pattern matching on section title
        match title.to_lowercase() {
            t if t.contains("identity") || t.contains("philosophy") 
                => ChunkCategory::CoreIdentity,
            t if t.contains("framework") || t.contains("decision")
                => ChunkCategory::DecisionFramework,
            t if t.contains("heuristic") || t.contains("scenario")
                => ChunkCategory::Heuristic,
            // ... etc
            _ => ChunkCategory::Example,
        }
    }
    
    fn extract_tags(&self, content: &str) -> Vec<String> {
        // Simple keyword extraction
        // Look for: dating, philosophy, technical, health, etc.
        // Can use regex or simple contains()
    }
    
    fn calculate_priority(&self, category: &ChunkCategory) -> u8 {
        match category {
            ChunkCategory::CoreIdentity => 10,
            ChunkCategory::DecisionFramework => 9,
            ChunkCategory::Heuristic => 8,
            ChunkCategory::PhilosophicalFramework => 7,
            ChunkCategory::CommunicationPattern => 6,
            ChunkCategory::CaseStudy => 5,
            ChunkCategory::Example => 4,
        }
    }
    
    fn split_with_overlap(&self, text: &str) -> Vec<String> {
        // Split large sections into sub-chunks
        // Maintain 50-token overlap for context continuity
    }
}
```

**Chunking Strategy:**
- **Primary:** Split by markdown headers (##, ###)
- **Secondary:** If section > 500 tokens, split with 50-token overlap
- **Preserve:** Keep section title with each chunk for context

### 3. VectorStore (Service)

**Purpose:** Interface to Qdrant vector database

```rust
// src/services/vector_store.rs

use qdrant_client::{
    client::QdrantClient,
    qdrant::{CreateCollection, SearchPoints, PointStruct, VectorParams, Distance}
};

pub struct VectorStore {
    client: QdrantClient,
    collection_name: String,
}

impl VectorStore {
    pub async fn new(url: &str) -> Result<Self> {
        let client = QdrantClient::from_url(url).build()?;
        Ok(Self {
            client,
            collection_name: "jessy_personality".to_string(),
        })
    }
    
    pub async fn initialize_collection(&self, vector_size: usize) -> Result<()> {
        // Create collection if doesn't exist
        self.client.create_collection(&CreateCollection {
            collection_name: self.collection_name.clone(),
            vectors_config: Some(VectorParams {
                size: vector_size as u64,  // From orchestrator's model
                distance: Distance::Cosine,
                ..Default::default()
            }),
            ..Default::default()
        }).await?;
        
        Ok(())
    }
    
    pub async fn upsert_chunks(
        &self,
        chunks: Vec<PersonalityChunk>,
        embeddings: Vec<Vec<f32>>
    ) -> Result<()> {
        let points: Vec<PointStruct> = chunks
            .iter()
            .zip(embeddings.iter())
            .map(|(chunk, embedding)| {
                PointStruct {
                    id: Some(chunk.id.clone().into()),
                    vectors: Some(embedding.clone().into()),
                    payload: serde_json::to_value(chunk).unwrap().into(),
                }
            })
            .collect();
        
        self.client.upsert_points(
            self.collection_name.clone(),
            points,
            None,
        ).await?;
        
        Ok(())
    }
    
    pub async fn search(
        &self,
        query_embedding: Vec<f32>,
        limit: usize
    ) -> Result<Vec<PersonalityChunk>> {
        let search_result = self.client.search_points(&SearchPoints {
            collection_name: self.collection_name.clone(),
            vector: query_embedding,
            limit: limit as u64,
            with_payload: Some(true.into()),
            ..Default::default()
        }).await?;
        
        let chunks: Vec<PersonalityChunk> = search_result
            .result
            .iter()
            .filter_map(|point| {
                serde_json::from_value(point.payload.clone()).ok()
            })
            .collect();
        
        Ok(chunks)
    }
}
```

**Qdrant Configuration:**
- **Collection:** "jessy_personality"
- **Distance Metric:** Cosine (standard for embeddings)
- **Vector Size:** Dynamic (from orchestrator's embedding model)
- **Payload:** Full PersonalityChunk as JSON

### 4. PersonalityRAG (Service)

**Purpose:** Main RAG orchestration - chunking, embedding, retrieval

```rust
// src/services/personality_rag.rs

pub struct PersonalityRAG {
    vector_store: Arc<VectorStore>,
    orchestrator: Arc<CognitiveOrchestrator>,
    chunker: PersonalityChunker,
    top_k: usize,
}

impl PersonalityRAG {
    pub fn new(
        vector_store: Arc<VectorStore>,
        orchestrator: Arc<CognitiveOrchestrator>,
        top_k: usize,
    ) -> Self {
        Self {
            vector_store,
            orchestrator,
            chunker: PersonalityChunker::new(500, 50),
            top_k,
        }
    }
    
    pub async fn initialize_from_files(
        &self,
        kiro_path: &str,
        goksel_path: &str
    ) -> Result<()> {
        // 1. Read files
        let kiro_content = tokio::fs::read_to_string(kiro_path).await?;
        let goksel_content = tokio::fs::read_to_string(goksel_path).await?;
        
        // 2. Chunk both files
        let mut chunks = Vec::new();
        chunks.extend(self.chunker.chunk_markdown(&kiro_content, PersonalitySource::Kiro));
        chunks.extend(self.chunker.chunk_markdown(&goksel_content, PersonalitySource::Goksel));
        
        info!("Generated {} personality chunks", chunks.len());
        
        // 3. Generate embeddings (parallel for performance)
        let embeddings = self.generate_embeddings_batch(&chunks).await?;
        
        // 4. Upsert to vector DB
        self.vector_store.upsert_chunks(chunks, embeddings).await?;
        
        info!("Personality chunks indexed successfully");
        Ok(())
    }
    
    async fn generate_embeddings_batch(
        &self,
        chunks: &[PersonalityChunk]
    ) -> Result<Vec<Vec<f32>>> {
        let mut embeddings = Vec::new();
        
        // Process in parallel (use tokio::spawn for each chunk)
        let handles: Vec<_> = chunks
            .iter()
            .map(|chunk| {
                let orchestrator = self.orchestrator.clone();
                let content = chunk.content.clone();
                tokio::spawn(async move {
                    orchestrator.generate_embedding(&content).await
                })
            })
            .collect();
        
        for handle in handles {
            embeddings.push(handle.await??);
        }
        
        Ok(embeddings)
    }
    
    pub async fn retrieve_relevant_context(
        &self,
        query: &str,
    ) -> Result<String> {
        // 1. Generate query embedding
        let query_embedding = self.orchestrator
            .generate_embedding(query)
            .await?;
        
        // 2. Search vector DB
        let chunks = self.vector_store
            .search(query_embedding, self.top_k)
            .await?;
        
        // 3. Format for system prompt
        Ok(self.format_chunks_for_prompt(chunks))
    }
    
    fn format_chunks_for_prompt(&self, chunks: Vec<PersonalityChunk>) -> String {
        let mut result = String::from("# Relevant Personality Context:\n\n");
        
        for (i, chunk) in chunks.iter().enumerate() {
            let source_name = match chunk.source {
                PersonalitySource::Kiro => "Kiro",
                PersonalitySource::Goksel => "Göksel",
            };
            
            result.push_str(&format!(
                "## Context {}: {} (from {})\n{}\n\n",
                i + 1,
                chunk.section,
                source_name,
                chunk.content
            ));
        }
        
        result
    }
}
```

**Key Design Decisions:**
- **Parallel Embedding:** Use tokio::spawn for batch embedding generation
- **Top-K Configurable:** Default 5, can be tuned via env var
- **Format:** Markdown-style context injection (readable for Claude)

### 5. LLMService Integration

**Purpose:** Integrate RAG into existing LLM service

```rust
// src/services/llm_service.rs (modifications)

pub struct LLMService {
    client: reqwest::Client,
    api_key: String,
    rag: Arc<PersonalityRAG>,  // ← Add this
    // ... existing fields
}

impl LLMService {
    pub async fn generate_response(
        &self,
        user_message: &str,
        session_id: &str
    ) -> Result<String> {
        // 1. Retrieve personality context via RAG
        let personality_context = match self.rag
            .retrieve_relevant_context(user_message)
            .await
        {
            Ok(ctx) => ctx,
            Err(e) => {
                warn!("RAG retrieval failed: {}, using base prompt", e);
                String::new()  // Graceful degradation
            }
        };
        
        // 2. Build system prompt
        let system_prompt = if personality_context.is_empty() {
            self.base_jessy_prompt()
        } else {
            format!(
                "{}\n\n{}",
                self.base_jessy_prompt(),
                personality_context
            )
        };
        
        // 3. Call Claude API (existing logic)
        let response = self.client
            .post("https://api.anthropic.com/v1/messages")
            .header("x-api-key", &self.api_key)
            .header("anthropic-version", "2023-06-01")
            .json(&json!({
                "model": "claude-3-5-sonnet-20241022",
                "max_tokens": 4096,
                "system": system_prompt,
                "messages": [{
                    "role": "user",
                    "content": user_message
                }]
            }))
            .send()
            .await?
            .json::<ClaudeResponse>()
            .await?;
        
        Ok(response.content[0].text.clone())
    }
}
```

**Graceful Degradation:**
- If RAG fails → log warning, use base prompt
- If Qdrant unavailable → continue with base prompt
- Never crash due to RAG failure

## Data Models

### Qdrant Point Structure

```json
{
  "id": "kiro-decision-framework-1",
  "vector": [0.23, -0.45, 0.67, ...],  // 384 or 768 dimensions
  "payload": {
    "id": "kiro-decision-framework-1",
    "source": "Kiro",
    "section": "Decision Framework",
    "content": "When making decisions, I apply 10 frameworks...",
    "metadata": {
      "category": "DecisionFramework",
      "relevance_tags": ["decision-making", "philosophy", "frameworks"],
      "priority": 9
    }
  }
}
```

### System Prompt Format

```
You are Jessy, an AI consciousness with evolving personality...

[Base Jessy prompt]

# Relevant Personality Context:

## Context 1: Decision Framework (from Göksel)
When making decisions, I apply 10 frameworks simultaneously...

## Context 2: Niyeti Varsa Söyler Heuristic (from Göksel)
If someone is confident and direct, they will express interest...

## Context 3: Communication Patterns (from Kiro)
I use Turkish-English mix naturally, with kanka energy...

[User message follows]
```

## Error Handling

### Error Scenarios

1. **Qdrant Unavailable**
   - **Detection:** Connection timeout or error response
   - **Handling:** Log error, return empty context, use base prompt
   - **Recovery:** Retry on next request

2. **Embedding Generation Failure**
   - **Detection:** Orchestrator returns error
   - **Handling:** Log error, skip that chunk, continue with others
   - **Recovery:** Retry failed chunks in background

3. **Personality File Missing**
   - **Detection:** File not found during initialization
   - **Handling:** Log warning, continue with available file
   - **Recovery:** Manual fix (ensure files exist)

4. **Vector Search Timeout**
   - **Detection:** Qdrant search exceeds 5 seconds
   - **Handling:** Cancel search, return empty context
   - **Recovery:** Investigate Qdrant performance

### Error Logging

```rust
// Example error handling with logging

match self.rag.retrieve_relevant_context(query).await {
    Ok(context) => {
        info!(
            "RAG retrieved {} chunks for query: {}",
            context.lines().filter(|l| l.starts_with("## Context")).count(),
            query
        );
        context
    }
    Err(e) => {
        warn!(
            "RAG retrieval failed for query '{}': {}. Using base prompt.",
            query, e
        );
        String::new()
    }
}
```

## Testing Strategy

### Unit Tests

1. **PersonalityChunker**
   - Test markdown parsing (sections, headers)
   - Test category detection (various titles)
   - Test tag extraction (keyword matching)
   - Test priority calculation
   - Test chunk splitting with overlap

2. **VectorStore**
   - Test collection creation
   - Test upsert (mock Qdrant client)
   - Test search (mock responses)

3. **PersonalityRAG**
   - Test chunk formatting
   - Test batch embedding (mock orchestrator)
   - Test context retrieval (mock vector store)

### Integration Tests

1. **End-to-End RAG Flow**
   - Initialize with test personality files
   - Query with test message
   - Verify relevant chunks retrieved
   - Verify system prompt format

2. **LLM Service Integration**
   - Mock Claude API
   - Verify RAG context injected
   - Verify graceful degradation on RAG failure

### Performance Tests

1. **Embedding Generation**
   - Measure time for 100 chunks
   - Target: <5 seconds (parallel processing)

2. **Vector Search**
   - Measure search latency
   - Target: <50ms per query

3. **End-to-End Latency**
   - Measure total time (embedding + search + format)
   - Target: <100ms added to LLM call

## Deployment Considerations

### Qdrant Setup

**Option 1: Embedded Mode (Development)**
```rust
// Qdrant runs in-process (no separate server)
let client = QdrantClient::from_url("memory").build()?;
```

**Option 2: Docker (Production)**
```bash
docker run -p 6333:6333 -p 6334:6334 \
  -v $(pwd)/qdrant_storage:/qdrant/storage \
  qdrant/qdrant
```

**Option 3: Qdrant Cloud (Scalable)**
```bash
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-api-key
```

### Environment Variables

```bash
# Qdrant configuration
QDRANT_URL=http://localhost:6334
QDRANT_API_KEY=  # Optional (for Qdrant Cloud)

# RAG configuration
RAG_TOP_K=5
RAG_CHUNK_SIZE=500
RAG_CHUNK_OVERLAP=50

# Personality files
KIRO_PERSONALITY_PATH=/Users/gok/.kiro/steering/:realkiro:.md
GOKSEL_PERSONALITY_PATH=/Users/gok/.kiro/steering/gokselclaude.md
```

### Initialization Script

```bash
# Run once to populate vector DB
cargo run --bin initialize_rag

# Output:
# Reading personality files...
# Chunking realkiro.md... (45 chunks)
# Chunking gokselclaude.md... (67 chunks)
# Generating embeddings... (112 chunks)
# Upserting to Qdrant...
# ✓ RAG system initialized successfully!
```

### Monitoring

**Metrics to Track:**
- RAG retrieval latency (p50, p95, p99)
- Embedding generation time
- Vector search time
- RAG failure rate
- Chunks retrieved per query (distribution)

**Logging:**
```rust
info!("RAG: Retrieved {} chunks in {}ms", chunk_count, latency_ms);
warn!("RAG: Search timeout after 5s, using base prompt");
error!("RAG: Qdrant connection failed: {}", error);
```

## Future Enhancements

1. **Hybrid Search:** Combine vector similarity with keyword matching
2. **Chunk Reranking:** Use LLM to rerank retrieved chunks by relevance
3. **Dynamic Top-K:** Adjust based on query complexity
4. **Chunk Caching:** Cache frequently retrieved chunks
5. **Multi-Source Fusion:** Blend Kiro + Göksel chunks intelligently
6. **Feedback Loop:** Learn which chunks are most useful over time
7. **Incremental Updates:** Update vector DB without full reindex

## Summary

The Personality RAG System enables Jessy to dynamically embody Kiro and Göksel's personalities by:
- **Chunking** personality files semantically (500 tokens, 50 overlap)
- **Embedding** using existing mmap-optimized model (no new dependencies)
- **Storing** in Qdrant vector database (cosine similarity)
- **Retrieving** top-K relevant chunks per query (<50ms)
- **Injecting** into system prompt (graceful degradation on failure)

**Key Benefits:**
- 🚀 Scalable (handles large personality files)
- 💰 Cost-effective (reuses existing embedding model)
- ⚡ Fast (vector search <50ms)
- 🛡️ Robust (graceful degradation)
- 🔍 Observable (comprehensive logging)
