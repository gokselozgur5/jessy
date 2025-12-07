use crate::models::personality_chunk::PersonalityChunk;
use anyhow::Result;
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::info;

/// Simple in-memory vector store using cosine similarity
pub struct InMemoryVectorStore {
    /// Storage for chunks and their embeddings
    storage: Arc<RwLock<Vec<StoredVector>>>,
}

#[derive(Clone)]
struct StoredVector {
    chunk: PersonalityChunk,
    embedding: Vec<f32>,
}

impl InMemoryVectorStore {
    /// Create a new in-memory vector store
    pub fn new() -> Self {
        Self {
            storage: Arc::new(RwLock::new(Vec::new())),
        }
    }

    /// Initialize collection (no-op for in-memory, always ready)
    pub async fn initialize_collection(&self, _vector_size: u64) -> Result<()> {
        info!("In-memory vector store ready (vector_size: {})", _vector_size);
        Ok(())
    }

    /// Upsert personality chunks with embeddings
    pub async fn upsert_chunks(
        &self,
        chunks: Vec<PersonalityChunk>,
        embeddings: Vec<Vec<f32>>,
    ) -> Result<()> {
        if chunks.len() != embeddings.len() {
            anyhow::bail!(
                "Chunks and embeddings length mismatch: {} vs {}",
                chunks.len(),
                embeddings.len()
            );
        }

        let chunk_count = chunks.len();
        let mut storage = self.storage.write().await;

        for (chunk, embedding) in chunks.into_iter().zip(embeddings.into_iter()) {
            storage.push(StoredVector { chunk, embedding });
        }

        info!("Upserted {} chunks to in-memory store (total: {})",
              chunk_count,
              storage.len());

        Ok(())
    }

    /// Search for similar chunks using cosine similarity
    pub async fn search(
        &self,
        query_embedding: Vec<f32>,
        limit: usize,
    ) -> Result<Vec<PersonalityChunk>> {
        let storage = self.storage.read().await;

        if storage.is_empty() {
            return Ok(Vec::new());
        }

        // Calculate cosine similarity for each stored vector
        let mut scored_chunks: Vec<(f32, PersonalityChunk)> = storage
            .iter()
            .map(|stored| {
                let similarity = cosine_similarity(&query_embedding, &stored.embedding);
                (similarity, stored.chunk.clone())
            })
            .collect();

        // Sort by similarity descending
        scored_chunks.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        // Take top K
        let results: Vec<PersonalityChunk> = scored_chunks
            .into_iter()
            .take(limit)
            .map(|(_, chunk)| chunk)
            .collect();

        info!("Found {} relevant chunks (from {} total)", results.len(), storage.len());
        Ok(results)
    }

    /// Get total number of stored vectors
    pub async fn count(&self) -> usize {
        self.storage.read().await.len()
    }

    /// Clear all stored vectors
    pub async fn clear(&self) {
        self.storage.write().await.clear();
        info!("Cleared in-memory vector store");
    }
}

/// Calculate cosine similarity between two vectors
fn cosine_similarity(a: &[f32], b: &[f32]) -> f32 {
    if a.len() != b.len() {
        return 0.0;
    }

    let dot_product: f32 = a.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
    let magnitude_a: f32 = a.iter().map(|x| x * x).sum::<f32>().sqrt();
    let magnitude_b: f32 = b.iter().map(|x| x * x).sum::<f32>().sqrt();

    if magnitude_a == 0.0 || magnitude_b == 0.0 {
        return 0.0;
    }

    dot_product / (magnitude_a * magnitude_b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::personality_chunk::{
        ChunkCategory, ChunkMetadata, PersonalitySource,
    };

    fn create_test_chunk(id: &str, content: &str) -> PersonalityChunk {
        PersonalityChunk {
            id: id.to_string(),
            source: PersonalitySource::Kiro,
            section: "Test Section".to_string(),
            content: content.to_string(),
            metadata: ChunkMetadata {
                category: ChunkCategory::CoreIdentity,
                relevance_tags: vec!["test".to_string()],
                priority: 10,
            },
        }
    }

    #[tokio::test]
    async fn test_in_memory_store_creation() {
        let store = InMemoryVectorStore::new();
        assert_eq!(store.count().await, 0);
    }

    #[tokio::test]
    async fn test_upsert_and_count() {
        let store = InMemoryVectorStore::new();

        let chunks = vec![
            create_test_chunk("1", "test content 1"),
            create_test_chunk("2", "test content 2"),
        ];

        let embeddings = vec![
            vec![0.1, 0.2, 0.3],
            vec![0.4, 0.5, 0.6],
        ];

        store.upsert_chunks(chunks, embeddings).await.unwrap();
        assert_eq!(store.count().await, 2);
    }

    #[tokio::test]
    async fn test_search_similar() {
        let store = InMemoryVectorStore::new();

        let chunks = vec![
            create_test_chunk("1", "philosophy"),
            create_test_chunk("2", "coding"),
            create_test_chunk("3", "existentialism"),
        ];

        let embeddings = vec![
            vec![1.0, 0.0, 0.0],  // philosophy
            vec![0.0, 1.0, 0.0],  // coding
            vec![0.9, 0.1, 0.0],  // existentialism (similar to philosophy)
        ];

        store.upsert_chunks(chunks, embeddings).await.unwrap();

        // Query similar to philosophy
        let query = vec![1.0, 0.0, 0.0];
        let results = store.search(query, 2).await.unwrap();

        assert_eq!(results.len(), 2);
        assert_eq!(results[0].id, "1");  // Most similar
        assert_eq!(results[1].id, "3");  // Second most similar
    }

    #[test]
    fn test_cosine_similarity() {
        let a = vec![1.0, 0.0, 0.0];
        let b = vec![1.0, 0.0, 0.0];
        assert!((cosine_similarity(&a, &b) - 1.0).abs() < 0.001);

        let c = vec![1.0, 0.0, 0.0];
        let d = vec![0.0, 1.0, 0.0];
        assert!((cosine_similarity(&c, &d) - 0.0).abs() < 0.001);

        let e = vec![1.0, 1.0, 0.0];
        let f = vec![1.0, 1.0, 0.0];
        assert!((cosine_similarity(&e, &f) - 1.0).abs() < 0.001);
    }

    #[tokio::test]
    async fn test_clear() {
        let store = InMemoryVectorStore::new();

        let chunks = vec![create_test_chunk("1", "test")];
        let embeddings = vec![vec![0.1, 0.2]];

        store.upsert_chunks(chunks, embeddings).await.unwrap();
        assert_eq!(store.count().await, 1);

        store.clear().await;
        assert_eq!(store.count().await, 0);
    }
}
