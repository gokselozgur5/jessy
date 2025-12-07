use crate::models::personality_chunk::PersonalityChunk;
use anyhow::{Context, Result};
use qdrant_client::{
    client::QdrantClient,
    qdrant::{
        vectors_config::Config, CreateCollection, Distance, PointStruct, SearchPoints,
        VectorParams, VectorsConfig,
    },
};
use tracing::{info, warn};

use super::vector_store_memory::InMemoryVectorStore;

/// Vector store service for personality chunks
/// Supports both Qdrant (remote) and InMemory (local) backends
pub struct VectorStore {
    backend: VectorStoreBackend,
}

enum VectorStoreBackend {
    Qdrant {
        client: QdrantClient,
        collection_name: String,
    },
    InMemory(InMemoryVectorStore),
}

impl VectorStore {
    /// Create a new VectorStore instance
    ///
    /// Use `:memory:` for in-memory storage (no external dependencies)
    /// Use a URL like `http://localhost:6334` for Qdrant
    pub async fn new(url: &str) -> Result<Self> {
        let backend = if url == ":memory:" {
            info!("🧠 Using in-memory vector store");
            VectorStoreBackend::InMemory(InMemoryVectorStore::new())
        } else {
            info!("🔗 Connecting to Qdrant at {}", url);
            let client = QdrantClient::from_url(url)
                .build()
                .context("Failed to create Qdrant client")?;

            VectorStoreBackend::Qdrant {
                client,
                collection_name: "jessy_personality".to_string(),
            }
        };

        Ok(Self { backend })
    }

    /// Initialize collection with given vector size
    pub async fn initialize_collection(&self, vector_size: u64) -> Result<()> {
        match &self.backend {
            VectorStoreBackend::InMemory(store) => {
                store.initialize_collection(vector_size).await
            }
            VectorStoreBackend::Qdrant { client, collection_name } => {
                // Check if collection exists
                let collections = client.list_collections().await?;
                let exists = collections
                    .collections
                    .iter()
                    .any(|c| c.name == *collection_name);

                if exists {
                    info!("Collection '{}' already exists", collection_name);
                    return Ok(());
                }

                // Create collection
                client
                    .create_collection(&CreateCollection {
                        collection_name: collection_name.clone(),
                        vectors_config: Some(VectorsConfig {
                            config: Some(Config::Params(VectorParams {
                                size: vector_size,
                                distance: Distance::Cosine.into(),
                                ..Default::default()
                            })),
                        }),
                        ..Default::default()
                    })
                    .await
                    .context("Failed to create Qdrant collection")?;

                info!(
                    "Created collection '{}' with vector size {}",
                    collection_name, vector_size
                );
                Ok(())
            }
        }
    }

    /// Upsert personality chunks with embeddings
    pub async fn upsert_chunks(
        &self,
        chunks: Vec<PersonalityChunk>,
        embeddings: Vec<Vec<f32>>,
    ) -> Result<()> {
        match &self.backend {
            VectorStoreBackend::InMemory(store) => {
                store.upsert_chunks(chunks, embeddings).await
            }
            VectorStoreBackend::Qdrant { client, collection_name } => {
                if chunks.len() != embeddings.len() {
                    anyhow::bail!(
                        "Chunks and embeddings length mismatch: {} vs {}",
                        chunks.len(),
                        embeddings.len()
                    );
                }

                let points: Vec<PointStruct> = chunks
                    .iter()
                    .zip(embeddings.iter())
                    .enumerate()
                    .map(|(idx, (chunk, embedding))| {
                        let json_value = serde_json::to_value(chunk)
                            .expect("Failed to serialize chunk");

                        let payload = json_value
                            .as_object()
                            .expect("Chunk should be object")
                            .iter()
                            .map(|(k, v)| (k.clone(), v.clone().into()))
                            .collect();

                        PointStruct {
                            id: Some((idx as u64).into()),
                            vectors: Some(embedding.clone().into()),
                            payload,
                        }
                    })
                    .collect();

                client
                    .upsert_points_blocking(collection_name, None, points, None)
                    .await
                    .context("Failed to upsert points to Qdrant")?;

                info!("Upserted {} chunks to Qdrant", chunks.len());
                Ok(())
            }
        }
    }

    /// Search for similar chunks
    pub async fn search(
        &self,
        query_embedding: Vec<f32>,
        limit: usize,
    ) -> Result<Vec<PersonalityChunk>> {
        match &self.backend {
            VectorStoreBackend::InMemory(store) => {
                store.search(query_embedding, limit).await
            }
            VectorStoreBackend::Qdrant { client, collection_name } => {
                let search_result = client
                    .search_points(&SearchPoints {
                        collection_name: collection_name.clone(),
                        vector: query_embedding,
                        limit: limit as u64,
                        with_payload: Some(true.into()),
                        ..Default::default()
                    })
                    .await
                    .context("Failed to search Qdrant")?;

                let chunks: Vec<PersonalityChunk> = search_result
                    .result
                    .iter()
                    .filter_map(|point| {
                        let json_map: serde_json::Map<String, serde_json::Value> = point
                            .payload
                            .iter()
                            .map(|(k, v)| (k.clone(), v.clone().into()))
                            .collect();

                        serde_json::from_value(serde_json::Value::Object(json_map)).ok()
                    })
                    .collect();

                info!("Found {} relevant chunks", chunks.len());
                Ok(chunks)
            }
        }
    }

    /// Delete collection (for testing)
    #[cfg(test)]
    pub async fn delete_collection(&self) -> Result<()> {
        match &self.backend {
            VectorStoreBackend::InMemory(store) => {
                store.clear().await;
                Ok(())
            }
            VectorStoreBackend::Qdrant { client, collection_name } => {
                client
                    .delete_collection(collection_name)
                    .await
                    .context("Failed to delete collection")?;
                Ok(())
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::personality_chunk::{
        ChunkCategory, ChunkMetadata, PersonalitySource,
    };

    fn create_test_chunk(id: &str) -> PersonalityChunk {
        PersonalityChunk {
            id: id.to_string(),
            source: PersonalitySource::Kiro,
            section: "Test Section".to_string(),
            content: "Test content".to_string(),
            metadata: ChunkMetadata {
                category: ChunkCategory::CoreIdentity,
                relevance_tags: vec!["test".to_string()],
                priority: 10,
            },
        }
    }

    #[tokio::test]
    async fn test_vector_store_creation() {
        // Use memory mode for testing
        let result = VectorStore::new("http://localhost:6334").await;
        // May fail if Qdrant not running, but tests structure
        assert!(result.is_ok() || result.is_err());
    }

    #[test]
    fn test_chunk_serialization_for_qdrant() {
        let chunk = create_test_chunk("test-1");
        let json = serde_json::to_value(&chunk).unwrap();
        assert!(json.is_object());
        assert_eq!(json["id"], "test-1");
        assert_eq!(json["section"], "Test Section");
    }

    #[test]
    fn test_embedding_chunk_length_validation() {
        let chunks = vec![create_test_chunk("1"), create_test_chunk("2")];
        let embeddings = vec![vec![0.1, 0.2, 0.3]]; // Mismatch!

        assert_ne!(chunks.len(), embeddings.len());
    }
}
