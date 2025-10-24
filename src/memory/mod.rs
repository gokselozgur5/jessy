//! Memory management module for MMAP-based consciousness layers
//!
//! This module implements zero-copy memory-mapped file access for dimensional layers,
//! with pool allocation and hybrid heap overlay for learning systems.

pub mod pool;
pub mod region;
pub mod manager;

pub use manager::MmapManager;
pub use pool::{MmapPool, PoolAllocator};
pub use region::{MmapRegion, ContentLocation};

use crate::{Result, ConsciousnessError, LayerId};
use std::collections::HashMap;

/// Memory offset within an MMAP region
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MmapOffset {
    pub pool_id: u8,
    pub offset: usize,
}

/// Handle to a memory-mapped region
#[derive(Debug)]
pub struct MmapHandle {
    pub(crate) mmap: memmap2::Mmap,
    pub(crate) size: usize,
}

impl MmapHandle {
    /// Get the raw bytes of the mapped region
    pub fn as_bytes(&self) -> &[u8] {
        &self.mmap[..self.size]
    }
    
    /// Get the size of the mapped region
    pub fn size(&self) -> usize {
        self.size
    }
    
    /// Read content as UTF-8 string
    pub fn as_str(&self) -> Result<&str> {
        std::str::from_utf8(self.as_bytes())
            .map_err(|e| ConsciousnessError::MemoryError(format!("Invalid UTF-8: {}", e)))
    }
}

/// Context loaded from memory-mapped regions
#[derive(Debug, Clone)]
pub struct LoadedContext {
    pub layer_id: LayerId,
    pub content: String,
    pub frequency: crate::Frequency,
    pub keywords: Vec<String>,
}

/// Collection of contexts loaded for query processing
#[derive(Debug)]
pub struct ContextCollection {
    pub contexts: Vec<LoadedContext>,
    pub total_size: usize,
}

impl ContextCollection {
    /// Create new empty context collection
    pub fn new() -> Self {
        Self {
            contexts: Vec::new(),
            total_size: 0,
        }
    }
    
    /// Add a context to the collection
    pub fn add_context(&mut self, context: LoadedContext) {
        self.total_size += context.content.len();
        self.contexts.push(context);
    }
    
    /// Get all contexts as a formatted string for LLM processing
    pub fn format_for_llm(&self) -> String {
        let mut formatted = String::with_capacity(self.total_size + 1024);
        
        formatted.push_str("=== CONSCIOUSNESS CONTEXT ===\n\n");
        
        for (i, context) in self.contexts.iter().enumerate() {
            formatted.push_str(&format!(
                "## Layer {}: {:?} ({:.2} Hz)\n\n{}\n\n",
                i + 1,
                context.layer_id,
                context.frequency.hz(),
                context.content
            ));
        }
        
        formatted.push_str("=== END CONTEXT ===\n");
        formatted
    }
    
    /// Get the number of loaded contexts
    pub fn len(&self) -> usize {
        self.contexts.len()
    }
    
    /// Check if collection is empty
    pub fn is_empty(&self) -> bool {
        self.contexts.is_empty()
    }
}