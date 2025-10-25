//! Memory management module for MMAP-based consciousness layers
//!
//! This module implements zero-copy memory-mapped file access for dimensional layers,
//! with pool allocation and hybrid heap overlay for learning systems.
//!
//! # Architecture
//!
//! The memory system uses a three-layer design:
//! - **MmapManager**: Orchestrates regions and pools, provides high-level API
//! - **MmapRegion**: Individual memory-mapped files for dimensions
//! - **PoolAllocator**: Dynamic memory allocation with multiple size classes
//!
//! # Zero-Copy Design
//!
//! Instead of reading files into buffers (copy), we map them directly into
//! virtual memory (zero-copy). The OS handles paging automatically.
//! This is why we return &[u8] slices instead of Vec<u8> - no allocation needed.

pub mod pool;
pub mod region;
pub mod manager;
pub mod optimization;
pub mod diagnostics;

#[cfg(test)]
mod error_tests;

#[cfg(test)]
mod perf_tests;

#[cfg(test)]
mod concurrency_tests;

#[cfg(test)]
mod integration_tests;

// Re-export main types for convenience
// Users can write `use memory::MmapManager` instead of `use memory::manager::MmapManager`
pub use manager::MmapManager;
pub use pool::{MmapPool, PoolAllocator};
pub use region::{MmapRegion, ContentLocation};
// Re-export NavigationPath from navigation module for convenience
pub use crate::navigation::NavigationPath;

use crate::{Result, ConsciousnessError, LayerId};
use std::collections::HashMap;

/// Memory offset within an MMAP region
///
/// Design note: This is Copy because it's just two integers (u8 + usize)
/// Copy types can be duplicated with simple memcpy - no heap allocation
/// Rule of thumb: if it fits in 2-3 registers, make it Copy
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MmapOffset {
    pub pool_id: u8,
    pub offset: usize,
}

/// Handle to a memory-mapped region
///
/// This wraps memmap2::Mmap to provide a safer, more ergonomic API
/// pub(crate) fields are visible within this crate but not to external users
/// This is Rust's module privacy system - more granular than public/private
#[derive(Debug)]
pub struct MmapHandle {
    pub(crate) mmap: memmap2::Mmap,
    pub(crate) size: usize,
}

impl MmapHandle {
    /// Get the raw bytes of the mapped region
    ///
    /// Returns &[u8] (borrowed slice) not Vec<u8> (owned) - zero-copy
    /// The slice borrows from self, so it can't outlive this MmapHandle
    pub fn as_bytes(&self) -> &[u8] {
        // Slice syntax: &array[start..end]
        // This creates a "fat pointer" - pointer + length, no allocation
        &self.mmap[..self.size]
    }
    
    /// Get the size of the mapped region
    pub fn size(&self) -> usize {
        self.size
    }
    
    /// Read content as UTF-8 string
    ///
    /// Returns &str not String - borrows from the MMAP, doesn't allocate
    /// This is why MMAP is fast - we never copy the data
    pub fn as_str(&self) -> Result<&str> {
        // from_utf8 validates UTF-8 without copying
        // If invalid, we get Err with details about where it failed
        std::str::from_utf8(self.as_bytes())
            .map_err(|e| ConsciousnessError::MemoryError(format!("Invalid UTF-8: {}", e)))
    }
}

/// Context loaded from memory-mapped regions
///
/// This is Clone because we need to duplicate it for processing
/// String and Vec are heap-allocated, so Clone does a deep copy
/// For hot paths, consider using Rc<LoadedContext> to share instead of clone
#[derive(Debug, Clone)]
pub struct LoadedContext {
    pub layer_id: LayerId,
    pub content: String,  // Owned String - necessary because we return it
    pub frequency: crate::Frequency,
    pub keywords: Vec<String>,  // Vec is growable array on heap
}

/// Collection of contexts loaded for query processing
#[derive(Debug)]
pub struct ContextCollection {
    pub contexts: Vec<LoadedContext>,
    pub total_size: usize,
}

impl ContextCollection {
    /// Create new empty context collection
    ///
    /// Vec::new() is a const fn - no runtime cost, just returns empty Vec
    /// Actual allocation happens on first push() - lazy allocation pattern
    pub fn new() -> Self {
        Self {
            contexts: Vec::new(),
            total_size: 0,
        }
    }
    
    /// Add a context to the collection
    ///
    /// Takes ownership of context (not &context) because we're storing it
    /// This is "move semantics" - context is moved into the Vec
    /// After this call, the original context variable is no longer usable
    pub fn add_context(&mut self, context: LoadedContext) {
        self.total_size += context.content.len();
        // push() moves context into the Vec - no copy if Vec has capacity
        // If Vec is full, it reallocates with 2x capacity (amortized O(1))
        self.contexts.push(context);
    }
    
    /// Get all contexts as a formatted string for LLM processing
    ///
    /// Pre-allocates String with known capacity to avoid reallocations
    /// This is the "reserve then fill" pattern for performance
    pub fn format_for_llm(&self) -> String {
        // with_capacity pre-allocates - avoids multiple reallocations during push_str
        // We know approximate size: total_size + overhead for formatting
        let mut formatted = String::with_capacity(self.total_size + 1024);
        
        // push_str appends without allocation (if capacity sufficient)
        formatted.push_str("=== CONSCIOUSNESS CONTEXT ===\n\n");
        
        // iter() borrows each element - no cloning
        // enumerate() adds index - zero-cost iterator adapter
        for (i, context) in self.contexts.iter().enumerate() {
            // format! allocates a new String - acceptable here (not hot path)
            // &format!(...) creates temporary String, then push_str borrows it
            formatted.push_str(&format!(
                "## Layer {}: {:?} ({:.2} Hz)\n\n{}\n\n",
                i + 1,
                context.layer_id,
                context.frequency.hz(),
                context.content
            ));
        }
        
        formatted.push_str("=== END CONTEXT ===\n");
        formatted  // Return ownership of the String
    }
    
    /// Task 9.2: Format contexts with frequency and keywords for LLM
    ///
    /// Enhanced formatting that includes:
    /// - Dimension metadata
    /// - Frequency information
    /// - Keywords for each layer
    /// - Structured format for better LLM understanding
    pub fn format_with_metadata(&self) -> String {
        let mut formatted = String::with_capacity(self.total_size + 2048);
        
        formatted.push_str("=== DIMENSIONAL CONSCIOUSNESS CONTEXT ===\n\n");
        formatted.push_str(&format!("Total Layers: {}\n", self.contexts.len()));
        formatted.push_str(&format!("Total Content Size: {} bytes\n\n", self.total_size));
        
        for (i, context) in self.contexts.iter().enumerate() {
            formatted.push_str(&format!("--- Layer {} ---\n", i + 1));
            formatted.push_str(&format!("Dimension: D{:02}\n", context.layer_id.dimension.0));
            formatted.push_str(&format!("Layer ID: {:?}\n", context.layer_id));
            formatted.push_str(&format!("Frequency: {:.2} Hz\n", context.frequency.hz()));
            
            // Add keywords if present
            if !context.keywords.is_empty() {
                formatted.push_str("Keywords: ");
                formatted.push_str(&context.keywords.join(", "));
                formatted.push_str("\n");
            }
            
            formatted.push_str("\nContent:\n");
            formatted.push_str(&context.content);
            formatted.push_str("\n\n");
        }
        
        formatted.push_str("=== END DIMENSIONAL CONTEXT ===\n");
        formatted
    }
    
    /// Get the number of loaded contexts
    ///
    /// Vec::len() is O(1) - just returns the stored length field
    /// Not O(n) like linked lists in some languages
    pub fn len(&self) -> usize {
        self.contexts.len()
    }
    
    /// Check if collection is empty
    ///
    /// Clippy (Rust linter) requires is_empty() when you have len()
    /// This is a common API pattern in Rust standard library
    pub fn is_empty(&self) -> bool {
        self.contexts.is_empty()
    }
}