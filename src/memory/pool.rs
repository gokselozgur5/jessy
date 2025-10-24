//! Memory pool allocation for MMAP regions
//!
//! Implements fixed-size block allocation with multiple pool sizes for efficient
//! memory management of consciousness layers.

use crate::{Result, ConsciousnessError};
use crate::memory::{MmapOffset, MmapHandle};
use bitvec::prelude::*;
use memmap2::{MmapOptions, MmapMut};
use std::collections::HashMap;
use std::fs::OpenOptions;
use std::path::Path;

/// Memory pool with fixed block size
#[derive(Debug)]
pub struct MmapPool {
    /// Memory-mapped region for this pool
    mmap: MmapMut,
    
    /// Size of each block in bytes
    block_size: usize,
    
    /// Total number of blocks in this pool
    block_count: usize,
    
    /// Bitmap tracking which blocks are allocated
    allocation_bitmap: BitVec,
    
    /// Pool identifier
    pool_id: u8,
}

impl MmapPool {
    /// Create new memory pool with specified block size and count
    pub fn new(pool_id: u8, block_size: usize, block_count: usize) -> Result<Self> {
        let total_size = block_size * block_count;
        
        // Create temporary file for memory mapping
        let temp_path = format!("/tmp/jessy_pool_{}.mmap", pool_id);
        let file = OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .truncate(true)
            .open(&temp_path)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to create pool file: {}", e)))?;
        
        // Set file size
        file.set_len(total_size as u64)
            .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to set file size: {}", e)))?;
        
        // Memory map the file
        let mmap = unsafe {
            MmapOptions::new()
                .map_mut(&file)
                .map_err(|e| ConsciousnessError::MemoryError(format!("Failed to mmap pool: {}", e)))?
        };
        
        // Initialize allocation bitmap (all blocks free)
        let allocation_bitmap = bitvec![0; block_count];
        
        Ok(Self {
            mmap,
            block_size,
            block_count,
            allocation_bitmap,
            pool_id,
        })
    }
    
    /// Allocate a block from this pool
    pub fn allocate_block(&mut self) -> Option<MmapOffset> {
        // Find first free block
        for (index, mut bit) in self.allocation_bitmap.iter_mut().enumerate() {
            if !*bit {
                // Mark block as allocated
                bit.set(true);
                
                return Some(MmapOffset {
                    pool_id: self.pool_id,
                    offset: index * self.block_size,
                });
            }
        }
        
        None // Pool is full
    }
    
    /// Free a block in this pool
    pub fn free_block(&mut self, offset: MmapOffset) -> Result<()> {
        if offset.pool_id != self.pool_id {
            return Err(ConsciousnessError::MemoryError(
                "Offset belongs to different pool".to_string()
            ));
        }
        
        let block_index = offset.offset / self.block_size;
        
        if block_index >= self.block_count {
            return Err(ConsciousnessError::MemoryError(
                "Block index out of bounds".to_string()
            ));
        }
        
        // Mark block as free
        self.allocation_bitmap.set(block_index, false);
        
        Ok(())
    }
    
    /// Get raw bytes for a block
    pub fn get_block_bytes(&self, offset: MmapOffset) -> Result<&[u8]> {
        if offset.pool_id != self.pool_id {
            return Err(ConsciousnessError::MemoryError(
                "Offset belongs to different pool".to_string()
            ));
        }
        
        let start = offset.offset;
        let end = start + self.block_size;
        
        if end > self.mmap.len() {
            return Err(ConsciousnessError::MemoryError(
                "Block offset out of bounds".to_string()
            ));
        }
        
        Ok(&self.mmap[start..end])
    }
    
    /// Get mutable bytes for a block
    pub fn get_block_bytes_mut(&mut self, offset: MmapOffset) -> Result<&mut [u8]> {
        if offset.pool_id != self.pool_id {
            return Err(ConsciousnessError::MemoryError(
                "Offset belongs to different pool".to_string()
            ));
        }
        
        let start = offset.offset;
        let end = start + self.block_size;
        
        if end > self.mmap.len() {
            return Err(ConsciousnessError::MemoryError(
                "Block offset out of bounds".to_string()
            ));
        }
        
        Ok(&mut self.mmap[start..end])
    }
    
    /// Get pool statistics
    pub fn stats(&self) -> PoolStats {
        let allocated_blocks = self.allocation_bitmap.count_ones();
        let free_blocks = self.block_count - allocated_blocks;
        
        PoolStats {
            pool_id: self.pool_id,
            block_size: self.block_size,
            total_blocks: self.block_count,
            allocated_blocks,
            free_blocks,
            fragmentation_ratio: allocated_blocks as f32 / self.block_count as f32,
        }
    }
    
    /// Check if pool can grow (extend file size)
    pub fn can_grow(&self) -> bool {
        // For now, pools have fixed size
        // Future: implement mremap-based growth
        false
    }
}

/// Pool allocation statistics
#[derive(Debug, Clone)]
pub struct PoolStats {
    pub pool_id: u8,
    pub block_size: usize,
    pub total_blocks: usize,
    pub allocated_blocks: usize,
    pub free_blocks: usize,
    pub fragmentation_ratio: f32,
}

/// Multi-pool allocator managing different block sizes
#[derive(Debug)]
pub struct PoolAllocator {
    /// Pools indexed by pool ID
    pools: HashMap<u8, MmapPool>,
    
    /// Block sizes for each pool (in ascending order)
    block_sizes: Vec<usize>,
}

impl PoolAllocator {
    /// Create new pool allocator with standard block sizes
    pub fn new() -> Result<Self> {
        let block_sizes = vec![
            4 * 1024,    // 4KB - small layers
            16 * 1024,   // 16KB - medium layers  
            64 * 1024,   // 64KB - large layers
            256 * 1024,  // 256KB - very large layers
        ];
        
        let mut pools = HashMap::new();
        
        // Create pools with different block sizes
        for (pool_id, &block_size) in block_sizes.iter().enumerate() {
            let pool_id = pool_id as u8;
            
            // Calculate block count based on total memory budget
            let blocks_per_pool = match block_size {
                4096 => 1024,    // 4MB total for 4KB blocks
                16384 => 512,    // 8MB total for 16KB blocks
                65536 => 256,    // 16MB total for 64KB blocks
                262144 => 128,   // 32MB total for 256KB blocks
                _ => 64,         // Default
            };
            
            let pool = MmapPool::new(pool_id, block_size, blocks_per_pool)?;
            pools.insert(pool_id, pool);
        }
        
        Ok(Self {
            pools,
            block_sizes,
        })
    }
    
    /// Allocate memory for given size, selecting appropriate pool
    pub fn allocate(&mut self, size: usize) -> Result<MmapOffset> {
        // Find smallest pool that can fit the size
        let pool_id = self.find_suitable_pool(size)?;
        
        let pool = self.pools.get_mut(&pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Pool not found".to_string()))?;
        
        pool.allocate_block()
            .ok_or_else(|| ConsciousnessError::MemoryError("Pool exhausted".to_string()))
    }
    
    /// Free allocated memory
    pub fn free(&mut self, offset: MmapOffset) -> Result<()> {
        let pool = self.pools.get_mut(&offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Pool not found".to_string()))?;
        
        pool.free_block(offset)
    }
    
    /// Get bytes for reading
    pub fn get_bytes(&self, offset: MmapOffset) -> Result<&[u8]> {
        let pool = self.pools.get(&offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Pool not found".to_string()))?;
        
        pool.get_block_bytes(offset)
    }
    
    /// Get bytes for writing
    pub fn get_bytes_mut(&mut self, offset: MmapOffset) -> Result<&mut [u8]> {
        let pool = self.pools.get_mut(&offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError("Pool not found".to_string()))?;
        
        pool.get_block_bytes_mut(offset)
    }
    
    /// Find suitable pool for given size
    fn find_suitable_pool(&self, size: usize) -> Result<u8> {
        for (pool_id, &block_size) in self.block_sizes.iter().enumerate() {
            if size <= block_size {
                return Ok(pool_id as u8);
            }
        }
        
        Err(ConsciousnessError::MemoryError(
            format!("Size {} too large for any pool", size)
        ))
    }
    
    /// Get statistics for all pools
    pub fn get_stats(&self) -> Vec<PoolStats> {
        self.pools.values()
            .map(|pool| pool.stats())
            .collect()
    }
    
    /// Get total memory usage
    pub fn total_memory_usage(&self) -> usize {
        self.pools.values()
            .map(|pool| pool.block_size * pool.block_count)
            .sum()
    }
}

impl Default for PoolAllocator {
    fn default() -> Self {
        Self::new().expect("Failed to create default pool allocator")
    }
}