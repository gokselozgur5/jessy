//! Memory pool allocator for MMAP regions
//!
//! Implements a multi-pool allocator with different block sizes for efficient
//! memory management of consciousness layers.

use crate::{Result, ConsciousnessError};
use bitvec::prelude::*;
use memmap2::{MmapMut, MmapOptions};
use std::collections::HashMap;

/// Pool allocator managing multiple memory pools with different block sizes
#[derive(Debug)]
pub struct PoolAllocator {
    pools: HashMap<u8, MmapPool>,
    next_pool_id: u8,
}

impl PoolAllocator {
    /// Create new pool allocator
    pub fn new() -> Self {
        Self {
            pools: HashMap::new(),
            next_pool_id: 0,
        }
    }
    
    /// Add a new memory pool with specified block size
    pub fn add_pool(&mut self, size_mb: usize, block_size: usize) -> Result<u8> {
        let pool_id = self.next_pool_id;
        self.next_pool_id += 1;
        
        let pool = MmapPool::new(pool_id, size_mb, block_size)?;
        self.pools.insert(pool_id, pool);
        
        Ok(pool_id)
    }
    
    /// Allocate a block from the best-fit pool
    pub fn allocate(&mut self, size: usize) -> Result<super::MmapOffset> {
        // Find the smallest pool that can accommodate this size
        let pool_id = self.pools
            .iter()
            .filter(|(_, pool)| pool.block_size >= size)
            .min_by_key(|(_, pool)| pool.block_size)
            .map(|(id, _)| *id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("No pool available for size {}", size)
            ))?;
        
        let pool = self.pools.get_mut(&pool_id).unwrap();
        let offset = pool.allocate_block()?;
        
        Ok(super::MmapOffset { pool_id, offset })
    }
    
    /// Deallocate a block
    pub fn deallocate(&mut self, mmap_offset: super::MmapOffset) -> Result<()> {
        let pool = self.pools.get_mut(&mmap_offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("Pool {} not found", mmap_offset.pool_id)
            ))?;
        
        pool.deallocate_block(mmap_offset.offset)
    }
    
    /// Get pool statistics
    pub fn get_stats(&self) -> PoolStats {
        let mut stats = PoolStats::default();
        
        for pool in self.pools.values() {
            stats.total_pools += 1;
            stats.total_size += pool.total_size;
            stats.allocated_size += pool.allocated_size();
            stats.free_blocks += pool.free_blocks();
        }
        
        stats
    }
    
    /// Get raw pointer to allocated block
    pub fn get_ptr(&self, mmap_offset: super::MmapOffset) -> Result<*mut u8> {
        let pool = self.pools.get(&mmap_offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("Pool {} not found", mmap_offset.pool_id)
            ))?;
        
        Ok(unsafe { pool.mmap.as_mut_ptr().add(mmap_offset.offset) })
    }
    
    /// Get slice reference to allocated block
    pub fn get_slice(&self, mmap_offset: super::MmapOffset, size: usize) -> Result<&[u8]> {
        let pool = self.pools.get(&mmap_offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("Pool {} not found", mmap_offset.pool_id)
            ))?;
        
        if mmap_offset.offset + size > pool.total_size {
            return Err(ConsciousnessError::MemoryError(
                "Block access out of bounds".to_string()
            ));
        }
        
        Ok(&pool.mmap[mmap_offset.offset..mmap_offset.offset + size])
    }
}

/// Individual memory pool with fixed block size
#[derive(Debug)]
pub struct MmapPool {
    pub pool_id: u8,
    pub block_size: usize,
    pub total_size: usize,
    pub mmap: MmapMut,
    pub free_bitmap: BitVec,
}

impl MmapPool {
    /// Create new memory pool
    pub fn new(pool_id: u8, size_mb: usize, block_size: usize) -> Result<Self> {
        let total_size = size_mb * 1024 * 1024;
        let num_blocks = total_size / block_size;
        
        // Create anonymous memory mapping
        let mmap = MmapOptions::new()
            .len(total_size)
            .map_anon()
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to create mmap: {}", e)
            ))?;
        
        // Initialize all blocks as free
        let mut free_bitmap = BitVec::with_capacity(num_blocks);
        free_bitmap.resize(num_blocks, true);
        
        Ok(Self {
            pool_id,
            block_size,
            total_size,
            mmap,
            free_bitmap,
        })
    }
    
    /// Allocate a block from this pool
    pub fn allocate_block(&mut self) -> Result<usize> {
        // Find first free block
        let block_idx = self.free_bitmap
            .first_one()
            .ok_or_else(|| ConsciousnessError::MemoryError(
                "Pool is full".to_string()
            ))?;
        
        // Mark block as used
        self.free_bitmap.set(block_idx, false);
        
        Ok(block_idx * self.block_size)
    }
    
    /// Deallocate a block
    pub fn deallocate_block(&mut self, offset: usize) -> Result<()> {
        let block_idx = offset / self.block_size;
        
        if block_idx >= self.free_bitmap.len() {
            return Err(ConsciousnessError::MemoryError(
                "Invalid block offset".to_string()
            ));
        }
        
        // Mark block as free
        self.free_bitmap.set(block_idx, true);
        
        Ok(())
    }
    
    /// Get number of allocated bytes
    pub fn allocated_size(&self) -> usize {
        let used_blocks = self.free_bitmap.count_zeros();
        used_blocks * self.block_size
    }
    
    /// Get number of free blocks
    pub fn free_blocks(&self) -> usize {
        self.free_bitmap.count_ones()
    }
    
    /// Check if pool can grow (for future mremap support)
    pub fn can_grow(&self) -> bool {
        // For now, pools have fixed size
        // Future: implement mremap-based growth
        false
    }
}

/// Statistics for pool allocator
#[derive(Debug, Default)]
pub struct PoolStats {
    pub total_pools: usize,
    pub total_size: usize,
    pub allocated_size: usize,
    pub free_blocks: usize,
}

impl PoolStats {
    /// Get utilization percentage
    pub fn utilization(&self) -> f32 {
        if self.total_size == 0 {
            0.0
        } else {
            (self.allocated_size as f32 / self.total_size as f32) * 100.0
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_pool_creation() {
        let pool = MmapPool::new(0, 1, 4096).unwrap(); // 1MB pool, 4KB blocks
        assert_eq!(pool.pool_id, 0);
        assert_eq!(pool.block_size, 4096);
        assert_eq!(pool.total_size, 1024 * 1024);
        assert_eq!(pool.free_blocks(), 256); // 1MB / 4KB = 256 blocks
    }
    
    #[test]
    fn test_allocation_deallocation() {
        let mut pool = MmapPool::new(0, 1, 4096).unwrap();
        
        // Allocate first block
        let offset1 = pool.allocate_block().unwrap();
        assert_eq!(offset1, 0);
        assert_eq!(pool.free_blocks(), 255);
        
        // Allocate second block
        let offset2 = pool.allocate_block().unwrap();
        assert_eq!(offset2, 4096);
        assert_eq!(pool.free_blocks(), 254);
        
        // Deallocate first block
        pool.deallocate_block(offset1).unwrap();
        assert_eq!(pool.free_blocks(), 255);
        
        // Next allocation should reuse first block
        let offset3 = pool.allocate_block().unwrap();
        assert_eq!(offset3, 0);
    }
    
    #[test]
    fn test_pool_allocator() {
        let mut allocator = PoolAllocator::new();
        
        // Add pools with different block sizes
        let pool_4k = allocator.add_pool(1, 4096).unwrap();
        let pool_16k = allocator.add_pool(1, 16384).unwrap();
        
        // Allocate small block (should use 4K pool)
        let offset1 = allocator.allocate(2048).unwrap();
        assert_eq!(offset1.pool_id, pool_4k);
        
        // Allocate large block (should use 16K pool)
        let offset2 = allocator.allocate(8192).unwrap();
        assert_eq!(offset2.pool_id, pool_16k);
        
        // Get statistics
        let stats = allocator.get_stats();
        assert_eq!(stats.total_pools, 2);
        assert!(stats.allocated_size > 0);
    }
}