//! Memory pool allocator for MMAP regions
//!
//! Implements a multi-pool allocator with different block sizes for efficient
//! memory management of consciousness layers.
//!
//! # Design Pattern: Segregated Free Lists
//!
//! This is similar to how jemalloc and tcmalloc work - multiple pools with
//! different block sizes reduce fragmentation. Small allocations don't waste
//! space in large blocks, and large allocations don't fragment small blocks.
//!
//! # Why Multiple Pools?
//!
//! Single pool with variable-size blocks → fragmentation over time
//! Multiple pools with fixed-size blocks → predictable, fast allocation

use crate::{Result, ConsciousnessError};
use bitvec::prelude::*;  // Efficient bitmap operations
use memmap2::{MmapMut, MmapOptions};
use std::collections::HashMap;

/// Pool allocator managing multiple memory pools with different block sizes
///
/// Design: Each pool has fixed-size blocks tracked by a bitmap
/// u8 pool_id limits us to 256 pools - sufficient for our use case
/// Using u8 instead of usize saves memory in MmapOffset structs
#[derive(Debug)]
pub struct PoolAllocator {
    // HashMap for O(1) pool lookup by ID
    pools: HashMap<u8, MmapPool>,
    // Simple counter for generating unique pool IDs
    // u8 wraps at 255, but we'll never have that many pools
    next_pool_id: u8,
}

impl PoolAllocator {
    /// Create new pool allocator
    ///
    /// No allocation happens here - HashMap::new() is zero-cost
    /// This is the "lazy initialization" pattern
    pub fn new() -> Self {
        Self {
            pools: HashMap::new(),
            next_pool_id: 0,
        }
    }
    
    /// Add a new memory pool with specified block size
    ///
    /// This is called during initialization to set up size classes
    /// Returns pool_id for tracking, though usually not needed
    pub fn add_pool(&mut self, size_mb: usize, block_size: usize) -> Result<u8> {
        let pool_id = self.next_pool_id;
        // Post-increment pattern: use current value, then increment
        self.next_pool_id += 1;
        
        // ? operator propagates errors up - if MmapPool::new fails, we fail
        let pool = MmapPool::new(pool_id, size_mb, block_size)?;
        // insert returns Option<old_value> - we ignore it since IDs are unique
        self.pools.insert(pool_id, pool);
        
        Ok(pool_id)
    }
    
    /// Allocate a block from the best-fit pool
    ///
    /// Best-fit strategy: find smallest block that fits the request
    /// This minimizes wasted space (internal fragmentation)
    /// Alternative would be first-fit (faster) or worst-fit (more fragmentation)
    pub fn allocate(&mut self, size: usize) -> Result<super::MmapOffset> {
        // Iterator chain - zero-cost abstraction, compiles to efficient loop
        // This is functional programming style, but with C-like performance
        let pool_id = self.pools
            .iter()  // Borrow each (key, value) pair
            .filter(|(_, pool)| pool.block_size >= size)  // Only pools big enough
            .min_by_key(|(_, pool)| pool.block_size)  // Find smallest
            .map(|(id, _)| *id)  // Extract just the ID, dereference it
            .ok_or_else(|| ConsciousnessError::AllocationFailed(
                format!("No pool available for size {} bytes", size)
            ))?;  // Convert Option to Result
        
        // unwrap() is safe here - we just found this pool_id in the iterator
        // In production code, we'd use expect() with a message for debugging
        let pool = self.pools.get_mut(&pool_id).unwrap();
        let offset = pool.allocate_block()?;
        
        // Struct literal - all fields must be specified
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
    ///
    /// Aggregates stats from all pools - useful for monitoring
    /// Takes &self (immutable borrow) so it doesn't interfere with allocations
    pub fn get_stats(&self) -> PoolStats {
        let mut stats = PoolStats::default();
        
        // values() returns iterator over pool references
        // No cloning, just borrowing - zero-cost
        for pool in self.pools.values() {
            stats.total_pools += 1;
            stats.total_size += pool.total_size;
            stats.allocated_size += pool.allocated_size();
            stats.free_blocks += pool.free_blocks();
        }
        
        stats
    }
    
    /// Get the number of pools
    pub fn pool_count(&self) -> usize {
        self.pools.len()
    }
    
    /// Get raw pointer to allocated block
    ///
    /// # Safety
    /// Returns raw pointer - caller must ensure:
    /// - Pointer is not used after deallocation
    /// - No aliasing with other references
    /// - Writes don't exceed block size
    ///
    /// This is marked safe (not `unsafe fn`) because returning a pointer isn't
    /// unsafe - only dereferencing it is. The unsafety is at the call site.
    pub fn get_ptr(&self, mmap_offset: super::MmapOffset) -> Result<*mut u8> {
        let pool = self.pools.get(&mmap_offset.pool_id)
            .ok_or_else(|| ConsciousnessError::MemoryError(
                format!("Pool {} not found", mmap_offset.pool_id)
            ))?;
        
        // unsafe block: pointer arithmetic with add()
        // as_ptr() returns *const u8, we cast to *mut for write access
        // This is safe because MmapMut provides interior mutability
        // add() is unsafe because it can create out-of-bounds pointers
        Ok(unsafe { 
            let ptr = pool.mmap.as_ptr() as *mut u8;
            ptr.add(mmap_offset.offset)
        })
    }
    
    /// Get slice reference to allocated block
    ///
    /// This is the safe alternative to get_ptr() - returns a slice with bounds
    /// Rust's borrow checker ensures the slice can't outlive the pool
    pub fn get_slice(&self, mmap_offset: super::MmapOffset, size: usize) -> Result<&[u8]> {
        let pool = self.pools.get(&mmap_offset.pool_id)
            .ok_or_else(|| ConsciousnessError::RegionNotFound {
                region_id: mmap_offset.pool_id as u32,
            })?;
        
        // Bounds check before creating slice - prevents undefined behavior
        // This is why safe Rust can't have buffer overflows
        if mmap_offset.offset + size > pool.total_size {
            return Err(ConsciousnessError::OutOfBounds {
                offset: mmap_offset.offset,
                size,
                region_size: pool.total_size,
            });
        }
        
        // Slice creation is safe after bounds check
        // Returns borrowed slice - no allocation, no copy
        Ok(&pool.mmap[mmap_offset.offset..mmap_offset.offset + size])
    }
}

/// Individual memory pool with fixed block size
///
/// Design: All blocks in a pool are the same size
/// This makes allocation O(1) - just find first free bit in bitmap
/// Deallocation is also O(1) - just set the bit
///
/// MmapMut is mutable MMAP - we can write to it
/// BitVec is from bitvec crate - space-efficient bitmap (1 bit per block)
#[derive(Debug)]
pub struct MmapPool {
    pub pool_id: u8,
    pub block_size: usize,  // Fixed size for all blocks in this pool
    pub total_size: usize,  // Total pool size in bytes
    pub mmap: MmapMut,  // Mutable memory-mapped region
    pub free_bitmap: BitVec,  // 1 = free, 0 = allocated
}

impl MmapPool {
    /// Create new memory pool
    ///
    /// Task 11.1: Cross-platform MMAP using memmap2
    /// 
    /// Platform-specific behavior:
    /// - Linux: Uses mmap() with MAP_ANONYMOUS flag
    /// - macOS: Uses mmap() with MAP_ANON flag (same as Linux)
    /// - Windows: Uses CreateFileMapping with INVALID_HANDLE_VALUE
    ///
    /// Page sizes:
    /// - Linux/Intel: 4KB pages
    /// - macOS Intel: 4KB pages
    /// - macOS Apple Silicon: 16KB pages (handled automatically by memmap2)
    /// - Windows: 4KB or 64KB pages
    ///
    /// memmap2 crate abstracts all platform differences
    pub fn new(pool_id: u8, size_mb: usize, block_size: usize) -> Result<Self> {
        let total_size = size_mb * 1024 * 1024;
        let num_blocks = total_size / block_size;
        
        // map_anon() creates anonymous mapping - not backed by file
        // This is like malloc() but using MMAP for better control
        // OS doesn't allocate physical memory until we write to pages
        // memmap2 handles platform differences:
        // - Unix: mmap(MAP_ANONYMOUS or MAP_ANON)
        // - Windows: CreateFileMapping(INVALID_HANDLE_VALUE)
        let mmap = MmapOptions::new()
            .len(total_size)
            .map_anon()  // Cross-platform anonymous mapping
            .map_err(|e| ConsciousnessError::MemoryError(
                format!("Failed to create mmap: {}", e)
            ))?;
        
        // BitVec is more efficient than Vec<bool>
        // Vec<bool> uses 1 byte per bool, BitVec uses 1 bit
        // For 1000 blocks: Vec<bool> = 1KB, BitVec = 125 bytes
        let mut free_bitmap = BitVec::with_capacity(num_blocks);
        free_bitmap.resize(num_blocks, true);  // All blocks start free
        
        Ok(Self {
            pool_id,
            block_size,
            total_size,
            mmap,
            free_bitmap,
        })
    }
    
    /// Allocate a block from this pool
    ///
    /// O(1) average case - BitVec::first_one() uses CPU bit-scan instructions
    /// Modern CPUs have BSF/BSR (bit scan forward/reverse) for fast bit finding
    pub fn allocate_block(&mut self) -> Result<usize> {
        // first_one() finds first set bit (free block) using hardware instructions
        // Returns Option<usize> - None if no free blocks
        let block_idx = self.free_bitmap
            .first_one()
            .ok_or_else(|| ConsciousnessError::AllocationFailed(
                format!("Pool {} is full (block_size={}, total_size={})", 
                    self.pool_id, self.block_size, self.total_size)
            ))?;
        
        // Mark block as used (false = allocated, true = free)
        // set() is O(1) - just flips one bit
        self.free_bitmap.set(block_idx, false);
        
        // Convert block index to byte offset
        // This is the actual offset into the MMAP region
        Ok(block_idx * self.block_size)
    }
    
    /// Deallocate a block
    ///
    /// O(1) operation - just sets a bit in the bitmap
    /// No need to zero memory - we'll overwrite it on next allocation
    pub fn deallocate_block(&mut self, offset: usize) -> Result<()> {
        // Convert byte offset back to block index
        let block_idx = offset / self.block_size;
        
        // Bounds check - prevent bitmap corruption
        if block_idx >= self.free_bitmap.len() {
            return Err(ConsciousnessError::MemoryError(
                "Invalid block offset".to_string()
            ));
        }
        
        // Mark block as free
        // Note: We don't check if it was already free (double-free)
        // In production, you might want to assert!(!self.free_bitmap[block_idx])
        self.free_bitmap.set(block_idx, true);
        
        Ok(())
    }
    
    /// Get number of allocated bytes
    ///
    /// count_zeros() is optimized - uses POPCNT instruction on modern CPUs
    /// POPCNT counts set bits in parallel, much faster than looping
    pub fn allocated_size(&self) -> usize {
        let used_blocks = self.free_bitmap.count_zeros();
        used_blocks * self.block_size
    }
    
    /// Get number of free blocks
    ///
    /// count_ones() also uses POPCNT - hardware-accelerated bit counting
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
#[derive(Debug, Default, Clone)]
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