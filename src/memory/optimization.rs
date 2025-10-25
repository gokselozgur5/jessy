//! Performance optimizations for memory manager
//!
//! Cache alignment, prefetching, and zero-copy optimizations

use std::alloc::{alloc, dealloc, Layout};

/// Cache-aligned allocation for hot path structures
///
/// Modern CPUs have 64-byte cache lines. Aligning frequently accessed
/// data to cache line boundaries reduces false sharing and improves
/// cache utilization.
#[repr(align(64))]
pub struct CacheAligned<T> {
    inner: T,
}

impl<T> CacheAligned<T> {
    pub fn new(value: T) -> Self {
        Self { inner: value }
    }
    
    pub fn get(&self) -> &T {
        &self.inner
    }
    
    pub fn get_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

/// OS prefetching hints for predictable access patterns
#[cfg(target_os = "linux")]
pub fn prefetch_region(ptr: *const u8, size: usize) {
    use libc::{madvise, MADV_WILLNEED};
    
    unsafe {
        // MADV_WILLNEED tells kernel to prefetch pages into RAM
        // This is a hint - kernel may ignore it under memory pressure
        madvise(
            ptr as *mut libc::c_void,
            size,
            MADV_WILLNEED
        );
    }
}

#[cfg(not(target_os = "linux"))]
pub fn prefetch_region(_ptr: *const u8, _size: usize) {
    // No-op on non-Linux platforms
    // macOS and Windows have different APIs (madvise/PrefetchVirtualMemory)
    // For MVP, we only optimize Linux
}

/// Zero-copy slice window
///
/// Returns a slice reference without copying data
/// This is the core of our zero-copy design
#[inline(always)]
pub fn zero_copy_window(data: &[u8], offset: usize, size: usize) -> Option<&[u8]> {
    // Bounds check
    if offset + size > data.len() {
        return None;
    }
    
    // Slice syntax creates fat pointer (ptr + len) - no allocation
    Some(&data[offset..offset + size])
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_cache_aligned_size() {
        let aligned = CacheAligned::new(42u64);
        
        // Verify alignment
        let ptr = &aligned as *const _ as usize;
        assert_eq!(ptr % 64, 0, "CacheAligned should be 64-byte aligned");
    }
    
    #[test]
    fn test_zero_copy_window() {
        let data = vec![1, 2, 3, 4, 5];
        
        let window = zero_copy_window(&data, 1, 3);
        assert_eq!(window, Some(&[2, 3, 4][..]));
        
        // Out of bounds
        let window = zero_copy_window(&data, 3, 5);
        assert_eq!(window, None);
    }
}
