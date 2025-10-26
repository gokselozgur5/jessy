//! Circular buffer implementation for observations

/// Circular buffer with fixed capacity
///
/// When the buffer is full, new items overwrite the oldest items.
/// This ensures bounded memory usage while maintaining recent history.
#[derive(Debug, Clone)]
pub struct CircularBuffer<T> {
    buffer: Vec<T>,
    capacity: usize,
    head: usize,
    size: usize,
}

impl<T> CircularBuffer<T> {
    /// Create new circular buffer with given capacity
    pub fn new(capacity: usize) -> Self {
        Self {
            buffer: Vec::with_capacity(capacity),
            capacity,
            head: 0,
            size: 0,
        }
    }
    
    /// Push item into buffer
    ///
    /// If buffer is full, overwrites oldest item
    pub fn push(&mut self, item: T) {
        if self.size < self.capacity {
            self.buffer.push(item);
            self.size += 1;
        } else {
            self.buffer[self.head] = item;
            self.head = (self.head + 1) % self.capacity;
        }
    }
    
    /// Get current number of items in buffer
    pub fn len(&self) -> usize {
        self.size
    }
    
    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }
    
    /// Get buffer capacity
    pub fn capacity(&self) -> usize {
        self.capacity
    }
    
    /// Iterate over items in chronological order (oldest to newest)
    pub fn iter(&self) -> CircularBufferIter<T> {
        CircularBufferIter {
            buffer: &self.buffer,
            head: self.head,
            current: 0,
            size: self.size,
            capacity: self.capacity,
        }
    }
}

/// Iterator for circular buffer
pub struct CircularBufferIter<'a, T> {
    buffer: &'a [T],
    head: usize,
    current: usize,
    size: usize,
    capacity: usize,
}

impl<'a, T> Iterator for CircularBufferIter<'a, T> {
    type Item = &'a T;
    
    fn next(&mut self) -> Option<Self::Item> {
        if self.current >= self.size {
            None
        } else {
            let index = if self.size < self.capacity {
                // Buffer not full, items are in order
                self.current
            } else {
                // Buffer is full, start from head
                (self.head + self.current) % self.capacity
            };
            self.current += 1;
            Some(&self.buffer[index])
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_circular_buffer_creation() {
        let buffer: CircularBuffer<i32> = CircularBuffer::new(10);
        assert_eq!(buffer.len(), 0);
        assert_eq!(buffer.capacity(), 10);
        assert!(buffer.is_empty());
    }
    
    #[test]
    fn test_push_within_capacity() {
        let mut buffer = CircularBuffer::new(5);
        buffer.push(1);
        buffer.push(2);
        buffer.push(3);
        
        assert_eq!(buffer.len(), 3);
        assert!(!buffer.is_empty());
        
        let items: Vec<_> = buffer.iter().copied().collect();
        assert_eq!(items, vec![1, 2, 3]);
    }
    
    #[test]
    fn test_push_exceeds_capacity() {
        let mut buffer = CircularBuffer::new(3);
        buffer.push(1);
        buffer.push(2);
        buffer.push(3);
        buffer.push(4); // Overwrites 1
        buffer.push(5); // Overwrites 2
        
        assert_eq!(buffer.len(), 3);
        
        let items: Vec<_> = buffer.iter().copied().collect();
        assert_eq!(items, vec![3, 4, 5]);
    }
    
    #[test]
    fn test_circular_overwrite() {
        let mut buffer = CircularBuffer::new(4);
        
        // Fill buffer
        for i in 1..=4 {
            buffer.push(i);
        }
        assert_eq!(buffer.len(), 4);
        
        // Overwrite oldest
        buffer.push(5);
        buffer.push(6);
        
        let items: Vec<_> = buffer.iter().copied().collect();
        assert_eq!(items, vec![3, 4, 5, 6]);
    }
    
    #[test]
    fn test_iter_chronological_order() {
        let mut buffer = CircularBuffer::new(5);
        
        // Add items
        for i in 1..=7 {
            buffer.push(i);
        }
        
        // Should have [3, 4, 5, 6, 7] in chronological order
        let items: Vec<_> = buffer.iter().copied().collect();
        assert_eq!(items, vec![3, 4, 5, 6, 7]);
    }
}
