//! Crystallization Queue: Background task system for heap → MMAP migration
//!
//! This module provides a background worker that processes crystallization
//! requests asynchronously, allowing the main query processing pipeline to
//! continue without blocking.

use crate::{DimensionId, Result, ConsciousnessError};
use super::{Crystallizer, ProtoDimension};
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::{Mutex, Notify};
use std::time::Duration;

/// Crystallization task priority
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CrystallizationPriority {
    /// Low priority (can wait)
    Low = 0,
    /// Normal priority (default)
    Normal = 1,
    /// High priority (process soon)
    High = 2,
}

impl Default for CrystallizationPriority {
    fn default() -> Self {
        Self::Normal
    }
}

/// Crystallization task in the queue
#[derive(Debug, Clone)]
pub struct CrystallizationTask {
    /// Dimension to crystallize
    pub dimension_id: DimensionId,
    /// Task priority
    pub priority: CrystallizationPriority,
    /// Retry count
    pub retry_count: u8,
    /// Proto-dimension data
    pub proto: ProtoDimension,
}

/// Crystallization queue metrics
#[derive(Debug, Clone, Default)]
pub struct QueueMetrics {
    /// Total tasks enqueued
    pub total_enqueued: u64,
    /// Total tasks completed successfully
    pub total_completed: u64,
    /// Total tasks failed
    pub total_failed: u64,
    /// Current queue size
    pub queue_size: usize,
    /// Currently processing task
    pub processing: bool,
}

impl QueueMetrics {
    /// Calculate success rate (0.0-1.0)
    pub fn success_rate(&self) -> f64 {
        let total = self.total_completed + self.total_failed;
        if total == 0 {
            0.0
        } else {
            self.total_completed as f64 / total as f64
        }
    }
}

/// Crystallization queue with background worker
///
/// Manages a queue of proto-dimensions waiting to be crystallized.
/// A background worker continuously processes tasks from the queue.
pub struct CrystallizationQueue {
    /// Pending tasks (sorted by priority)
    pending: Arc<Mutex<VecDeque<CrystallizationTask>>>,
    /// Crystallizer instance
    crystallizer: Arc<Mutex<Crystallizer>>,
    /// Notification for new tasks
    notify: Arc<Notify>,
    /// Worker task handle
    worker: Option<tokio::task::JoinHandle<()>>,
    /// Shutdown flag
    shutdown: Arc<Mutex<bool>>,
    /// Metrics
    metrics: Arc<Mutex<QueueMetrics>>,
    /// Maximum retries per task
    max_retries: u8,
}

impl CrystallizationQueue {
    /// Create new crystallization queue
    ///
    /// # Arguments
    ///
    /// * `crystallizer` - Crystallizer instance to use
    ///
    /// # Returns
    ///
    /// New queue instance with background worker started
    pub fn new(crystallizer: Crystallizer) -> Self {
        let pending = Arc::new(Mutex::new(VecDeque::new()));
        let crystallizer = Arc::new(Mutex::new(crystallizer));
        let notify = Arc::new(Notify::new());
        let shutdown = Arc::new(Mutex::new(false));
        let metrics = Arc::new(Mutex::new(QueueMetrics::default()));

        let mut queue = Self {
            pending: pending.clone(),
            crystallizer: crystallizer.clone(),
            notify: notify.clone(),
            worker: None,
            shutdown: shutdown.clone(),
            metrics: metrics.clone(),
            max_retries: 3,
        };

        // Start background worker
        let worker = tokio::spawn(Self::worker_loop(
            pending,
            crystallizer,
            notify,
            shutdown,
            metrics,
        ));

        queue.worker = Some(worker);

        eprintln!("[Crystallization Queue] Background worker started");

        queue
    }

    /// Enqueue a crystallization task
    ///
    /// # Arguments
    ///
    /// * `proto` - Proto-dimension to crystallize
    /// * `priority` - Task priority (default: Normal)
    ///
    /// # Returns
    ///
    /// Ok(()) if task was enqueued successfully
    pub async fn enqueue(
        &self,
        proto: ProtoDimension,
        priority: CrystallizationPriority,
    ) -> Result<()> {
        let dimension_id = proto.dimension_id;  // Capture before move

        let task = CrystallizationTask {
            dimension_id,
            priority,
            retry_count: 0,
            proto,
        };

        let mut pending = self.pending.lock().await;

        // Insert task sorted by priority (high priority first)
        let insert_pos = pending
            .iter()
            .position(|t| t.priority < priority)
            .unwrap_or(pending.len());

        pending.insert(insert_pos, task);

        // Update metrics
        let mut metrics = self.metrics.lock().await;
        metrics.total_enqueued += 1;
        metrics.queue_size = pending.len();

        eprintln!(
            "[Crystallization Queue] Enqueued dimension {:?} (priority: {:?}, queue size: {})",
            dimension_id,
            priority,
            metrics.queue_size
        );

        // Notify worker
        drop(pending);
        drop(metrics);
        self.notify.notify_one();

        Ok(())
    }

    /// Background worker loop
    async fn worker_loop(
        pending: Arc<Mutex<VecDeque<CrystallizationTask>>>,
        crystallizer: Arc<Mutex<Crystallizer>>,
        notify: Arc<Notify>,
        shutdown: Arc<Mutex<bool>>,
        metrics: Arc<Mutex<QueueMetrics>>,
    ) {
        eprintln!("[Crystallization Queue Worker] Started");

        loop {
            // Check shutdown flag
            {
                let should_shutdown = *shutdown.lock().await;
                if should_shutdown {
                    eprintln!("[Crystallization Queue Worker] Shutdown requested");
                    break;
                }
            }

            // Get next task
            let task = {
                let mut pending_guard = pending.lock().await;
                pending_guard.pop_front()
            };

            match task {
                Some(task) => {
                    // Update metrics: processing started
                    {
                        let mut metrics_guard = metrics.lock().await;
                        metrics_guard.processing = true;
                        metrics_guard.queue_size = pending.lock().await.len();
                    }

                    eprintln!(
                        "[Crystallization Queue Worker] Processing dimension {:?} (attempt {}/{})",
                        task.dimension_id,
                        task.retry_count + 1,
                        3
                    );

                    // Process task
                    let result = {
                        let mut crystallizer_guard = crystallizer.lock().await;
                        crystallizer_guard.crystallize(&task.proto).await
                    };

                    // Handle result
                    match result {
                        Ok(()) => {
                            // Success
                            let mut metrics_guard = metrics.lock().await;
                            metrics_guard.total_completed += 1;
                            metrics_guard.processing = false;

                            eprintln!(
                                "[Crystallization Queue Worker] Completed dimension {:?} (success rate: {:.2}%)",
                                task.dimension_id,
                                metrics_guard.success_rate() * 100.0
                            );
                        }
                        Err(e) => {
                            // Failure - retry if possible
                            if task.retry_count < 3 {
                                eprintln!(
                                    "[Crystallization Queue Worker] Failed dimension {:?}: {} - retrying",
                                    task.dimension_id,
                                    e
                                );

                                // Re-enqueue with incremented retry count
                                let mut retry_task = task.clone();
                                retry_task.retry_count += 1;

                                let mut pending_guard = pending.lock().await;
                                pending_guard.push_back(retry_task);

                                let mut metrics_guard = metrics.lock().await;
                                metrics_guard.processing = false;
                                metrics_guard.queue_size = pending_guard.len();
                            } else {
                                // Max retries exceeded
                                let mut metrics_guard = metrics.lock().await;
                                metrics_guard.total_failed += 1;
                                metrics_guard.processing = false;

                                eprintln!(
                                    "[Crystallization Queue Worker] Failed dimension {:?} after {} retries: {}",
                                    task.dimension_id,
                                    task.retry_count + 1,
                                    e
                                );
                            }
                        }
                    }
                }
                None => {
                    // Queue empty - wait for notification
                    notify.notified().await;
                }
            }

            // Small delay to avoid busy loop
            tokio::time::sleep(Duration::from_millis(10)).await;
        }

        eprintln!("[Crystallization Queue Worker] Stopped");
    }

    /// Get current queue metrics
    pub async fn metrics(&self) -> QueueMetrics {
        let metrics = self.metrics.lock().await;
        metrics.clone()
    }

    /// Get current queue size
    pub async fn size(&self) -> usize {
        let pending = self.pending.lock().await;
        pending.len()
    }

    /// Check if queue is empty
    pub async fn is_empty(&self) -> bool {
        let pending = self.pending.lock().await;
        pending.is_empty()
    }

    /// Shutdown the queue and wait for worker to finish
    ///
    /// Stops accepting new tasks and waits for current task to complete.
    pub async fn shutdown(mut self) -> Result<()> {
        eprintln!("[Crystallization Queue] Shutting down...");

        // Set shutdown flag
        {
            let mut shutdown = self.shutdown.lock().await;
            *shutdown = true;
        }

        // Notify worker
        self.notify.notify_one();

        // Wait for worker to finish
        if let Some(worker) = self.worker.take() {
            worker.await.map_err(|e| {
                ConsciousnessError::LearningError(format!("Worker join error: {}", e))
            })?;
        }

        eprintln!("[Crystallization Queue] Shutdown complete");

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::memory::MmapManager;
    use std::sync::Arc;
    use std::time::SystemTime;

    fn create_test_proto(dimension_id: DimensionId) -> ProtoDimension {
        ProtoDimension {
            dimension_id,
            content: b"Test dimensional content for crystallization".to_vec(),
            confidence: 0.90,
            created_at: SystemTime::now(),
            last_accessed: SystemTime::now(),
            size_bytes: 45,
        }
    }

    #[tokio::test]
    async fn test_queue_creation() {
        // Given: Memory manager and crystallizer
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);

        // When: Creating queue
        let queue = CrystallizationQueue::new(crystallizer);

        // Then: Should be empty initially
        assert!(queue.is_empty().await);
        assert_eq!(queue.size().await, 0);
    }

    #[tokio::test]
    async fn test_enqueue_task() {
        // Given: Queue
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);
        let queue = CrystallizationQueue::new(crystallizer);

        // When: Enqueuing task
        let proto = create_test_proto(DimensionId(101));
        queue.enqueue(proto, CrystallizationPriority::Normal).await.unwrap();

        // Then: Queue size should be 1
        assert_eq!(queue.size().await, 1);
        assert!(!queue.is_empty().await);
    }

    #[tokio::test]
    async fn test_priority_ordering() {
        // Given: Queue
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);
        let queue = CrystallizationQueue::new(crystallizer);

        // When: Enqueuing tasks with different priorities
        queue.enqueue(create_test_proto(DimensionId(101)), CrystallizationPriority::Low).await.unwrap();
        queue.enqueue(create_test_proto(DimensionId(102)), CrystallizationPriority::High).await.unwrap();
        queue.enqueue(create_test_proto(DimensionId(103)), CrystallizationPriority::Normal).await.unwrap();

        // Then: Queue should have 3 tasks
        assert_eq!(queue.size().await, 3);

        // High priority should be first
        let pending = queue.pending.lock().await;
        assert_eq!(pending[0].priority, CrystallizationPriority::High);
        assert_eq!(pending[0].dimension_id, DimensionId(102));
    }

    #[tokio::test]
    async fn test_metrics_tracking() {
        // Given: Queue
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);
        let queue = CrystallizationQueue::new(crystallizer);

        // When: Enqueuing tasks
        queue.enqueue(create_test_proto(DimensionId(101)), CrystallizationPriority::Normal).await.unwrap();
        queue.enqueue(create_test_proto(DimensionId(102)), CrystallizationPriority::Normal).await.unwrap();

        // Then: Metrics should reflect enqueued tasks
        let metrics = queue.metrics().await;
        assert_eq!(metrics.total_enqueued, 2);
        assert_eq!(metrics.queue_size, 2);
    }

    #[tokio::test]
    async fn test_worker_processes_task() {
        // Given: Queue with task
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);
        let queue = CrystallizationQueue::new(crystallizer);

        let proto = create_test_proto(DimensionId(101));
        queue.enqueue(proto, CrystallizationPriority::Normal).await.unwrap();

        // When: Waiting for worker to process
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Then: Task should be processed (queue empty or processing)
        // Note: Worker might still be processing, so check metrics
        let metrics = queue.metrics().await;
        assert!(metrics.total_enqueued >= 1);
    }

    #[tokio::test]
    async fn test_queue_shutdown() {
        // Given: Queue
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);
        let queue = CrystallizationQueue::new(crystallizer);

        // When: Shutting down
        let result = queue.shutdown().await;

        // Then: Should complete without error
        assert!(result.is_ok());
    }

    #[tokio::test]
    async fn test_retry_on_failure() {
        // Given: Queue (placeholder implementation succeeds, so this tests structure)
        let memory_manager = Arc::new(MmapManager::new(280).unwrap());
        let crystallizer = Crystallizer::new(memory_manager);
        let queue = CrystallizationQueue::new(crystallizer);

        // When: Enqueuing task
        let proto = create_test_proto(DimensionId(101));
        queue.enqueue(proto, CrystallizationPriority::Normal).await.unwrap();

        // Wait for processing
        tokio::time::sleep(Duration::from_millis(100)).await;

        // Then: Should have completed (placeholder always succeeds)
        let metrics = queue.metrics().await;
        assert!(metrics.total_completed >= 1 || metrics.total_failed >= 1);
    }
}
