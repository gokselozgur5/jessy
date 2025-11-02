//! Test helpers and fixtures for memory module testing
//!
//! Provides common utilities, mock data generators, and test fixtures
//! for unit, integration, and performance tests in the memory subsystem.

use crate::{DimensionId, LayerId, MemoryConfig, MemoryStats, Result};
use std::path::PathBuf;
use tempfile::TempDir;

/// Test fixture for creating temporary test directories
pub struct TestEnvironment {
    pub temp_dir: TempDir,
    pub data_dir: PathBuf,
    pub mmap_dir: PathBuf,
}

impl TestEnvironment {
    /// Create a new test environment with temporary directories
    pub fn new() -> Result<Self> {
        let temp_dir = tempfile::tempdir()
            .map_err(|e| crate::ConsciousnessError::MemoryError(
                format!("Failed to create temp directory: {}", e)
            ))?;

        let data_dir = temp_dir.path().join("data");
        let mmap_dir = data_dir.join("mmap");

        std::fs::create_dir_all(&data_dir)
            .map_err(|e| crate::ConsciousnessError::MemoryError(
                format!("Failed to create data directory: {}", e)
            ))?;

        std::fs::create_dir_all(&mmap_dir)
            .map_err(|e| crate::ConsciousnessError::MemoryError(
                format!("Failed to create mmap directory: {}", e)
            ))?;

        Ok(Self {
            temp_dir,
            data_dir,
            mmap_dir,
        })
    }

    /// Get path to MMAP directory
    pub fn mmap_path(&self) -> &PathBuf {
        &self.mmap_dir
    }

    /// Get path to data directory
    pub fn data_path(&self) -> &PathBuf {
        &self.data_dir
    }

    /// Create a test MMAP file for a dimension
    pub fn create_test_mmap_file(&self, dim_id: DimensionId, size_mb: usize) -> Result<PathBuf> {
        let dim_dir = self.mmap_dir.join(format!("D{:02}", dim_id.0));
        std::fs::create_dir_all(&dim_dir)
            .map_err(|e| crate::ConsciousnessError::MemoryError(
                format!("Failed to create dimension directory: {}", e)
            ))?;

        let file_path = dim_dir.join("region.mmap");
        let size_bytes = size_mb * 1024 * 1024;

        // Create file with test data
        let test_data = generate_test_dimension_data(dim_id, size_bytes);
        std::fs::write(&file_path, test_data)
            .map_err(|e| crate::ConsciousnessError::MemoryError(
                format!("Failed to write test MMAP file: {}", e)
            ))?;

        Ok(file_path)
    }
}

/// Generate test dimension data with realistic structure
fn generate_test_dimension_data(dim_id: DimensionId, size_bytes: usize) -> Vec<u8> {
    let mut data = Vec::with_capacity(size_bytes);

    // Header
    let header = format!(
        "DIMENSION:{}|NAME:{}|SIZE:{}|LAYERS:4\n",
        dim_id.0,
        get_dimension_name(dim_id),
        size_bytes
    );
    data.extend_from_slice(header.as_bytes());

    // Layer metadata
    for layer_num in 0..4 {
        let layer_header = format!(
            "LAYER:{}|DEPTH:{}|FREQ:{:.2}|KEYWORDS:test,keywords,layer{}\n",
            layer_num,
            layer_num,
            2.0 + (layer_num as f32 * 0.15),
            layer_num
        );
        data.extend_from_slice(layer_header.as_bytes());

        // Layer content
        let content = format!(
            "# Test Layer {} Content\n\nThis is test data for dimension {} layer {}.\n\n",
            layer_num, dim_id.0, layer_num
        );
        data.extend_from_slice(content.as_bytes());
    }

    // Pad to requested size
    while data.len() < size_bytes {
        data.push(0);
    }

    data.truncate(size_bytes);
    data
}

fn get_dimension_name(dim_id: DimensionId) -> &'static str {
    match dim_id.0 {
        1 => "Emotion",
        2 => "Cognition",
        3 => "Intention",
        4 => "Social",
        5 => "Temporal",
        6 => "Philosophy",
        7 => "Technical",
        8 => "Creative",
        9 => "Ethical",
        10 => "Meta",
        11 => "Ecological",
        12 => "Positivity",
        13 => "Balance",
        14 => "Security",
        _ => "Unknown",
    }
}

/// Create a test MemoryConfig with small sizes for testing
pub fn test_memory_config() -> MemoryConfig {
    MemoryConfig {
        total_memory_mb: 10, // Small for fast tests
        base_path: PathBuf::from("test_data"),
        pool_configs: vec![
            (2, 4096),   // 2MB of 4KB blocks
            (4, 16384),  // 4MB of 16KB blocks
            (2, 65536),  // 2MB of 64KB blocks
            (2, 262144), // 2MB of 256KB blocks
        ],
        enable_warnings: false, // Disable warnings in tests
        warning_threshold: 75.0,
        eviction_threshold: 85.0,
        critical_threshold: 95.0,
    }
}

/// Create a MemoryStats with test data
pub fn test_memory_stats() -> MemoryStats {
    MemoryStats {
        total_allocated_bytes: 10 * 1024 * 1024,
        total_used_bytes: 5 * 1024 * 1024,
        dimensions_loaded: 3,
        layers_loaded: 12,
        active_regions: 3,
        peak_usage_bytes: 6 * 1024 * 1024,
        allocation_count: 150,
        allocation_failures: 2,
        avg_access_latency_us: 0.5,
        simulation_mode: true,
        usage_percentage: 50.0,
        pool_stats: vec![
            (4096, 100, 412),
            (16384, 50, 206),
            (65536, 20, 12),
            (262144, 5, 3),
        ],
    }
}

/// Generate test layer IDs for a dimension
pub fn test_layer_ids(dim_id: DimensionId, count: usize) -> Vec<LayerId> {
    (0..count)
        .map(|i| LayerId {
            dimension: dim_id,
            layer: i as u16,
        })
        .collect()
}

/// Generate test dimension IDs
pub fn test_dimension_ids(count: usize) -> Vec<DimensionId> {
    (1..=count).map(|i| DimensionId(i as u8)).collect()
}

/// Assert that memory usage is within expected range
#[macro_export]
macro_rules! assert_memory_usage {
    ($stats:expr, $min_mb:expr, $max_mb:expr) => {
        let used_mb = $stats.total_used_bytes as f64 / (1024.0 * 1024.0);
        assert!(
            used_mb >= $min_mb && used_mb <= $max_mb,
            "Memory usage {:.2}MB not in range [{}, {}]",
            used_mb,
            $min_mb,
            $max_mb
        );
    };
}

/// Assert that access latency is below threshold
#[macro_export]
macro_rules! assert_latency {
    ($stats:expr, $max_us:expr) => {
        assert!(
            $stats.avg_access_latency_us <= $max_us,
            "Average latency {:.2}μs exceeds threshold {}μs",
            $stats.avg_access_latency_us,
            $max_us
        );
    };
}

/// Assert that allocation succeeded
#[macro_export]
macro_rules! assert_allocation_success {
    ($result:expr) => {
        assert!($result.is_ok(), "Allocation failed: {:?}", $result.err());
    };
}

/// Assert that allocation failed with expected error
#[macro_export]
macro_rules! assert_allocation_failure {
    ($result:expr) => {
        assert!($result.is_err(), "Expected allocation to fail");
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_environment_creation() {
        let env = TestEnvironment::new().expect("Failed to create test environment");
        assert!(env.data_path().exists());
        assert!(env.mmap_path().exists());
    }

    #[test]
    fn test_mmap_file_creation() {
        let env = TestEnvironment::new().unwrap();
        let dim_id = DimensionId(1);
        let path = env.create_test_mmap_file(dim_id, 1).unwrap();

        assert!(path.exists());
        let metadata = std::fs::metadata(&path).unwrap();
        assert_eq!(metadata.len(), 1024 * 1024); // 1MB
    }

    #[test]
    fn test_memory_config_validation() {
        let config = test_memory_config();
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_stats_calculations() {
        let mut stats = test_memory_stats();
        stats.calculate_usage_percentage(10 * 1024 * 1024);
        assert_eq!(stats.usage_percentage, 50.0);
    }

    #[test]
    fn test_layer_id_generation() {
        let dim_id = DimensionId(2);
        let layers = test_layer_ids(dim_id, 4);
        assert_eq!(layers.len(), 4);
        assert_eq!(layers[0].dimension, dim_id);
        assert_eq!(layers[0].layer, 0);
        assert_eq!(layers[3].layer, 3);
    }

    #[test]
    fn test_dimension_id_generation() {
        let dims = test_dimension_ids(5);
        assert_eq!(dims.len(), 5);
        assert_eq!(dims[0].0, 1);
        assert_eq!(dims[4].0, 5);
    }
}
