//! Diagnostic utilities for memory manager
//!
//! Provides comprehensive state dumps for debugging and monitoring

use serde::{Serialize, Deserialize};
use std::collections::HashMap;
use crate::{LayerId, DimensionId};

/// Complete memory manager state snapshot
///
/// This structure captures everything needed to diagnose issues:
/// - All loaded regions and their metadata
/// - Complete layer index
/// - Pool allocator state
/// - Current memory usage
/// - Historical statistics
#[derive(Debug, Serialize, Deserialize)]
pub struct MemoryStateDump {
    pub timestamp: String,
    pub regions: Vec<RegionDump>,
    pub layer_index: Vec<LayerIndexEntry>,
    pub pool_state: PoolStateDump,
    pub memory_usage: MemoryUsageDump,
    pub statistics: StatisticsDump,
}

/// Region state snapshot
#[derive(Debug, Serialize, Deserialize)]
pub struct RegionDump {
    pub region_id: u32,
    pub dimension_id: u8,
    pub dimension_name: String,
    pub file_path: String,
    pub size_bytes: usize,
    pub layer_count: usize,
    pub layers: Vec<LayerDump>,
}

/// Layer information in region
#[derive(Debug, Serialize, Deserialize)]
pub struct LayerDump {
    pub layer_id: String,
    pub name: String,
    pub offset: usize,
    pub size: usize,
    pub frequency: f64,
    pub keywords: Vec<String>,
}

/// Layer index entry
#[derive(Debug, Serialize, Deserialize)]
pub struct LayerIndexEntry {
    pub layer_id: String,
    pub dimension: u8,
    pub layer: u16,  // Changed from u8 to match LayerId.layer type
    pub location_type: String, // "MMAP", "Heap", "Hybrid"
    pub region_id: u32,
    pub offset: usize,
    pub size: usize,
}

/// Pool allocator state
#[derive(Debug, Serialize, Deserialize)]
pub struct PoolStateDump {
    pub total_pools: usize,
    pub pools: Vec<PoolDump>,
}

/// Individual pool state
#[derive(Debug, Serialize, Deserialize)]
pub struct PoolDump {
    pub pool_id: usize,
    pub block_size: usize,
    pub total_blocks: usize,
    pub used_blocks: usize,
    pub free_blocks: usize,
    pub utilization_percent: f32,
    pub total_size_mb: f32,
}

/// Memory usage snapshot
#[derive(Debug, Serialize, Deserialize)]
pub struct MemoryUsageDump {
    pub total_limit_mb: usize,
    pub current_allocated_mb: usize,
    pub available_mb: usize,
    pub utilization_percent: f32,
    pub fragmentation_ratio: f32,
}

/// Historical statistics
#[derive(Debug, Serialize, Deserialize)]
pub struct StatisticsDump {
    pub total_allocations: usize,
    pub total_deallocations: usize,
    pub allocation_failures: usize,
    pub peak_concurrent_readers: usize,
    pub regions_loaded: usize,
    pub layers_indexed: usize,
}

impl MemoryStateDump {
    /// Create a new empty dump
    pub fn new() -> Self {
        Self {
            timestamp: chrono::Utc::now().to_rfc3339(),
            regions: Vec::new(),
            layer_index: Vec::new(),
            pool_state: PoolStateDump {
                total_pools: 0,
                pools: Vec::new(),
            },
            memory_usage: MemoryUsageDump {
                total_limit_mb: 0,
                current_allocated_mb: 0,
                available_mb: 0,
                utilization_percent: 0.0,
                fragmentation_ratio: 0.0,
            },
            statistics: StatisticsDump {
                total_allocations: 0,
                total_deallocations: 0,
                allocation_failures: 0,
                peak_concurrent_readers: 0,
                regions_loaded: 0,
                layers_indexed: 0,
            },
        }
    }
    
    /// Serialize to JSON string
    pub fn to_json(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }
    
    /// Serialize to JSON file
    pub fn to_json_file(&self, path: &std::path::Path) -> Result<(), Box<dyn std::error::Error>> {
        let json = self.to_json()?;
        std::fs::write(path, json)?;
        Ok(())
    }
    
    /// Get summary statistics
    pub fn summary(&self) -> String {
        format!(
            "Memory State Dump @ {}\n\
             Regions: {} | Layers: {} | Pools: {}\n\
             Memory: {}/{} MB ({:.1}%)\n\
             Allocations: {} | Failures: {} | Peak Readers: {}",
            self.timestamp,
            self.regions.len(),
            self.layer_index.len(),
            self.pool_state.total_pools,
            self.memory_usage.current_allocated_mb,
            self.memory_usage.total_limit_mb,
            self.memory_usage.utilization_percent,
            self.statistics.total_allocations,
            self.statistics.allocation_failures,
            self.statistics.peak_concurrent_readers
        )
    }
}

impl Default for MemoryStateDump {
    fn default() -> Self {
        Self::new()
    }
}
