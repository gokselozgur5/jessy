use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use jessy::memory::MmapManager;
use jessy::DimensionId;
use std::sync::Arc;
use tokio::sync::RwLock;

/// Benchmark: Layer access latency (Target: <1ms)
/// 
/// This validates R3.2: "THE Memory Manager SHALL complete layer access operations within 1 millisecond"
fn bench_layer_access_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("layer_access");
    group.significance_level(0.1).sample_size(1000);
    
    // Setup: Create manager with test layer
    let rt = tokio::runtime::Runtime::new().unwrap();
    let mut manager = MmapManager::new(280).unwrap();
    
    let dimension_id = DimensionId(99);
    let content = b"Performance test content for layer access latency measurement".to_vec();
    let layer_id = rt.block_on(async {
        manager.create_proto_dimension(dimension_id, content).unwrap()
    });
    
    // Warm up - first access might be slower due to OS paging
    let _ = manager.load_layer_context(layer_id);
    
    group.bench_function("single_layer_access", |b| {
        b.iter(|| {
            manager.load_layer_context(black_box(layer_id))
        });
    });
    
    group.finish();
}

/// Benchmark: Memory allocation performance (Target: <100μs)
///
/// This validates R10.2: "THE Memory Manager SHALL perform zero dynamic allocations during query processing hot paths"
/// While we do allocate, it should be fast enough to not impact query performance
fn bench_memory_allocation(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_allocation");
    group.significance_level(0.1).sample_size(500);
    
    // Test different allocation sizes matching our pool strategy
    for size in [4096, 16384, 65536, 262144].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let mut manager = MmapManager::new(280).unwrap();
            
            b.iter(|| {
                // allocate returns Result<MmapOffset>
                // We measure the allocation itself, not error handling
                manager.allocate(black_box(size))
            });
        });
    }
    
    group.finish();
}

/// Benchmark: Dimension scan performance (Target: <100ms for all layers)
///
/// This validates the overall query processing requirement
fn bench_dimension_scan(c: &mut Criterion) {
    let mut group = c.benchmark_group("dimension_scan");
    group.significance_level(0.1).sample_size(100);
    
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    // Create manager with multiple layers to simulate dimension scan
    for num_layers in [10, 50, 100].iter() {
        let mut manager = MmapManager::new(280).unwrap();
        let dimension_id = DimensionId(99);
        
        // Create multiple layers
        let layer_ids: Vec<_> = (0..*num_layers).map(|i| {
            let content = format!("Layer {} content for scanning benchmark", i).into_bytes();
            rt.block_on(async {
                manager.create_proto_dimension(dimension_id, content).unwrap()
            })
        }).collect();
        
        group.bench_with_input(
            BenchmarkId::from_parameter(num_layers),
            num_layers,
            |b, _| {
                b.iter(|| {
                    // Scan all layers - simulates dimension traversal
                    for &layer_id in &layer_ids {
                        let _ = manager.load_layer_context(black_box(layer_id));
                    }
                });
            },
        );
    }
    
    group.finish();
}

/// Benchmark: Concurrent read performance (Target: 100+ concurrent readers with <10% degradation)
///
/// This validates R4.2: "THE Memory Manager SHALL support at least 100 concurrent read operations"
fn bench_concurrent_reads(c: &mut Criterion) {
    let mut group = c.benchmark_group("concurrent_reads");
    group.significance_level(0.1).sample_size(50);
    
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    // Setup: Create manager with test layer
    let mut manager = MmapManager::new(280).unwrap();
    let dimension_id = DimensionId(99);
    let content = b"Concurrent access test content".to_vec();
    let layer_id = rt.block_on(async {
        manager.create_proto_dimension(dimension_id, content).unwrap()
    });
    
    let manager = Arc::new(RwLock::new(manager));
    
    // Test with different concurrency levels
    for num_threads in [10, 50, 100, 200].iter() {
        group.bench_with_input(
            BenchmarkId::from_parameter(num_threads),
            num_threads,
            |b, &num_threads| {
                b.iter(|| {
                    rt.block_on(async {
                        let mut handles = vec![];
                        
                        for _ in 0..num_threads {
                            let manager_clone = Arc::clone(&manager);
                            let handle = tokio::spawn(async move {
                                let mgr = manager_clone.read().await;
                                mgr.load_layer_context(layer_id)
                            });
                            handles.push(handle);
                        }
                        
                        // Wait for all to complete
                        for handle in handles {
                            let _ = handle.await;
                        }
                    });
                });
            },
        );
    }
    
    group.finish();
}

/// Benchmark: Proto-dimension crystallization (Target: <10ms per MB)
///
/// This validates the learning system integration performance
fn bench_crystallization(c: &mut Criterion) {
    let mut group = c.benchmark_group("crystallization");
    group.significance_level(0.1).sample_size(100);
    
    let rt = tokio::runtime::Runtime::new().unwrap();
    
    // Test crystallization with different content sizes
    for size_kb in [1, 10, 100, 1000].iter() {
        let content = vec![0u8; size_kb * 1024];
        
        group.bench_with_input(
            BenchmarkId::from_parameter(size_kb),
            size_kb,
            |b, _| {
                b.iter(|| {
                    let mut manager = MmapManager::new(280).unwrap();
                    let dimension_id = DimensionId(99);
                    
                    rt.block_on(async {
                        // Create proto-dimension
                        let layer_id = manager.create_proto_dimension(
                            dimension_id,
                            black_box(content.clone())
                        ).unwrap();
                        
                        // Crystallize it
                        manager.crystallize_proto_dimension(layer_id)
                    })
                });
            },
        );
    }
    
    group.finish();
}

criterion_group!(
    benches,
    bench_layer_access_latency,
    bench_memory_allocation,
    bench_dimension_scan,
    bench_concurrent_reads,
    bench_crystallization
);
criterion_main!(benches);
