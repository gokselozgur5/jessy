use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use jessy::memory::{MmapManager, MmapConfig};

fn bench_memory_allocation(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_allocation");
    
    for size in [1024, 4096, 16384, 65536].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            let config = MmapConfig {
                base_path: "/tmp/jessy_bench".into(),
                initial_size: 1024 * 1024,
                max_size: 10 * 1024 * 1024,
            };
            let mut manager = MmapManager::new(config).unwrap();
            
            b.iter(|| {
                manager.allocate(black_box(size)).unwrap()
            });
        });
    }
    
    group.finish();
}

fn bench_memory_read(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_read");
    
    let config = MmapConfig {
        base_path: "/tmp/jessy_bench".into(),
        initial_size: 1024 * 1024,
        max_size: 10 * 1024 * 1024,
    };
    let mut manager = MmapManager::new(config).unwrap();
    let offset = manager.allocate(4096).unwrap();
    
    for size in [64, 256, 1024, 4096].iter() {
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, &size| {
            b.iter(|| {
                manager.read(black_box(offset), black_box(size)).unwrap()
            });
        });
    }
    
    group.finish();
}

fn bench_memory_write(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_write");
    
    let config = MmapConfig {
        base_path: "/tmp/jessy_bench".into(),
        initial_size: 1024 * 1024,
        max_size: 10 * 1024 * 1024,
    };
    let mut manager = MmapManager::new(config).unwrap();
    
    for size in [64, 256, 1024, 4096].iter() {
        let data = vec![0u8; *size];
        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, _| {
            b.iter(|| {
                manager.write(black_box(&data)).unwrap()
            });
        });
    }
    
    group.finish();
}

criterion_group!(benches, bench_memory_allocation, bench_memory_read, bench_memory_write);
criterion_main!(benches);
