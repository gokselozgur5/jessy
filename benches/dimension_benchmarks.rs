use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use jessy::dimensions::{DimensionRegistry, Dimension, DimensionId};

fn bench_dimension_creation(c: &mut Criterion) {
    c.bench_function("dimension_creation", |b| {
        let mut registry = DimensionRegistry::new();
        let mut counter = 0u64;
        
        b.iter(|| {
            let id = DimensionId(counter);
            counter += 1;
            registry.create_dimension(black_box(id), black_box(1.0))
        });
    });
}

fn bench_dimension_lookup(c: &mut Criterion) {
    let mut group = c.benchmark_group("dimension_lookup");
    
    for num_dimensions in [10, 100, 1000].iter() {
        let mut registry = DimensionRegistry::new();
        
        // Pre-populate registry
        for i in 0..*num_dimensions {
            registry.create_dimension(DimensionId(i), 1.0).unwrap();
        }
        
        group.bench_with_input(
            BenchmarkId::from_parameter(num_dimensions),
            num_dimensions,
            |b, &num| {
                b.iter(|| {
                    let id = DimensionId(black_box(num / 2));
                    registry.get_dimension(&id)
                });
            },
        );
    }
    
    group.finish();
}

fn bench_dimension_scan(c: &mut Criterion) {
    let mut group = c.benchmark_group("dimension_scan");
    
    for num_dimensions in [10, 100, 1000].iter() {
        let mut registry = DimensionRegistry::new();
        
        // Pre-populate registry
        for i in 0..*num_dimensions {
            registry.create_dimension(DimensionId(i), (i as f64) / 100.0).unwrap();
        }
        
        group.bench_with_input(
            BenchmarkId::from_parameter(num_dimensions),
            num_dimensions,
            |b, _| {
                b.iter(|| {
                    registry.scan_dimensions(black_box(0.5), black_box(0.1))
                });
            },
        );
    }
    
    group.finish();
}

criterion_group!(benches, bench_dimension_creation, bench_dimension_lookup, bench_dimension_scan);
criterion_main!(benches);
