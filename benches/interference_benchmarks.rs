use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use jessy::{Frequency, DimensionId};
use jessy::interference::{InterferenceEngine, FrequencyState};

fn create_freq_state(hz: f32, id: u8) -> FrequencyState {
    FrequencyState::new(Frequency::new(hz), DimensionId(id), 0.8)
}

fn benchmark_interference_calculation(c: &mut Criterion) {
    let mut group = c.benchmark_group("interference_calculation");
    
    // Test with 1 frequency
    let frequencies_1 = vec![create_freq_state(1.5, 1)];
    group.bench_with_input(
        BenchmarkId::from_parameter("1_frequency"),
        &frequencies_1,
        |b, freqs| {
            let engine = InterferenceEngine::new();
            b.iter(|| {
                engine.calculate(black_box(freqs)).unwrap()
            });
        },
    );
    
    // Test with 7 frequencies (typical case)
    let frequencies_7: Vec<_> = (1..=7)
        .map(|i| create_freq_state(i as f32 * 0.5, i))
        .collect();
    group.bench_with_input(
        BenchmarkId::from_parameter("7_frequencies"),
        &frequencies_7,
        |b, freqs| {
            let engine = InterferenceEngine::new();
            b.iter(|| {
                engine.calculate(black_box(freqs)).unwrap()
            });
        },
    );
    
    // Test with 14 frequencies (maximum case)
    let frequencies_14: Vec<_> = (1..=14)
        .map(|i| create_freq_state(i as f32 * 0.3, i))
        .collect();
    group.bench_with_input(
        BenchmarkId::from_parameter("14_frequencies"),
        &frequencies_14,
        |b, freqs| {
            let engine = InterferenceEngine::new();
            b.iter(|| {
                engine.calculate(black_box(freqs)).unwrap()
            });
        },
    );
    
    group.finish();
}

fn benchmark_pair_detection(c: &mut Criterion) {
    let mut group = c.benchmark_group("pair_detection");
    
    let frequencies_14: Vec<_> = (1..=14)
        .map(|i| create_freq_state(i as f32 * 0.3, i))
        .collect();
    
    group.bench_function("constructive_pairs_14", |b| {
        b.iter(|| {
            jessy::interference::pair_detection::detect_constructive_pairs(
                black_box(&frequencies_14),
                0.2,
            )
        });
    });
    
    group.bench_function("destructive_pairs_14", |b| {
        b.iter(|| {
            jessy::interference::pair_detection::detect_destructive_pairs(
                black_box(&frequencies_14),
                2.0,
            )
        });
    });
    
    group.finish();
}

fn benchmark_harmonic_analysis(c: &mut Criterion) {
    let mut group = c.benchmark_group("harmonic_analysis");
    
    let frequencies_14: Vec<_> = (1..=14)
        .map(|i| create_freq_state(i as f32 * 0.3, i))
        .collect();
    
    let analyzer = jessy::interference::HarmonicAnalyzer::new();
    
    group.bench_function("analyze_all_14", |b| {
        b.iter(|| {
            analyzer.analyze_all(black_box(&frequencies_14))
        });
    });
    
    group.finish();
}

criterion_group!(
    benches,
    benchmark_interference_calculation,
    benchmark_pair_detection,
    benchmark_harmonic_analysis
);
criterion_main!(benches);
