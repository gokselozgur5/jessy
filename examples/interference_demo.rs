//! Interference Engine Demonstration
//!
//! This example demonstrates the full capabilities of the Interference Engine:
//! - Harmonic detection between frequencies
//! - Balance modulation for extreme frequencies
//! - Modulation suggestions for optimization
//! - Return-to-source detection for complexity management

use jessy::{Frequency, DimensionId};
use jessy::interference::{InterferenceEngine, FrequencyState, ModulationReason};

fn main() {
    println!("=== Interference Engine Demonstration ===\n");
    
    // Create the interference engine
    let engine = InterferenceEngine::new();
    
    // Demo 1: Harmonic Resonance
    println!("Demo 1: Harmonic Resonance");
    println!("---------------------------");
    demo_harmonic_resonance(&engine);
    println!();
    
    // Demo 2: Extreme Frequency Balance
    println!("Demo 2: Extreme Frequency Balance");
    println!("----------------------------------");
    demo_extreme_balance(&engine);
    println!();
    
    // Demo 3: Dissonant Conflict
    println!("Demo 3: Dissonant Conflict");
    println!("---------------------------");
    demo_dissonant_conflict(&engine);
    println!();
    
    // Demo 4: Complex Multi-Dimensional State
    println!("Demo 4: Complex Multi-Dimensional State");
    println!("----------------------------------------");
    demo_complex_state(&engine);
    println!();
    
    // Demo 5: Balanced Consciousness
    println!("Demo 5: Balanced Consciousness");
    println!("-------------------------------");
    demo_balanced_state(&engine);
    println!();
}

fn demo_harmonic_resonance(engine: &InterferenceEngine) {
    println!("Simulating harmonically aligned thought patterns...");
    
    let frequencies = vec![
        FrequencyState::new(Frequency::new(1.0), DimensionId(1), 0.9),  // Base frequency
        FrequencyState::new(Frequency::new(2.0), DimensionId(2), 0.8),  // Octave (2:1)
        FrequencyState::new(Frequency::new(1.5), DimensionId(3), 0.85), // Perfect 5th (3:2)
    ];
    
    let result = engine.calculate(&frequencies).unwrap();
    
    println!("Input frequencies: 1.0 Hz, 2.0 Hz, 1.5 Hz");
    println!("Dominant frequency: {:.2} Hz", result.pattern.dominant_frequency.hz());
    println!("Harmonics detected: {}", result.pattern.harmonics.len());
    
    for (i, harmonic) in result.pattern.harmonics.iter().enumerate() {
        println!("  Harmonic {}: {:?} (strength: {:.2})", 
                 i + 1, harmonic.relationship_type, harmonic.strength);
    }
    
    println!("Pattern stability: {}", if result.pattern.is_stable() { "STABLE" } else { "UNSTABLE" });
    println!("Complexity score: {:.2}", result.pattern.complexity_score);
}

fn demo_extreme_balance(engine: &InterferenceEngine) {
    println!("Simulating extreme frequency requiring balance...");
    
    let frequencies = vec![
        FrequencyState::new(Frequency::new(4.5), DimensionId(1), 0.9),  // Extreme!
        FrequencyState::new(Frequency::new(1.0), DimensionId(2), 0.7),  // Normal
    ];
    
    let result = engine.calculate(&frequencies).unwrap();
    
    println!("Input frequencies: 4.5 Hz (EXTREME), 1.0 Hz");
    println!("Dominant frequency: {:.2} Hz", result.pattern.dominant_frequency.hz());
    println!("Balance activation needed: {}", result.balance_activation_needed);
    println!("Modulation suggestions: {}", result.modulation_suggestions.len());
    
    for suggestion in &result.modulation_suggestions {
        println!("  Dimension {:?}:", suggestion.dimension_id);
        println!("    Current: {:.2} Hz → Suggested: {:.2} Hz", 
                 suggestion.current_frequency.hz(),
                 suggestion.suggested_frequency.hz());
        println!("    Reason: {:?}, Priority: {:?}, Strength: {:.2}",
                 suggestion.reason, suggestion.priority, suggestion.strength);
    }
}

fn demo_dissonant_conflict(engine: &InterferenceEngine) {
    println!("Simulating cognitive dissonance...");
    
    let frequencies = vec![
        FrequencyState::new(Frequency::new(1.0), DimensionId(1), 0.8),
        FrequencyState::new(Frequency::new(3.7), DimensionId(2), 0.8),  // Dissonant
        FrequencyState::new(Frequency::new(2.3), DimensionId(3), 0.7),  // More dissonance
    ];
    
    let result = engine.calculate(&frequencies).unwrap();
    
    println!("Input frequencies: 1.0 Hz, 3.7 Hz, 2.3 Hz");
    println!("Dominant frequency: {:.2} Hz", result.pattern.dominant_frequency.hz());
    println!("Destructive pairs: {}", result.pattern.destructive_pairs.len());
    println!("Dissonances: {}", result.pattern.dissonances.len());
    println!("Pattern stability: {}", if result.pattern.is_stable() { "STABLE" } else { "UNSTABLE" });
    
    let dissonant_suggestions: Vec<_> = result.modulation_suggestions.iter()
        .filter(|s| s.reason == ModulationReason::Dissonant)
        .collect();
    
    println!("Dissonance resolution suggestions: {}", dissonant_suggestions.len());
    for suggestion in dissonant_suggestions {
        println!("  Dimension {:?}: {:.2} Hz → {:.2} Hz",
                 suggestion.dimension_id,
                 suggestion.current_frequency.hz(),
                 suggestion.suggested_frequency.hz());
    }
}

fn demo_complex_state(engine: &InterferenceEngine) {
    println!("Simulating complex multi-dimensional activation...");
    
    let frequencies: Vec<_> = (1..=8)
        .map(|i| FrequencyState::new(
            Frequency::new(i as f32 * 0.5),
            DimensionId(i),
            0.7
        ))
        .collect();
    
    let result = engine.calculate(&frequencies).unwrap();
    
    println!("Input: 8 active dimensions");
    println!("Dominant frequency: {:.2} Hz", result.pattern.dominant_frequency.hz());
    println!("Complexity score: {:.2}", result.pattern.complexity_score);
    println!("Return-to-source suggested: {}", result.return_to_source_suggested);
    println!("Constructive pairs: {}", result.pattern.constructive_pairs.len());
    println!("Destructive pairs: {}", result.pattern.destructive_pairs.len());
    println!("Harmonics detected: {}", result.pattern.harmonics.len());
    
    if result.return_to_source_suggested {
        println!("\n⚠️  COMPLEXITY WARNING: Too many dimensions active!");
        println!("   Suggestion: Return to source (D14) to simplify");
    }
}

fn demo_balanced_state(engine: &InterferenceEngine) {
    println!("Simulating balanced, healthy consciousness state...");
    
    let frequencies = vec![
        FrequencyState::new(Frequency::new(0.8), DimensionId(1), 0.8),   // Technical
        FrequencyState::new(Frequency::new(1.2), DimensionId(2), 0.9),   // Emotional
        FrequencyState::new(Frequency::new(0.6), DimensionId(3), 0.7),   // Philosophical
        FrequencyState::new(Frequency::new(1.2), DimensionId(13), 0.85), // Balance
    ];
    
    let result = engine.calculate(&frequencies).unwrap();
    
    println!("Input frequencies: 0.8 Hz, 1.2 Hz, 0.6 Hz, 1.2 Hz (Balance)");
    println!("Dominant frequency: {:.2} Hz", result.pattern.dominant_frequency.hz());
    println!("Balance activation needed: {}", result.balance_activation_needed);
    println!("Pattern stability: {}", if result.pattern.is_stable() { "STABLE" } else { "UNSTABLE" });
    println!("Complexity score: {:.2}", result.pattern.complexity_score);
    println!("Modulation suggestions: {}", result.modulation_suggestions.len());
    
    println!("\n✓ Healthy consciousness state detected!");
    println!("  - All frequencies in normal range (0.6-1.8 Hz)");
    println!("  - Balance dimension active");
    println!("  - Low complexity");
    println!("  - Stable pattern");
}
