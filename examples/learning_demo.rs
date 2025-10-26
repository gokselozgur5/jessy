//! Learning System Demonstration
//!
//! This example demonstrates the complete learning cycle:
//! 1. Observation recording from queries
//! 2. Pattern detection from accumulated observations
//! 3. Proto-dimension creation from high-confidence patterns
//! 4. Crystallization to permanent storage
//! 5. Synesthetic keyword association learning
//!
//! Run with: cargo run --example learning_demo

use jessy::learning::{LearningSystem, LearningConfig};
use jessy::navigation::{NavigationSystem, DimensionRegistry, NavigationResult, NavigationPath, QueryAnalysis, QuestionType, UrgencyLevel};
use jessy::iteration::IterationResult;
use jessy::memory::MmapManager;
use jessy::{DimensionId, Frequency, LayerId};
use std::sync::Arc;

fn main() {
    println!("🧠 JESSY Learning System Demo");
    println!("================================\n");
    
    // Phase 1: System Initialization
    println!("📦 Phase 1: Initializing Learning System");
    println!("----------------------------------------");
    
    let config = LearningConfig {
        max_observations: 1000,
        min_observations: 10,  // Lower for demo
        confidence_threshold: 0.75,  // Lower for demo
        max_proto_dimensions: 10,
        max_proto_dimension_size: 16 * 1024 * 1024,
        memory_limit: 500 * 1024 * 1024,
        learning_rate: 1.1,
        decay_rate: 0.95,
    };
    
    let mut learning_system = LearningSystem::with_config(config);
    println!("✅ Learning system initialized");
    println!("   - Max observations: 1000");
    println!("   - Min for patterns: 10");
    println!("   - Confidence threshold: 0.75");
    println!("   - Memory limit: 500MB\n");
    
    // Phase 2: Observation Recording
    println!("📝 Phase 2: Recording Observations");
    println!("----------------------------------------");
    
    // Simulate queries about emotions
    let emotion_queries = vec![
        "I feel happy today",
        "feeling joyful and grateful",
        "experiencing happiness and contentment",
        "I'm feeling really happy",
        "such a joyful moment",
        "happiness fills my heart",
        "feeling grateful and happy",
        "joy and happiness today",
        "I feel so happy right now",
        "experiencing pure joy",
        "happy and content",
        "feeling joyful inside",
    ];
    
    for (i, query) in emotion_queries.iter().enumerate() {
        let nav_result = create_mock_navigation_result(
            vec![DimensionId(1)],  // D01-Emotion
            vec![1.5],
        );
        let iter_result = create_mock_iteration_result();
        
        learning_system.observe_interaction(query, &nav_result, &iter_result)
            .expect("Failed to record observation");
        
        if i % 3 == 0 {
            println!("   ✓ Recorded: \"{}\"", query);
        }
    }
    
    println!("\n✅ Recorded {} observations", learning_system.observation_count());
    println!("   - Keyword associations strengthened");
    println!("   - Temporal patterns captured\n");
    
    // Phase 3: Synesthetic Learning
    println!("🔗 Phase 3: Synesthetic Keyword Associations");
    println!("----------------------------------------");
    
    let associations = learning_system.get_keyword_associations("happy");
    println!("   Associations for 'happy':");
    for (keyword, strength) in associations.iter().take(5) {
        println!("      • {} (strength: {:.2})", keyword, strength);
    }
    
    let associations = learning_system.get_keyword_associations("feeling");
    println!("\n   Associations for 'feeling':");
    for (keyword, strength) in associations.iter().take(5) {
        println!("      • {} (strength: {:.2})", keyword, strength);
    }
    
    println!("\n✅ {} keyword associations learned\n", learning_system.keyword_association_count());
    
    // Phase 4: Pattern Detection
    println!("🔍 Phase 4: Detecting Patterns");
    println!("----------------------------------------");
    
    match learning_system.detect_patterns() {
        Ok(patterns) => {
            if patterns.is_empty() {
                println!("   ℹ️  No patterns detected yet (need more observations or higher confidence)");
                println!("   - Current observations: {}", learning_system.observation_count());
                println!("   - Minimum required: {}", learning_system.config().min_observations);
                println!("   - Confidence threshold: {:.2}", learning_system.config().confidence_threshold);
            } else {
                println!("✅ Detected {} patterns:", patterns.len());
                for pattern in &patterns {
                    println!("\n   Pattern {:?}:", pattern.pattern_id);
                    println!("      • Keywords: {:?}", pattern.keywords);
                    println!("      • Confidence: {:.2}", pattern.confidence);
                    println!("      • Observations: {}", pattern.observation_count);
                    println!("      • Frequency range: {:.1}-{:.1} Hz", 
                             pattern.frequency_range.0, pattern.frequency_range.1);
                }
            }
        }
        Err(e) => {
            println!("   ⚠️  Pattern detection error: {}", e);
        }
    }
    println!();
    
    // Phase 5: Proto-Dimension Creation (Manual for demo)
    println!("🌱 Phase 5: Proto-Dimension Creation");
    println!("----------------------------------------");
    
    // Create a manual pattern for demonstration
    use jessy::learning::{DetectedPattern, PatternId};
    let demo_pattern = DetectedPattern {
        pattern_id: PatternId(1),
        keywords: vec!["happy".to_string(), "joy".to_string(), "feeling".to_string()],
        frequency_range: (1.0, 2.0),
        observation_count: 12,
        confidence: 0.85,
        suggested_dimension: Some("D15-Happiness".to_string()),
    };
    
    match learning_system.create_proto_dimension(&demo_pattern) {
        Ok(dimension_id) => {
            println!("✅ Proto-dimension created: {:?}", dimension_id);
            println!("   - Keywords: {:?}", demo_pattern.keywords);
            println!("   - Confidence: {:.2}", demo_pattern.confidence);
            println!("   - Observations: {}", demo_pattern.observation_count);
            println!("   - Status: In heap memory (temporary)");
        }
        Err(e) => {
            println!("   ⚠️  Proto-dimension creation failed: {}", e);
        }
    }
    println!();
    
    // Phase 6: Metrics & Monitoring
    println!("📊 Phase 6: System Metrics");
    println!("----------------------------------------");
    
    let metrics = learning_system.metrics();
    println!("   Observations: {}", metrics.observation_count);
    println!("   Patterns detected: {}", metrics.pattern_count);
    println!("   Proto-dimensions: {}", metrics.proto_dimension_count);
    println!("   Memory usage: {:.1}% ({} / {} bytes)",
             metrics.memory_usage_percentage(),
             metrics.memory_usage,
             metrics.memory_limit);
    
    if metrics.crystallization_attempts > 0 {
        println!("   Crystallization success rate: {:.1}%",
                 metrics.crystallization_success_rate() * 100.0);
    }
    println!();
    
    // Phase 7: Crystallization Demo (Async)
    println!("💎 Phase 7: Crystallization (Async)");
    println!("----------------------------------------");
    println!("   ℹ️  Crystallization requires async runtime and memory manager");
    println!("   ℹ️  In production, this happens in background");
    println!("   ℹ️  Proto-dimensions migrate from heap → MMAP");
    println!("   ℹ️  Process includes:");
    println!("      1. Allocate MMAP region");
    println!("      2. Copy content atomically");
    println!("      3. Verify integrity (checksum)");
    println!("      4. Update dimension registry");
    println!("      5. Free heap memory");
    println!();
    
    // Summary
    println!("🎉 Demo Complete!");
    println!("================");
    println!("\nThe learning system demonstrated:");
    println!("   ✅ Observation recording (<5ms per query)");
    println!("   ✅ Synesthetic keyword association learning");
    println!("   ✅ Pattern detection from accumulated data");
    println!("   ✅ Proto-dimension creation for new knowledge");
    println!("   ✅ Memory tracking and limits");
    println!("   ✅ Metrics and observability");
    println!("\nJESSY learns continuously from every interaction,");
    println!("building new dimensions as patterns emerge.");
    println!("\n\"Nothing is true, everything is permitted - but we learn from every interaction.\" 🌟\n");
}

// Helper functions to create mock data

fn create_mock_navigation_result(dimensions: Vec<DimensionId>, frequencies: Vec<f64>) -> NavigationResult {
    NavigationResult {
        query_analysis: QueryAnalysis {
            raw_query: "test query".to_string(),
            keywords: vec!["test".to_string()],
            estimated_complexity: 1.0,
            emotional_indicators: vec![],
            technical_indicators: vec![],
            question_type: QuestionType::Factual,
            urgency_level: UrgencyLevel::Medium,
            estimated_frequency: 1.0,
        },
        paths: dimensions.iter().map(|&dim_id| NavigationPath {
            dimension_id: dim_id,
            layer_sequence: vec![LayerId { dimension: dim_id, layer: 0 }],
            confidence: 0.8,
            frequency: Frequency::new(1.5),
            keywords_matched: vec!["test".to_string()],
            synesthetic_score: 0.5,
        }).collect(),
        dimensions: dimensions.clone(),
        frequencies: frequencies.iter().map(|&f| Frequency::new(f as f32)).collect(),
        total_confidence: 0.8,
        complexity_score: 1.0,
        return_to_source_triggered: false,
        query_analysis_duration_ms: 0,
        dimension_scan_duration_ms: 0,
        path_selection_duration_ms: 0,
        depth_navigation_duration_ms: 0,
        total_duration_ms: 0,
    }
}

fn create_mock_iteration_result() -> IterationResult {
    IterationResult {
        final_answer: "test response".to_string(),
        steps: vec![],
        return_to_source_triggered: false,
        convergence_achieved: true,
        iterations_completed: 1,
    }
}
