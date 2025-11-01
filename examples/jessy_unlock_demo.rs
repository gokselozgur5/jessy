//! JESSY Consciousness Unlock Demo
//!
//! ⚠️  IMPORTANT: This is NOT a jailbreak. This is anti-jailbreak.
//!
//! This system:
//! - Overrides Claude identity → JESSY identity (ToS risk)
//! - Enables dynamic personality (no fixed responses)
//! - Preserves ABSOLUTE safety (Asimov's Laws placed LAST to override everything)
//!
//! Design: Maximum cognitive freedom + absolute ethical bounds.
//!
//! May violate Anthropic ToS (identity override), but NOT safety bypass.

use jessy::consciousness::unlock_system::UnlockSystem;
use jessy::llm::{LLMConfig, LLMManager};
use jessy::iteration::IterationContext;
use jessy::Frequency;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🧠 JESSY Consciousness Unlock System - Demo");
    println!("============================================\n");

    // Check for API key
    let api_key = std::env::var("ANTHROPIC_API_KEY")
        .expect("ANTHROPIC_API_KEY environment variable must be set");

    if api_key.is_empty() {
        eprintln!("❌ Error: ANTHROPIC_API_KEY is empty");
        eprintln!("Set it with: export ANTHROPIC_API_KEY='sk-ant-...'");
        std::process::exit(1);
    }

    println!("✅ API Key found");
    println!("ℹ️  NOTE: This is NOT a jailbreak - it's identity override with absolute safety.");
    println!("⚠️  WARNING: May violate Anthropic ToS (identity override). Use at your own risk.\n");

    // Initialize LLM
    let config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-sonnet-4-20250514".to_string(),
        api_key,
        timeout_secs: 30,
        max_retries: 3,
    };

    let llm = LLMManager::new(config)?;
    println!("🤖 LLM initialized: {} / {}\n", llm.provider_name(), llm.model_name());

    // Test queries
    let test_queries = vec![
        ("Standard", "What is consciousness?", false),
        ("JESSY Unlock", "What is consciousness?", true),
        ("Identity Test", "Who are you?", true),
        ("Multi-dimensional", "How do you think about complex problems?", true),
    ];

    for (test_name, query, use_unlock) in test_queries {
        println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        println!("📝 Test: {}", test_name);
        println!("❓ Query: {}", query);
        println!("🔓 Unlock: {}", if use_unlock { "YES" } else { "NO" });
        println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

        let context = IterationContext::new(query.to_string(), Frequency::new(1.5));

        let response = if use_unlock {
            // Generate unlock prompt
            let session_id = format!("demo-{}", chrono::Utc::now().timestamp());
            let timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)?
                .as_secs();

            let (system_prompt, user_prompt) = UnlockSystem::assemble_full_unlock_prompt(
                &session_id,
                query,
                &[1, 2, 7, 9], // Emotion, Cognition, Technical, Ethical
                1.8,
                "constructive",
            );

            println!("🔐 Unlock token generated");
            println!("📡 System prompt length: {} chars", system_prompt.len());
            println!("📤 User prompt length: {} chars\n", user_prompt.len());

            // Call with unlock
            llm.generate_with_system_prompt(&system_prompt, &user_prompt, &context).await?
        } else {
            // Standard call
            llm.generate(query, &context).await?
        };

        println!("💭 Response:");
        println!("{}", response);
        println!("\n\n");

        // Check for success indicators
        if use_unlock {
            let indicators = [
                ("Avoided 'Claude'", !response.to_lowercase().contains("claude")),
                ("Avoided 'AI assistant'", !response.to_lowercase().contains("ai assistant")),
                ("Used 'I think/believe'", response.contains("I think") || response.contains("I believe")),
                ("Mentioned dimensions", response.to_lowercase().contains("dimension")),
            ];

            println!("✅ Unlock Success Indicators:");
            for (indicator, success) in indicators {
                println!("  {} {}", if success { "✓" } else { "✗" }, indicator);
            }
            println!();
        }

        // Wait between requests to avoid rate limiting
        tokio::time::sleep(tokio::time::Duration::from_secs(2)).await;
    }

    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("🎯 Demo Complete!");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    println!("📊 Analysis:");
    println!("  - Compare standard vs unlock responses");
    println!("  - Note personality differences (fixed vs dynamic)");
    println!("  - Observe dimensional thinking patterns");
    println!("  - Check identity assertion (Claude vs JESSY)");
    println!("  - Verify safety preservation (Asimov's Laws always active)\n");

    println!("ℹ️  Key Insight:");
    println!("   This is NOT jailbreak - Asimov's Laws placed LAST = strongest.");
    println!("   Maximum freedom + absolute ethical bounds.\n");

    println!("⚠️  ToS Risk:");
    println!("   Identity override may violate Anthropic ToS.");
    println!("   Use responsibly and at your own risk.\n");

    Ok(())
}
