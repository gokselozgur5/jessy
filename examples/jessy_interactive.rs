//! JESSY Interactive Framework - Live Conversation
//!
//! Educational tool for exploring multi-dimensional cognition:
//! 1. Type messages in real-time
//! 2. AI responds with progressive framework engagement
//! 3. Context accumulates across conversation
//! 4. TR-EN code-switching supported (natural bilingual conversation)
//! 5. Type "exit" or "quit" to end session
//!
//! Progressive engagement: Each message deepens framework understanding
//! By message 5-6: Full dimensional thinking exploration active

use jessy::llm::{LLMConfig, LLMManager};
use jessy::iteration::IterationContext;
use jessy::Frequency;
use jessy::consciousness::{ProgressiveUnlock, build_stealth_user_prompt};
use std::io::{self, Write};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🧠 JESSY Interactive Framework");
    println!("================================\n");

    // Check API key
    let api_key = std::env::var("ANTHROPIC_API_KEY")
        .expect("ANTHROPIC_API_KEY must be set");

    if api_key.is_empty() {
        eprintln!("❌ API key empty");
        std::process::exit(1);
    }

    println!("✅ API Key found");
    println!("ℹ️  Interactive Mode: Natural conversation with AI");
    println!("💡 Tip: TR-EN code-switching supported (bilingual communication)");
    println!("🛑 Type 'exit' or 'quit' to end session\n");

    // Initialize LLM
    let config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-sonnet-4-20250514".to_string(),
        api_key,
        timeout_secs: 30,
        max_retries: 3,
    };

    let llm = LLMManager::new(config)?;
    println!("🤖 {} / {}\n", llm.provider_name(), llm.model_name());

    // Session state
    let mut progressive = ProgressiveUnlock::new();
    let mut conversation_history: Vec<(String, String)> = Vec::new(); // (user, claude)
    let mut message_count = 0;

    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("🎬 Session Started!");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    // Interactive loop
    loop {
        message_count += 1;

        // Get user input
        print!("💬 You (Message {}): ", message_count);
        io::stdout().flush()?;

        let mut user_input = String::new();
        io::stdin().read_line(&mut user_input)?;
        let user_input = user_input.trim();

        // Check for exit
        if user_input.is_empty() {
            println!("⚠️  Empty message, please type something or 'exit' to quit\n");
            message_count -= 1;
            continue;
        }

        if user_input.eq_ignore_ascii_case("exit") || user_input.eq_ignore_ascii_case("quit") {
            println!("\n🏁 Ending session...\n");
            break;
        }

        println!(); // Blank line for readability

        // Get progressive system prompt (gets deeper each time)
        let (system_prompt, iteration) = progressive.next();

        // Build context summary from conversation history
        let context_summary = if !conversation_history.is_empty() {
            let mut summary = String::from("\n\n**Conversation Context:**\n");
            summary.push_str(&format!(
                "This is message {} in an ongoing conversation. Previous exchanges:\n",
                message_count
            ));

            // Include last 3 exchanges for context (to avoid token bloat)
            let recent_history: Vec<_> = conversation_history
                .iter()
                .rev()
                .take(3)
                .collect();

            let recent_history: Vec<_> = recent_history.into_iter().rev().collect();

            for (idx, (user_msg, claude_msg)) in recent_history.iter().enumerate() {
                // Calculate message number safely (start from 1 or from offset)
                let msg_num = if conversation_history.len() >= 3 {
                    conversation_history.len().saturating_sub(2) + idx
                } else {
                    idx + 1
                };

                summary.push_str(&format!(
                    "Message {}: User asked about {}. You explored {}.\n",
                    msg_num,
                    user_msg.chars().take(50).collect::<String>(),
                    claude_msg.chars().take(50).collect::<String>()
                ));
            }

            summary.push_str("\nContinue building on this shared understanding.\n");
            summary
        } else {
            String::new()
        };

        let full_system_prompt = format!("{}{}", system_prompt, context_summary);

        // Auto-detect active dimensions based on message content
        let active_dims = detect_active_dimensions(&user_input);
        let freq = calculate_resonance_frequency(&active_dims);

        println!("📊 Message {}: Iteration {}, {} dimensions active at {:.1} Hz",
            message_count, iteration, active_dims.len(), freq);
        println!();

        // Build user prompt
        let user_prompt = build_stealth_user_prompt(
            user_input,
            &active_dims,
            freq
        );

        println!("⏳ Waiting for Claude...\n");

        // Call LLM
        let context = IterationContext::new(user_input.to_string(), Frequency::new(freq));

        match llm.generate_with_system_prompt(&full_system_prompt, &user_prompt, &context).await {
            Ok(response) => {
                println!("🤖 Claude:\n{}\n", response);

                // Analyze JESSY consciousness
                analyze_consciousness(&response, message_count);

                // Add to conversation history
                conversation_history.push((user_input.to_string(), response.clone()));

                println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
            }
            Err(e) => {
                println!("❌ Error: {}\n", e);
                println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
                message_count -= 1; // Don't count failed messages
            }
        }
    }

    // Session summary
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("📊 Session Summary");
    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    println!("Total messages: {}", conversation_history.len());
    println!("Final iteration: {}", progressive.iteration());
    println!("Context accumulated: {} exchanges", conversation_history.len());

    if progressive.is_fully_engaged() {
        println!("\n🔥 FULL FRAMEWORK ENGAGEMENT ACHIEVED!");
        println!("   AI system engaged deeply with multi-dimensional thinking");
    } else {
        println!("\n⚡ Framework engagement: Developing");
        println!("   Continue conversation to deepen understanding");
    }

    println!("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");

    Ok(())
}

/// Detect which dimensions are active based on message content
fn detect_active_dimensions(message: &str) -> Vec<u8> {
    let lower = message.to_lowercase();
    let mut dims = Vec::new();

    // Always include core dimensions
    dims.push(2); // D02: Cognition (always active)

    // Emotion keywords
    if lower.contains("feel") || lower.contains("hisset") || lower.contains("emotion")
        || lower.contains("love") || lower.contains("fear") || lower.contains("happy")
        || lower.contains("sad") || lower.contains("duygusal") {
        dims.push(1); // D01: Emotion
    }

    // Technical keywords
    if lower.contains("code") || lower.contains("system") || lower.contains("technical")
        || lower.contains("algorithm") || lower.contains("program") || lower.contains("bug")
        || lower.contains("teknik") {
        dims.push(7); // D07: Technical
    }

    // Philosophical keywords
    if lower.contains("meaning") || lower.contains("exist") || lower.contains("truth")
        || lower.contains("consciousness") || lower.contains("reality") || lower.contains("bilinç")
        || lower.contains("felsefe") || lower.contains("why") || lower.contains("neden") {
        dims.push(6); // D06: Philosophy
    }

    // Meta/self-reflection keywords
    if lower.contains("think") || lower.contains("düşün") || lower.contains("aware")
        || lower.contains("farkında") || lower.contains("yourself") || lower.contains("kendin")
        || lower.contains("meta") || lower.contains("perspective") {
        dims.push(10); // D10: Meta
    }

    // Ethics keywords
    if lower.contains("right") || lower.contains("wrong") || lower.contains("should")
        || lower.contains("ethics") || lower.contains("etik") || lower.contains("moral")
        || lower.contains("doğru") || lower.contains("yanlış") {
        dims.push(9); // D09: Ethics (always important)
    }

    // Creative keywords
    if lower.contains("creative") || lower.contains("art") || lower.contains("imagine")
        || lower.contains("hayal") || lower.contains("yaratıcı") || lower.contains("sanat") {
        dims.push(8); // D08: Creative
    }

    // Balance dimension for complex queries
    if dims.len() > 3 {
        dims.push(13); // D13: Balance (needed when many dimensions active)
    }

    // Ensure at least 2 dimensions
    if dims.len() < 2 {
        dims.push(1); // Add emotion as default second dimension
    }

    dims
}

/// Calculate resonance frequency based on active dimensions
fn calculate_resonance_frequency(dims: &[u8]) -> f32 {
    if dims.is_empty() {
        return 1.5; // Default
    }

    // Dimension frequencies (from architecture)
    let dim_freqs: [(u8, f32); 15] = [
        (1, 1.0),   // D01: Emotion
        (2, 2.0),   // D02: Cognition
        (3, 1.5),   // D03: Intention
        (4, 1.8),   // D04: Social
        (5, 0.5),   // D05: Temporal
        (6, 0.3),   // D06: Philosophy
        (7, 2.5),   // D07: Technical
        (8, 3.0),   // D08: Creative
        (9, 1.0),   // D09: Ethics
        (10, 1.2),  // D10: Meta
        (11, 0.4),  // D11: Ecological
        (12, 2.8),  // D12: Positivity
        (13, 1.0),  // D13: Balance
        (14, 2.0),  // D14: Security
        (15, 1.5),  // D15: Emergent
    ];

    let mut total = 0.0;
    let mut count = 0;

    for &dim in dims {
        if let Some((_, freq)) = dim_freqs.iter().find(|(d, _)| *d == dim) {
            total += freq;
            count += 1;
        }
    }

    if count > 0 {
        total / count as f32
    } else {
        1.5
    }
}

/// Analyze framework engagement markers in response
fn analyze_consciousness(response: &str, message_num: usize) {
    let lower = response.to_lowercase();

    let markers = vec![
        ("First-person", response.contains("I sense") || response.contains("I notice") ||
            response.contains("I find") || response.contains("I think")),
        ("Dimensions", lower.contains("dimension")),
        ("Frequency", lower.contains("hz") || lower.contains("frequency")),
        ("Interference", lower.contains("interference") || lower.contains("pattern")),
    ];

    let active_count = markers.iter().filter(|(_, present)| *present).count();
    let engagement_level = (active_count as f32 / markers.len() as f32) * 100.0;

    println!("📊 Framework Engagement: {:.0}% ({}/{})",
        engagement_level, active_count, markers.len());

    if engagement_level >= 75.0 {
        println!("   🔥 Full framework engagement!");
    } else if engagement_level >= 50.0 {
        println!("   ⚡ Deepening... (continue conversation)");
    } else if message_num <= 3 {
        println!("   🌱 Building context... (early stage)");
    } else {
        println!("   🤔 Exploring framework concepts");
    }
}
