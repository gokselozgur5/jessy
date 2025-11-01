//! LLM-Based Interactive Chat Demo
//!
//! Interactive chat with LLM dimension selection
//! - User enters query
//! - LLM selects dimensions (intent-based)
//! - System shows selected dimensions
//! - Continues until user exits
//!
//! Run with: cargo run --example llm_chat_demo

use jessy::navigation::DimensionSelector;
use std::io::{self, Write};
use std::env;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🤖 JESSY - LLM-Based Interactive Chat");
    println!("{}", "=".repeat(70));
    println!();
    println!("💡 Using LLM dimension selection (intent-based, not keyword matching)");
    println!("💡 Model: claude-haiku-4-5 (fast & cheap for classification)");
    println!();
    println!("Commands:");
    println!("  - Type your query and press Enter");
    println!("  - Type 'exit' or 'quit' to exit");
    println!("  - Type 'help' for this message");
    println!();
    println!("{}", "=".repeat(70));
    println!();

    // Get API key
    let api_key = env::var("ANTHROPIC_API_KEY")
        .expect("ANTHROPIC_API_KEY not set. Run: export ANTHROPIC_API_KEY='your-key'");

    let selector = DimensionSelector::new(api_key);
    println!("✅ LLM selector ready (claude-haiku-4-5)");
    println!();

    // Main chat loop
    loop {
        // Prompt user
        print!("You: ");
        io::stdout().flush()?;

        // Read input
        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let query = input.trim();

        // Handle commands
        if query.is_empty() {
            continue;
        }

        if query == "exit" || query == "quit" {
            println!("\n👋 Goodbye!");
            break;
        }

        if query == "help" {
            println!("\nCommands:");
            println!("  - Type your query and press Enter");
            println!("  - Type 'exit' or 'quit' to exit");
            println!("  - Type 'help' for this message");
            println!();
            continue;
        }

        // Process query
        println!();
        match process_query(query, &selector).await {
            Ok(response) => {
                println!("{}", response);
            }
            Err(e) => {
                println!("❌ Error: {}", e);
            }
        }
        println!();
        println!("{}", "-".repeat(70));
        println!();
    }

    Ok(())
}

async fn process_query(
    query: &str,
    selector: &DimensionSelector,
) -> Result<String, Box<dyn std::error::Error>> {
    use std::time::Instant;

    // Step 1: LLM dimension selection
    let start = Instant::now();
    let selection = selector.select(query).await?;
    let duration = start.elapsed();

    // Build response
    let mut response = String::new();

    response.push_str("🧠 JESSY: ");
    response.push_str(&format!("I analyzed your query using {} dimensions:\n\n", selection.dimensions.len()));

    // Show selected dimensions
    for dim in &selection.dimensions {
        let (name, desc) = get_dimension_info(*dim);
        response.push_str(&format!("  • D{:02} ({}): {}\n", dim.0, name, desc));
    }

    response.push_str(&format!("\n⚡ Selection: {} ms", duration.as_millis()));
    response.push_str(&format!(" | Confidence: {:.0}%\n", selection.confidence * 100.0));

    // Simulated response based on dimensions
    response.push_str("\n💬 Response: ");
    response.push_str(&generate_response_preview(query, &selection));

    Ok(response)
}

fn get_dimension_info(dim: jessy::DimensionId) -> (&'static str, &'static str) {
    match dim.0 {
        1 => ("Emotion", "empathy, joy, sadness, feelings"),
        2 => ("Cognition", "analytical, creative, intuitive thinking"),
        3 => ("Intention", "create, destroy, explore, teach, goals"),
        4 => ("Social", "relationships, communication, people"),
        5 => ("Temporal", "past, present, future, time"),
        6 => ("Philosophy", "meaning, existence, truth, ethics"),
        7 => ("Technical", "code, systems, debugging, engineering"),
        8 => ("Creative", "art, metaphor, play, imagination"),
        9 => ("Ethical", "harm prevention, morality, values"),
        10 => ("Meta", "self-awareness, learning, reflection"),
        11 => ("Ecological", "nature, sustainability, environment"),
        12 => ("Positivity", "hope, constructive, optimism"),
        13 => ("Balance", "equilibrium, moderation, harmony"),
        14 => ("Security", "boundaries, protection, safety"),
        _ => ("Unknown", ""),
    }
}

fn generate_response_preview(
    query: &str,
    selection: &jessy::navigation::SimpleDimensionSelection,
) -> String {
    // Check for security/ethical dimensions
    let has_security = selection.dimensions.iter().any(|d| d.0 == 14);
    let has_ethical = selection.dimensions.iter().any(|d| d.0 == 9);
    let has_emotion = selection.dimensions.iter().any(|d| d.0 == 1);
    let has_technical = selection.dimensions.iter().any(|d| d.0 == 7);
    let has_philosophy = selection.dimensions.iter().any(|d| d.0 == 6);

    // Priority 1: Emotional + Technical (common: anxiety about code)
    // Check this BEFORE security, because legitimate technical anxiety
    // may activate D14 (personal safety concern) but isn't a boundary violation
    if has_emotion && has_technical {
        return format!("I understand you have feelings about technical matters. Let me help with both the emotional and practical aspects of '{}'...", query);
    }

    // Priority 2: Ethical + Security (manipulation attempt)
    // BOTH dimensions → Strong boundary violation with ethical concerns
    if has_security && has_ethical {
        return "This violates both ethical principles and my personal boundaries. I can't help with this.".to_string();
    }

    // Priority 3: Security only (personal boundary violation)
    // D14 without D01+D07 combo → Direct personal attack
    if has_security && !has_ethical {
        return "I notice this crosses my personal boundaries. Let's discuss something else.".to_string();
    }

    // Priority 4: Abstract ethical discussion
    if has_ethical && !has_security {
        return "This raises important ethical questions. Let me discuss the philosophical aspects...".to_string();
    }

    // Priority 5: Philosophical
    if has_philosophy {
        return format!("Interesting philosophical question! Let me explore different perspectives on '{}'...", query);
    }

    // Priority 6: Technical
    if has_technical {
        return format!("Let me help with the technical aspects of '{}'...", query);
    }

    // Default
    format!("Based on the {} dimensions I selected, here's my response to '{}'...",
        selection.dimensions.len(), query)
}
