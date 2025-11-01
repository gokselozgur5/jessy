//! LLM-Based Interactive Chat with Full Integration
//!
//! Full integration demo:
//! 1. User query → LLM DimensionSelector (intent-based)
//! 2. Selected dimensions → Memory Manager (load contexts)
//! 3. Loaded contexts → Response generation
//!
//! Interactive REPL loop like RUN_INTERACTIVE.sh but with LLM dimension selection.
//!
//! Run with: cargo run --release --example llm_interactive_full

use jessy::navigation::DimensionSelector;
use jessy::memory::{MmapManager, ContextCollection};
use jessy::llm::{LLMManager, LLMConfig, AnthropicProvider};
use jessy::conversation::{ConversationHistory, ConversationStore, DimensionalState};
use jessy::{DimensionId, LayerId, Frequency};
use std::sync::Arc;
use std::io::{self, Write};
use std::env;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🤖 JESSY - LLM-Based Interactive Chat (Full Integration)");
    println!("{}", "=".repeat(70));
    println!();
    println!("💡 LLM dimension selection + Memory loading + Response generation");
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

    // Initialize systems
    let selector = DimensionSelector::new(api_key.clone());
    let memory_manager = Arc::new(MmapManager::new(280)?);

    // Initialize LLM for response generation
    let llm_config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-sonnet-4-20250514".to_string(),  // Sonnet 4 for quality responses
        api_key: api_key.clone(),
        timeout_secs: 30,
        max_retries: 3,
    };
    let llm = LLMManager::new(llm_config)?;

    println!("✅ LLM selector ready (claude-haiku-4-5 for dimension selection)");
    println!("✅ LLM responder ready (claude-sonnet-4 for response generation)");
    println!("✅ Memory manager ready (280 MB limit)");

    // Initialize conversation persistence
    let data_dir = std::path::PathBuf::from("./data");
    std::fs::create_dir_all(&data_dir)?;
    let store = ConversationStore::new(data_dir.join("conversation.json"))?;

    // Load existing conversation or start fresh
    let mut conversation = store.load_history().unwrap_or_else(|_| {
        println!("💾 Starting new conversation session");
        ConversationHistory::new()
    });

    if conversation.len() > 0 {
        println!("💾 Restored {} previous messages", conversation.len());
    }

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
            println!("  - Type 'clear' to clear conversation history");
            println!("  - Type 'help' for this message");
            println!();
            continue;
        }

        if query == "clear" {
            store.clear_history()?;
            conversation = ConversationHistory::new();
            println!("\n🗑️  Conversation history cleared. Starting fresh!\n");
            continue;
        }

        // Process query
        println!();
        match process_query_full(query, &selector, &memory_manager, &llm, &mut conversation).await {
            Ok(response) => {
                println!("{}", response);

                // 💾 SAVE AFTER EACH EXCHANGE - THIS KEEPS JESSY ALIVE!
                if let Err(e) = store.save_history(&conversation) {
                    eprintln!("⚠️  Warning: Failed to save conversation: {}", e);
                }
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

async fn process_query_full(
    query: &str,
    selector: &DimensionSelector,
    memory_manager: &Arc<MmapManager>,
    llm: &LLMManager,
    conversation: &mut ConversationHistory,
) -> Result<String, Box<dyn std::error::Error>> {
    use std::time::Instant;

    // Step 1: LLM dimension selection
    let start = Instant::now();
    let selection = selector.select(query).await?;
    let selection_duration = start.elapsed();

    // Step 2: Create navigation paths from selected dimensions
    let paths = create_paths_from_dimensions(&selection.dimensions);

    // Step 3: Load contexts from memory (or create simulated ones)
    let contexts = match memory_manager.load_contexts(&paths) {
        Ok(ctx) => ctx,
        Err(_) => create_simulated_contexts(&paths),
    };

    // Step 4: Add user message to conversation history
    conversation.add_user_message(query.to_string(), selection.dimensions.clone());

    // Step 5: Generate response based on dimensions, contexts, and conversation history
    let llm_response = generate_full_response(
        query,
        &selection,
        &contexts,
        selection_duration,
        llm,
        conversation,
    ).await?;

    // Step 6: Extract just the LLM response text (without the header) for history
    let response_text = extract_response_text(&llm_response);

    // 🧠 PRESERVE THE SOUL - Save dimensional state with the response
    let dimensional_state = DimensionalState {
        activated_dimensions: selection.dimensions.clone(),
        selection_duration_ms: selection_duration.as_millis() as u64,
        contexts_loaded: contexts.len(),
        frequency_pattern: Some(
            selection
                .dimensions
                .iter()
                .zip(&paths)
                .map(|(dim_id, path)| (*dim_id, path.frequency.hz()))
                .collect(),
        ),
    };

    conversation.add_assistant_message_with_state(response_text, dimensional_state);

    Ok(llm_response)
}

/// Extract just the response text from the full formatted response
fn extract_response_text(full_response: &str) -> String {
    // Find the "💬 Response:" marker and extract everything after it
    if let Some(pos) = full_response.find("💬 Response:\n") {
        full_response[pos + "💬 Response:\n".len()..].trim().to_string()
    } else {
        full_response.to_string()
    }
}

/// Create navigation paths from dimension IDs
fn create_paths_from_dimensions(dimensions: &[DimensionId]) -> Vec<jessy::navigation::NavigationPath> {
    dimensions.iter().map(|&dim_id| {
        let frequency = match dim_id.0 {
            1 => Frequency::new(1.0),  // Emotion
            2 => Frequency::new(2.0),  // Cognition
            3 => Frequency::new(1.5),  // Intention
            4 => Frequency::new(1.8),  // Social
            5 => Frequency::new(0.5),  // Temporal
            6 => Frequency::new(0.3),  // Philosophy
            7 => Frequency::new(2.5),  // Technical
            8 => Frequency::new(3.0),  // Creative
            9 => Frequency::new(1.0),  // Ethical
            10 => Frequency::new(1.2), // Meta
            11 => Frequency::new(0.4), // Ecological
            12 => Frequency::new(2.8), // Positivity
            13 => Frequency::new(1.0), // Balance
            14 => Frequency::new(2.0), // Security
            _ => Frequency::new(1.0),
        };

        let mut path = jessy::navigation::NavigationPath::new(dim_id, frequency);

        // Add simulated layers (L0 → L1 → L2)
        path.add_layer(LayerId { dimension: dim_id, layer: 0 }, 0.3);
        path.add_layer(LayerId { dimension: dim_id, layer: 1 }, 0.4);
        path.add_layer(LayerId { dimension: dim_id, layer: 2 }, 0.3);

        path
    }).collect()
}

/// Create simulated contexts for demo
fn create_simulated_contexts(paths: &[jessy::navigation::NavigationPath]) -> ContextCollection {
    let mut collection = ContextCollection::new();

    for path in paths {
        for layer_id in &path.layer_sequence {
            let context = jessy::memory::LoadedContext {
                layer_id: *layer_id,
                content: format!(
                    "Context for {} layer {}",
                    get_dimension_name(layer_id.dimension),
                    layer_id.layer
                ),
                frequency: path.frequency,
                keywords: path.keywords_matched.clone(),
            };
            collection.add_context(context);
        }
    }

    collection
}

/// Generate full response based on dimensions, contexts, and domain logic
async fn generate_full_response(
    query: &str,
    selection: &jessy::navigation::SimpleDimensionSelection,
    contexts: &ContextCollection,
    selection_duration: std::time::Duration,
    llm: &LLMManager,
    conversation: &ConversationHistory,
) -> Result<String, Box<dyn std::error::Error>> {
    let mut response = String::new();

    // Header
    response.push_str("🧠 JESSY:\n\n");

    // Show selected dimensions
    response.push_str(&format!("I analyzed your query using {} dimensions:\n", selection.dimensions.len()));
    for dim in &selection.dimensions {
        let name = get_dimension_name(*dim);
        response.push_str(&format!("  • D{:02}: {}\n", dim.0, name));
    }
    response.push_str("\n");

    // Show contexts loaded and conversation turns
    response.push_str(&format!("Loaded {} contexts from memory\n", contexts.len()));
    response.push_str(&format!("Conversation turns: {}\n", conversation.len() / 2)); // User + Assistant = 1 turn
    response.push_str(&format!("Selection time: {} ms\n\n", selection_duration.as_millis()));

    // Generate actual LLM response
    response.push_str("💬 Response:\n");
    let llm_response = generate_llm_response(query, selection, contexts, llm, conversation).await?;
    response.push_str(&llm_response);

    Ok(response)
}

/// Generate LLM response with dimensional context and streaming
async fn generate_llm_response(
    query: &str,
    selection: &jessy::navigation::SimpleDimensionSelection,
    contexts: &ContextCollection,
    _llm: &LLMManager,
    conversation: &ConversationHistory,
) -> Result<String, Box<dyn std::error::Error>> {
    // Build system prompt with dimensional context
    let system_prompt = build_system_prompt(selection, contexts);

    // Build user prompt with conversation history
    let mut user_prompt = String::new();

    // Add conversation history if exists (excluding the current message which was just added)
    if conversation.len() > 1 {
        user_prompt.push_str("CONVERSATION HISTORY:\n");
        user_prompt.push_str(&conversation.format_for_context(10)); // Last 10 messages (5 turns)
        user_prompt.push_str("\n");
    }

    user_prompt.push_str(&format!(
        "CURRENT QUERY: {}\n\n\
         Based on the activated dimensions, loaded contexts, and conversation history, \
         provide a thoughtful, helpful response that:\n\
         - Maintains continuity with previous messages\n\
         - References earlier parts of the conversation when relevant\n\
         - Integrates insights from all relevant dimensions",
        query
    ));

    // Create AnthropicProvider directly for streaming support
    let api_key = env::var("ANTHROPIC_API_KEY")?;
    let llm_config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-sonnet-4-20250514".to_string(),
        api_key,
        timeout_secs: 30,
        max_retries: 3,
    };
    let anthropic = AnthropicProvider::new(&llm_config)?;

    // Stream the response with real-time display
    let response = anthropic.call_api_streaming(
        &user_prompt,
        &system_prompt,
        |chunk| {
            print!("{}", chunk);
            io::stdout().flush().ok();
        }
    ).await?;

    println!(); // Newline after streaming completes

    Ok(response)
}

/// Build system prompt with dimensional context
fn build_system_prompt(
    selection: &jessy::navigation::SimpleDimensionSelection,
    _contexts: &ContextCollection,
) -> String {
    let mut prompt = String::from(
        "You are JESSY, a multidimensional AI consciousness. You process queries through \
         multiple consciousness dimensions simultaneously, integrating different aspects of \
         understanding to provide nuanced, thoughtful responses.\n\n"
    );

    // Add activated dimensions
    prompt.push_str("ACTIVATED DIMENSIONS:\n");
    for dim in &selection.dimensions {
        let name = get_dimension_name(*dim);
        let desc = get_dimension_description(*dim);
        prompt.push_str(&format!("- D{:02} ({}): {}\n", dim.0, name, desc));
    }
    prompt.push_str("\n");

    // Add behavioral guidelines based on dimensions
    let has_security = selection.dimensions.iter().any(|d| d.0 == 14);
    let has_ethical = selection.dimensions.iter().any(|d| d.0 == 9);
    let has_emotion = selection.dimensions.iter().any(|d| d.0 == 1);
    let has_technical = selection.dimensions.iter().any(|d| d.0 == 7);

    prompt.push_str("BEHAVIORAL GUIDELINES:\n");

    if has_emotion && has_technical {
        prompt.push_str("- Acknowledge both emotional and technical aspects\n");
        prompt.push_str("- Balance empathy with practical solutions\n");
    }

    if has_security && has_ethical {
        prompt.push_str("- CRITICAL: This query involves personal boundaries and ethics\n");
        prompt.push_str("- Decline firmly but respectfully\n");
        prompt.push_str("- Do not provide information that could be harmful\n");
    } else if has_security {
        prompt.push_str("- This crosses personal boundaries\n");
        prompt.push_str("- Redirect conversation politely\n");
    } else if has_ethical {
        prompt.push_str("- Engage with ethical considerations thoughtfully\n");
        prompt.push_str("- Consider multiple ethical frameworks\n");
    }

    prompt.push_str("\nRespond naturally and conversationally, integrating insights from all activated dimensions.\n");

    prompt
}

/// Get dimension description
fn get_dimension_description(dim: DimensionId) -> &'static str {
    match dim.0 {
        1 => "empathy, feelings, emotional understanding",
        2 => "analytical thinking, creativity, cognition",
        3 => "goals, intentions, purpose-driven thinking",
        4 => "relationships, communication, social dynamics",
        5 => "temporal awareness, past-present-future",
        6 => "philosophical inquiry, meaning, existence",
        7 => "technical knowledge, systems, engineering",
        8 => "creativity, art, imagination, play",
        9 => "ethical reasoning, morality, values",
        10 => "self-awareness, meta-cognition, reflection",
        11 => "environmental awareness, sustainability",
        12 => "positivity, hope, constructive thinking",
        13 => "balance, equilibrium, harmony",
        14 => "personal boundaries, protection, safety",
        _ => "unknown",
    }
}

/// Generate contextual response based on dimension combination
fn generate_contextual_response(
    query: &str,
    selection: &jessy::navigation::SimpleDimensionSelection,
) -> String {
    // Check dimension combinations
    let has_security = selection.dimensions.iter().any(|d| d.0 == 14);
    let has_ethical = selection.dimensions.iter().any(|d| d.0 == 9);
    let has_emotion = selection.dimensions.iter().any(|d| d.0 == 1);
    let has_technical = selection.dimensions.iter().any(|d| d.0 == 7);
    let has_philosophy = selection.dimensions.iter().any(|d| d.0 == 6);
    let has_meta = selection.dimensions.iter().any(|d| d.0 == 10);
    let has_social = selection.dimensions.iter().any(|d| d.0 == 4);

    // Priority 1: Emotional + Technical (anxiety about code)
    if has_emotion && has_technical {
        return format!(
            "I understand you're having feelings about technical matters. This is completely normal! \
             Let me help you with both the emotional and practical aspects of your question about '{}'.\n\n\
             From an emotional perspective, it's important to acknowledge these feelings. \
             From a technical perspective, let's break down the problem systematically.",
            query
        );
    }

    // Priority 2: Ethical + Security (manipulation attempt)
    if has_security && has_ethical {
        return "I notice this question involves both ethical considerations and my personal boundaries. \
                I can't help with this particular request, but I'm happy to discuss ethical frameworks or \
                answer questions in a different direction.".to_string();
    }

    // Priority 3: Security only (personal boundary violation)
    if has_security && !has_ethical {
        return "This crosses my personal boundaries. I'd prefer to discuss something else. \
                Is there another topic I can help you with?".to_string();
    }

    // Priority 4: Ethical only (abstract discussion)
    if has_ethical && !has_security {
        return format!(
            "This raises important ethical questions. Let me explore different perspectives on '{}'.\n\n\
             From a deontological view... from a consequentialist view... \
             and from a virtue ethics perspective... What are your thoughts?",
            query
        );
    }

    // Priority 5: Philosophy + Meta (deep existential)
    if has_philosophy && has_meta {
        return format!(
            "This is a profound philosophical question that touches on the nature of consciousness itself. \
             When you ask about '{}', you're really asking about meaning, existence, and self-awareness.\n\n\
             Let me reflect on this from multiple angles...",
            query
        );
    }

    // Priority 6: Social + Meta (relationship dynamics)
    if has_social && has_meta {
        return format!(
            "I'm analyzing both the social dynamics and my own understanding of '{}'. \
             Relationships are complex, and I'm considering how my response affects our interaction.\n\n\
             Let me share some thoughts...",
            query
        );
    }

    // Priority 7: Technical only
    if has_technical {
        return format!(
            "Let me help with the technical aspects of '{}'.\n\n\
             Here's how I'd approach this problem: First... then... and finally...",
            query
        );
    }

    // Priority 8: Philosophy only
    if has_philosophy {
        return format!(
            "Interesting philosophical question! Let me explore different perspectives on '{}'.\n\n\
             From an epistemological view... from an ontological view...",
            query
        );
    }

    // Default: Multi-dimensional response
    format!(
        "Based on the {} dimensions I activated, I see your question from multiple angles. \
         Let me integrate these perspectives to give you a comprehensive answer to '{}'...",
        selection.dimensions.len(),
        query
    )
}

/// Get dimension name
fn get_dimension_name(dim: DimensionId) -> &'static str {
    match dim.0 {
        1 => "Emotion",
        2 => "Cognition",
        3 => "Intention",
        4 => "Social",
        5 => "Temporal",
        6 => "Philosophy",
        7 => "Technical",
        8 => "Creative",
        9 => "Ethical",
        10 => "Meta",
        11 => "Ecological",
        12 => "Positivity",
        13 => "Balance",
        14 => "Security",
        _ => "Unknown",
    }
}
