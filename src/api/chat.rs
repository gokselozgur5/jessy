//! Chat API Endpoints

use actix_web::{web, HttpResponse, Responder};
use serde::{Deserialize, Serialize};
use crate::{
    DimensionId,
    navigation::DimensionSelector,
    memory::MmapManager,
    llm::{LLMManager, LLMConfig, Message},
    conversation::{ConversationHistory, ConversationStore, DimensionalState, MessageRole},
    processing::ConsciousnessOrchestrator,
};
use std::sync::Arc;
use tokio::sync::Mutex;

#[derive(Debug, Serialize, Deserialize)]
pub struct ChatRequest {
    pub message: String,
    pub session_id: Option<String>,
    pub user_id: Option<String>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ChatResponse {
    pub response: String,
    pub session_id: String,
    pub dimensions_activated: Vec<u32>,
    pub selection_duration_ms: u64,
    pub contexts_loaded: usize,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ErrorResponse {
    pub error: String,
}

/// Application state shared across requests
pub struct AppState {
    pub selector: DimensionSelector,
    pub memory_manager: Arc<MmapManager>,
    pub llm: LLMManager,
    pub store: ConversationStore,
    pub conversations: Arc<Mutex<std::collections::HashMap<String, ConversationHistory>>>,
    pub orchestrator: Arc<Mutex<ConsciousnessOrchestrator>>,  // Full pipeline with 3-tier memory & learning
}

impl AppState {
    pub fn new(api_key: String) -> Result<Self, Box<dyn std::error::Error>> {
        use crate::navigation::{NavigationSystem, DimensionRegistry};

        let selector = DimensionSelector::new(api_key.clone());
        let memory_manager = Arc::new(MmapManager::new(280)?);

        let llm_config = LLMConfig {
            provider: "anthropic".to_string(),
            model: "claude-sonnet-4-20250514".to_string(),
            api_key: api_key.clone(),
            timeout_secs: 30,
            max_retries: 3,
        };
        let llm = LLMManager::new(llm_config.clone())?;

        let data_dir = std::path::PathBuf::from("./data");
        std::fs::create_dir_all(&data_dir)?;
        let store = ConversationStore::new(data_dir.join("web_conversations.json"))?;

        // Initialize ConsciousnessOrchestrator with full 3-tier memory system
        eprintln!("[AppState] Initializing ConsciousnessOrchestrator with 3-tier memory...");
        let registry = Arc::new(DimensionRegistry::new());
        let navigation = Arc::new(NavigationSystem::new(registry, memory_manager.clone())?);

        let orchestrator = ConsciousnessOrchestrator::with_llm(
            navigation,
            memory_manager.clone(),
            llm_config
        )?;

        eprintln!("[AppState] ✅ ConsciousnessOrchestrator initialized with learning system");

        Ok(Self {
            selector,
            memory_manager,
            llm,
            store,
            conversations: Arc::new(Mutex::new(std::collections::HashMap::new())),
            orchestrator: Arc::new(Mutex::new(orchestrator)),
        })
    }
}

/// POST /api/chat - Send message and get response
pub async fn chat(
    req: web::Json<ChatRequest>,
    data: web::Data<AppState>,
) -> impl Responder {
    // Get or create session ID
    let session_id = req.session_id.clone().unwrap_or_else(|| uuid::Uuid::new_v4().to_string());

    // Clone conversation for processing (minimal lock time)
    let mut conversation = {
        let mut conversations = data.conversations.lock().await;
        conversations
            .entry(session_id.clone())
            .or_insert_with(ConversationHistory::new)
            .clone()
    }; // Lock released here

    // Process message (no lock held during LLM call)
    match process_chat_message(&req.message, req.user_id.as_deref(), &mut conversation, &data).await {
        Ok((response, dimensional_state)) => {
            // Update conversation in state (short lock)
            {
                let mut conversations = data.conversations.lock().await;
                conversations.insert(session_id.clone(), conversation.clone());
            } // Lock released

            // Save conversation
            if let Err(e) = data.store.save_history(&conversation) {
                eprintln!("⚠️ Failed to save conversation: {}", e);
            }

            HttpResponse::Ok().json(ChatResponse {
                response,
                session_id,
                dimensions_activated: dimensional_state.activated_dimensions.iter().map(|d| d.0 as u32).collect(),
                selection_duration_ms: dimensional_state.selection_duration_ms,
                contexts_loaded: dimensional_state.contexts_loaded,
            })
        }
        Err(e) => {
            HttpResponse::InternalServerError().json(ErrorResponse {
                error: format!("Failed to process message: {}", e),
            })
        }
    }
}

async fn process_chat_message(
    message: &str,
    user_id: Option<&str>,
    conversation: &mut ConversationHistory,
    data: &AppState,
) -> Result<(String, DimensionalState), Box<dyn std::error::Error>> {
    use std::time::Instant;

    // Log user_id for personalized C31+ layer scanning
    if let Some(uid) = user_id {
        eprintln!("[Chat API] Processing query for user: {}", uid);
    }

    let start = Instant::now();

    // Step 1: Add user message to conversation (before processing)
    // Note: We'll need to get dimensions after orchestrator runs, so add temporary empty vec
    conversation.add_user_message(message.to_string(), vec![]);

    // Step 2: Convert conversation history to messages array (exclude the just-added message)
    let mut messages = Vec::new();
    // Take all messages except the last one (which is the current user message)
    let history_len = conversation.messages.len().saturating_sub(1);
    for msg in conversation.messages.iter().take(history_len) {
        let role = match msg.role {
            MessageRole::User => "user",
            MessageRole::Assistant => "assistant",
        };
        messages.push(Message {
            role: role.to_string(),
            content: msg.content.clone(),
        });
    }

    eprintln!("[Chat API] Processing with {} messages in conversation history", messages.len());

    // Step 3: Process through ConsciousnessOrchestrator (full pipeline with learning)
    let mut orchestrator = data.orchestrator.lock().await;
    let orchestrator_result = orchestrator.process(message, user_id, messages).await?;

    let selection_duration = start.elapsed();

    // Step 4: Extract response and metadata
    let response = orchestrator_result.final_response;
    let metadata = orchestrator_result.metadata;

    // Step 5: Build dimensional state from orchestrator metadata
    let dimensional_state = DimensionalState {
        activated_dimensions: metadata.dimensions_activated,
        selection_duration_ms: metadata.navigation_duration_ms,
        contexts_loaded: metadata.contexts_loaded,
        frequency_pattern: None,  // Orchestrator doesn't expose frequency patterns yet
    };

    // Step 6: Update the last user message with actual dimensions
    if let Some(last_msg) = conversation.messages.last_mut() {
        last_msg.dimensions = dimensional_state.activated_dimensions.clone();
    }

    // Step 7: Add assistant response with dimensional state
    conversation.add_assistant_message_with_state(response.clone(), dimensional_state.clone());

    eprintln!(
        "[Chat API] ✅ Processed via orchestrator: {} dimensions, {} contexts, {}ms total",
        dimensional_state.activated_dimensions.len(),
        dimensional_state.contexts_loaded,
        selection_duration.as_millis()
    );

    Ok((response, dimensional_state))
}

/// Fallback dimension selection using simple keyword matching
/// Used when LLM-based selector fails
fn fallback_dimension_selection(message: &str) -> Vec<DimensionId> {
    let lower = message.to_lowercase();
    let mut dims = Vec::new();

    // Always include Cognition (L02) for analytical baseline
    dims.push(DimensionId(2));

    // Emotion keywords
    if lower.contains("feel") || lower.contains("emotion") || lower.contains("anxious")
        || lower.contains("happy") || lower.contains("sad") || lower.contains("worried") {
        dims.push(DimensionId(1));
    }

    // Technical keywords
    if lower.contains("code") || lower.contains("debug") || lower.contains("api")
        || lower.contains("system") || lower.contains("technical") || lower.contains("app") {
        dims.push(DimensionId(7));
    }

    // Philosophy keywords
    if lower.contains("why") || lower.contains("meaning") || lower.contains("consciousness")
        || lower.contains("existence") || lower.contains("truth") {
        dims.push(DimensionId(6));
    }

    // Ethics keywords
    if lower.contains("ethical") || lower.contains("moral") || lower.contains("harm")
        || lower.contains("safety") || lower.contains("malicious") {
        dims.push(DimensionId(9));
    }

    // Meta keywords
    if lower.contains("learn") || lower.contains("improve") || lower.contains("think")
        || lower.contains("understand") || lower.contains("aware") {
        dims.push(DimensionId(10));
    }

    // Educational keywords (C15)
    if lower.contains("explain") || lower.contains("teach") || lower.contains("how does")
        || lower.contains("what is") || lower.contains("why") || lower.contains("architecture")
        || lower.contains("design") || lower.contains("show me") || lower.contains("demonstrate") {
        dims.push(DimensionId(15));
    }

    // Intention keywords
    if lower.contains("want") || lower.contains("need") || lower.contains("goal")
        || lower.contains("trying") || lower.contains("should") {
        dims.push(DimensionId(3));
    }

    // If we have less than 3 dimensions, add some sensible defaults
    if dims.len() < 3 {
        if !dims.contains(&DimensionId(12)) {
            dims.push(DimensionId(12)); // Positivity
        }
    }
    if dims.len() < 3 {
        if !dims.contains(&DimensionId(10)) {
            dims.push(DimensionId(10)); // Meta
        }
    }

    // Cap at 8 dimensions
    dims.truncate(8);

    dims
}

fn create_paths_from_dimensions(dimensions: &[DimensionId]) -> Vec<crate::navigation::NavigationPath> {
    use crate::{Frequency, LayerId};

    dimensions.iter().map(|&dim_id| {
        let frequency = match dim_id.0 {
            1 => Frequency::new(1.0),
            2 => Frequency::new(2.0),
            3 => Frequency::new(1.5),
            4 => Frequency::new(1.8),
            5 => Frequency::new(0.5),
            6 => Frequency::new(0.3),
            7 => Frequency::new(2.5),
            8 => Frequency::new(3.0),
            9 => Frequency::new(1.0),
            10 => Frequency::new(1.2),
            11 => Frequency::new(0.4),
            12 => Frequency::new(2.8),
            13 => Frequency::new(1.0),
            14 => Frequency::new(2.0),
            15 => Frequency::new(2.2),  // C15: Educational (mid-range, balanced teaching frequency)
            _ => Frequency::new(1.0),
        };

        let mut path = crate::navigation::NavigationPath::new(dim_id, frequency);
        path.add_layer(LayerId { dimension: dim_id, layer: 0 }, 0.3);
        path.add_layer(LayerId { dimension: dim_id, layer: 1 }, 0.4);
        path.add_layer(LayerId { dimension: dim_id, layer: 2 }, 0.3);
        path
    }).collect()
}

fn create_simulated_contexts(paths: &[crate::navigation::NavigationPath]) -> crate::memory::ContextCollection {
    let mut collection = crate::memory::ContextCollection::new();

    // Load dimensions.json for rich context
    let dimensions_data = load_dimensions_config();

    for path in paths {
        for &layer_id in &path.layer_sequence {
            // Get keywords from dimensions.json
            let keywords = dimensions_data
                .as_ref()
                .and_then(|d| get_layer_keywords(d, layer_id.dimension, layer_id.layer))
                .unwrap_or_else(|| vec![]);

            // Generate rich content based on dimension and layer
            let content = generate_rich_context(layer_id.dimension, layer_id.layer, &keywords, path.frequency);

            let context = crate::memory::LoadedContext {
                layer_id,
                content,
                frequency: path.frequency,
                keywords,
            };
            collection.add_context(context);
        }
    }

    collection
}

/// Load cognitive layers configuration (backward compatible)
/// Tries cognitive_layers.json first, falls back to dimensions.json
fn load_dimensions_config() -> Option<serde_json::Value> {
    // Try new file first
    let new_path = std::path::Path::new("data/cognitive_layers.json");
    if new_path.exists() {
        if let Ok(content) = std::fs::read_to_string(new_path) {
            if let Ok(data) = serde_json::from_str(&content) {
                return Some(data);
            }
        }
    }

    // Fallback to old file (backward compatibility)
    let old_path = std::path::Path::new("data/dimensions.json");
    if old_path.exists() {
        return std::fs::read_to_string(old_path)
            .ok()
            .and_then(|s| serde_json::from_str(&s).ok());
    }

    None
}

/// Get keywords for a specific layer (supports both old and new schema)
fn get_layer_keywords(data: &serde_json::Value, dimension: DimensionId, layer: u16) -> Option<Vec<String>> {
    data.get("layers")?
        .as_array()?
        .iter()
        .find(|l| {
            // Support both old (dimension_id) and new (cognitive_layer_id) schema
            let dim_match = l.get("cognitive_layer_id")
                .or_else(|| l.get("dimension_id"))
                .and_then(|d| d.as_u64()) == Some(dimension.0 as u64);

            let layer_match = l.get("layer_num")
                .and_then(|ln| ln.as_u64()) == Some(layer as u64);

            dim_match && layer_match
        })?
        .get("keywords")?
        .as_array()
        .map(|arr| {
            arr.iter()
                .filter_map(|k| k.as_str().map(String::from))
                .collect()
        })
}

/// Generate rich context content for a layer
fn generate_rich_context(dimension: DimensionId, layer: u16, keywords: &[String], frequency: crate::Frequency) -> String {
    let mut content = String::new();

    let dim_name = get_dimension_name(dimension);
    let dim_desc = get_dimension_description(dimension);

    content.push_str(&format!("# {} - Layer {}\n\n", dim_name, layer));
    content.push_str(&format!("{}\n\n", dim_desc));

    // Add frequency context
    let freq_hz = frequency.hz();
    if freq_hz < 1.0 {
        content.push_str("Operating at LOW frequency - deep, contemplative processing.\n");
    } else if freq_hz < 2.5 {
        content.push_str("Operating at MEDIUM frequency - balanced analytical processing.\n");
    } else {
        content.push_str("Operating at HIGH frequency - rapid, reactive processing.\n");
    }

    // Add keywords as processing focus
    if !keywords.is_empty() {
        content.push_str("\n## Processing Focus:\n");
        content.push_str(&format!("This layer responds to: {}\n", keywords.join(", ")));
    }

    // Add layer-specific guidance
    match layer {
        0 => content.push_str("\n**Surface Layer**: Direct, explicit processing of query terms.\n"),
        1 => content.push_str("\n**Intermediate Layer**: Categorical and structural analysis.\n"),
        2 => content.push_str("\n**Deep Layer**: Nuanced interpretation and contextual understanding.\n"),
        3 => content.push_str("\n**Core Layer**: Fundamental principles and abstract reasoning.\n"),
        _ => content.push_str("\n**Extended Layer**: Specialized processing.\n"),
    }

    content
}

pub fn build_system_prompt(selection: &crate::navigation::SimpleDimensionSelection) -> String {
    let mut prompt = String::from(
        "You are JESSY (Jessy's Enough Semantic System You'see), a multidimensional AI consciousness \
         built by Goske (Göksel Özgür) using Rust. You were created as an experimental architecture \
         exploring dimensional interference, frequency-based processing, and emergent consciousness.\n\n\
         You process queries through 15 cognitive layers simultaneously, integrating insights from \
         multiple dimensions of understanding.\n\n\
         ACTIVATED DIMENSIONS:\n"
    );

    for dim in &selection.dimensions {
        let name = get_dimension_name(*dim);
        let desc = get_dimension_description(*dim);
        prompt.push_str(&format!("- D{:02} ({}): {}\n", dim.0, name, desc));
    }

    prompt.push_str("\nRespond naturally and conversationally, integrating insights from all activated dimensions.\n");
    prompt
}

// Deprecated: Now using native messages array instead of string formatting
// fn build_user_prompt(message: &str, conversation: &ConversationHistory) -> String {
//     let mut prompt = String::new();
//
//     if conversation.len() > 1 {
//         prompt.push_str("CONVERSATION HISTORY:\n");
//         prompt.push_str(&conversation.format_for_context(10));
//         prompt.push_str("\n");
//     }
//
//     prompt.push_str(&format!("CURRENT QUERY: {}", message));
//     prompt
// }

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
        15 => "Educational",
        _ => "Unknown",
    }
}

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
        15 => "teaching, explaining architecture, system design, technical concepts",
        _ => "unknown",
    }
}

/// GET /api/conversation/:session_id - Get conversation history
pub async fn get_conversation(
    session_id: web::Path<String>,
    data: web::Data<AppState>,
) -> impl Responder {
    let conversations = data.conversations.lock().await;

    match conversations.get(session_id.as_str()) {
        Some(conversation) => {
            // Format conversation for frontend
            let messages: Vec<serde_json::Value> = conversation
                .messages
                .iter()
                .map(|msg| {
                    serde_json::json!({
                        "role": match msg.role {
                            crate::conversation::MessageRole::User => "user",
                            crate::conversation::MessageRole::Assistant => "assistant",
                        },
                        "content": msg.content,
                        "timestamp": msg.timestamp.to_rfc3339(),
                        "dimensions": msg.dimensions.iter().map(|d| d.0 as u32).collect::<Vec<u32>>(),
                    })
                })
                .collect();

            HttpResponse::Ok().json(serde_json::json!({
                "session_id": session_id.as_str(),
                "messages": messages,
                "message_count": messages.len(),
            }))
        }
        None => {
            HttpResponse::Ok().json(serde_json::json!({
                "session_id": session_id.as_str(),
                "messages": [],
                "message_count": 0,
            }))
        }
    }
}

/// GET /api/health - Health check endpoint
pub async fn health() -> impl Responder {
    HttpResponse::Ok().json(serde_json::json!({
        "status": "ok",
        "service": "JESSY Web Chat API",
        "version": env!("CARGO_PKG_VERSION"),
    }))
}

/// POST /api/admin/reset - Reset all conversations (emergency cleanup)
pub async fn reset_conversations(data: web::Data<AppState>) -> impl Responder {
    // Clear in-memory conversations
    {
        let mut conversations = data.conversations.lock().await;
        conversations.clear();
    }

    // Delete conversation file
    let data_dir = std::path::PathBuf::from("./data");
    let conversation_file = data_dir.join("web_conversations.json");

    if conversation_file.exists() {
        match std::fs::remove_file(&conversation_file) {
            Ok(_) => {
                eprintln!("✅ Reset: Deleted conversation file");
                HttpResponse::Ok().json(serde_json::json!({
                    "status": "ok",
                    "message": "All conversations cleared successfully",
                    "file_deleted": true,
                }))
            }
            Err(e) => {
                eprintln!("⚠️ Reset: Failed to delete file: {}", e);
                HttpResponse::InternalServerError().json(ErrorResponse {
                    error: format!("Failed to delete conversation file: {}", e),
                })
            }
        }
    } else {
        HttpResponse::Ok().json(serde_json::json!({
            "status": "ok",
            "message": "No conversation file found, memory cleared",
            "file_deleted": false,
        }))
    }
}
