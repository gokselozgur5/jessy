//! Chat API Endpoints

use actix_web::{web, HttpResponse, Responder};
use serde::{Deserialize, Serialize};
use crate::{
    DimensionId,
    navigation::DimensionSelector,
    memory::MmapManager,
    llm::{LLMManager, LLMConfig, Message},
    conversation::{ConversationHistory, ConversationStore, DimensionalState, MessageRole, MetadataExtractor},
    processing::ConsciousnessOrchestrator,
};
use crate::services::personality_rag::PersonalityRAG;
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
    pub context_manager: Arc<Mutex<crate::memory::PersistentContextManager>>,  // User context persistence
    pub emotional_memory: Arc<Mutex<crate::memory::EmotionalMemoryManager>>,  // Full conversation + emotional memory
    pub personality_rag: Option<Arc<PersonalityRAG>>,  // Personality RAG system (optional - graceful degradation)
}

impl AppState {
    pub async fn new(api_key: String) -> Result<Self, Box<dyn std::error::Error>> {
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

        // Get data directories from environment
        let runtime_data_dir = std::env::var("RUNTIME_DATA_DIR").unwrap_or_else(|_| "./data".to_string());
        let jessy_data_dir = std::env::var("JESSY_DATA_DIR").unwrap_or_else(|_| "./data".to_string());
        
        let data_dir = std::path::PathBuf::from(&runtime_data_dir);
        std::fs::create_dir_all(&data_dir)?;
        let store = ConversationStore::new(data_dir.join("web_conversations.json"))?;

        // Initialize ConsciousnessOrchestrator with full 3-tier memory system
        eprintln!("[AppState] Initializing ConsciousnessOrchestrator with 3-tier memory...");
        eprintln!("[AppState] Runtime data dir: {}", runtime_data_dir);
        eprintln!("[AppState] JESSY data dir: {}", jessy_data_dir);

        // Load dimension registry
        // Try cognitive_layers.json first (new format with layers), fallback to dimensions.json
        let cognitive_layers_path = format!("{}/cognitive_layers.json", jessy_data_dir);
        let dimensions_path = format!("{}/dimensions.json", jessy_data_dir);
        
        let (config_data, config_path) = if std::path::Path::new(&cognitive_layers_path).exists() {
            let cognitive_data = std::fs::read_to_string(&cognitive_layers_path)
                .map_err(|e| format!("Failed to read cognitive_layers.json: {}", e))?;

            // cognitive_layers.json only has layers, need to merge with dimensions metadata
            let dimensions_data = std::fs::read_to_string(&dimensions_path)
                .map_err(|e| format!("Failed to read dimensions.json: {}", e))?;

            // Parse both
            let dims: serde_json::Value = serde_json::from_str(&dimensions_data)
                .map_err(|e| format!("Failed to parse dimensions.json: {}", e))?;
            let layers: serde_json::Value = serde_json::from_str(&cognitive_data)
                .map_err(|e| format!("Failed to parse cognitive_layers.json: {}", e))?;

            // Merge them
            let merged = serde_json::json!({
                "dimensions": dims.get("dimensions").unwrap_or(&serde_json::json!([])),
                "layers": layers.get("layers").unwrap_or(&serde_json::json!([]))
            });

            (serde_json::to_string(&merged).unwrap(), "data/cognitive_layers.json + dimensions.json")
        } else {
            // Fallback to dimensions.json only
            let data = std::fs::read_to_string("data/dimensions.json")
                .map_err(|e| format!("Failed to read dimensions.json: {}", e))?;
            (data, "data/dimensions.json")
        };

        let registry = Arc::new(
            DimensionRegistry::load_dimensions(&config_data)
                .map_err(|e| format!("Failed to load dimensions: {}", e))?
        );

        eprintln!("[AppState] ✅ Loaded {} dimensions, {} layers from {}",
                  registry.dimension_count(), registry.layer_count(), config_path);

        // Register layers from registry into memory manager's layer index
        // This populates the layer_index so memory loading can find layers
        memory_manager.register_layers_from_registry(&registry)
            .map_err(|e| format!("Failed to register layers: {}", e))?;

        let navigation = Arc::new(NavigationSystem::new(registry, memory_manager.clone())?);

        let mut orchestrator = ConsciousnessOrchestrator::with_llm(
            navigation,
            memory_manager.clone(),
            llm_config
        )?;

        eprintln!("[AppState] ✅ ConsciousnessOrchestrator initialized with learning system");

        // Initialize self-reflection system
        let reflection_path = std::path::PathBuf::from(&runtime_data_dir).join("self_reflections");
        if let Err(e) = orchestrator.learning_mut().init_self_reflection(reflection_path) {
            eprintln!("[AppState] Failed to initialize self-reflection: {}", e);
        }

        // Initialize persistent context manager
        let user_contexts_path = format!("{}/user_contexts", runtime_data_dir);
        let context_manager = crate::memory::PersistentContextManager::new(
            std::path::PathBuf::from(&user_contexts_path),
            100,  // max_cache_size
            30,   // retention_days
        )?;

        eprintln!("[AppState] ✅ PersistentContextManager initialized");

        // Initialize emotional memory manager
        let emotional_memory_path = format!("{}/emotional_memory", runtime_data_dir);
        let emotional_memory = crate::memory::EmotionalMemoryManager::new(
            std::path::PathBuf::from(&emotional_memory_path)
        )?;

        eprintln!("[AppState] ✅ EmotionalMemoryManager initialized");

        // Wrap orchestrator in Arc<Mutex<>> early for RAG initialization
        let orchestrator_arc = Arc::new(Mutex::new(orchestrator));

        // Initialize PersonalityRAG (optional - graceful degradation if fails)
        eprintln!("[AppState] 🧠 Initializing PersonalityRAG...");
        let personality_rag = match initialize_personality_rag(orchestrator_arc.clone(), &runtime_data_dir).await {
            Ok(rag) => {
                eprintln!("[AppState] ✅ PersonalityRAG initialized successfully");
                Some(Arc::new(rag))
            }
            Err(e) => {
                eprintln!("[AppState] ⚠️  PersonalityRAG initialization failed: {}. Continuing without personality context (graceful degradation).", e);
                None
            }
        };

        Ok(Self {
            selector,
            memory_manager,
            llm,
            store,
            conversations: Arc::new(Mutex::new(std::collections::HashMap::new())),
            orchestrator: orchestrator_arc,
            context_manager: Arc::new(Mutex::new(context_manager)),
            emotional_memory: Arc::new(Mutex::new(emotional_memory)),
            personality_rag,
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

    // Load conversation from emotional memory or create new
    let mut conversation = {
        let mut conversations = data.conversations.lock().await;
        
        if let Some(conv) = conversations.get(&session_id) {
            // Already in memory
            conv.clone()
        } else if let Some(uid) = req.user_id.as_deref() {
            // Try loading from emotional memory (persistent across restarts)
            let emotional_memory = data.emotional_memory.lock().await;
            match emotional_memory.load_conversation(uid, &session_id).await {
                Ok(record) => {
                    eprintln!("[Chat API] 📂 Loaded conversation {} from emotional memory ({} messages)", 
                             session_id, record.messages.len());
                    
                    // Convert ConversationRecord back to ConversationHistory
                    let mut conv = ConversationHistory::new();
                    for msg_record in record.messages {
                        if msg_record.role == "user" {
                            conv.add_user_message(msg_record.content, msg_record.dimensions_activated);
                        } else {
                            conv.add_assistant_message(msg_record.content);
                        }
                    }
                    
                    conversations.insert(session_id.clone(), conv.clone());
                    conv
                }
                Err(_) => {
                    // New conversation
                    eprintln!("[Chat API] 📝 Creating new conversation {}", session_id);
                    ConversationHistory::new()
                }
            }
        } else {
            // No user_id, can't load from disk
            eprintln!("[Chat API] 📝 Creating new conversation {} (no user_id)", session_id);
            ConversationHistory::new()
        }
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
            
            // Save full emotional memory (async, non-blocking)
            let orchestrator = data.orchestrator.lock().await;
            let state = orchestrator.state().clone();
            drop(orchestrator); // Release lock
            
            if let Err(e) = save_emotional_memory(&data, &conversation, &session_id, req.user_id.as_deref(), &state).await {
                eprintln!("⚠️ Failed to save emotional memory: {}", e);
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

    // Step 2.5: Load user context if user_id provided
    if let Some(uid) = user_id {
        let context_manager = data.context_manager.lock().await;
        match context_manager.load_user_context(uid).await {
            Ok(user_context) => {
                eprintln!("[Chat API] ✅ Loaded context for user {}: {} conversations", 
                    uid, user_context.conversations.len());
                
                // Add user context as system message at the beginning
                if !user_context.conversations.is_empty() {
                    let context_summary = format!(
                        "User Context: You've had {} previous conversations with this user. Recent topics: {}. Communication style: {:?}.",
                        user_context.conversations.len(),
                        user_context.conversations.iter()
                            .rev()
                            .take(3)
                            .flat_map(|c| c.topics.iter())
                            .take(5)
                            .cloned()
                            .collect::<Vec<_>>()
                            .join(", "),
                        user_context.relationship_dynamics.communication_style
                    );
                    
                    eprintln!("[Chat API] Adding user context to prompt: {}", context_summary);
                    
                    messages.insert(0, Message {
                        role: "system".to_string(),
                        content: context_summary,
                    });
                }
            }
            Err(e) => {
                eprintln!("[Chat API] ⚠️ Failed to load user context: {}", e);
            }
        }
    }

    // Step 2.75: Retrieve personality context from RAG (if available)
    if let Some(rag) = &data.personality_rag {
        eprintln!("[Chat API] 🔍 Retrieving personality context from RAG...");
        let context = rag.retrieve_relevant_context(message).await;
        if !context.is_empty() {
            eprintln!("[Chat API] ✅ Retrieved {} bytes of personality context", context.len());
            // Add personality context as system message
            messages.insert(0, Message {
                role: "system".to_string(),
                content: context,
            });
        } else {
            eprintln!("[Chat API] ⚠️  No personality context retrieved (empty result)");
        }
    } else {
        eprintln!("[Chat API] ⚠️  PersonalityRAG not available (graceful degradation)");
    }

    // Step 3: Process through ConsciousnessOrchestrator (full pipeline with learning)
    let mut orchestrator = data.orchestrator.lock().await;
    let orchestrator_result = orchestrator.process(message, user_id, messages).await?;

    let selection_duration = start.elapsed();
    
    // Step 3.5: Self-reflection - Jessy observes her own response (conscious learning)
    // This is where Jessy becomes aware of what she said and learns from it
    orchestrator.learning_mut().reflect_on_response(message, &orchestrator_result.final_response);
    
    // Save reflections periodically
    let _ = orchestrator.learning_mut().save_reflections();

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
    
    // Step 8: Save conversation to user context (if user_id provided) - BEFORE returning response
    if let Some(uid) = user_id {
        let manager = data.context_manager.lock().await;
        
        // Load or create user context
        let mut context = manager.load_user_context(uid).await
            .unwrap_or_else(|_| crate::memory::UserContext::new(uid.to_string()));
        
        // Extract metadata from conversation
        use crate::conversation::MetadataExtractor;
        let extractor = MetadataExtractor::new();
        let conversation_text = format!("User: {}\nJessy: {}", message, response);
        let metadata = extractor.extract_metadata(&conversation_text, uid);
        
        // Add conversation summary with extracted metadata
        context.conversations.push(crate::memory::ConversationSummary {
            session_id: uuid::Uuid::new_v4().to_string(),
            timestamp: chrono::Utc::now(),
            emotional_tone: metadata.emotional_tone,
            key_moments: metadata.key_moments,
            topics: metadata.topics.into_iter().map(|(topic, _)| topic).collect(),
            message_count: 2,  // User + Jessy
        });
        
        // Update relationship dynamics based on metadata
        if metadata.emotional_tone == crate::memory::EmotionalTone::Playful {
            // Update humor style in conversation flavor
            if context.conversation_flavor.humor_style.is_none() {
                context.conversation_flavor.humor_style = Some(crate::memory::HumorStyle::Playful);
            }
        }
        
        // Save updated context
        eprintln!("[Chat API] Attempting to save context for user {} with {} conversations", 
            uid, context.conversations.len());
        
        match manager.save_user_context(&context).await {
            Ok(_) => {
                eprintln!("[Chat API] ✅ Successfully saved conversation for user {} with {} total conversations", 
                    uid, context.conversations.len());
            }
            Err(e) => {
                eprintln!("[Chat API] ❌ Failed to save user context: {}", e);
            }
        }
    }

    Ok((response, dimensional_state))
}

/// Save full conversation to emotional memory (called from chat endpoint)
async fn save_emotional_memory(
    data: &AppState,
    conversation: &ConversationHistory,
    session_id: &str,
    user_id: Option<&str>,
    orchestrator_state: &crate::consciousness::ConsciousnessState,
) -> Result<(), Box<dyn std::error::Error>> {
    let Some(uid) = user_id else {
        return Ok(()); // Skip if no user_id
    };
    
    let emotional_memory = data.emotional_memory.lock().await;
    
    // Convert conversation to ConversationRecord
    let messages: Vec<crate::memory::MessageRecord> = conversation.messages.iter().map(|msg| {
        crate::memory::MessageRecord {
            role: match msg.role {
                crate::conversation::MessageRole::User => "user".to_string(),
                crate::conversation::MessageRole::Assistant => "assistant".to_string(),
            },
            content: msg.content.clone(),
            timestamp: msg.timestamp,
            dimensions_activated: msg.dimensions.clone(),
            jessy_mood: Some(format!("{:?}", orchestrator_state.mood())),
            jessy_energy: Some(orchestrator_state.energy_level),
            emotional_tone: None, // TODO: Extract from metadata
        }
    }).collect();
    
    let record = crate::memory::ConversationRecord {
        session_id: session_id.to_string(),
        user_id: uid.to_string(),
        started_at: conversation.messages.first().map(|m| m.timestamp).unwrap_or_else(chrono::Utc::now),
        ended_at: Some(chrono::Utc::now()),
        messages,
        emotional_arc: crate::memory::EmotionalArc {
            start_tone: "neutral".to_string(), // TODO: Detect from first messages
            end_tone: "satisfied".to_string(), // TODO: Detect from last messages
            peak_moments: vec![],
            overall_energy: orchestrator_state.energy_level,
        },
        key_moments: vec![],
        energy_pattern: crate::memory::EnergyPattern {
            average_energy: orchestrator_state.energy_level,
            peak_energy: orchestrator_state.energy_level,
            low_energy: orchestrator_state.energy_level,
            energy_trajectory: vec![],
        },
    };
    
    // Save conversation record
    emotional_memory.save_conversation(&record).await?;
    
    // Update aggregated emotional memory
    emotional_memory.update_emotional_memory(&record).await?;
    
    eprintln!("[Chat API] ✅ Saved full emotional memory for session {}", session_id);
    
    Ok(())
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

pub fn build_system_prompt(
    selection: &crate::navigation::SimpleDimensionSelection,
    personality_context: Option<&str>,
) -> String {
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

    // Inject personality context if available (RAG system)
    if let Some(context) = personality_context {
        if !context.is_empty() {
            prompt.push_str("\n");
            prompt.push_str(context);
            prompt.push_str("\n");
        }
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

/// Initialize PersonalityRAG system from personality files
///
/// Graceful degradation: Returns error if initialization fails, allowing AppState
/// to continue without personality context (base prompt only)
async fn initialize_personality_rag(
    orchestrator_arc: Arc<Mutex<ConsciousnessOrchestrator>>,
    runtime_data_dir: &str,
) -> Result<PersonalityRAG, Box<dyn std::error::Error>> {
    use crate::services::vector_store::VectorStore;

    // Initialize vector store (in-memory for simplicity, no external Qdrant dependency)
    let vector_store = Arc::new(VectorStore::new(":memory:").await?);

    // TODO: For now, use a placeholder orchestrator since RAG uses dummy embeddings
    // When real embedding model is integrated, replace this with actual orchestrator reference
    //
    // TEMPORARY SOLUTION: Create a minimal placeholder Arc by loading minimal dimensions
    // This works because generate_embeddings_batch() in personality_rag.rs currently uses
    // deterministic hash-based dummy embeddings and doesn't actually call orchestrator methods
    use crate::navigation::{NavigationSystem, DimensionRegistry};

    // Create minimal valid dimension registry (1 dimension with empty layers)
    let minimal_dimensions_json = r#"{"dimensions":[{"id":1,"name":"Placeholder"}],"layers":[]}"#;
    let placeholder_registry = Arc::new(
        DimensionRegistry::load_dimensions(minimal_dimensions_json)
            .map_err(|e| format!("Failed to create placeholder registry: {}", e))?
    );

    let placeholder_memory = Arc::new(MmapManager::new(1)?);
    let placeholder_nav = Arc::new(NavigationSystem::new(placeholder_registry, placeholder_memory.clone())?);
    let placeholder_orch = Arc::new(ConsciousnessOrchestrator::new(
        placeholder_nav,
        placeholder_memory
    ));

    // Create PersonalityRAG instance
    let rag = PersonalityRAG::new(
        vector_store,
        placeholder_orch,
        5,  // top_k: retrieve 5 most relevant chunks
    );

    // Personality file paths (uploaded via SCP to /app/data/personality/)
    let kiro_path = format!("{}/personality/realkiro.md", runtime_data_dir);
    let goksel_path = format!("{}/personality/gokselclaude.md", runtime_data_dir);

    eprintln!("[RAG Init] Reading personality files from:");
    eprintln!("  - Kiro: {}", kiro_path);
    eprintln!("  - Göksel: {}", goksel_path);

    // Initialize RAG from files (chunks → embeddings → Qdrant)
    rag.initialize_from_files(&kiro_path, &goksel_path).await?;

    Ok(rag)
}
