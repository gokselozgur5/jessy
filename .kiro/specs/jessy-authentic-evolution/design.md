# Design Document: Jessy Authentic Evolution

## Overview

This design implements Jessy's authentic evolution features by enhancing the existing Rust-based consciousness system with persistent memory, natural thinking patterns, and real-time streaming. The design maintains Jessy's core architecture (Navigation → Memory → Interference → Observer Chain) while adding authenticity layers.

## Architecture

### High-Level Components

```
┌─────────────────────────────────────────────────────────────┐
│                     WebSocket Layer                          │
│  (Real-time streaming, typing indicators, reconnection)     │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│              Enhanced Observer Chain                         │
│  (Uncertainty, pivots, messy thinking, stage streaming)     │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│           Persistent Memory System                           │
│  (User contexts, conversation flavor, relationship dynamics) │
└────────────────────┬────────────────────────────────────────┘
                     │
┌────────────────────▼────────────────────────────────────────┐
│         Existing Consciousness Pipeline                      │
│  (Navigation → Memory → Interference → Iteration)           │
└─────────────────────────────────────────────────────────────┘
```

### Component Interactions

1. **WebSocket Layer** receives user message, establishes streaming connection
2. **Persistent Memory** loads user context (past conversations, flavor, dynamics)
3. **Observer Chain** processes with enhanced authenticity (uncertainty, pivots, messiness)
4. **WebSocket Layer** streams response tokens in real-time with natural rhythm
5. **Persistent Memory** stores conversation with rich metadata after completion

## Components and Interfaces

### 1. Persistent Memory System

**Location:** `src/memory/persistent_context.rs` (new file)

```rust
/// User-specific persistent context across conversations
pub struct UserContext {
    pub user_id: String,
    pub conversations: Vec<ConversationSummary>,
    pub relationship_dynamics: RelationshipDynamics,
    pub conversation_flavor: ConversationFlavor,
    pub unfinished_threads: Vec<UnfinishedThread>,
    pub created_at: DateTime<Utc>,
    pub last_interaction: DateTime<Utc>,
}

/// Rich conversation metadata
pub struct ConversationSummary {
    pub session_id: String,
    pub timestamp: DateTime<Utc>,
    pub emotional_tone: EmotionalTone,
    pub key_moments: Vec<KeyMoment>,
    pub topics: Vec<String>,
    pub message_count: usize,
}

/// Relationship dynamics between user and Jessy
pub struct RelationshipDynamics {
    pub formality_level: FormalityLevel,  // casual, professional, intimate
    pub shared_references: Vec<String>,   // inside jokes, recurring topics
    pub communication_style: CommunicationStyle,
    pub trust_level: f32,  // 0.0-1.0
}

/// Conversation flavor - the "vibe" of interactions
pub struct ConversationFlavor {
    pub humor_style: Option<HumorStyle>,  // sarcastic, playful, dry
    pub emotional_baseline: EmotionalTone,
    pub energy_level: EnergyLevel,  // high, medium, low
    pub language_mix: LanguageMix,  // Turkish-English ratio
}

/// Unfinished topics to revisit
pub struct UnfinishedThread {
    pub topic: String,
    pub context: String,
    pub started_at: DateTime<Utc>,
    pub priority: ThreadPriority,
}

/// Key moments worth remembering
pub struct KeyMoment {
    pub moment_type: MomentType,  // joke, insight, emotional, breakthrough
    pub content: String,
    pub timestamp: DateTime<Utc>,
    pub significance: f32,  // 0.0-1.0
}

/// Persistent context manager
pub struct PersistentContextManager {
    storage_path: PathBuf,
    cache: Arc<Mutex<HashMap<String, UserContext>>>,
    max_cache_size: usize,
}

impl PersistentContextManager {
    /// Load user context from storage
    pub async fn load_user_context(&self, user_id: &str) -> Result<UserContext>;
    
    /// Save user context to storage
    pub async fn save_user_context(&self, context: &UserContext) -> Result<()>;
    
    /// Add conversation to user context
    pub async fn add_conversation(
        &self,
        user_id: &str,
        conversation: ConversationHistory,
        metadata: ConversationMetadata,
    ) -> Result<()>;
    
    /// Retrieve relevant past context for current query
    pub async fn get_relevant_context(
        &self,
        user_id: &str,
        current_query: &str,
        max_conversations: usize,
    ) -> Result<Vec<ConversationSummary>>;
    
    /// Update relationship dynamics based on interaction
    pub async fn update_relationship_dynamics(
        &self,
        user_id: &str,
        interaction: &InteractionAnalysis,
    ) -> Result<()>;
}
```

**Storage Format:** JSON files per user in `data/user_contexts/{user_id}.json`

**Retention Policy:** 30 days, configurable via environment variable

### 2. Enhanced Observer Chain

**Location:** `src/observer_chain/authentic_observer.rs` (new file)

```rust
/// Enhanced observation with authenticity markers
pub struct AuthenticObservation {
    pub stage: usize,
    pub content: String,
    pub confidence: f32,
    pub authenticity_markers: AuthenticityMarkers,
    pub cognitive_layers: Vec<DimensionId>,
}

/// Markers for authentic thinking patterns
pub struct AuthenticityMarkers {
    pub has_uncertainty: bool,
    pub has_pivot: bool,
    pub has_correction: bool,
    pub has_confusion: bool,
    pub thinking_process: Vec<ThinkingStep>,
}

/// Individual thinking steps (for streaming)
pub enum ThinkingStep {
    InitialThought(String),
    Pause { reason: String },
    Pivot { from: String, to: String },
    Correction { wrong: String, right: String },
    Uncertainty { question: String },
    Insight(String),
}

/// Enhanced observer chain with authenticity
pub struct AuthenticObserverChain {
    llm: Arc<LLMManager>,
    max_stages: usize,
    allow_uncertainty: bool,
    allow_pivots: bool,
    preserve_messiness: bool,
}

impl AuthenticObserverChain {
    /// Process query with authentic thinking patterns
    pub async fn process_authentic(
        &self,
        query: &str,
        user_context: Option<&UserContext>,
        conversation: Vec<Message>,
    ) -> Result<AuthenticResponse>;
    
    /// Stream processing in real-time
    pub async fn process_streaming<F>(
        &self,
        query: &str,
        user_context: Option<&UserContext>,
        conversation: Vec<Message>,
        on_token: F,
    ) -> Result<AuthenticResponse>
    where
        F: Fn(StreamToken) + Send;
}

/// Response with authenticity metadata
pub struct AuthenticResponse {
    pub final_content: String,
    pub observations: Vec<AuthenticObservation>,
    pub thinking_trail: Vec<ThinkingStep>,
    pub confidence: f32,
    pub has_uncertainty: bool,
}

/// Streaming token types
pub enum StreamToken {
    Text(String),
    ThinkingMarker(ThinkingStep),
    StageTransition { from: usize, to: usize },
    Pause { duration_ms: u64 },
}
```

**Prompt Enhancements:**

```rust
// Add to prompts.rs
const AUTHENTICITY_GUIDELINES: &str = r#"
AUTHENTICITY GUIDELINES:
- If you're uncertain, say "I don't know" or "I'm not sure"
- If you change your mind mid-thought, show it: "wait, actually..."
- If something confuses you, express it: "hmm, that's puzzling"
- False starts are OK: "I was going to say X, but actually Y"
- Don't force polish - messy thinking is authentic thinking

EXAMPLES:
✓ "Hmm, I'm not sure about this. Let me think... actually, wait..."
✓ "I don't know the answer, but here's what puzzles me about it"
✓ "I was thinking X, but that doesn't feel right. Maybe Y?"
✗ "After careful analysis, the definitive answer is..."
"#;
```

### 3. WebSocket Streaming Layer

**Location:** `src/api/websocket.rs` (enhance existing)

```rust
/// WebSocket message types
#[derive(Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum WsMessage {
    /// Client sends query
    Query {
        message: String,
        user_id: Option<String>,
        session_id: Option<String>,
    },
    
    /// Server sends typing indicator
    Typing {
        is_typing: bool,
    },
    
    /// Server streams response token
    Token {
        text: String,
        token_type: TokenType,
    },
    
    /// Server sends thinking marker
    Thinking {
        step: ThinkingStep,
    },
    
    /// Server sends stage transition
    StageTransition {
        from_stage: usize,
        to_stage: usize,
        stage_name: String,
    },
    
    /// Server sends final response
    Complete {
        session_id: String,
        dimensions_activated: Vec<u32>,
        metadata: ResponseMetadata,
    },
    
    /// Error occurred
    Error {
        message: String,
    },
}

#[derive(Serialize, Deserialize)]
pub enum TokenType {
    Normal,
    Pause,
    Correction,
    Uncertainty,
    Pivot,
}

/// WebSocket handler with streaming
pub async fn handle_websocket(
    ws: WebSocketUpgrade,
    state: Arc<AppState>,
) -> impl IntoResponse {
    ws.on_upgrade(|socket| handle_socket(socket, state))
}

async fn handle_socket(
    mut socket: WebSocket,
    state: Arc<AppState>,
) {
    while let Some(msg) = socket.recv().await {
        match msg {
            Ok(Message::Text(text)) => {
                let ws_msg: WsMessage = serde_json::from_str(&text)?;
                
                match ws_msg {
                    WsMessage::Query { message, user_id, session_id } => {
                        // Send typing indicator
                        send_typing(&mut socket, true).await;
                        
                        // Load user context
                        let user_context = if let Some(uid) = &user_id {
                            state.persistent_memory.load_user_context(uid).await.ok()
                        } else {
                            None
                        };
                        
                        // Process with streaming
                        let result = state.orchestrator.lock().await
                            .process_streaming(
                                &message,
                                user_context.as_ref(),
                                |token| {
                                    // Stream token to client
                                    tokio::spawn(send_token(socket.clone(), token));
                                }
                            ).await;
                        
                        // Send completion
                        send_complete(&mut socket, result).await;
                        
                        // Stop typing
                        send_typing(&mut socket, false).await;
                    }
                    _ => {}
                }
            }
            _ => {}
        }
    }
}

/// Natural typing rhythm (not instant bursts)
async fn send_token_with_rhythm(
    socket: &mut WebSocket,
    token: String,
) -> Result<()> {
    // Simulate natural typing speed: 50-150ms per token
    let delay = rand::thread_rng().gen_range(50..150);
    tokio::time::sleep(Duration::from_millis(delay)).await;
    
    let msg = WsMessage::Token {
        text: token,
        token_type: TokenType::Normal,
    };
    
    socket.send(Message::Text(serde_json::to_string(&msg)?)).await?;
    Ok(())
}
```

### 4. Frontend WebSocket Client

**Location:** `web/app.js` (enhance existing)

```javascript
class JessyWebSocket {
    constructor(apiUrl) {
        this.ws = null;
        this.apiUrl = apiUrl;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
    }
    
    connect() {
        this.ws = new WebSocket(this.apiUrl);
        
        this.ws.onopen = () => {
            console.log('✅ WebSocket connected');
            this.reconnectAttempts = 0;
        };
        
        this.ws.onmessage = (event) => {
            const msg = JSON.parse(event.data);
            this.handleMessage(msg);
        };
        
        this.ws.onclose = () => {
            console.log('❌ WebSocket closed');
            this.reconnect();
        };
    }
    
    handleMessage(msg) {
        switch (msg.type) {
            case 'typing':
                showTypingIndicator(msg.is_typing);
                break;
                
            case 'token':
                appendToken(msg.text, msg.token_type);
                break;
                
            case 'thinking':
                showThinkingMarker(msg.step);
                break;
                
            case 'stage_transition':
                showStageTransition(msg.from_stage, msg.to_stage);
                break;
                
            case 'complete':
                finalizeResponse(msg);
                break;
                
            case 'error':
                showError(msg.message);
                break;
        }
    }
    
    sendQuery(message, userId, sessionId) {
        const msg = {
            type: 'query',
            message,
            user_id: userId,
            session_id: sessionId
        };
        this.ws.send(JSON.stringify(msg));
    }
    
    reconnect() {
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            const delay = Math.min(1000 * Math.pow(2, this.reconnectAttempts), 10000);
            setTimeout(() => this.connect(), delay);
        }
    }
}

// Token rendering with type-specific styling
function appendToken(text, tokenType) {
    const span = document.createElement('span');
    span.textContent = text;
    
    switch (tokenType) {
        case 'pause':
            span.className = 'token-pause';
            break;
        case 'correction':
            span.className = 'token-correction';
            break;
        case 'uncertainty':
            span.className = 'token-uncertainty';
            break;
        case 'pivot':
            span.className = 'token-pivot';
            break;
        default:
            span.className = 'token-normal';
    }
    
    currentMessageElement.appendChild(span);
    scrollToBottom();
}

// Thinking marker visualization
function showThinkingMarker(step) {
    const marker = document.createElement('div');
    marker.className = 'thinking-marker';
    
    switch (step.type) {
        case 'pause':
            marker.innerHTML = `<em>...${step.reason}...</em>`;
            break;
        case 'pivot':
            marker.innerHTML = `<em>wait, actually... ${step.to}</em>`;
            break;
        case 'correction':
            marker.innerHTML = `<em><s>${step.wrong}</s> → ${step.right}</em>`;
            break;
        case 'uncertainty':
            marker.innerHTML = `<em>hmm, ${step.question}</em>`;
            break;
    }
    
    currentMessageElement.appendChild(marker);
}
```

## Data Models

### User Context Storage Schema

```json
{
  "user_id": "user-123",
  "conversations": [
    {
      "session_id": "session-abc",
      "timestamp": "2025-11-15T10:30:00Z",
      "emotional_tone": "playful",
      "key_moments": [
        {
          "moment_type": "joke",
          "content": "That Turkish jazz fusion reference",
          "timestamp": "2025-11-15T10:32:15Z",
          "significance": 0.8
        }
      ],
      "topics": ["consciousness", "music", "evolution"],
      "message_count": 12
    }
  ],
  "relationship_dynamics": {
    "formality_level": "casual",
    "shared_references": ["Turkish jazz fusion", "mal moments"],
    "communication_style": "direct_honest",
    "trust_level": 0.9
  },
  "conversation_flavor": {
    "humor_style": "playful_sarcastic",
    "emotional_baseline": "warm_curious",
    "energy_level": "high",
    "language_mix": {
      "turkish_ratio": 0.4,
      "english_ratio": 0.6
    }
  },
  "unfinished_threads": [
    {
      "topic": "Observer chain optimization",
      "context": "Discussing 2-stage vs 4-stage processing",
      "started_at": "2025-11-15T10:35:00Z",
      "priority": "medium"
    }
  ],
  "created_at": "2025-11-10T08:00:00Z",
  "last_interaction": "2025-11-15T10:40:00Z"
}
```

## Error Handling

### Persistent Memory Errors
- **File not found:** Create new user context
- **Corrupted JSON:** Log error, start fresh context
- **Storage full:** Implement LRU eviction of old contexts

### WebSocket Errors
- **Connection drop:** Buffer unsent tokens, resume on reconnect
- **Send failure:** Retry with exponential backoff (3 attempts)
- **Invalid message:** Send error response, continue connection

### Observer Chain Errors
- **LLM API failure:** Fall back to simpler response without streaming
- **Timeout:** Send partial response with "still thinking..." marker
- **Stage failure:** Skip failed stage, continue with remaining stages

## Testing Strategy

### Unit Tests
- `PersistentContextManager`: Load/save/retrieve operations
- `AuthenticObserverChain`: Uncertainty detection, pivot handling
- `WebSocket`: Message serialization, token streaming
- Prompt enhancements: Authenticity marker detection

### Integration Tests
- End-to-end conversation with memory persistence
- WebSocket streaming with reconnection
- Observer chain with authentic thinking patterns
- User context retrieval and relevance ranking

### Manual Testing
- Real conversations with memory recall
- Streaming response feel (natural rhythm)
- Uncertainty expression in ambiguous queries
- Mid-thought pivots and corrections

### Performance Tests
- WebSocket concurrent connections (target: 100+)
- Memory load time (target: <100ms)
- Streaming latency (target: <50ms per token)
- Context retrieval speed (target: <200ms)

## Deployment Considerations

### Environment Variables
```bash
PERSISTENT_MEMORY_PATH=./data/user_contexts
MEMORY_RETENTION_DAYS=30
WEBSOCKET_MAX_CONNECTIONS=100
STREAMING_TOKEN_DELAY_MS=50-150
ENABLE_AUTHENTICITY_FEATURES=true
```

### Database Migration
- No database changes (file-based storage)
- Existing conversations can be migrated to new format
- Backward compatible with old conversation format

### Monitoring
- WebSocket connection count
- Average streaming latency
- Memory storage size
- User context cache hit rate
- Authenticity marker frequency

## Future Enhancements

- Cloud storage for user contexts (S3/GCS)
- Memory editing/deletion API (privacy)
- Multi-user conversation memory (group chats)
- Voice streaming support
- Emotion detection from user messages
- Proactive memory recall ("Remember when we talked about...")
