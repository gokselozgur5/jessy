//! WebSocket Streaming API for Real-Time Jessy Conversations
//!
//! Provides WebSocket endpoint for streaming responses with natural rhythm,
//! thinking markers, and stage transitions.

use actix::{Actor, ActorContext, AsyncContext, StreamHandler};
use actix_web::{web, Error, HttpRequest, HttpResponse};
use actix_web_actors::ws;
use serde::{Deserialize, Serialize};
use std::time::{Duration, Instant};
use uuid::Uuid;

/// WebSocket message types for client-server communication
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum WsMessage {
    /// Client sends a chat message
    Chat {
        message: String,
        user_id: Option<String>,
        session_id: Option<String>,
    },
    
    /// Server sends a token with optional styling
    Token {
        content: String,
        token_type: TokenType,
    },
    
    /// Server sends thinking marker (pause, pivot, correction)
    ThinkingMarker {
        marker_type: ThinkingMarkerType,
        content: Option<String>,
    },
    
    /// Server sends stage transition notification
    StageTransition {
        from_stage: String,
        to_stage: String,
    },
    
    /// Server sends typing indicator
    Typing {
        is_typing: bool,
    },
    
    /// Server sends completion notification
    Complete {
        session_id: String,
    },
    
    /// Server sends error
    Error {
        message: String,
    },
    
    /// Ping/Pong for connection health
    Ping,
    Pong,
}

/// Token types for different styling
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TokenType {
    Normal,
    Emphasis,
    Uncertainty,
    Correction,
    Technical,
}

/// Thinking marker types
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ThinkingMarkerType {
    Pause,
    Pivot,
    Correction,
    Uncertainty,
    Insight,
}

/// WebSocket actor for handling individual connections
pub struct JessyWebSocket {
    /// Unique connection ID
    id: Uuid,
    /// Last heartbeat time
    hb: Instant,
    /// User ID (if authenticated)
    user_id: Option<String>,
    /// Session ID for this conversation
    session_id: Option<String>,
}

impl JessyWebSocket {
    pub fn new() -> Self {
        Self {
            id: Uuid::new_v4(),
            hb: Instant::now(),
            user_id: None,
            session_id: None,
        }
    }

    /// Send heartbeat ping to client
    fn hb(&self, ctx: &mut ws::WebsocketContext<Self>) {
        ctx.run_interval(Duration::from_secs(5), |act, ctx| {
            if Instant::now().duration_since(act.hb) > Duration::from_secs(10) {
                // Client hasn't responded in 10 seconds, disconnect
                tracing::warn!("WebSocket client {} timed out, disconnecting", act.id);
                ctx.stop();
                return;
            }
            
            ctx.ping(b"");
        });
    }

    /// Send a message to the client
    fn send_message(&self, ctx: &mut ws::WebsocketContext<Self>, msg: WsMessage) {
        if let Ok(json) = serde_json::to_string(&msg) {
            ctx.text(json);
        }
    }

    /// Send a single token with natural rhythm delay
    fn send_token_with_rhythm(&self, ctx: &mut ws::WebsocketContext<Self>, token: String, token_type: TokenType) {
        use rand::Rng;
        let mut rng = rand::thread_rng();
        
        // Random delay between 50-150ms for natural typing rhythm
        let delay_ms = rng.gen_range(50..150);
        
        ctx.run_later(Duration::from_millis(delay_ms), move |act, ctx| {
            act.send_message(ctx, WsMessage::Token {
                content: token,
                token_type,
            });
        });
    }
}

/// Internal message for streaming tokens from async tasks
#[derive(actix::Message)]
#[rtype(result = "()")]
struct StreamToken {
    message: WsMessage,
}

impl actix::Handler<StreamToken> for JessyWebSocket {
    type Result = ();

    fn handle(&mut self, msg: StreamToken, ctx: &mut Self::Context) {
        self.send_message(ctx, msg.message);
    }
}

impl Actor for JessyWebSocket {
    type Context = ws::WebsocketContext<Self>;

    fn started(&mut self, ctx: &mut Self::Context) {
        tracing::info!("WebSocket connection {} started", self.id);
        self.hb(ctx);
    }

    fn stopped(&mut self, _ctx: &mut Self::Context) {
        tracing::info!("WebSocket connection {} stopped", self.id);
    }
}

/// Handle incoming WebSocket messages
impl StreamHandler<Result<ws::Message, ws::ProtocolError>> for JessyWebSocket {
    fn handle(&mut self, msg: Result<ws::Message, ws::ProtocolError>, ctx: &mut Self::Context) {
        match msg {
            Ok(ws::Message::Ping(msg)) => {
                self.hb = Instant::now();
                ctx.pong(&msg);
            }
            Ok(ws::Message::Pong(_)) => {
                self.hb = Instant::now();
            }
            Ok(ws::Message::Text(text)) => {
                self.hb = Instant::now();
                
                // Parse incoming message
                match serde_json::from_str::<WsMessage>(&text) {
                    Ok(WsMessage::Chat { message, user_id, session_id }) => {
                        tracing::info!("Received chat message from {}: {}", 
                            user_id.as_deref().unwrap_or("anonymous"), message);
                        
                        self.user_id = user_id.clone();
                        self.session_id = session_id.or_else(|| Some(Uuid::new_v4().to_string()));
                        
                        // Send typing indicator
                        self.send_message(ctx, WsMessage::Typing { is_typing: true });
                        
                        // Process through consciousness orchestrator
                        // Clone necessary data for async processing
                        let message_clone = message.clone();
                        let user_id_clone = user_id.clone();
                        let session_id_clone = self.session_id.clone();
                        let addr = ctx.address();
                        
                        // Spawn async task to process message
                        let fut = async move {
                            // TODO: Get AppState from somewhere - for now use test response
                            // This will be properly integrated when we wire up AppState to WebSocket
                            
                            // Simulate processing delay
                            tokio::time::sleep(Duration::from_millis(500)).await;
                            
                            // Test response for now
                            let response = format!(
                                "I received your message: \"{}\". WebSocket is connected to the handler! 🎉\n\n\
                                Next: wire up AppState to access the consciousness orchestrator.",
                                message_clone
                            );
                            
                            // Send stage transition
                            addr.do_send(StreamToken {
                                message: WsMessage::StageTransition {
                                    from_stage: "Navigation".to_string(),
                                    to_stage: "Response".to_string(),
                                },
                            });
                            
                            // Stream response word by word
                            for word in response.split_whitespace() {
                                addr.do_send(StreamToken {
                                    message: WsMessage::Token {
                                        content: format!("{} ", word),
                                        token_type: TokenType::Normal,
                                    },
                                });
                                tokio::time::sleep(Duration::from_millis(80)).await;
                            }
                            
                            // Send completion
                            addr.do_send(StreamToken {
                                message: WsMessage::Typing { is_typing: false },
                            });
                            addr.do_send(StreamToken {
                                message: WsMessage::Complete {
                                    session_id: session_id_clone.unwrap_or_default(),
                                },
                            });
                        };
                        
                        actix_web::rt::spawn(fut);
                    }
                    Ok(WsMessage::Ping) => {
                        self.send_message(ctx, WsMessage::Pong);
                    }
                    Err(e) => {
                        tracing::error!("Failed to parse WebSocket message: {}", e);
                        self.send_message(ctx, WsMessage::Error {
                            message: format!("Invalid message format: {}", e),
                        });
                    }
                    _ => {
                        tracing::warn!("Unexpected message type from client");
                    }
                }
            }
            Ok(ws::Message::Binary(_)) => {
                tracing::warn!("Binary messages not supported");
            }
            Ok(ws::Message::Close(reason)) => {
                tracing::info!("WebSocket connection {} closed: {:?}", self.id, reason);
                ctx.stop();
            }
            _ => {}
        }
    }
}

/// WebSocket upgrade handler
pub async fn websocket_handler(
    req: HttpRequest,
    stream: web::Payload,
) -> Result<HttpResponse, Error> {
    tracing::info!("WebSocket upgrade request from {:?}", req.peer_addr());
    
    let ws = JessyWebSocket::new();
    let resp = ws::start(ws, &req, stream)?;
    
    Ok(resp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ws_message_serialization() {
        let msg = WsMessage::Token {
            content: "Hello".to_string(),
            token_type: TokenType::Normal,
        };
        
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("token"));
        assert!(json.contains("Hello"));
    }

    #[test]
    fn test_thinking_marker_serialization() {
        let msg = WsMessage::ThinkingMarker {
            marker_type: ThinkingMarkerType::Pause,
            content: Some("Hmm...".to_string()),
        };
        
        let json = serde_json::to_string(&msg).unwrap();
        assert!(json.contains("thinking_marker"));
        assert!(json.contains("pause"));
    }
}
