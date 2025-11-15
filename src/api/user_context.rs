//! User Context API Endpoints
//!
//! Provides endpoints for managing persistent user contexts

use actix_web::{web, HttpResponse, Responder};
use serde::{Deserialize, Serialize};
use crate::memory::PersistentContextManager;
use std::sync::Arc;
use tokio::sync::Mutex;

#[derive(Debug, Serialize, Deserialize)]
pub struct UserContextSummary {
    pub user_id: String,
    pub conversation_count: usize,
    pub relationship_dynamics: RelationshipDynamicsSummary,
    pub conversation_flavor: ConversationFlavorSummary,
    pub unfinished_threads: Vec<UnfinishedThreadSummary>,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RelationshipDynamicsSummary {
    pub formality_level: String,
    pub humor_frequency: f32,
    pub energy_level: f32,
    pub trust_level: f32,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ConversationFlavorSummary {
    pub typical_topics: Vec<String>,
    pub communication_style: String,
    pub preferred_depth: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct UnfinishedThreadSummary {
    pub topic: String,
    pub priority: f32,
    pub mentioned_at: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ResetContextRequest {
    pub keep_relationship: bool,
}

/// GET /api/user/:user_id/context - Get user context summary
pub async fn get_user_context(
    user_id: web::Path<String>,
    app_state: web::Data<crate::api::AppState>,
) -> impl Responder {
    let manager = app_state.context_manager.lock().await;
    
    match manager.load_user_context(&user_id).await {
        Ok(context) => {
            // Build summary (don't expose full conversation history)
            let summary = UserContextSummary {
                user_id: user_id.to_string(),
                conversation_count: context.conversations.len(),
                relationship_dynamics: RelationshipDynamicsSummary {
                    formality_level: format!("{:?}", context.relationship_dynamics.formality_level),
                    humor_frequency: context.conversation_flavor.humor_style.as_ref()
                        .map(|_| 0.7).unwrap_or(0.0),  // Simplified
                    energy_level: match context.conversation_flavor.energy_level {
                        crate::memory::EnergyLevel::Low => 0.3,
                        crate::memory::EnergyLevel::Medium => 0.6,
                        crate::memory::EnergyLevel::High => 0.9,
                    },
                    trust_level: context.relationship_dynamics.trust_level,
                },
                conversation_flavor: ConversationFlavorSummary {
                    typical_topics: context.conversations.iter()
                        .flat_map(|c| c.topics.clone())
                        .collect::<std::collections::HashSet<_>>()
                        .into_iter()
                        .collect(),
                    communication_style: format!("{:?}", context.relationship_dynamics.communication_style),
                    preferred_depth: format!("{:?}", context.conversation_flavor.emotional_baseline),
                },
                unfinished_threads: context.unfinished_threads.iter().map(|thread| {
                    UnfinishedThreadSummary {
                        topic: thread.topic.clone(),
                        priority: match thread.priority {
                            crate::memory::ThreadPriority::Low => 0.3,
                            crate::memory::ThreadPriority::Medium => 0.6,
                            crate::memory::ThreadPriority::High => 0.9,
                        },
                        mentioned_at: thread.started_at.to_rfc3339(),
                    }
                }).collect(),
            };
            
            HttpResponse::Ok().json(summary)
        }
        Err(e) => {
            eprintln!("Failed to load user context: {}", e);
            HttpResponse::InternalServerError().json(serde_json::json!({
                "error": format!("Failed to load user context: {}", e)
            }))
        }
    }
}

/// POST /api/user/:user_id/context/reset - Reset user context
pub async fn reset_user_context(
    user_id: web::Path<String>,
    req: web::Json<ResetContextRequest>,
    app_state: web::Data<crate::api::AppState>,
) -> impl Responder {
    let mut manager = app_state.context_manager.lock().await;
    
    match manager.load_user_context(&user_id).await {
        Ok(mut context) => {
            // Clear conversations
            context.conversations.clear();
            context.unfinished_threads.clear();
            
            // Optionally keep relationship dynamics
            if !req.keep_relationship {
                context.relationship_dynamics = crate::memory::RelationshipDynamics::default();
                context.conversation_flavor = crate::memory::ConversationFlavor::default();
            }
            
            // Save updated context
            match manager.save_user_context(&context).await {
                Ok(_) => {
                    eprintln!("[User Context] Reset context for user: {}", user_id);
                    HttpResponse::Ok().json(serde_json::json!({
                        "message": "User context reset successfully",
                        "kept_relationship": req.keep_relationship
                    }))
                }
                Err(e) => {
                    eprintln!("Failed to save reset context: {}", e);
                    HttpResponse::InternalServerError().json(serde_json::json!({
                        "error": format!("Failed to save reset context: {}", e)
                    }))
                }
            }
        }
        Err(e) => {
            eprintln!("Failed to load user context for reset: {}", e);
            HttpResponse::InternalServerError().json(serde_json::json!({
                "error": format!("Failed to load user context: {}", e)
            }))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_user_context_summary_serialization() {
        let summary = UserContextSummary {
            user_id: "test_user".to_string(),
            conversation_count: 5,
            relationship_dynamics: RelationshipDynamicsSummary {
                formality_level: 0.3,
                humor_frequency: 0.7,
                energy_level: 0.8,
                trust_level: 0.9,
            },
            conversation_flavor: ConversationFlavorSummary {
                typical_topics: vec!["rust".to_string(), "ai".to_string()],
                communication_style: "casual".to_string(),
                preferred_depth: "deep".to_string(),
            },
            unfinished_threads: vec![],
        };
        
        let json = serde_json::to_string(&summary).unwrap();
        assert!(json.contains("test_user"));
        assert!(json.contains("rust"));
    }
}
