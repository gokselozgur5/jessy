//! Server-Sent Events (SSE) for streaming responses

use actix_web::{web, HttpResponse, HttpRequest};
use actix_web::http::header;
use futures::stream::Stream;
use std::pin::Pin;
use std::task::{Context, Poll};
use tokio::sync::mpsc;

pub struct SseStream {
    rx: mpsc::UnboundedReceiver<String>,
}

impl Stream for SseStream {
    type Item = Result<web::Bytes, actix_web::Error>;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<Self::Item>> {
        match self.rx.poll_recv(cx) {
            Poll::Ready(Some(msg)) => {
                let data = format!("data: {}\n\n", msg);
                Poll::Ready(Some(Ok(web::Bytes::from(data))))
            }
            Poll::Ready(None) => Poll::Ready(None),
            Poll::Pending => Poll::Pending,
        }
    }
}

/// GET /api/chat/stream?message=...&session_id=... - Stream chat response
pub async fn chat_stream(
    _req: HttpRequest,
    query: web::Query<super::ChatRequest>,
    data: web::Data<super::AppState>,
) -> HttpResponse {
    let (tx, rx) = mpsc::unbounded_channel();

    // Spawn background task to process chat
    let message = query.message.clone();
    let session_id = query.session_id.clone();
    let api_key = std::env::var("ANTHROPIC_API_KEY").unwrap_or_default();

    actix_web::rt::spawn(async move {
        if let Err(e) = process_streaming_chat(&message, &session_id, &api_key, tx).await {
            eprintln!("Error processing streaming chat: {}", e);
        }
    });

    HttpResponse::Ok()
        .insert_header(header::ContentType(mime::TEXT_EVENT_STREAM))
        .insert_header(header::CacheControl(vec![header::CacheDirective::NoCache]))
        .insert_header(("X-Accel-Buffering", "no"))
        .streaming(SseStream { rx })
}

async fn process_streaming_chat(
    message: &str,
    _session_id: &Option<String>,
    api_key: &str,
    tx: mpsc::UnboundedSender<String>,
) -> Result<(), Box<dyn std::error::Error>> {
    use crate::llm::{LLMConfig, AnthropicProvider};
    use crate::navigation::DimensionSelector;

    // Create selector for dimension selection
    let selector = DimensionSelector::new(api_key.to_string());
    let selection = selector.select(message).await?;

    // Build prompts
    // TODO: Add PersonalityRAG context retrieval here when available
    let system_prompt = super::chat::build_system_prompt(&selection, None);
    let user_prompt = format!("CURRENT QUERY: {}", message);

    // Create provider
    let llm_config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-sonnet-4-20250514".to_string(),
        api_key: api_key.to_string(),
        timeout_secs: 30,
        max_retries: 3,
    };
    let anthropic = AnthropicProvider::new(&llm_config)?;

    // Stream response
    let _response = anthropic.call_api_streaming(
        &user_prompt,
        &system_prompt,
        |chunk| {
            // Send each chunk via SSE
            let json = serde_json::json!({
                "type": "chunk",
                "text": chunk,
            });
            let _ = tx.send(json.to_string());
        }
    ).await?;

    // Send completion signal
    let done = serde_json::json!({
        "type": "done",
    });
    let _ = tx.send(done.to_string());

    Ok(())
}
