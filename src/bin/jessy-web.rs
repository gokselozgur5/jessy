//! JESSY Web Chat Server
//!
//! REST API server for web-based chat interface

use actix_web::{middleware, web, App, HttpServer};
use actix_files as fs;
use actix_cors::Cors;
use jessy::api::{chat, sse, websocket, user_context, AppState};
use jessy::memory::PersistentContextManager;
use std::env;
use std::sync::Arc;
use tokio::sync::Mutex;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    // Initialize logging
    env_logger::init_from_env(env_logger::Env::new().default_filter_or("info"));

    // Get configuration from environment
    let api_key = env::var("ANTHROPIC_API_KEY")
        .expect("ANTHROPIC_API_KEY must be set");

    let host = env::var("HOST").unwrap_or_else(|_| "0.0.0.0".to_string());
    let port = env::var("PORT")
        .unwrap_or_else(|_| "8080".to_string())
        .parse::<u16>()
        .expect("PORT must be a valid number");

    eprintln!("🧠 JESSY Web Chat Server");
    eprintln!("========================");
    eprintln!("Starting server on {}:{}", host, port);

    // Initialize application state
    let app_state = web::Data::new(
        AppState::new(api_key.clone())
            .expect("Failed to initialize application state")
    );

    // Initialize persistent context manager
    let context_manager = web::Data::new(Arc::new(Mutex::new(
        PersistentContextManager::new(
            std::path::PathBuf::from("data/user_contexts"),
            100,  // max_cache_size
            30,   // retention_days
        ).expect("Failed to initialize persistent context manager")
    )));

    eprintln!("✅ Persistent context manager initialized");

    // Start HTTP server
    HttpServer::new(move || {
        // CORS configuration
        let cors = Cors::default()
            .allow_any_origin() // Allow GitHub Pages and any other origin
            .allow_any_method()
            .allow_any_header()
            .max_age(3600);

        let context_manager = context_manager.clone();
        
        App::new()
            .app_data(app_state.clone())
            .app_data(context_manager.clone())
            // Middleware
            .wrap(cors)
            .wrap(middleware::Logger::default())
            .wrap(middleware::Compress::default())
            // API routes
            .service(
                web::scope("/api")
                    .route("/health", web::get().to(chat::health))
                    .route("/chat", web::post().to(chat::chat))
                    .route("/chat/stream", web::get().to(sse::chat_stream))
                    .route("/ws", web::get().to(websocket::websocket_handler))
                    .route("/admin/reset", web::post().to(chat::reset_conversations))
                    .route("/user/{user_id}/context", web::get().to(user_context::get_user_context))
                    .route("/user/{user_id}/context/reset", web::post().to(user_context::reset_user_context))
            )
            // Static files (web UI)
            .service(fs::Files::new("/", "./web").index_file("index.html"))
    })
    .bind((host.as_str(), port))?
    .run()
    .await
}
