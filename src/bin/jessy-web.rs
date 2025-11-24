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
    
    // Persistent memory configuration
    let memory_path = env::var("PERSISTENT_MEMORY_PATH")
        .unwrap_or_else(|_| "./data/user_contexts".to_string());
    let memory_retention_days = env::var("MEMORY_RETENTION_DAYS")
        .unwrap_or_else(|_| "30".to_string())
        .parse::<i64>()
        .unwrap_or(30);
    let memory_cache_size = env::var("MEMORY_CACHE_SIZE")
        .unwrap_or_else(|_| "100".to_string())
        .parse::<usize>()
        .unwrap_or(100);
    
    // WebSocket configuration
    let ws_max_connections = env::var("WEBSOCKET_MAX_CONNECTIONS")
        .unwrap_or_else(|_| "1000".to_string())
        .parse::<usize>()
        .unwrap_or(1000);
    
    // Feature flags
    let enable_authenticity = env::var("ENABLE_AUTHENTICITY_FEATURES")
        .unwrap_or_else(|_| "true".to_string())
        .parse::<bool>()
        .unwrap_or(true);

    eprintln!("🧠 JESSY Web Chat Server");
    eprintln!("========================");
    eprintln!("Starting server on {}:{}", host, port);
    eprintln!("");
    eprintln!("Configuration:");
    eprintln!("  Memory Path: {}", memory_path);
    eprintln!("  Memory Retention: {} days", memory_retention_days);
    eprintln!("  Memory Cache Size: {} users", memory_cache_size);
    eprintln!("  WebSocket Max Connections: {}", ws_max_connections);
    eprintln!("  Authenticity Features: {}", if enable_authenticity { "enabled" } else { "disabled" });
    eprintln!("");

    // Initialize application state (includes context manager)
    let app_state = web::Data::new(
        AppState::new(api_key.clone()).await
            .expect("Failed to initialize application state")
    );

    // Start HTTP server
    HttpServer::new(move || {
        // CORS configuration
        let cors = Cors::default()
            .allow_any_origin() // Allow GitHub Pages and any other origin
            .allow_any_method()
            .allow_any_header()
            .max_age(3600);

        App::new()
            .app_data(app_state.clone())
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
