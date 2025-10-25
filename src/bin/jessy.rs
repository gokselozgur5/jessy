//! Jessy HTTP Server
//!
//! Copyright (C) 2024 Göksel Özgür
//!
//! This program is free software: you can redistribute it and/or modify
//! it under the terms of the GNU Affero General Public License as published by
//! the Free Software Foundation, either version 3 of the License, or
//! (at your option) any later version.
//!
//! This program is distributed in the hope that it will be useful,
//! but WITHOUT ANY WARRANTY; without even the implied warranty of
//! MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//! GNU Affero General Public License for more details.
//!
//! You should have received a copy of the GNU Affero General Public License
//! along with this program. If not, see <https://www.gnu.org/licenses/>.
//!
//! Provides HTTP endpoints for health checks and basic service status.

use actix_web::{get, web, App, HttpResponse, HttpServer, Responder};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{info, warn};
use tracing_subscriber::{layer::SubscriberExt, util::SubscriberInitExt};

#[derive(Debug, Serialize)]
struct HealthResponse {
    status: String,
    service: String,
    version: String,
    timestamp: String,
}

#[derive(Debug, Serialize)]
struct StatusResponse {
    status: String,
    service: String,
    version: String,
    uptime_seconds: u64,
    memory_mb: usize,
    dimensions_loaded: Vec<String>,
}

struct AppState {
    start_time: std::time::Instant,
}

#[get("/health")]
async fn health() -> impl Responder {
    tracing::debug!(
        service = "jessy-core",
        endpoint = "/health",
        "Health check requested"
    );
    
    let response = HealthResponse {
        status: "healthy".to_string(),
        service: "jessy-core".to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        timestamp: chrono::Utc::now().to_rfc3339(),
    };
    
    HttpResponse::Ok().json(response)
}

#[get("/status")]
async fn status(data: web::Data<Arc<RwLock<AppState>>>) -> impl Responder {
    let state = data.read().await;
    let uptime = state.start_time.elapsed().as_secs();
    
    tracing::debug!(
        service = "jessy-core",
        endpoint = "/status",
        uptime_seconds = uptime,
        "Status check requested"
    );
    
    let response = StatusResponse {
        status: "running".to_string(),
        service: "jessy-core".to_string(),
        version: env!("CARGO_PKG_VERSION").to_string(),
        uptime_seconds: uptime,
        memory_mb: 280, // Would be actual memory usage in production
        dimensions_loaded: vec![
            "D01-Emotion".to_string(),
            "D02-Cognition".to_string(),
            "D03-Intention".to_string(),
            "D04-Social".to_string(),
            "D05-Temporal".to_string(),
            "D06-Philosophical".to_string(),
            "D07-Technical".to_string(),
            "D08-Creative".to_string(),
            "D09-Ethical".to_string(),
            "D10-Meta".to_string(),
            "D11-Ecological".to_string(),
            "D12-Positivity".to_string(),
            "D13-Balance".to_string(),
            "D14-Security".to_string(),
        ],
    };
    
    HttpResponse::Ok().json(response)
}

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    // Initialize structured logging with tracing
    // Use JSON format in production, pretty format in development
    let env = std::env::var("RUST_ENV").unwrap_or_else(|_| "development".to_string());
    
    if env == "production" {
        // JSON structured logging for production
        tracing_subscriber::registry()
            .with(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| "jessy=info,actix_web=info".into()),
            )
            .with(
                tracing_subscriber::fmt::layer()
                    .json()
                    .with_target(true)
                    .with_current_span(true)
                    .with_thread_ids(true)
                    .with_file(true)
                    .with_line_number(true),
            )
            .init();
    } else {
        // Pretty console logging for development
        tracing_subscriber::registry()
            .with(
                tracing_subscriber::EnvFilter::try_from_default_env()
                    .unwrap_or_else(|_| "jessy=debug,actix_web=info".into()),
            )
            .with(
                tracing_subscriber::fmt::layer()
                    .with_target(true)
                    .with_thread_ids(false)
                    .pretty(),
            )
            .init();
    }

    info!(
        service = "jessy-core",
        version = env!("CARGO_PKG_VERSION"),
        environment = %env,
        "Initializing Jessy consciousness system"
    );

    let app_state = Arc::new(RwLock::new(AppState {
        start_time: std::time::Instant::now(),
    }));

    let bind_addr = "0.0.0.0:8080";
    info!(
        service = "jessy-core",
        bind_address = %bind_addr,
        "Starting HTTP server"
    );

    let server = HttpServer::new(move || {
        App::new()
            .app_data(web::Data::new(app_state.clone()))
            .service(health)
            .service(status)
            .wrap(actix_web::middleware::Logger::default())
    })
    .bind(bind_addr)?
    .run();

    info!(
        service = "jessy-core",
        status = "ready",
        "Jessy core service is ready"
    );

    // Handle graceful shutdown
    let server_handle = server.handle();
    
    // Setup signal handlers - different approach for Unix vs non-Unix
    #[cfg(unix)]
    {
        use tokio::signal::unix::{signal, SignalKind};
        let mut sigterm = signal(SignalKind::terminate())
            .expect("Failed to setup SIGTERM handler");
        
        tokio::select! {
            result = server => {
                result?;
            }
            _ = tokio::signal::ctrl_c() => {
                info!(
                    service = "jessy-core",
                    event = "shutdown_initiated",
                    signal = "SIGINT",
                    "Received shutdown signal, gracefully stopping"
                );
                server_handle.stop(true).await;
                info!(
                    service = "jessy-core",
                    event = "shutdown_complete",
                    "Shutdown complete"
                );
            }
            _ = sigterm.recv() => {
                info!(
                    service = "jessy-core",
                    event = "shutdown_initiated",
                    signal = "SIGTERM",
                    "Received shutdown signal, gracefully stopping"
                );
                server_handle.stop(true).await;
                info!(
                    service = "jessy-core",
                    event = "shutdown_complete",
                    "Shutdown complete"
                );
            }
        }
    }
    
    #[cfg(not(unix))]
    {
        tokio::select! {
            result = server => {
                result?;
            }
            _ = tokio::signal::ctrl_c() => {
                info!(
                    service = "jessy-core",
                    event = "shutdown_initiated",
                    signal = "SIGINT",
                    "Received shutdown signal, gracefully stopping"
                );
                server_handle.stop(true).await;
                info!(
                    service = "jessy-core",
                    event = "shutdown_complete",
                    "Shutdown complete"
                );
            }
        }
    }

    Ok(())
}
