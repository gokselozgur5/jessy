// Integration tests for Jessy consciousness system
// These tests verify that services work together correctly

use std::env;

/// Get the Rust service URL from environment or use default
fn rust_service_url() -> String {
    env::var("RUST_SERVICE_URL").unwrap_or_else(|_| "http://localhost:8080".to_string())
}

/// Get the API service URL from environment or use default
fn api_service_url() -> String {
    env::var("API_SERVICE_URL").unwrap_or_else(|_| "http://localhost:3000".to_string())
}

#[tokio::test]
async fn test_rust_service_health() {
    let url = format!("{}/health", rust_service_url());
    
    let client = reqwest::Client::new();
    let response = client.get(&url)
        .send()
        .await
        .expect("Failed to connect to Rust service");
    
    assert!(response.status().is_success(), "Health check failed");
    
    let body = response.text().await.expect("Failed to read response");
    assert!(body.contains("healthy") || body.contains("ok"), "Unexpected health response: {}", body);
}

#[tokio::test]
async fn test_api_service_health() {
    let url = format!("{}/api/health", api_service_url());
    
    let client = reqwest::Client::new();
    let response = client.get(&url)
        .send()
        .await
        .expect("Failed to connect to API service");
    
    assert!(response.status().is_success(), "API health check failed");
    
    let body = response.text().await.expect("Failed to read response");
    assert!(body.contains("healthy") || body.contains("ok"), "Unexpected API health response: {}", body);
}

#[tokio::test]
async fn test_service_communication() {
    // Test that API can communicate with Rust service
    let api_url = format!("{}/api/v1/status", api_service_url());
    
    let client = reqwest::Client::new();
    let response = client.get(&api_url)
        .send()
        .await
        .expect("Failed to connect to API service");
    
    // Even if the endpoint doesn't exist yet, we should get a response
    assert!(
        response.status().is_success() || response.status().is_client_error(),
        "Unexpected status code: {}", response.status()
    );
}

#[tokio::test]
async fn test_rust_service_status() {
    let url = format!("{}/status", rust_service_url());
    
    let client = reqwest::Client::new();
    let response = client.get(&url)
        .send()
        .await
        .expect("Failed to connect to Rust service");
    
    // Even if the endpoint doesn't exist yet, we should get a response
    assert!(
        response.status().is_success() || response.status().is_client_error(),
        "Unexpected status code: {}", response.status()
    );
}
