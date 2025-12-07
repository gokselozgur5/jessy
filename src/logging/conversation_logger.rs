//! Conversation Logger - Daily conversation tracking with IP-based analytics
//!
//! Logs all conversations to daily text files for feedback and improvement analysis.
//! Format: YYYY-MM-DD.txt with IP-based separators

use chrono::{DateTime, Utc};
use std::path::PathBuf;
use tokio::fs::OpenOptions;
use tokio::io::AsyncWriteExt;

/// Conversation logger for analytics and feedback
pub struct ConversationLogger {
    log_dir: PathBuf,
}

impl ConversationLogger {
    /// Create new conversation logger
    pub fn new(log_dir: PathBuf) -> Self {
        Self { log_dir }
    }

    /// Log a conversation exchange (user message + Jessy response)
    pub async fn log_conversation(
        &self,
        ip_address: &str,
        user_message: &str,
        jessy_response: &str,
        session_id: &str,
        user_id: Option<&str>,
    ) -> Result<(), std::io::Error> {
        // Create log directory if it doesn't exist
        tokio::fs::create_dir_all(&self.log_dir).await?;

        // Get today's log file name (YYYY-MM-DD.txt)
        let today = Utc::now().format("%Y-%m-%d").to_string();
        let log_file = self.log_dir.join(format!("{}.txt", today));

        // Format log entry
        let timestamp = Utc::now().to_rfc3339();
        let uid_str = user_id.unwrap_or("anonymous");

        let separator = "=".repeat(80);
        let log_entry = format!(
            r#"
{separator}
[{timestamp}] IP: {ip} | Session: {session} | User: {uid}
{separator}

USER:
{user_msg}

JESSY:
{jessy_msg}

{separator}

"#,
            separator = separator,
            timestamp = timestamp,
            ip = ip_address,
            session = session_id,
            uid = uid_str,
            user_msg = user_message,
            jessy_msg = jessy_response,
        );

        // Append to today's log file
        let mut file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(&log_file)
            .await?;

        file.write_all(log_entry.as_bytes()).await?;
        file.sync_all().await?;

        eprintln!("[ConversationLogger] ✅ Logged conversation from {} to {}", ip_address, log_file.display());

        Ok(())
    }

    /// Get daily summary statistics from log file
    pub async fn get_daily_summary(&self, date: &str) -> Result<DailySummary, std::io::Error> {
        let log_file = self.log_dir.join(format!("{}.txt", date));

        if !log_file.exists() {
            return Ok(DailySummary::default());
        }

        let content = tokio::fs::read_to_string(&log_file).await?;

        // Count unique IPs
        let mut unique_ips = std::collections::HashSet::new();
        let mut conversation_count = 0;

        for line in content.lines() {
            if line.contains("IP:") {
                if let Some(ip_part) = line.split("IP:").nth(1) {
                    if let Some(ip) = ip_part.split('|').next() {
                        unique_ips.insert(ip.trim().to_string());
                        conversation_count += 1;
                    }
                }
            }
        }

        Ok(DailySummary {
            date: date.to_string(),
            unique_ips: unique_ips.len(),
            total_conversations: conversation_count,
            log_file_size: content.len(),
        })
    }
}

/// Daily summary statistics
#[derive(Debug, Default)]
pub struct DailySummary {
    pub date: String,
    pub unique_ips: usize,
    pub total_conversations: usize,
    pub log_file_size: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_conversation_logger() {
        let temp_dir = std::env::temp_dir().join("jessy_test_logs");
        let logger = ConversationLogger::new(temp_dir.clone());

        let result = logger
            .log_conversation(
                "192.168.1.1",
                "What is consciousness?",
                "Consciousness is a fascinating topic...",
                "test-session-123",
                Some("test-user"),
            )
            .await;

        assert!(result.is_ok());

        // Cleanup
        tokio::fs::remove_dir_all(temp_dir).await.ok();
    }

    #[tokio::test]
    async fn test_daily_summary() {
        let temp_dir = std::env::temp_dir().join("jessy_test_logs_summary");
        let logger = ConversationLogger::new(temp_dir.clone());

        // Log a few conversations
        for i in 0..3 {
            logger
                .log_conversation(
                    &format!("192.168.1.{}", i),
                    "Test message",
                    "Test response",
                    &format!("session-{}", i),
                    None,
                )
                .await
                .unwrap();
        }

        let today = Utc::now().format("%Y-%m-%d").to_string();
        let summary = logger.get_daily_summary(&today).await.unwrap();

        assert_eq!(summary.unique_ips, 3);
        assert_eq!(summary.total_conversations, 3);

        // Cleanup
        tokio::fs::remove_dir_all(temp_dir).await.ok();
    }
}
