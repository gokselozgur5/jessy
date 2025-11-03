# Multi-stage build for JESSY - Context-Aware AI Assistant
# Stage 1: Build
# Updated: 2025-01-03 - Invalidate cache for conversation fix
FROM rust:1.83-slim as builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy manifests
COPY Cargo.toml Cargo.lock ./

# Copy source code and benches (needed for Cargo.toml validation)
COPY src ./src
COPY benches ./benches

# Build release binary for web server
RUN cargo build --release --bin jessy-web

# Stage 2: Runtime
FROM debian:bookworm-slim

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates \
    libssl3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy binary from builder
COPY --from=builder /app/target/release/jessy-web /app/jessy-web

# Copy web frontend
COPY web ./web

# Copy JESSY's soul - dimension and vocabulary data
COPY data ./data

# Create additional data directory for runtime persistence (conversations, etc)
RUN mkdir -p /app/data

# Set environment variables
ENV RUST_BACKTRACE=1
ENV RUST_LOG=info
ENV HOST=0.0.0.0
ENV PORT=8080

# Expose port
EXPOSE 8080

# Volume for persistent conversation storage
VOLUME ["/app/data"]

# Run JESSY Web Server
CMD ["/app/jessy-web"]
