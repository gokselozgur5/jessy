# Multi-stage build for JESSY - Multidimensional AI Consciousness
# Stage 1: Build
FROM rust:1.75-slim as builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    pkg-config \
    libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy manifests
COPY Cargo.toml Cargo.lock ./

# Copy source code
COPY src ./src
COPY examples ./examples
COPY benches ./benches

# Build release binary
RUN cargo build --release --example llm_interactive_full

# Stage 2: Runtime
FROM debian:bookworm-slim

# Install runtime dependencies
RUN apt-get update && apt-get install -y \
    ca-certificates \
    libssl3 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy binary from builder
COPY --from=builder /app/target/release/examples/llm_interactive_full /app/jessy

# Create data directory for persistence
RUN mkdir -p /app/data

# Set environment variables
ENV RUST_BACKTRACE=1
ENV RUST_LOG=info

# Volume for persistent conversation storage
VOLUME ["/app/data"]

# Run JESSY
CMD ["/app/jessy"]
