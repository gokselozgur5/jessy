# ============================================
# Stage 1: Builder
# ============================================
FROM golang:1.23-alpine as builder

WORKDIR /app

# Install dependencies
RUN apk add --no-cache git

# Copy go mod file
COPY api/go.mod ./

# Download dependencies (cached layer)
# This will create go.sum if it doesn't exist
RUN go mod download

# Copy source
COPY api/ .

# Build
RUN CGO_ENABLED=0 GOOS=linux go build -a -installsuffix cgo -o jessy-api .

# ============================================
# Stage 2: Development
# ============================================
FROM golang:1.23-alpine as development

WORKDIR /app

# Install wget for health checks and air for hot reload
# Using v1.61.1 which is compatible with Go 1.23
RUN apk add --no-cache wget git && \
    go install github.com/air-verse/air@v1.61.1

# Create MMAP data directory with proper permissions
RUN mkdir -p /app/data/mmap && \
    chmod 755 /app/data/mmap

# Copy go mod file first for caching
COPY api/go.mod ./

# Download dependencies (cached layer)
# This will create go.sum if it doesn't exist
RUN go mod download

# Copy air configuration
COPY api/.air.toml ./.air.toml

# Copy source
COPY api/ .

# Ensure go.mod and go.sum are in sync
RUN go mod tidy

# Use air for hot reload in development
CMD ["air", "-c", ".air.toml"]

# ============================================
# Stage 3: Production
# ============================================
FROM alpine:latest as production

WORKDIR /app

# Install ca-certificates
RUN apk --no-cache add ca-certificates curl

# Copy binary
COPY --from=builder /app/jessy-api .

# Create non-root user and MMAP data directory
RUN adduser -D -u 1000 jessy && \
    mkdir -p /app/data/mmap && \
    chown -R jessy:jessy /app

USER jessy

EXPOSE 3000

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:3000/api/health || exit 1

CMD ["./jessy-api"]
