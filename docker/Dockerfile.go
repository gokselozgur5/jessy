# Go API development environment
FROM golang:1.21-alpine

# Install system dependencies
RUN apk add --no-cache \
    build-base \
    git \
    curl

# Set working directory
WORKDIR /app

# Copy go.mod and go.sum for dependency caching
COPY api/go.mod api/go.sum ./

# Download dependencies (cache layer)
RUN go mod download

# Install development tools
RUN go install github.com/cosmtrek/air@latest

# Copy application code
COPY api/ .

# Expose ports
EXPOSE 8080 8081

# Default command (can be overridden)
CMD ["go", "run", "."]
