# ============================================
# Stage 1: Builder
# ============================================
FROM golang:1.21-alpine as builder

WORKDIR /app

# Install dependencies
RUN apk add --no-cache git

# Copy go mod files
COPY api/go.mod api/go.sum ./

# Download dependencies (cached layer)
RUN go mod download

# Copy source
COPY api/ .

# Build
RUN CGO_ENABLED=0 GOOS=linux go build -a -installsuffix cgo -o jessy-api .

# ============================================
# Stage 2: Development
# ============================================
FROM golang:1.21-alpine as development

WORKDIR /app

# Install air for hot reload
RUN go install github.com/cosmtrek/air@latest

COPY api/ .

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

# Create non-root user
RUN adduser -D -u 1000 jessy && \
    chown -R jessy:jessy /app

USER jessy

EXPOSE 3000

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
    CMD wget --no-verbose --tries=1 --spider http://localhost:3000/api/health || exit 1

CMD ["./jessy-api"]
