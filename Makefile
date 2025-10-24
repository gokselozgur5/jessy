.PHONY: help up down build test test-unit test-integration test-bdd clean logs shell

# Colors for output
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[0;33m
RED := \033[0;31m
NC := \033[0m # No Color

help: ## Show this help message
	@echo "$(BLUE)🎪 Jessy Development Commands$(NC)"
	@echo ""
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "$(GREEN)%-20s$(NC) %s\n", $$1, $$2}'

up: ## Start all services (docker-compose up)
	@echo "$(BLUE)🎪 Starting the maestro orchestra...$(NC)"
	docker-compose up -d
	@echo "$(GREEN)✅ All services are up!$(NC)"
	@echo "$(YELLOW)Rust Core: http://localhost:8080$(NC)"
	@echo "$(YELLOW)Go API: http://localhost:3000$(NC)"

down: ## Stop all services
	@echo "$(BLUE)🛑 Stopping services...$(NC)"
	docker-compose down
	@echo "$(GREEN)✅ All services stopped$(NC)"

build: ## Build all Docker images
	@echo "$(BLUE)🔨 Building images...$(NC)"
	docker-compose build
	@echo "$(GREEN)✅ Build complete$(NC)"

test: ## Run all tests in containers
	@echo "$(BLUE)🧪 Running all tests...$(NC)"
	docker-compose run --rm test-runner
	@echo "$(GREEN)✅ Tests complete$(NC)"

test-unit: ## Run unit tests only
	@echo "$(BLUE)🧪 Running unit tests...$(NC)"
	docker-compose run --rm test-runner cargo test --lib --all-features
	@echo "$(GREEN)✅ Unit tests complete$(NC)"

test-integration: ## Run integration tests
	@echo "$(BLUE)🧪 Running integration tests...$(NC)"
	docker-compose run --rm test-runner cargo test --test '*' --all-features
	@echo "$(GREEN)✅ Integration tests complete$(NC)"

test-bdd: ## Run BDD tests
	@echo "$(BLUE)🧪 Running BDD tests...$(NC)"
	docker-compose run --rm bdd-tests
	@echo "$(GREEN)✅ BDD tests complete$(NC)"

clean: ## Clean up containers, volumes, and build artifacts
	@echo "$(BLUE)🧹 Cleaning up...$(NC)"
	docker-compose down -v
	docker system prune -f
	@echo "$(GREEN)✅ Cleanup complete$(NC)"

logs: ## Show logs from all services
	docker-compose logs -f

logs-rust: ## Show logs from Rust service
	docker-compose logs -f jessy-core

logs-go: ## Show logs from Go API
	docker-compose logs -f jessy-api

logs-json: ## Show logs in JSON format (structured)
	@echo "$(BLUE)📋 Showing structured logs...$(NC)"
	docker-compose logs --no-color | grep -E '^\{.*\}$$' || docker-compose logs

logs-filter: ## Filter logs by service label
	@echo "$(BLUE)🔍 Filtering logs by service...$(NC)"
	@echo "$(YELLOW)Jessy Core logs:$(NC)"
	docker inspect --format='{{.Config.Labels.service}}' jessy-core 2>/dev/null || echo "Service not running"
	@echo ""
	@echo "$(YELLOW)Jessy API logs:$(NC)"
	docker inspect --format='{{.Config.Labels.service}}' jessy-api 2>/dev/null || echo "Service not running"

logs-test: ## Test log aggregation and structured logging
	@echo "$(BLUE)🧪 Testing log aggregation...$(NC)"
	@echo ""
	@echo "$(YELLOW)1. Starting services...$(NC)"
	@docker-compose up -d
	@sleep 5
	@echo ""
	@echo "$(YELLOW)2. Generating test traffic...$(NC)"
	@curl -s http://localhost:8080/health > /dev/null && echo "✓ Rust health check" || echo "✗ Rust health check failed"
	@curl -s http://localhost:8080/status > /dev/null && echo "✓ Rust status check" || echo "✗ Rust status check failed"
	@curl -s http://localhost:3000/api/health > /dev/null && echo "✓ Go health check" || echo "✗ Go health check failed"
	@curl -s http://localhost:3000/api/v1/status > /dev/null && echo "✓ Go status check" || echo "✗ Go status check failed"
	@echo ""
	@echo "$(YELLOW)3. Aggregated logs from all services:$(NC)"
	@docker-compose logs --tail=50
	@echo ""
	@echo "$(GREEN)✅ Log aggregation test complete$(NC)"

shell-rust: ## Open shell in Rust container
	docker-compose exec jessy-core /bin/bash

shell-go: ## Open shell in Go API container
	docker-compose exec jessy-api /bin/sh

fmt: ## Format code
	@echo "$(BLUE)🎨 Formatting code...$(NC)"
	docker-compose run --rm rust-dev cargo fmt --all
	@echo "$(GREEN)✅ Code formatted$(NC)"

clippy: ## Run clippy linter
	@echo "$(BLUE)📎 Running clippy...$(NC)"
	docker-compose run --rm rust-dev cargo clippy --all-features -- -D warnings
	@echo "$(GREEN)✅ Clippy checks passed$(NC)"

ci: fmt clippy test ## Run full CI pipeline locally
	@echo "$(GREEN)✅ Full CI pipeline complete!$(NC)"

watch: ## Start services with hot reload
	@echo "$(BLUE)👀 Starting with hot reload...$(NC)"
	docker-compose up

ps: ## Show running containers
	docker-compose ps

restart: down up ## Restart all services

rebuild: down build up ## Rebuild and restart all services
