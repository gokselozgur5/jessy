.PHONY: help up down build test test-unit test-integration test-bdd clean logs shell

# Docker Compose service name for Rust commands
RUST_SERVICE := unit-tests

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
	@docker-compose run --rm jessy-test
	@if [ $$? -eq 0 ]; then \
		echo "$(GREEN)✅ Tests complete$(NC)"; \
	else \
		echo "$(RED)❌ Tests failed$(NC)"; \
		exit 1; \
	fi

test-unit: ## Run unit tests only
	@echo "$(BLUE)🧪 Running unit tests...$(NC)"
	@docker-compose run --rm unit-tests
	@if [ $$? -eq 0 ]; then \
		echo "$(GREEN)✅ Unit tests complete$(NC)"; \
	else \
		echo "$(RED)❌ Unit tests failed$(NC)"; \
		exit 1; \
	fi

test-integration: ## Run integration tests
	@echo "$(BLUE)🧪 Running integration tests...$(NC)"
	@echo "$(YELLOW)⚠️  Starting required services...$(NC)"
	@docker-compose up -d jessy-core jessy-api
	@echo "$(YELLOW)⏳ Waiting for services to be healthy...$(NC)"
	@sleep 10
	@docker-compose run --rm integration-tests
	@if [ $$? -eq 0 ]; then \
		echo "$(GREEN)✅ Integration tests complete$(NC)"; \
		docker-compose down; \
	else \
		echo "$(RED)❌ Integration tests failed$(NC)"; \
		docker-compose down; \
		exit 1; \
	fi

test-integration-verbose: ## Run integration tests with enhanced health checking and logging
	@./scripts/run-integration-tests.sh

test-bdd: ## Run BDD tests
	@echo "$(BLUE)🧪 Running BDD tests...$(NC)"
	docker-compose run --rm bdd-tests
	@echo "$(GREEN)✅ BDD tests complete$(NC)"

coverage: ## Generate test coverage report
	@echo "$(BLUE)📊 Generating coverage report...$(NC)"
	@docker-compose run --rm coverage
	@if [ $$? -eq 0 ]; then \
		echo "$(GREEN)✅ Coverage report generated!$(NC)"; \
		echo "$(YELLOW)📄 Report available at: test-results/index.html$(NC)"; \
		echo "$(YELLOW)💡 Open with: open test-results/index.html$(NC)"; \
	else \
		echo "$(RED)❌ Coverage generation failed$(NC)"; \
		exit 1; \
	fi

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

cargo: ## Run cargo commands in Docker (usage: make cargo ARGS="test --lib")
	@if [ -z "$(ARGS)" ]; then \
		echo "$(RED)❌ Error: Please provide cargo arguments$(NC)"; \
		echo "$(YELLOW)Usage: make cargo ARGS=\"test --lib\"$(NC)"; \
		echo "$(YELLOW)Example: make cargo ARGS=\"build --release\"$(NC)"; \
		exit 1; \
	fi
	@echo "$(BLUE)🦀 Running: cargo $(ARGS)$(NC)"
	@echo "$(YELLOW)📦 Using service: $(RUST_SERVICE)$(NC)"
	@docker-compose run --rm $(RUST_SERVICE) cargo $(ARGS)

services: ## List all available Docker Compose services
	@echo "$(BLUE)📋 Available Docker Compose Services:$(NC)"
	@echo ""
	@echo "$(YELLOW)Main Services:$(NC)"
	@docker-compose config --services 2>/dev/null | while read service; do \
		echo "  • $$service"; \
	done
	@echo ""
	@echo "$(YELLOW)Test Services (profile: test):$(NC)"
	@docker-compose --profile test config --services 2>/dev/null | grep -E "(test|coverage|bdd)" | while read service; do \
		if [ "$$service" = "$(RUST_SERVICE)" ]; then \
			echo "  • $(GREEN)$$service$(NC) ⭐ (current Rust service)"; \
		else \
			echo "  • $$service"; \
		fi; \
	done
	@echo ""
	@echo "$(YELLOW)💡 Current Rust service: $(GREEN)$(RUST_SERVICE)$(NC)$(NC)"
	@echo "$(YELLOW)💡 To change, edit RUST_SERVICE in Makefile$(NC)"

fmt: ## Format code
	@echo "$(BLUE)🎨 Formatting code...$(NC)"
	@docker-compose run --rm $(RUST_SERVICE) cargo fmt --all
	@echo "$(GREEN)✅ Code formatted$(NC)"

clippy: ## Run clippy linter
	@echo "$(BLUE)📎 Running clippy...$(NC)"
	@docker compose run --rm $(RUST_SERVICE) cargo clippy --all-features -- -D warnings
	@echo "$(GREEN)✅ Clippy checks passed$(NC)"

ci: fmt clippy test ## Run full CI pipeline locally
	@echo "$(GREEN)✅ Full CI pipeline complete!$(NC)"

watch: ## Start services with hot reload
	@echo "$(BLUE)👀 Starting with hot reload...$(NC)"
	docker-compose up

test-hot-reload: ## Test hot reload functionality
	@echo "$(BLUE)🔥 Testing hot reload functionality...$(NC)"
	@./scripts/test-hot-reload.sh

bench: ## Run performance benchmarks
	@echo "$(BLUE)⚡ Running performance benchmarks...$(NC)"
	@docker-compose run --rm $(RUST_SERVICE) cargo bench --bench navigation_benchmarks --no-default-features
	@if [ $? -eq 0 ]; then \
		echo "$(GREEN)✅ Benchmarks complete!$(NC)"; \
		echo "$(YELLOW)📊 Reports available at: target/criterion/$(NC)"; \
		echo "$(YELLOW)💡 View HTML reports: open target/criterion/report/index.html$(NC)"; \
	else \
		echo "$(RED)❌ Benchmarks failed$(NC)"; \
		exit 1; \
	fi

bench-baseline: ## Save benchmark baseline for regression detection
	@echo "$(BLUE)📊 Saving benchmark baseline...$(NC)"
	@docker-compose run --rm $(RUST_SERVICE) cargo bench --bench navigation_benchmarks --no-default-features -- --save-baseline main
	@echo "$(GREEN)✅ Baseline saved!$(NC)"

bench-compare: ## Compare benchmarks against baseline
	@echo "$(BLUE)📊 Comparing benchmarks against baseline...$(NC)"
	@docker-compose run --rm $(RUST_SERVICE) cargo bench --bench navigation_benchmarks --no-default-features -- --baseline main
	@echo "$(GREEN)✅ Comparison complete!$(NC)"

bench-compare: ## Compare benchmarks against baseline
	@echo "$(BLUE)📊 Comparing benchmarks against baseline...$(NC)"
	@docker-compose run --rm jessy-test cargo bench --all-features -- --baseline main
	@echo "$(GREEN)✅ Comparison complete!$(NC)"

ps: ## Show running containers
	docker-compose ps

restart: down up ## Restart all services

rebuild: down build up ## Rebuild and restart all services

setup-hooks: ## Setup pre-commit hooks for development
	@echo "$(BLUE)🔧 Setting up development hooks...$(NC)"
	@./scripts/setup-hooks.sh

init-mmap: ## Initialize MMAP volume structure
	@echo "$(BLUE)🗂️  Initializing MMAP volumes...$(NC)"
	@./scripts/init-mmap-volumes.sh

test-mmap: ## Test MMAP volume access from containers
	@echo "$(BLUE)🧪 Testing MMAP volume access...$(NC)"
	@./scripts/test-mmap-access.sh

mmap-info: ## Show MMAP volume information
	@echo "$(BLUE)📊 MMAP Volume Information$(NC)"
	@echo ""
	@echo "$(YELLOW)Host directory:$(NC)"
	@ls -lah ./data/mmap 2>/dev/null || echo "  Not initialized (run 'make init-mmap')"
	@echo ""
	@echo "$(YELLOW)Docker volume:$(NC)"
	@docker volume inspect jessy_mmap-data 2>/dev/null | grep -A 5 "Mountpoint" || echo "  Volume not created (run 'docker-compose up')"
	@echo ""
	@echo "$(YELLOW)Container mounts:$(NC)"
	@docker-compose ps -q jessy-core 2>/dev/null | xargs -I {} docker inspect {} --format='  Rust: {{range .Mounts}}{{if eq .Destination "/app/data/mmap"}}{{.Source}} -> {{.Destination}} ({{.Mode}}){{end}}{{end}}' 2>/dev/null || echo "  Rust service not running"
	@docker-compose ps -q jessy-api 2>/dev/null | xargs -I {} docker inspect {} --format='  Go: {{range .Mounts}}{{if eq .Destination "/app/data/mmap"}}{{.Source}} -> {{.Destination}} ({{.Mode}}){{end}}{{end}}' 2>/dev/null || echo "  Go service not running"


cli: ## Run JESSY CLI interactively (requires .env with API key)
	@echo "$(BLUE)🤖 Starting JESSY CLI...$(NC)"
	@if [ ! -f .env ]; then \
		echo "$(RED)❌ .env file not found$(NC)"; \
		echo "$(YELLOW)💡 Copy .env.example to .env and add your API key$(NC)"; \
		exit 1; \
	fi
	@echo "$(YELLOW)Loading configuration from .env...$(NC)"
	@set -a && . ./.env && set +a && docker compose run --rm \
		-e ANTHROPIC_API_KEY="$$ANTHROPIC_API_KEY" \
		-e OPENAI_API_KEY="$$OPENAI_API_KEY" \
		-e LLM_PROVIDER="$$LLM_PROVIDER" \
		-e LLM_MODEL="$$LLM_MODEL" \
		-e RUST_ENV=development \
		unit-tests cargo run --bin jessy-cli

cli-native: ## Run JESSY CLI natively (no Docker, faster!)
	@echo "$(BLUE)🤖 Starting JESSY CLI (native)...$(NC)"
	@if [ ! -f .env ]; then \
		echo "$(RED)❌ .env file not found$(NC)"; \
		echo "$(YELLOW)💡 Copy .env.example to .env and add your API key$(NC)"; \
		exit 1; \
	fi
	@echo "$(YELLOW)Loading configuration from .env...$(NC)"
	@set -a && . ./.env && set +a && cargo run --bin jessy-cli

cli-release: ## Run JESSY CLI (release build, faster)
	@echo "$(BLUE)🤖 Starting JESSY CLI (release mode)...$(NC)"
	@if [ ! -f .env ]; then \
		echo "$(RED)❌ .env file not found$(NC)"; \
		echo "$(YELLOW)💡 Copy .env.example to .env and add your API key$(NC)"; \
		exit 1; \
	fi
	@echo "$(YELLOW)Loading configuration from .env...$(NC)"
	@set -a && . ./.env && set +a && docker compose run --rm \
		-e ANTHROPIC_API_KEY="$$ANTHROPIC_API_KEY" \
		-e OPENAI_API_KEY="$$OPENAI_API_KEY" \
		-e LLM_PROVIDER="$$LLM_PROVIDER" \
		-e LLM_MODEL="$$LLM_MODEL" \
		-e RUST_ENV=development \
		unit-tests cargo run --release --bin jessy-cli
