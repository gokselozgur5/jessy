# Jessy Development Makefile
# Optimized for MacBook M2 (Apple Silicon)

.PHONY: help up down restart logs test bench docs clean build

# Default target
.DEFAULT_GOAL := help

## help: Show this help message
help:
	@echo "Jessy Development Commands (MacBook M2)"
	@echo ""
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@sed -n 's/^##//p' ${MAKEFILE_LIST} | column -t -s ':' | sed -e 's/^/ /'

## up: Start development environment
up:
	docker-compose up rust-dev go-api

## down: Stop all services
down:
	docker-compose down

## restart: Restart all services
restart: down up

## logs: Show logs from all services
logs:
	docker-compose logs -f

## test: Run all tests
test:
	docker-compose --profile test up test-runner

## test-bdd: Run BDD integration tests
test-bdd:
	docker-compose --profile test up bdd-tests

## bench: Run performance benchmarks
bench:
	docker-compose --profile bench up benchmark

## docs: Generate and serve documentation
docs:
	docker-compose --profile docs up docs

## clean: Remove all containers and volumes
clean:
	docker-compose down -v
	docker system prune -f

## build: Rebuild all Docker images
build:
	docker-compose build --no-cache

## shell-rust: Open shell in Rust container
shell-rust:
	docker-compose run --rm rust-dev /bin/bash

## shell-go: Open shell in Go container
shell-go:
	docker-compose run --rm go-api /bin/sh

## check: Run cargo check
check:
	docker-compose run --rm rust-dev cargo check

## fmt: Format code
fmt:
	docker-compose run --rm rust-dev cargo fmt

## clippy: Run clippy linter
clippy:
	docker-compose run --rm rust-dev cargo clippy

## health: Check API health
health:
	@curl -s http://localhost:8080/api/v1/health | jq .

## status: Show container status
status:
	docker-compose ps

## platform: Show Docker platform info
platform:
	@echo "Docker Platform: $$(docker version --format '{{.Server.Os}}/{{.Server.Arch}}')"
	@echo "Expected: linux/arm64 (Apple Silicon)"
