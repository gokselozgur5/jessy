#!/bin/bash
# Run integration tests with proper service orchestration
# This script:
# 1. Starts required services
# 2. Waits for them to be healthy
# 3. Runs integration tests
# 4. Cleans up regardless of test outcome

set -e

# Colors
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🧪 Running integration tests...${NC}"

# Cleanup function to ensure services are stopped
cleanup() {
    echo -e "${YELLOW}🧹 Cleaning up services...${NC}"
    docker-compose down 2>/dev/null || true
}

# Register cleanup on exit
trap cleanup EXIT

# Start services
echo -e "${YELLOW}⚠️  Starting required services...${NC}"
docker-compose up -d jessy-core jessy-api

# Wait for services to be healthy
echo -e "${YELLOW}⏳ Waiting for services to be healthy...${NC}"
TIMEOUT=60
INTERVAL=2
elapsed=0

while [ $elapsed -lt $TIMEOUT ]; do
    # Check jessy-core health
    core_health=$(docker inspect --format='{{.State.Health.Status}}' jessy-core 2>/dev/null || echo "starting")
    
    # Check jessy-api health
    api_health=$(docker inspect --format='{{.State.Health.Status}}' jessy-api 2>/dev/null || echo "starting")
    
    if [ "$core_health" = "healthy" ] && [ "$api_health" = "healthy" ]; then
        echo -e "${GREEN}✅ All services are healthy${NC}"
        break
    fi
    
    if [ "$core_health" = "unhealthy" ]; then
        echo -e "${RED}❌ jessy-core is unhealthy${NC}"
        docker-compose logs --tail=30 jessy-core
        exit 1
    fi
    
    if [ "$api_health" = "unhealthy" ]; then
        echo -e "${RED}❌ jessy-api is unhealthy${NC}"
        docker-compose logs --tail=30 jessy-api
        exit 1
    fi
    
    echo -e "  jessy-core: $core_health, jessy-api: $api_health"
    sleep $INTERVAL
    elapsed=$((elapsed + INTERVAL))
done

if [ $elapsed -ge $TIMEOUT ]; then
    echo -e "${RED}❌ Timeout waiting for services to become healthy${NC}"
    docker-compose logs jessy-core jessy-api
    exit 1
fi

# Run integration tests
echo -e "${YELLOW}🧪 Running integration tests...${NC}"
echo ""

if docker-compose run --rm integration-tests; then
    echo ""
    echo -e "${GREEN}✅ Integration tests passed${NC}"
    exit 0
else
    echo ""
    echo -e "${RED}❌ Integration tests failed${NC}"
    echo -e "${YELLOW}📋 Service logs:${NC}"
    docker-compose logs --tail=50 jessy-core jessy-api
    exit 1
fi
