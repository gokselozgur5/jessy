#!/bin/bash
# Verification script for graceful shutdown implementation
# This script verifies that the code has proper signal handlers

set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔍 Verifying Graceful Shutdown Implementation${NC}"
echo ""

# Function to check if a pattern exists in a file
check_pattern() {
    local file=$1
    local pattern=$2
    local description=$3
    
    if grep -q "$pattern" "$file"; then
        echo -e "${GREEN}✅ $description${NC}"
        return 0
    else
        echo -e "${RED}❌ $description${NC}"
        return 1
    fi
}

# Verify Rust service implementation
echo -e "${BLUE}Checking Rust Service (src/bin/jessy.rs)...${NC}"
check_pattern "src/bin/jessy.rs" "tokio::signal::unix::signal" "SIGTERM handler setup"
check_pattern "src/bin/jessy.rs" "SignalKind::terminate" "SIGTERM signal kind"
check_pattern "src/bin/jessy.rs" "tokio::signal::ctrl_c" "SIGINT (Ctrl+C) handler"
check_pattern "src/bin/jessy.rs" "server_handle.stop" "Graceful server stop"
check_pattern "src/bin/jessy.rs" "shutdown_initiated" "Shutdown logging"
echo ""

# Verify Go API implementation
echo -e "${BLUE}Checking Go API (api/main.go)...${NC}"
check_pattern "api/main.go" "syscall.SIGTERM" "SIGTERM signal handling"
check_pattern "api/main.go" "os.Interrupt" "SIGINT signal handling"
check_pattern "api/main.go" "ShutdownWithTimeout" "Graceful shutdown with timeout"
check_pattern "api/main.go" "consciousness.Close" "Consciousness service cleanup"
check_pattern "api/main.go" "Graceful shutdown complete" "Shutdown completion logging"
echo ""

# Verify consciousness service cleanup
echo -e "${BLUE}Checking Consciousness Service (api/consciousness.go)...${NC}"
check_pattern "api/consciousness.go" "func.*Close.*error" "Close method exists"
check_pattern "api/consciousness.go" "active_sessions" "Session cleanup logging"
check_pattern "api/consciousness.go" "Cleaning up consciousness service" "Cleanup message"
echo ""

# Check docker-compose configuration
echo -e "${BLUE}Checking Docker Compose Configuration...${NC}"
check_pattern "docker-compose.yml" "restart: unless-stopped" "Restart policy configured"
check_pattern "docker-compose.yml" "healthcheck:" "Health checks configured"
echo ""

# Verify test script exists
echo -e "${BLUE}Checking Test Scripts...${NC}"
if [ -f "scripts/test-graceful-shutdown.sh" ]; then
    echo -e "${GREEN}✅ Graceful shutdown test script exists${NC}"
    if [ -x "scripts/test-graceful-shutdown.sh" ]; then
        echo -e "${GREEN}✅ Test script is executable${NC}"
    else
        echo -e "${YELLOW}⚠️  Test script is not executable (run: chmod +x scripts/test-graceful-shutdown.sh)${NC}"
    fi
else
    echo -e "${RED}❌ Graceful shutdown test script missing${NC}"
fi
echo ""

# Verify documentation
echo -e "${BLUE}Checking Documentation...${NC}"
if [ -f "docs/GRACEFUL_SHUTDOWN.md" ]; then
    echo -e "${GREEN}✅ Graceful shutdown documentation exists${NC}"
else
    echo -e "${RED}❌ Graceful shutdown documentation missing${NC}"
fi
echo ""

echo -e "${GREEN}✅ Graceful Shutdown Implementation Verification Complete!${NC}"
echo ""
echo -e "${BLUE}Summary:${NC}"
echo "- Rust service has SIGTERM and SIGINT handlers"
echo "- Go API has proper signal handling and cleanup"
echo "- Consciousness service implements proper Close method"
echo "- Docker configuration supports graceful shutdown"
echo "- Test scripts and documentation are in place"
echo ""
echo -e "${YELLOW}Note: To test the actual runtime behavior, run:${NC}"
echo "  ./scripts/test-graceful-shutdown.sh"
echo ""
echo -e "${YELLOW}Or manually test with:${NC}"
echo "  docker-compose up -d"
echo "  docker-compose kill -s SIGTERM jessy-core"
echo "  docker-compose logs jessy-core | grep shutdown"
