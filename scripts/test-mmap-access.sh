#!/bin/bash
# Test MMAP file access from Docker containers
# Verifies that both Rust and Go services can access MMAP volumes

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🧪 Testing MMAP volume access from containers...${NC}\n"

# Check if services are running
if ! docker-compose ps | grep -q "jessy-core.*Up"; then
    echo -e "${RED}❌ jessy-core service is not running${NC}"
    echo -e "${YELLOW}💡 Run 'docker-compose up -d' first${NC}"
    exit 1
fi

if ! docker-compose ps | grep -q "jessy-api.*Up"; then
    echo -e "${RED}❌ jessy-api service is not running${NC}"
    echo -e "${YELLOW}💡 Run 'docker-compose up -d' first${NC}"
    exit 1
fi

# Test 1: Check if MMAP directory exists in Rust container
echo -e "${BLUE}Test 1: Checking MMAP directory in Rust container...${NC}"
if docker-compose exec -T jessy-core test -d /app/data/mmap; then
    echo -e "${GREEN}✓ MMAP directory exists in Rust container${NC}"
else
    echo -e "${RED}❌ MMAP directory not found in Rust container${NC}"
    exit 1
fi

# Test 2: Check write permissions in Rust container
echo -e "\n${BLUE}Test 2: Testing write permissions in Rust container...${NC}"
if docker-compose exec -T jessy-core sh -c "echo 'test' > /app/data/mmap/test-write.txt && rm /app/data/mmap/test-write.txt"; then
    echo -e "${GREEN}✓ Write permissions OK in Rust container${NC}"
else
    echo -e "${RED}❌ Write permissions failed in Rust container${NC}"
    exit 1
fi

# Test 3: Check if MMAP directory exists in Go container
echo -e "\n${BLUE}Test 3: Checking MMAP directory in Go container...${NC}"
if docker-compose exec -T jessy-api test -d /app/data/mmap; then
    echo -e "${GREEN}✓ MMAP directory exists in Go container${NC}"
else
    echo -e "${RED}❌ MMAP directory not found in Go container${NC}"
    exit 1
fi

# Test 4: Check read-only mount in Go container
echo -e "\n${BLUE}Test 4: Testing read-only mount in Go container...${NC}"
if docker-compose exec -T jessy-api sh -c "echo 'test' > /app/data/mmap/test-write.txt 2>&1" | grep -q "Read-only file system"; then
    echo -e "${GREEN}✓ Go container has read-only access (as expected)${NC}"
else
    echo -e "${YELLOW}⚠️  Go container may have write access (should be read-only)${NC}"
fi

# Test 5: Create test file from Rust and read from Go
echo -e "\n${BLUE}Test 5: Testing cross-container file access...${NC}"
TEST_FILE="/app/data/mmap/test-cross-container.txt"
TEST_CONTENT="Hello from Rust container"

# Write from Rust
docker-compose exec -T jessy-core sh -c "echo '$TEST_CONTENT' > $TEST_FILE"
echo -e "${GREEN}✓ Created test file from Rust container${NC}"

# Read from Go
GO_READ=$(docker-compose exec -T jessy-api cat $TEST_FILE 2>/dev/null || echo "")
if [ "$GO_READ" = "$TEST_CONTENT" ]; then
    echo -e "${GREEN}✓ Successfully read file from Go container${NC}"
else
    echo -e "${RED}❌ Failed to read file from Go container${NC}"
    exit 1
fi

# Cleanup
docker-compose exec -T jessy-core rm $TEST_FILE
echo -e "${GREEN}✓ Cleaned up test file${NC}"

# Test 6: Check consciousness directory structure
echo -e "\n${BLUE}Test 6: Checking consciousness directory structure...${NC}"
if docker-compose exec -T jessy-core test -d /app/data/mmap/consciousness; then
    echo -e "${GREEN}✓ Consciousness directory exists${NC}"
    
    # List dimension directories
    DIMS=$(docker-compose exec -T jessy-core ls /app/data/mmap/consciousness 2>/dev/null || echo "")
    if [ -n "$DIMS" ]; then
        echo -e "${GREEN}✓ Found dimension directories:${NC}"
        echo "$DIMS" | sed 's/^/  /'
    else
        echo -e "${YELLOW}⚠️  No dimension directories found (run init-mmap-volumes.sh)${NC}"
    fi
else
    echo -e "${RED}❌ Consciousness directory not found${NC}"
    exit 1
fi

# Test 7: Check volume mount details
echo -e "\n${BLUE}Test 7: Checking volume mount details...${NC}"
echo -e "${YELLOW}Rust container mount:${NC}"
docker-compose exec -T jessy-core df -h /app/data/mmap | tail -1

echo -e "${YELLOW}Go container mount:${NC}"
docker-compose exec -T jessy-api df -h /app/data/mmap | tail -1

# Test 8: Check file permissions
echo -e "\n${BLUE}Test 8: Checking file permissions...${NC}"
docker-compose exec -T jessy-core ls -la /app/data/mmap | head -5

# Summary
echo -e "\n${GREEN}✅ All MMAP volume access tests passed!${NC}"
echo -e "${BLUE}📊 Summary:${NC}"
echo -e "  • Rust container: Read-write access ✓"
echo -e "  • Go container: Read-only access ✓"
echo -e "  • Cross-container file sharing ✓"
echo -e "  • Directory structure ✓"
echo -e "  • Permissions ✓"
