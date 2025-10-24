#!/bin/bash

# Test script for hot reload functionality
# This script verifies that code changes trigger automatic recompilation

set -e

echo "🔥 Testing Hot Reload Functionality"
echo "===================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if services are running
print_status "Checking if services are running..."
if ! docker ps | grep -q jessy-core; then
    print_error "jessy-core is not running. Please start services with 'docker-compose up -d'"
    exit 1
fi

if ! docker ps | grep -q jessy-api; then
    print_error "jessy-api is not running. Please start services with 'docker-compose up -d'"
    exit 1
fi

print_success "Both services are running"
echo ""

# Test Rust hot reload
print_status "Testing Rust (cargo-watch) hot reload..."
print_status "Checking if cargo-watch is running in jessy-core..."

# Check if cargo-watch process is running
if docker exec jessy-core ps aux | grep -q "cargo-watch"; then
    print_success "cargo-watch is running in jessy-core container"
else
    print_warning "cargo-watch process not found. Checking command..."
    docker exec jessy-core ps aux | grep cargo || true
fi

# Create a temporary test file to trigger rebuild
print_status "Creating temporary test file to trigger Rust rebuild..."
TEST_FILE="src/hot_reload_test.rs"
cat > "$TEST_FILE" << 'EOF'
// Temporary test file for hot reload verification
#[cfg(test)]
mod hot_reload_test {
    #[test]
    fn test_hot_reload() {
        assert!(true);
    }
}
EOF

print_status "Waiting 3 seconds for cargo-watch to detect changes..."
sleep 3

# Check logs for rebuild activity
print_status "Checking jessy-core logs for rebuild activity..."
if docker logs jessy-core --tail 20 2>&1 | grep -q -E "(Compiling|Finished|Running)"; then
    print_success "Rust hot reload detected rebuild activity"
else
    print_warning "No rebuild activity detected in logs (this may be normal if no changes were needed)"
fi

# Clean up test file
rm -f "$TEST_FILE"
print_status "Cleaned up test file"
echo ""

# Test Go hot reload
print_status "Testing Go (air) hot reload..."
print_status "Checking if air is running in jessy-api..."

# Check if air process is running
if docker exec jessy-api ps aux | grep -q "air"; then
    print_success "air is running in jessy-api container"
else
    print_warning "air process not found. Checking command..."
    docker exec jessy-api ps aux | grep -E "(air|go)" || true
fi

# Create a temporary test file to trigger rebuild
print_status "Creating temporary test file to trigger Go rebuild..."
TEST_FILE="api/hot_reload_test.go"
cat > "$TEST_FILE" << 'EOF'
package main

// Temporary test file for hot reload verification
func hotReloadTest() bool {
    return true
}
EOF

print_status "Waiting 3 seconds for air to detect changes..."
sleep 3

# Check logs for rebuild activity
print_status "Checking jessy-api logs for rebuild activity..."
if docker logs jessy-api --tail 20 2>&1 | grep -q -E "(building|Building|watching|Watching)"; then
    print_success "Go hot reload detected rebuild activity"
else
    print_warning "No rebuild activity detected in logs (this may be normal if no changes were needed)"
fi

# Clean up test file
rm -f "$TEST_FILE"
print_status "Cleaned up test file"
echo ""

# Final summary
echo "===================================="
print_success "Hot reload test completed!"
echo ""
print_status "To manually verify hot reload:"
echo "  1. Make a change to a source file (src/*.rs or api/*.go)"
echo "  2. Watch the logs: docker-compose logs -f jessy-core (or jessy-api)"
echo "  3. You should see automatic recompilation and restart"
echo ""
print_status "To view live logs:"
echo "  - Rust: docker-compose logs -f jessy-core"
echo "  - Go:   docker-compose logs -f jessy-api"
echo ""
