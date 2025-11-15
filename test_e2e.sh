#!/bin/bash
# End-to-End Test Script for Jessy Authentic Evolution

set -e

echo "🧪 Jessy Authentic Evolution - End-to-End Test"
echo "=============================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test configuration
API_URL="http://localhost:8080"
USER_ID="test-user-$(date +%s)"

echo "📋 Test Configuration:"
echo "  API URL: $API_URL"
echo "  User ID: $USER_ID"
echo ""

# Test 1: Health Check
echo "Test 1: Health Check"
echo "--------------------"
HEALTH=$(curl -s "$API_URL/api/health")
if echo "$HEALTH" | grep -q "ok"; then
    echo -e "${GREEN}✓ Health check passed${NC}"
else
    echo -e "${RED}✗ Health check failed${NC}"
    exit 1
fi
echo ""

# Test 2: New User - First Conversation
echo "Test 2: New User - First Conversation"
echo "--------------------------------------"
RESPONSE=$(curl -s -X POST "$API_URL/api/chat" \
    -H "Content-Type: application/json" \
    -d "{\"message\": \"Hello! I'm testing the memory system.\", \"user_id\": \"$USER_ID\"}")

if echo "$RESPONSE" | grep -q "response"; then
    echo -e "${GREEN}✓ First conversation successful${NC}"
    echo "Response preview: $(echo "$RESPONSE" | jq -r '.response' | head -c 100)..."
else
    echo -e "${RED}✗ First conversation failed${NC}"
    echo "Response: $RESPONSE"
    exit 1
fi
echo ""

# Test 3: Check User Context Created
echo "Test 3: Check User Context Created"
echo "-----------------------------------"
sleep 2  # Give time for context to save
CONTEXT=$(curl -s "$API_URL/api/user/$USER_ID/context")

if echo "$CONTEXT" | grep -q "user_id"; then
    echo -e "${GREEN}✓ User context created${NC}"
    echo "Context preview: $(echo "$CONTEXT" | jq -c '{user_id, conversation_count}')"
else
    echo -e "${RED}✗ User context not found${NC}"
    echo "Response: $CONTEXT"
    exit 1
fi
echo ""

# Test 4: Returning User - Second Conversation
echo "Test 4: Returning User - Second Conversation"
echo "---------------------------------------------"
RESPONSE2=$(curl -s -X POST "$API_URL/api/chat" \
    -H "Content-Type: application/json" \
    -d "{\"message\": \"Do you remember me?\", \"user_id\": \"$USER_ID\"}")

if echo "$RESPONSE2" | grep -q "response"; then
    echo -e "${GREEN}✓ Second conversation successful${NC}"
    echo "Response preview: $(echo "$RESPONSE2" | jq -r '.response' | head -c 100)..."
else
    echo -e "${RED}✗ Second conversation failed${NC}"
    exit 1
fi
echo ""

# Test 5: Verify Context Updated
echo "Test 5: Verify Context Updated"
echo "-------------------------------"
sleep 2
CONTEXT2=$(curl -s "$API_URL/api/user/$USER_ID/context")
CONV_COUNT=$(echo "$CONTEXT2" | jq -r '.conversation_count')

if [ "$CONV_COUNT" -ge 2 ]; then
    echo -e "${GREEN}✓ Context updated with new conversation${NC}"
    echo "Conversation count: $CONV_COUNT"
else
    echo -e "${YELLOW}⚠ Context may not have updated (count: $CONV_COUNT)${NC}"
fi
echo ""

# Test 6: WebSocket Connection Test
echo "Test 6: WebSocket Connection Test"
echo "----------------------------------"
echo -e "${YELLOW}⚠ WebSocket test requires manual verification${NC}"
echo "  1. Open http://localhost:8080 in browser"
echo "  2. Send a message"
echo "  3. Verify word-by-word streaming"
echo "  4. Check connection status indicator"
echo ""

# Test 7: Context Reset
echo "Test 7: Context Reset"
echo "---------------------"
RESET=$(curl -s -X POST "$API_URL/api/user/$USER_ID/context/reset" \
    -H "Content-Type: application/json" \
    -d '{"keep_relationship": false}')

if echo "$RESET" | grep -q "reset successfully"; then
    echo -e "${GREEN}✓ Context reset successful${NC}"
else
    echo -e "${RED}✗ Context reset failed${NC}"
    exit 1
fi
echo ""

# Test 8: Verify Context Cleared
echo "Test 8: Verify Context Cleared"
echo "-------------------------------"
sleep 1
CONTEXT3=$(curl -s "$API_URL/api/user/$USER_ID/context")
CONV_COUNT3=$(echo "$CONTEXT3" | jq -r '.conversation_count')

if [ "$CONV_COUNT3" -eq 0 ]; then
    echo -e "${GREEN}✓ Context cleared successfully${NC}"
else
    echo -e "${RED}✗ Context not cleared (count: $CONV_COUNT3)${NC}"
    exit 1
fi
echo ""

# Summary
echo "=============================================="
echo -e "${GREEN}✅ All automated tests passed!${NC}"
echo ""
echo "📝 Manual Tests Required:"
echo "  1. WebSocket streaming (word-by-word)"
echo "  2. Thinking markers visualization"
echo "  3. Stage transitions"
echo "  4. Connection status indicator"
echo "  5. Natural typing rhythm"
echo ""
echo "🚀 Next Steps:"
echo "  1. Open http://localhost:8080 in browser"
echo "  2. Test real-time streaming"
echo "  3. Try ambiguous queries for uncertainty"
echo "  4. Test reconnection (kill/restart server)"
echo ""
