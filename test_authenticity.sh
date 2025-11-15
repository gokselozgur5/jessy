#!/bin/bash
# Test Authenticity Features
# Tests the authentic observer chain's ability to detect and preserve natural thinking patterns

set -e

echo "🎭 Jessy Authenticity Features Test"
echo "===================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "Test 1: Authenticity Marker Detection"
echo "--------------------------------------"

# Test uncertainty detection
echo "Testing uncertainty detection..."
cargo test --lib authentic_observer::tests::test_detect_uncertainty 2>&1 | grep -q "test result: ok"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Uncertainty detection works${NC}"
else
    echo -e "${RED}✗ Uncertainty detection failed${NC}"
fi

# Test pivot detection
echo "Testing pivot detection..."
cargo test --lib authentic_observer::tests::test_detect_pivot 2>&1 | grep -q "test result: ok"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Pivot detection works${NC}"
else
    echo -e "${RED}✗ Pivot detection failed${NC}"
fi

# Test correction detection
echo "Testing correction detection..."
cargo test --lib authentic_observer::tests::test_detect_correction 2>&1 | grep -q "test result: ok"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Correction detection works${NC}"
else
    echo -e "${RED}✗ Correction detection failed${NC}"
fi

echo ""

echo "Test 2: Response Parsing"
echo "------------------------"
cargo test --lib authentic_observer::tests::test_parse_response_with_confidence 2>&1 | grep -q "test result: ok"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Response parsing works${NC}"
else
    echo -e "${RED}✗ Response parsing failed${NC}"
fi

echo ""

echo "Test 3: No False Positives"
echo "--------------------------"
cargo test --lib authentic_observer::tests::test_no_markers 2>&1 | grep -q "test result: ok"
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ No false positives in clean text${NC}"
else
    echo -e "${RED}✗ False positive detection${NC}"
fi

echo ""

echo "===================================="
echo -e "${GREEN}✅ Authenticity feature tests complete!${NC}"
echo ""
echo "📝 What was tested:"
echo "  ✓ Uncertainty marker detection ('I don't know')"
echo "  ✓ Pivot marker detection ('wait, actually')"
echo "  ✓ Correction marker detection ('I mean')"
echo "  ✓ Response confidence parsing"
echo "  ✓ No false positives on clean text"
echo ""
echo "🎯 Authenticity Markers Detected:"
echo "  • Uncertainty: 'I don't know', 'I'm not sure', 'hmm'"
echo "  • Pivots: 'wait, actually', 'on second thought'"
echo "  • Corrections: 'I mean', 'rather', 'not X but Y'"
echo "  • Confusion: 'confused', 'doesn't make sense'"
echo ""
echo "💡 These markers are DETECTED, not FORCED"
echo "   The system recognizes natural patterns when they occur,"
echo "   but doesn't make Jessy perform authenticity."
echo ""
