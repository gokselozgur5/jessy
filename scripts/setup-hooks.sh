#!/bin/bash
# Setup development hooks for Jessy project

set -e

BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔧 Setting up development hooks...${NC}"

# Check if pre-commit is installed
if ! command -v pre-commit &> /dev/null; then
    echo -e "${YELLOW}⚠️  pre-commit not found. Installing...${NC}"
    
    # Try pip
    if command -v pip3 &> /dev/null; then
        pip3 install pre-commit
    elif command -v pip &> /dev/null; then
        pip install pre-commit
    else
        echo -e "${RED}❌ pip not found. Please install Python and pip first.${NC}"
        echo -e "${YELLOW}💡 Visit: https://pip.pypa.io/en/stable/installation/${NC}"
        exit 1
    fi
fi

# Verify pre-commit is now available
if ! command -v pre-commit &> /dev/null; then
    echo -e "${RED}❌ Failed to install pre-commit${NC}"
    exit 1
fi

echo -e "${GREEN}✅ pre-commit is installed${NC}"

# Install pre-commit hooks
echo -e "${BLUE}📦 Installing pre-commit hooks...${NC}"
pre-commit install

# Install commit-msg hook
echo -e "${BLUE}📦 Installing commit-msg hook...${NC}"
pre-commit install --hook-type commit-msg

# Create secrets baseline if it doesn't exist
if [ ! -f .secrets.baseline ]; then
    echo -e "${BLUE}🔒 Creating secrets baseline...${NC}"
    detect-secrets scan > .secrets.baseline || true
fi

# Run hooks on all files to verify setup
echo -e "${BLUE}🧪 Testing hooks on all files...${NC}"
if pre-commit run --all-files; then
    echo -e "${GREEN}✅ All hooks passed!${NC}"
else
    echo -e "${YELLOW}⚠️  Some hooks failed. This is normal for first run.${NC}"
    echo -e "${YELLOW}💡 Run 'pre-commit run --all-files' to fix issues.${NC}"
fi

echo ""
echo -e "${GREEN}✅ Development hooks setup complete!${NC}"
echo ""
echo -e "${BLUE}📝 Hooks will now run automatically on:${NC}"
echo -e "  - ${GREEN}git commit${NC} (pre-commit hooks)"
echo -e "  - ${GREEN}git push${NC} (pre-push hooks)"
echo ""
echo -e "${BLUE}💡 Useful commands:${NC}"
echo -e "  - ${GREEN}pre-commit run --all-files${NC}  # Run all hooks manually"
echo -e "  - ${GREEN}pre-commit run <hook-id>${NC}    # Run specific hook"
echo -e "  - ${GREEN}git commit --no-verify${NC}      # Skip hooks (not recommended)"
echo -e "  - ${GREEN}pre-commit autoupdate${NC}       # Update hook versions"
echo ""
