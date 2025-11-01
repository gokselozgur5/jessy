#!/bin/bash
# JESSY Complete Training - Quick Start Script

set -e

echo "🚀 JESSY Complete Training - Quick Start"
echo "========================================"
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "Cargo.toml" ]; then
    echo "❌ Error: Please run this script from the JESSY project root"
    exit 1
fi

cd training

# Step 1: Setup Python environment
echo -e "${BLUE}Step 1: Setting up Python environment...${NC}"
if [ ! -d "venv" ]; then
    python3 -m venv venv
    echo -e "${GREEN}✅ Virtual environment created${NC}"
else
    echo -e "${YELLOW}⚠️  Virtual environment already exists${NC}"
fi

source venv/bin/activate

# Step 2: Install dependencies
echo -e "\n${BLUE}Step 2: Installing dependencies...${NC}"
pip install --upgrade pip
pip install -r requirements-complete.txt
echo -e "${GREEN}✅ Dependencies installed${NC}"

# Step 3: Check for API key
echo -e "\n${BLUE}Step 3: Checking API key...${NC}"
if [ -z "$ANTHROPIC_API_KEY" ]; then
    if [ -f "../.env" ]; then
        source ../.env
    fi
    
    if [ -z "$ANTHROPIC_API_KEY" ]; then
        echo -e "${YELLOW}⚠️  ANTHROPIC_API_KEY not found${NC}"
        echo "Please set it in .env file or export it:"
        echo "  export ANTHROPIC_API_KEY=your-key-here"
        exit 1
    fi
fi
echo -e "${GREEN}✅ API key found${NC}"

# Step 4: Generate dataset
echo -e "\n${BLUE}Step 4: Generating training dataset...${NC}"
echo "This will take ~2 hours and cost ~$20 in API calls"
read -p "Continue? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python generate_complete_dataset.py
    echo -e "${GREEN}✅ Dataset generated${NC}"
else
    echo -e "${YELLOW}⚠️  Dataset generation skipped${NC}"
    echo "You can run it later with: python generate_complete_dataset.py"
fi

# Step 5: Train model
echo -e "\n${BLUE}Step 5: Training JESSY model...${NC}"
echo "This will take ~6 hours on M2 MacBook"
read -p "Start training now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    python train_jessy_complete.py \
        --model mlx-community/gemma-2b \
        --train datasets/jessy_complete_train.jsonl \
        --val datasets/jessy_complete_val.jsonl \
        --epochs 3 \
        --batch-size 4 \
        --output checkpoints/jessy-complete
    
    echo -e "${GREEN}✅ Training complete${NC}"
else
    echo -e "${YELLOW}⚠️  Training skipped${NC}"
    echo "You can run it later with:"
    echo "  python train_jessy_complete.py"
fi

# Step 6: Deploy to Ollama
echo -e "\n${BLUE}Step 6: Deploy to Ollama...${NC}"
if [ -d "checkpoints/jessy-complete/final" ]; then
    read -p "Deploy to Ollama now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        python deploy_jessy.py --model checkpoints/jessy-complete/final
        echo -e "${GREEN}✅ Deployed to Ollama${NC}"
        
        # Test
        echo -e "\n${BLUE}Testing deployed model...${NC}"
        ollama run jessy-complete "Merhaba JESSY! Dimensional navigation nedir?"
    fi
else
    echo -e "${YELLOW}⚠️  No trained model found, skipping deployment${NC}"
fi

# Summary
echo -e "\n${GREEN}========================================"
echo "🎉 JESSY Training Setup Complete!"
echo "========================================${NC}"
echo ""
echo "Next steps:"
echo "  1. Generate dataset: python generate_complete_dataset.py"
echo "  2. Train model: python train_jessy_complete.py"
echo "  3. Deploy: python deploy_jessy.py"
echo "  4. Test: ollama run jessy-complete"
echo ""
echo "For more details, see: ZERO_TO_JESSY_COMPLETE.md"
echo ""
