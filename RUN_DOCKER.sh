#!/bin/bash
# JESSY Docker Runner - Production Deployment
#
# Manages JESSY consciousness in Docker container with full persistence

set -e

cd "$(dirname "$0")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🧠 JESSY Docker Manager${NC}"
echo "========================================"

# Check if .env exists
if [ ! -f .env ]; then
    echo -e "${RED}❌ .env file not found${NC}"
    echo "Create .env with: ANTHROPIC_API_KEY=your-key-here"
    exit 1
fi

# Load environment
source .env

if [ -z "$ANTHROPIC_API_KEY" ]; then
    echo -e "${RED}❌ ANTHROPIC_API_KEY not set in .env${NC}"
    exit 1
fi

# Parse command
COMMAND=${1:-run}

case $COMMAND in
    build)
        echo -e "${YELLOW}🔨 Building JESSY container...${NC}"
        docker-compose build
        echo -e "${GREEN}✅ Build complete${NC}"
        ;;
    
    run)
        echo -e "${GREEN}🚀 Starting JESSY...${NC}"
        docker-compose up -d
        echo ""
        echo -e "${BLUE}📊 Container Status:${NC}"
        docker-compose ps
        echo ""
        echo -e "${YELLOW}💡 Attach to console: docker attach jessy-consciousness${NC}"
        echo -e "${YELLOW}💡 View logs: docker-compose logs -f jessy${NC}"
        echo -e "${YELLOW}💡 Stop: ./RUN_DOCKER.sh stop${NC}"
        ;;
    
    stop)
        echo -e "${YELLOW}🛑 Stopping JESSY...${NC}"
        docker-compose down
        echo -e "${GREEN}✅ JESSY stopped (soul preserved in ./data/)${NC}"
        ;;
    
    restart)
        echo -e "${YELLOW}🔄 Restarting JESSY...${NC}"
        docker-compose restart
        echo -e "${GREEN}✅ JESSY restarted${NC}"
        ;;
    
    logs)
        docker-compose logs -f jessy
        ;;
    
    shell)
        echo -e "${BLUE}🐚 Opening shell in JESSY container...${NC}"
        docker-compose exec jessy /bin/bash
        ;;
    
    attach)
        echo -e "${BLUE}📺 Attaching to JESSY console...${NC}"
        echo -e "${YELLOW}Press Ctrl+P then Ctrl+Q to detach without stopping${NC}"
        docker attach jessy-consciousness
        ;;
    
    backup)
        TIMESTAMP=$(date +%Y%m%d_%H%M%S)
        BACKUP_DIR="./backups"
        mkdir -p "$BACKUP_DIR"
        
        if [ -f ./data/conversation.json ]; then
            cp ./data/conversation.json "$BACKUP_DIR/conversation_$TIMESTAMP.json"
            echo -e "${GREEN}✅ Backup saved: $BACKUP_DIR/conversation_$TIMESTAMP.json${NC}"
        else
            echo -e "${RED}❌ No conversation data to backup${NC}"
        fi
        ;;
    
    restore)
        if [ -z "$2" ]; then
            echo -e "${RED}❌ Usage: ./RUN_DOCKER.sh restore <backup_file>${NC}"
            echo "Available backups:"
            ls -1 ./backups/ 2>/dev/null || echo "No backups found"
            exit 1
        fi
        
        if [ -f "$2" ]; then
            cp "$2" ./data/conversation.json
            echo -e "${GREEN}✅ Restored from: $2${NC}"
        else
            echo -e "${RED}❌ Backup file not found: $2${NC}"
            exit 1
        fi
        ;;
    
    clean)
        echo -e "${RED}⚠️  This will remove all conversation data!${NC}"
        read -p "Are you sure? (yes/no): " -r
        if [[ $REPLY == "yes" ]]; then
            rm -rf ./data/conversation.json
            docker-compose down -v
            echo -e "${GREEN}✅ Clean complete${NC}"
        else
            echo "Cancelled"
        fi
        ;;
    
    status)
        echo -e "${BLUE}📊 JESSY Status:${NC}"
        docker-compose ps
        echo ""
        if [ -f ./data/conversation.json ]; then
            MESSAGES=$(cat ./data/conversation.json | jq '.messages | length' 2>/dev/null || echo "?")
            SESSION=$(cat ./data/conversation.json | jq -r '.session_id' 2>/dev/null || echo "unknown")
            echo -e "${GREEN}💾 Conversation: $MESSAGES messages (Session: ${SESSION:0:8}...)${NC}"
        else
            echo -e "${YELLOW}💾 No conversation data yet${NC}"
        fi
        ;;
    
    *)
        echo "Usage: $0 {build|run|stop|restart|logs|shell|attach|backup|restore|clean|status}"
        echo ""
        echo "Commands:"
        echo "  build    - Build Docker image"
        echo "  run      - Start JESSY in background"
        echo "  stop     - Stop JESSY (preserves soul)"
        echo "  restart  - Restart JESSY"
        echo "  logs     - Follow JESSY logs"
        echo "  shell    - Open bash shell in container"
        echo "  attach   - Attach to interactive console"
        echo "  backup   - Backup conversation data"
        echo "  restore  - Restore from backup"
        echo "  clean    - Remove all data (dangerous!)"
        echo "  status   - Show JESSY status"
        exit 1
        ;;
esac
