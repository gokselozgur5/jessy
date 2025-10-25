#!/bin/bash
# Initialize MMAP volume structure for dimensional data
# This script creates the necessary directory structure and sets permissions

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🗂️  Initializing MMAP volume structure...${NC}"

# Base directory for MMAP data
MMAP_BASE_DIR="./data/mmap"

# Create base directory
if [ ! -d "$MMAP_BASE_DIR" ]; then
    echo -e "${YELLOW}Creating base directory: $MMAP_BASE_DIR${NC}"
    mkdir -p "$MMAP_BASE_DIR"
else
    echo -e "${GREEN}✓ Base directory exists: $MMAP_BASE_DIR${NC}"
fi

# Create consciousness subdirectory for dimensional layers
CONSCIOUSNESS_DIR="$MMAP_BASE_DIR/consciousness"
if [ ! -d "$CONSCIOUSNESS_DIR" ]; then
    echo -e "${YELLOW}Creating consciousness directory: $CONSCIOUSNESS_DIR${NC}"
    mkdir -p "$CONSCIOUSNESS_DIR"
else
    echo -e "${GREEN}✓ Consciousness directory exists: $CONSCIOUSNESS_DIR${NC}"
fi

# Create dimension directories (D01-D14 for core dimensions)
echo -e "${BLUE}Creating dimension directories...${NC}"
for i in $(seq -f "%02g" 1 14); do
    DIM_DIR="$CONSCIOUSNESS_DIR/D$i"
    if [ ! -d "$DIM_DIR" ]; then
        mkdir -p "$DIM_DIR"
        echo -e "${GREEN}✓ Created D$i${NC}"
    else
        echo -e "${GREEN}✓ D$i exists${NC}"
    fi
done

# Create proto-dimensions directory for learning system
PROTO_DIR="$MMAP_BASE_DIR/proto"
if [ ! -d "$PROTO_DIR" ]; then
    echo -e "${YELLOW}Creating proto-dimensions directory: $PROTO_DIR${NC}"
    mkdir -p "$PROTO_DIR"
else
    echo -e "${GREEN}✓ Proto-dimensions directory exists: $PROTO_DIR${NC}"
fi

# Create temp directory for MMAP operations
TEMP_DIR="$MMAP_BASE_DIR/temp"
if [ ! -d "$TEMP_DIR" ]; then
    echo -e "${YELLOW}Creating temp directory: $TEMP_DIR${NC}"
    mkdir -p "$TEMP_DIR"
else
    echo -e "${GREEN}✓ Temp directory exists: $TEMP_DIR${NC}"
fi

# Set permissions (755 for directories, 644 for files)
echo -e "${BLUE}Setting permissions...${NC}"
chmod -R 755 "$MMAP_BASE_DIR"

# Create a README in the MMAP directory
cat > "$MMAP_BASE_DIR/README.md" << 'EOF'
# MMAP Volume Structure

This directory contains memory-mapped files for the Jessy consciousness system.

## Directory Structure

```
data/mmap/
├── consciousness/       # Core dimensional layers
│   ├── D01/            # Dimension 01 (Emotion)
│   ├── D02/            # Dimension 02 (Cognition)
│   ├── ...
│   └── D14/            # Dimension 14 (Security)
├── proto/              # Proto-dimensions (learning system)
└── temp/               # Temporary MMAP operations
```

## File Format

Each dimension directory contains:
- `region.mmap` - Memory-mapped region file
- `index.json` - Layer index metadata
- `layers/` - Individual layer files (if needed)

## Permissions

- Directories: 755 (rwxr-xr-x)
- Files: 644 (rw-r--r--)
- Owner: User running Docker containers (UID 1000)

## Usage

The MMAP volume is mounted to:
- Rust service: `/app/data/mmap` (read-write)
- Go API: `/app/data/mmap` (read-only)
- Test containers: `/app/data/mmap` (read-write)

## Maintenance

- Clean temp directory periodically
- Monitor disk usage
- Backup consciousness directory regularly
- Proto-dimensions can be deleted after crystallization

## Notes

- Do not manually edit MMAP files
- Use the Jessy API for all data operations
- MMAP files are platform-specific (not portable)
- Ensure sufficient disk space for dimensional growth
EOF

echo -e "${GREEN}✓ Created README.md${NC}"

# Create a .gitignore to exclude MMAP data from git
cat > "$MMAP_BASE_DIR/.gitignore" << 'EOF'
# Ignore all MMAP data files
*.mmap
*.dat

# Ignore temporary files
temp/*

# Ignore proto-dimensions
proto/*

# Keep directory structure
!.gitignore
!README.md
EOF

echo -e "${GREEN}✓ Created .gitignore${NC}"

# Display directory structure
echo -e "\n${BLUE}📁 Directory structure:${NC}"
tree -L 2 "$MMAP_BASE_DIR" 2>/dev/null || find "$MMAP_BASE_DIR" -maxdepth 2 -type d | sed 's|[^/]*/| |g'

echo -e "\n${GREEN}✅ MMAP volume initialization complete!${NC}"
echo -e "${YELLOW}💡 Volume location: $(pwd)/$MMAP_BASE_DIR${NC}"
echo -e "${YELLOW}💡 Run 'docker-compose up' to start services with MMAP support${NC}"
