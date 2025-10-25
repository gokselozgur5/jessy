# MMAP Volumes for Dimensional Data

## Overview

The Jessy consciousness system uses memory-mapped files (MMAP) for zero-copy access to dimensional layer data. This document describes the MMAP volume structure, configuration, and usage.

## Architecture

### Memory-Mapped I/O

MMAP provides direct memory access to file contents without explicit read/write operations:

```
┌─────────────────────────────────────────────────────────┐
│                    Application                          │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Memory Access (ptr dereference)                 │  │
│  └────────────────┬─────────────────────────────────┘  │
│                   │                                     │
│                   ▼                                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Virtual Memory (MMAP region)                    │  │
│  └────────────────┬─────────────────────────────────┘  │
└───────────────────┼─────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────┐
│              Operating System Kernel                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Page Table (virtual → physical mapping)        │  │
│  └────────────────┬─────────────────────────────────┘  │
│                   │                                     │
│                   ▼                                     │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Page Cache (hot data in RAM)                   │  │
│  └────────────────┬─────────────────────────────────┘  │
└───────────────────┼─────────────────────────────────────┘
                    │
                    ▼
┌─────────────────────────────────────────────────────────┐
│                  File System                            │
│  ┌──────────────────────────────────────────────────┐  │
│  │  Dimensional Layer Files (*.mmap)               │  │
│  └──────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

### Benefits

1. **Zero-copy**: Direct memory access, no buffer copies
2. **Lazy loading**: Pages loaded on-demand via page faults
3. **OS-managed caching**: Kernel handles hot/cold data automatically
4. **Shared memory**: Multiple processes can access same data
5. **Persistence**: Changes can be synced back to disk

## Directory Structure

```
data/mmap/
├── consciousness/              # Core dimensional layers
│   ├── D01/                   # Dimension 01 (Emotion)
│   │   ├── region.mmap        # Memory-mapped region file
│   │   ├── index.json         # Layer index metadata
│   │   └── layers/            # Individual layer files (optional)
│   ├── D02/                   # Dimension 02 (Cognition)
│   ├── D03/                   # Dimension 03 (Intention)
│   ├── D04/                   # Dimension 04 (Social)
│   ├── D05/                   # Dimension 05 (Temporal)
│   ├── D06/                   # Dimension 06 (Philosophical)
│   ├── D07/                   # Dimension 07 (Technical)
│   ├── D08/                   # Dimension 08 (Creative)
│   ├── D09/                   # Dimension 09 (Ethical)
│   ├── D10/                   # Dimension 10 (Meta)
│   ├── D11/                   # Dimension 11 (Ecological)
│   ├── D12/                   # Dimension 12 (Positivity)
│   ├── D13/                   # Dimension 13 (Balance)
│   └── D14/                   # Dimension 14 (Security)
├── proto/                     # Proto-dimensions (learning system)
│   └── *.mmap                 # Temporary proto-dimension files
├── temp/                      # Temporary MMAP operations
├── README.md                  # Volume documentation
└── .gitignore                 # Exclude MMAP data from git
```

## Initialization

### First-Time Setup

```bash
# Initialize MMAP directory structure
make init-mmap

# Verify structure
ls -la data/mmap/consciousness/

# Check permissions
ls -la data/mmap/
```

### What Gets Created

1. **Base directory**: `data/mmap/`
2. **Consciousness directory**: 14 dimension subdirectories (D01-D14)
3. **Proto directory**: For learning system proto-dimensions
4. **Temp directory**: For temporary MMAP operations
5. **README.md**: Documentation
6. **. gitignore**: Excludes MMAP data from version control

## Docker Configuration

### Volume Definition

In `docker-compose.yml`:

```yaml
volumes:
  mmap-data:
    driver: local
    driver_opts:
      type: none
      o: bind
      device: ${PWD}/data/mmap
```

This creates a bind mount from the host directory to the Docker volume.

### Service Mounts

#### Rust Service (jessy-core)

**Read-write access** for creating and modifying dimensional data:

```yaml
services:
  jessy-core:
    volumes:
      - mmap-data:/app/data/mmap
```

#### Go API (jessy-api)

**Read-only access** for querying dimensional data:

```yaml
services:
  jessy-api:
    volumes:
      - mmap-data:/app/data/mmap:ro
```

#### Test Containers

**Read-write access** for testing:

```yaml
services:
  jessy-test:
    volumes:
      - mmap-data:/app/data/mmap
```

### Environment Variables

Configure MMAP behavior in `.env`:

```bash
# Base path for MMAP files (inside container)
MMAP_BASE_PATH=/app/data/mmap

# Initial MMAP region size (1MB)
MMAP_INITIAL_SIZE=1048576

# Maximum MMAP region size (10MB)
MMAP_MAX_SIZE=10485760
```

## File Format

### Region File (region.mmap)

Binary file containing packed layer data:

```
┌─────────────────────────────────────────┐
│  Header (256 bytes)                     │
│  - Magic number: 0x4D4D4150 ("MMAP")   │
│  - Version: 1                           │
│  - Dimension ID                         │
│  - Layer count                          │
│  - Total size                           │
│  - Checksum                             │
├─────────────────────────────────────────┤
│  Layer 1 Data                           │
│  - Content (variable size)              │
│  - Padding to 4KB boundary              │
├─────────────────────────────────────────┤
│  Layer 2 Data                           │
│  - Content (variable size)              │
│  - Padding to 4KB boundary              │
├─────────────────────────────────────────┤
│  ...                                    │
└─────────────────────────────────────────┘
```

### Index File (index.json)

JSON metadata for fast layer lookup:

```json
{
  "dimension_id": 1,
  "dimension_name": "Emotion",
  "version": 1,
  "created_at": "2024-10-24T10:00:00Z",
  "updated_at": "2024-10-24T12:00:00Z",
  "layer_count": 5,
  "total_size": 20480,
  "layers": [
    {
      "layer_id": 1,
      "offset": 256,
      "size": 4096,
      "frequency": 1.5,
      "keywords": ["emotion", "feeling", "affect"],
      "checksum": "sha256:abc123..."
    },
    {
      "layer_id": 2,
      "offset": 4352,
      "size": 8192,
      "frequency": 2.0,
      "keywords": ["mood", "sentiment"],
      "checksum": "sha256:def456..."
    }
  ]
}
```

## Usage

### From Rust Code

```rust
use jessy::memory::MmapManager;

// Create manager
let mut manager = MmapManager::new(280)?; // 280MB total

// Load dimension
let region_id = manager.load_dimension(DimensionId(1))?;

// Load layer context
let context = manager.load_layer_context(LayerId {
    dimension: DimensionId(1),
    layer: 1,
})?;

// Access content (zero-copy)
println!("Content: {}", context.content);
```

### From Go API

```go
import "jessy/api/consciousness"

// Read dimensional data (read-only)
data, err := consciousness.LoadDimension(1)
if err != nil {
    log.Fatal(err)
}

// Access layer content
content := data.GetLayer(1)
fmt.Println("Content:", content)
```

## Testing

### Verify Configuration

```bash
# Run comprehensive verification
make test-mmap

# Or use the verification script directly
./scripts/verify-mmap-volume.sh
```

### Manual Testing

```bash
# Check volume exists
docker volume inspect kiroxnaut_mmap-data

# Test Rust container access
docker-compose exec jessy-core ls -la /app/data/mmap

# Test Go container access (read-only)
docker-compose exec jessy-api ls -la /app/data/mmap

# Create test file from Rust
docker-compose exec jessy-core sh -c "echo 'test' > /app/data/mmap/test.txt"

# Read from Go
docker-compose exec jessy-api cat /app/data/mmap/test.txt

# Try to write from Go (should fail)
docker-compose exec jessy-api sh -c "echo 'test' > /app/data/mmap/test2.txt"
# Expected: "Read-only file system" error
```

### View Volume Info

```bash
# Show MMAP volume information
make mmap-info

# Inspect volume details
docker volume inspect kiroxnaut_mmap-data

# Check disk usage
du -sh data/mmap/
```

## Performance

### Characteristics

- **Layer load time**: <1ms (cached), <10ms (cold)
- **Memory overhead**: ~4KB per layer (page table entries)
- **Disk I/O**: Only on page faults, then cached by OS
- **Concurrent access**: Multiple readers, single writer per region

### Optimization Tips

1. **Align to page boundaries**: Use 4KB alignment for better performance
2. **Pre-fault pages**: Touch pages during initialization to avoid runtime faults
3. **Use madvise**: Hint kernel about access patterns
4. **Monitor page faults**: Track with `perf` or similar tools

### Benchmarks

```bash
# Run MMAP benchmarks
make bench

# View results
open target/criterion/report/index.html
```

## Backup and Recovery

### Backup

```bash
# Backup all dimensional data
tar -czf mmap-backup-$(date +%Y%m%d).tar.gz data/mmap/consciousness/

# Backup specific dimension
tar -czf D01-backup.tar.gz data/mmap/consciousness/D01/

# Incremental backup (rsync)
rsync -av --progress data/mmap/ backup/mmap/
```

### Restore

```bash
# Restore from backup
tar -xzf mmap-backup-20241024.tar.gz

# Verify integrity
./scripts/verify-mmap-volume.sh

# Restart services
docker-compose restart jessy-core
```

### Disaster Recovery

```bash
# Reinitialize from scratch
rm -rf data/mmap/
make init-mmap

# Rebuild dimensional data (if source available)
# This would typically involve re-running data ingestion pipelines
```

## Troubleshooting

### Permission Denied Errors

**Problem**: Cannot read/write MMAP files

**Solution**:
```bash
# Check permissions
ls -la data/mmap/

# Fix permissions
chmod -R 755 data/mmap/

# Restart containers
docker-compose restart
```

### MMAP Files Not Found

**Problem**: Region files missing

**Solution**:
```bash
# Verify directory structure
ls -la data/mmap/consciousness/

# Reinitialize if needed
make init-mmap

# Check volume mount
docker inspect jessy-core --format='{{range .Mounts}}{{.Source}} -> {{.Destination}}{{end}}'
```

### Out of Memory Errors

**Problem**: MMAP allocation fails

**Solution**:
```bash
# Check current limits
docker-compose exec jessy-core env | grep MMAP

# Increase limits in .env
MMAP_MAX_SIZE=20971520  # 20MB

# Restart services
docker-compose restart
```

### Stale Data After Updates

**Problem**: Changes not reflected

**Solution**:
```bash
# Clear page cache (Linux)
sync && echo 3 > /proc/sys/vm/drop_caches

# Or restart containers
docker-compose restart jessy-core

# Or clear temp directory
rm -rf data/mmap/temp/*
```

### Volume Mount Issues

**Problem**: Volume not mounting correctly

**Solution**:
```bash
# Remove and recreate volume
docker-compose down -v
docker volume rm kiroxnaut_mmap-data
make init-mmap
docker-compose up -d

# Verify mount
docker volume inspect kiroxnaut_mmap-data
```

## Best Practices

### Development

1. **Initialize first**: Always run `make init-mmap` before first use
2. **Test access**: Use `make test-mmap` to verify configuration
3. **Monitor size**: Keep an eye on MMAP directory size
4. **Clean temp**: Periodically clean `data/mmap/temp/`
5. **Backup regularly**: Backup consciousness directory

### Production

1. **Use separate volumes**: Don't mix dev and prod data
2. **Set resource limits**: Configure MMAP size limits appropriately
3. **Monitor performance**: Track page faults and I/O
4. **Implement checksums**: Verify data integrity
5. **Plan for growth**: Ensure sufficient disk space

### Security

1. **Read-only for API**: Go API has read-only access
2. **Validate inputs**: Check all MMAP operations
3. **Limit file sizes**: Enforce maximum region sizes
4. **Audit access**: Log all MMAP operations
5. **Encrypt at rest**: Consider disk encryption for sensitive data

## Related Documentation

- [Docker Setup](DOCKER_SETUP.md)
- [Memory Manager Architecture](../ARCHITECTURE.md)
- [Testing Infrastructure](TESTING_INFRASTRUCTURE.md)
- [Performance Benchmarking](BENCHMARKING.md)

---

*"Memory-mapped I/O: Where the file system meets virtual memory. Zero-copy magic. 🗺️"*
