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
