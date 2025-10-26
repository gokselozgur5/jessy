#!/bin/bash
# JESSY'nin tüm bilgisini topla

OUTPUT_DIR="training/knowledge_base"
mkdir -p "$OUTPUT_DIR"

echo "🧠 Collecting JESSY's knowledge..."

# 1. Steering Files (Philosophy & Principles)
echo "📚 Collecting steering files..."
mkdir -p "$OUTPUT_DIR/steering"
cp -r .kiro/steering/*.md "$OUTPUT_DIR/steering/" 2>/dev/null || true

# 2. Documentation
echo "📖 Collecting documentation..."
mkdir -p "$OUTPUT_DIR/docs"
cp -r docs/*.md "$OUTPUT_DIR/docs/" 2>/dev/null || true
cp README.md "$OUTPUT_DIR/docs/" 2>/dev/null || true
cp VISION.md "$OUTPUT_DIR/docs/" 2>/dev/null || true
cp QUICKSTART.md "$OUTPUT_DIR/docs/" 2>/dev/null || true

# 3. Specs (Requirements, Design, Tasks)
echo "📋 Collecting specs..."
mkdir -p "$OUTPUT_DIR/specs"
find .kiro/specs -name "*.md" -exec cp {} "$OUTPUT_DIR/specs/" \; 2>/dev/null || true

# 4. Progress & Session Notes
echo "📝 Collecting progress notes..."
mkdir -p "$OUTPUT_DIR/progress"
cp *PROGRESS*.md "$OUTPUT_DIR/progress/" 2>/dev/null || true
cp *COMPLETE*.md "$OUTPUT_DIR/progress/" 2>/dev/null || true
cp SESSION*.md "$OUTPUT_DIR/progress/" 2>/dev/null || true
cp TASK*.md "$OUTPUT_DIR/progress/" 2>/dev/null || true

# 5. Code Documentation (key files)
echo "💻 Collecting code docs..."
mkdir -p "$OUTPUT_DIR/code"
find src -name "*.rs" -type f | head -20 | while read file; do
    # Extract doc comments
    grep -E "^//[!/]" "$file" > "$OUTPUT_DIR/code/$(basename $file).docs" 2>/dev/null || true
done

# 6. Data (dimensions, etc)
echo "🗄️ Collecting data..."
mkdir -p "$OUTPUT_DIR/data"
cp data/*.json "$OUTPUT_DIR/data/" 2>/dev/null || true

# Statistics
echo ""
echo "✅ Collection complete!"
echo "📊 Statistics:"
echo "   Steering: $(find $OUTPUT_DIR/steering -type f | wc -l) files"
echo "   Docs: $(find $OUTPUT_DIR/docs -type f | wc -l) files"
echo "   Specs: $(find $OUTPUT_DIR/specs -type f | wc -l) files"
echo "   Progress: $(find $OUTPUT_DIR/progress -type f | wc -l) files"
echo "   Code: $(find $OUTPUT_DIR/code -type f | wc -l) files"
echo "   Data: $(find $OUTPUT_DIR/data -type f | wc -l) files"
echo ""
echo "   Total: $(find $OUTPUT_DIR -type f | wc -l) files"
echo "   Total size: $(du -sh $OUTPUT_DIR | cut -f1)"
