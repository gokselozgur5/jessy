#!/bin/bash

# Mass commit script - adds each untracked file as separate commit
# For maximum GitHub green squares 💚

cd /Users/gok/Projects/DeepCode/jessy

# Get all untracked files
git status --porcelain | grep '^??' | cut -c4- | while read file; do
    # Skip if it's a directory
    if [ -f "$file" ]; then
        # Add file
        git add "$file"

        # Create commit message based on file type
        if [[ "$file" == *.md ]]; then
            TYPE="docs"
        elif [[ "$file" == *.rs ]]; then
            TYPE="feat"
        elif [[ "$file" == *.py ]]; then
            TYPE="chore"
        elif [[ "$file" == *.sh ]]; then
            TYPE="chore"
        elif [[ "$file" == *.json ]]; then
            TYPE="data"
        else
            TYPE="chore"
        fi

        # Get just the filename
        FILENAME=$(basename "$file")

        # Commit
        git commit -m "$TYPE: Add $FILENAME

Development history file.

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>"

        echo "✅ Committed: $file"
    fi
done

echo ""
echo "🎉 All files committed! Now pushing..."
git push origin main

echo "💚 GitHub green squares incoming!"
