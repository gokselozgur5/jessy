#!/usr/bin/env python3
"""Add common conversational keywords to dimension layers"""

import json

# Load dimensions.json
with open('data/dimensions.json', 'r') as f:
    data = json.load(f)

# Common conversational keywords to add to each dimension
conversational_keywords = {
    1: ["happy", "sad", "feel", "feeling", "love", "hate", "like", "enjoy"],  # Emotion
    2: ["think", "know", "understand", "learn", "remember", "wonder", "believe"],  # Cognition
    3: ["want", "need", "try", "help", "do", "make", "get"],  # Intention
    4: ["hi", "hello", "hey", "how", "you", "me", "we", "us", "greet", "greeting", "talk", "chat", "conversation"],  # Social Context
    5: ["today", "now", "when", "time", "day", "yesterday", "tomorrow"],  # Temporal State
    10: ["i", "am", "are", "is", "be", "being"],  # Meta-Awareness
}

# Add keywords to root layers (depth 0)
for layer in data['layers']:
    if layer['depth'] == 0:
        dim_id = layer['dimension_id']
        if dim_id in conversational_keywords:
            # Add new keywords without duplicates
            existing = set(layer['keywords'])
            new_keywords = conversational_keywords[dim_id]
            for kw in new_keywords:
                if kw not in existing:
                    layer['keywords'].append(kw)
            print(f"Dimension {dim_id}: Added {len(new_keywords)} conversational keywords")

# Save updated dimensions.json
with open('data/dimensions.json', 'w') as f:
    json.dump(data, f, indent=2)

print("\n✅ Updated dimensions.json with conversational keywords")
