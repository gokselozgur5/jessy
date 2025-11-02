#!/usr/bin/env python3
"""
Initialize MMAP files from dimensions.json
Quick and simple Python script to create MMAP region files
"""

import json
import os
from pathlib import Path

def main():
    print("🔧 JESSY MMAP Initializer")
    print("========================\n")

    # Read dimensions.json
    dimensions_file = Path("data/dimensions.json")
    if not dimensions_file.exists():
        print("❌ Error: data/dimensions.json not found")
        return 1

    with open(dimensions_file) as f:
        data = json.load(f)

    print(f"✅ Loaded {len(data['dimensions'])} dimensions, {len(data['layers'])} layers")

    # Create data/mmap directory
    mmap_dir = Path("data/mmap")
    mmap_dir.mkdir(exist_ok=True)
    print("✅ Created data/mmap directory")

    # Create MMAP file for each dimension
    for dimension in data['dimensions']:
        create_dimension_mmap(dimension, data['layers'], mmap_dir)

    total_size = sum(d['size_bytes'] for d in data['dimensions'])
    print(f"\n✨ MMAP initialization complete!")
    print(f"📊 Total MMAP size: {total_size // 1024 // 1024} MB")

    return 0


def create_dimension_mmap(dimension, all_layers, mmap_dir):
    dim_id = dimension['id']
    dim_name = dimension['name']
    print(f"\n📦 Creating MMAP for D{dim_id:02d} ({dim_name})")

    # Filter layers for this dimension
    dimension_layers = [l for l in all_layers if l['dimension_id'] == dim_id]

    if not dimension_layers:
        print("   ⚠️  No layers found, skipping")
        return

    # Create dimension-specific directory
    dimension_dir = mmap_dir / f"D{dim_id:02d}"
    dimension_dir.mkdir(exist_ok=True)

    # Create region.mmap file
    region_file = dimension_dir / "region.mmap"

    with open(region_file, 'w') as f:
        # Write header (metadata about this dimension)
        header = (f"DIMENSION:{dim_id:02d}|NAME:{dim_name}|"
                 f"SIZE:{dimension['size_bytes']}|LAYERS:{len(dimension_layers)}\n")
        f.write(header)

        # Write each layer's content
        current_offset = len(header)
        for layer in sorted(dimension_layers, key=lambda l: l['layer_num']):
            layer_num = layer['layer_num']
            depth = layer['depth']
            freq = layer['frequency']

            print(f"   📄 Layer {layer_num} (depth {depth}, freq {freq:.1f} Hz)")

            # Generate layer content
            content = generate_layer_content(dimension, layer)

            # Write layer metadata + content
            keywords_str = ",".join(layer['keywords'])
            layer_header = (f"LAYER:{layer_num}|DEPTH:{depth}|FREQ:{freq:.2f}|"
                          f"KEYWORDS:{keywords_str}|OFFSET:{current_offset}|SIZE:{len(content)}\n")

            f.write(layer_header)
            f.write(content)
            f.write("\n")

            current_offset += len(layer_header) + len(content) + 1

    file_size_kb = region_file.stat().st_size // 1024
    print(f"   ✅ Created {region_file} ({file_size_kb} KB)")


def generate_layer_content(dimension, layer):
    """Generate rich context content for this layer"""
    content = []

    dim_name = dimension['name']
    layer_num = layer['layer_num']
    depth = layer['depth']
    freq = layer['frequency']
    keywords = layer['keywords']

    content.append(f"# {dim_name} - Layer {layer_num}\n")
    content.append(f"Depth: {depth} | Frequency: {freq:.2f} Hz\n\n")

    # Add keywords as context
    content.append("## Keywords:\n")
    for keyword in keywords:
        content.append(f"- {keyword}\n")
    content.append("\n")

    # Add dimensional context
    content.append("## Context:\n")
    content.append(f"This layer operates within the {dim_name.lower()} dimension, ")
    content.append(f"processing queries through {', '.join(keywords)}.\n")

    # Add frequency context
    if freq < 1.0:
        content.append("Operating at LOW frequency - deep, contemplative processing.\n")
    elif freq < 2.5:
        content.append("Operating at MEDIUM frequency - balanced analytical processing.\n")
    else:
        content.append("Operating at HIGH frequency - rapid, reactive processing.\n")

    # Add depth-specific guidance
    depth_guidance = {
        0: "\nSurface layer: Direct, explicit processing of query terms.\n",
        1: "\nIntermediate layer: Categorical and structural analysis.\n",
        2: "\nDeep layer: Nuanced interpretation and contextual understanding.\n",
        3: "\nCore layer: Fundamental principles and abstract reasoning.\n",
    }
    content.append(depth_guidance.get(depth, "\nExtended layer: Specialized processing.\n"))

    return "".join(content)


if __name__ == "__main__":
    exit(main())
