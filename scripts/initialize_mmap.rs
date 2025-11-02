//! Initialize MMAP files from dimensions.json
//!
//! This script reads dimensions.json and creates MMAP region files
//! that can be loaded by MmapManager for zero-copy access.

use std::fs::{File, create_dir_all};
use std::io::{Write as IoWrite, BufWriter};
use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
struct DimensionsData {
    dimensions: Vec<Dimension>,
    layers: Vec<Layer>,
}

#[derive(Debug, Deserialize, Serialize)]
struct Dimension {
    id: u8,
    name: String,
    frequency_min: f32,
    frequency_max: f32,
    size_bytes: usize,
}

#[derive(Debug, Deserialize, Serialize)]
struct Layer {
    dimension_id: u8,
    layer_num: u8,
    depth: u8,
    parent_layer: Option<u8>,
    keywords: Vec<String>,
    frequency: f32,
    mmap_offset: usize,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("🔧 JESSY MMAP Initializer");
    println!("========================\n");

    // Read dimensions.json
    let dimensions_file = Path::new("data/dimensions.json");
    if !dimensions_file.exists() {
        eprintln!("❌ Error: data/dimensions.json not found");
        return Err("dimensions.json not found".into());
    }

    let data: DimensionsData = serde_json::from_reader(File::open(dimensions_file)?)?;
    println!("✅ Loaded {} dimensions, {} layers", data.dimensions.len(), data.layers.len());

    // Create data/mmap directory
    let mmap_dir = Path::new("data/mmap");
    create_dir_all(mmap_dir)?;
    println!("✅ Created data/mmap directory");

    // Create MMAP file for each dimension
    for dimension in &data.dimensions {
        create_dimension_mmap(&dimension, &data.layers, mmap_dir)?;
    }

    println!("\n✨ MMAP initialization complete!");
    println!("📊 Total MMAP size: {} MB", calculate_total_size(&data.dimensions) / 1024 / 1024);

    Ok(())
}

fn create_dimension_mmap(
    dimension: &Dimension,
    all_layers: &[Layer],
    mmap_dir: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("\n📦 Creating MMAP for D{:02} ({})", dimension.id, dimension.name);

    // Filter layers for this dimension
    let dimension_layers: Vec<&Layer> = all_layers
        .iter()
        .filter(|l| l.dimension_id == dimension.id)
        .collect();

    if dimension_layers.is_empty() {
        println!("   ⚠️  No layers found, skipping");
        return Ok(());
    }

    // Create dimension-specific directory
    let dimension_dir = mmap_dir.join(format!("D{:02}", dimension.id));
    create_dir_all(&dimension_dir)?;

    // Create region.mmap file
    let region_file = dimension_dir.join("region.mmap");
    let mut writer = BufWriter::new(File::create(&region_file)?);

    // Write header (metadata about this dimension)
    let header = format!(
        "DIMENSION:{:02}|NAME:{}|SIZE:{}|LAYERS:{}\n",
        dimension.id,
        dimension.name,
        dimension.size_bytes,
        dimension_layers.len()
    );
    writer.write_all(header.as_bytes())?;

    // Write each layer's content
    let mut current_offset = header.len();
    for layer in &dimension_layers {
        println!("   📄 Layer {} (depth {}, freq {:.1} Hz)",
            layer.layer_num, layer.depth, layer.frequency);

        // Generate layer content
        let content = generate_layer_content(dimension, layer);

        // Write layer metadata + content
        let layer_header = format!(
            "LAYER:{}|DEPTH:{}|FREQ:{:.2}|KEYWORDS:{}|OFFSET:{}|SIZE:{}\n",
            layer.layer_num,
            layer.depth,
            layer.frequency,
            layer.keywords.join(","),
            current_offset,
            content.len()
        );

        writer.write_all(layer_header.as_bytes())?;
        writer.write_all(content.as_bytes())?;
        writer.write_all(b"\n")?;

        current_offset += layer_header.len() + content.len() + 1;
    }

    writer.flush()?;
    println!("   ✅ Created {} ({} KB)",
        region_file.display(),
        current_offset / 1024);

    Ok(())
}

fn generate_layer_content(dimension: &Dimension, layer: &Layer) -> String {
    // Generate rich context content for this layer
    let mut content = String::new();

    content.push_str(&format!("# {} - Layer {}\n\n", dimension.name, layer.layer_num));
    content.push_str(&format!("Depth: {} | Frequency: {:.2} Hz\n\n", layer.depth, layer.frequency));

    // Add keywords as context
    content.push_str("## Keywords:\n");
    for keyword in &layer.keywords {
        content.push_str(&format!("- {}\n", keyword));
    }
    content.push_str("\n");

    // Add dimensional context
    content.push_str("## Context:\n");
    content.push_str(&format!(
        "This layer operates within the {} dimension, processing queries through {}.\n",
        dimension.name.to_lowercase(),
        layer.keywords.join(", ")
    ));

    // Add frequency context
    if layer.frequency < 1.0 {
        content.push_str("Operating at LOW frequency - deep, contemplative processing.\n");
    } else if layer.frequency < 2.5 {
        content.push_str("Operating at MEDIUM frequency - balanced analytical processing.\n");
    } else {
        content.push_str("Operating at HIGH frequency - rapid, reactive processing.\n");
    }

    // Add depth-specific guidance
    match layer.depth {
        0 => content.push_str("\nSurface layer: Direct, explicit processing of query terms.\n"),
        1 => content.push_str("\nIntermediate layer: Categorical and structural analysis.\n"),
        2 => content.push_str("\nDeep layer: Nuanced interpretation and contextual understanding.\n"),
        3 => content.push_str("\nCore layer: Fundamental principles and abstract reasoning.\n"),
        _ => content.push_str("\nExtended layer: Specialized processing.\n"),
    }

    content
}

fn calculate_total_size(dimensions: &[Dimension]) -> usize {
    dimensions.iter().map(|d| d.size_bytes).sum()
}
