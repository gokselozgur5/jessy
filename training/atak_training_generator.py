#!/usr/bin/env python3
"""
ATAK Training Data Generator
Generates comprehensive Q&A pairs for ATAK (Android Team Awareness Kit) training
"""

import json
import random
from typing import List, Dict

# ATAK Domain Knowledge Base
ATAK_KNOWLEDGE = {
    "basics": [
        {
            "q": "What is ATAK?",
            "a": "ATAK (Android Team Awareness Kit) is a geospatial situational awareness application for Android devices. It provides real-time mapping, navigation, and communication capabilities for tactical operations."
        },
        {
            "q": "What are the main features of ATAK?",
            "a": "ATAK's main features include: real-time mapping and navigation, marker placement and management, route planning, range and bearing tools, team tracking, chat and messaging, plugin support, and offline map capabilities."
        },
        {
            "q": "What devices can run ATAK?",
            "a": "ATAK runs on Android devices (phones and tablets) running Android 5.0 (Lollipop) or higher. It's optimized for rugged tactical devices but works on consumer Android devices as well."
        }
    ],
    
    "markers": [
        {
            "q": "How do you add a marker in ATAK?",
            "a": "To add a marker: 1) Tap the marker tool icon, 2) Either tap a location on the map or enter coordinates manually, 3) Customize the marker type, color, and label, 4) Tap 'Done' to place the marker."
        },
        {
            "q": "How do you edit an existing marker?",
            "a": "To edit a marker: 1) Tap the marker on the map, 2) Select 'Edit' from the radial menu, 3) Modify the marker properties (name, type, color, etc.), 4) Tap 'Done' to save changes."
        },
        {
            "q": "How do you delete a marker?",
            "a": "To delete a marker: 1) Tap the marker on the map, 2) Select 'Delete' from the radial menu, or 3) Long-press the marker and select 'Delete' from the context menu."
        },
        {
            "q": "What marker types are available in ATAK?",
            "a": "ATAK supports various marker types including: friendly units, hostile units, neutral units, waypoints, objectives, hazards, and custom markers. Each type has different icons and colors for quick identification."
        }
    ],
    
    "navigation": [
        {
            "q": "How do you create a route in ATAK?",
            "a": "To create a route: 1) Tap the route tool, 2) Tap waypoints on the map in sequence, 3) Adjust waypoints by dragging, 4) Set route properties (name, color, method), 5) Tap 'Done' to save the route."
        },
        {
            "q": "How do you navigate to a waypoint?",
            "a": "To navigate to a waypoint: 1) Tap the waypoint marker, 2) Select 'Navigate' from the radial menu, 3) ATAK will display bearing, distance, and ETA, 4) Follow the navigation arrow on the map."
        },
        {
            "q": "What navigation methods does ATAK support?",
            "a": "ATAK supports multiple navigation methods: driving (road-based), walking (pedestrian), straight line (direct), and aviation. Each method calculates routes differently based on the terrain and available paths."
        }
    ],
    
    "tools": [
        {
            "q": "How do you measure distance in ATAK?",
            "a": "To measure distance: 1) Open Range Tools, 2) Select 'Range & Bearing', 3) Tap two points on the map, 4) ATAK displays distance, bearing, and elevation difference between the points."
        },
        {
            "q": "How do you use the bullseye tool?",
            "a": "The bullseye tool: 1) Select 'Bullseye' from Range Tools, 2) Set the center point, 3) Configure ring spacing and count, 4) Use for reference points and distance estimation in tactical operations."
        },
        {
            "q": "What is the radial menu in ATAK?",
            "a": "The radial menu appears when you tap a marker or location. It provides quick access to common actions like Edit, Delete, Navigate, Share, and tool-specific options. It's designed for one-handed operation."
        }
    ],
    
    "communication": [
        {
            "q": "How does team tracking work in ATAK?",
            "a": "Team tracking: 1) Team members' locations appear as markers on the map, 2) Updates occur in real-time via network connection, 3) Each team member has a unique callsign and icon, 4) You can see bearing, distance, and status of team members."
        },
        {
            "q": "How do you send a chat message in ATAK?",
            "a": "To send a chat message: 1) Open the chat panel, 2) Select recipient (individual or group), 3) Type your message, 4) Optionally attach location or markers, 5) Tap send. Messages are delivered over the TAK server network."
        },
        {
            "q": "How do you share a marker with team members?",
            "a": "To share a marker: 1) Tap the marker, 2) Select 'Share' from the radial menu, 3) Choose recipients (individual, group, or all), 4) The marker appears on recipients' maps with your callsign."
        }
    ],
    
    "maps": [
        {
            "q": "How do you import offline maps in ATAK?",
            "a": "To import offline maps: 1) Copy map files (.mbtiles, .sqlite, or imagery) to device storage, 2) In ATAK, go to Settings > Map Layers, 3) Select 'Import', 4) Choose the map file, 5) The map becomes available offline."
        },
        {
            "q": "What map formats does ATAK support?",
            "a": "ATAK supports multiple map formats: MBTiles, SQLite databases, GeoTIFF imagery, KML/KMZ files, shapefiles, and various tile packages. It can also connect to WMS/WMTS servers for online maps."
        },
        {
            "q": "How do you change the base map layer?",
            "a": "To change base map: 1) Tap the layer icon, 2) Select 'Base Maps', 3) Choose from available maps (satellite, street, terrain, etc.), 4) The map view updates immediately."
        }
    ],
    
    "settings": [
        {
            "q": "How do you configure your callsign in ATAK?",
            "a": "To set callsign: 1) Go to Settings > Device Preferences, 2) Enter your callsign in the 'Callsign' field, 3) This identifies you to other team members, 4) Choose a unique, recognizable callsign."
        },
        {
            "q": "How do you connect to a TAK server?",
            "a": "To connect to TAK server: 1) Go to Settings > Network Preferences, 2) Add server connection, 3) Enter server address, port, and credentials, 4) Enable the connection, 5) ATAK connects and syncs with the server."
        },
        {
            "q": "How do you customize the self-marker appearance?",
            "a": "To customize self-marker: 1) Go to Settings > My Location, 2) Choose marker icon and color, 3) Set display options (heading, speed, altitude), 4) Configure GPS update frequency."
        }
    ],
    
    "plugins": [
        {
            "q": "How do you install ATAK plugins?",
            "a": "To install plugins: 1) Download the plugin APK file, 2) Install it on your Android device, 3) Open ATAK, 4) The plugin appears in the tools menu, 5) Some plugins require additional configuration."
        },
        {
            "q": "What are some popular ATAK plugins?",
            "a": "Popular ATAK plugins include: Video streaming plugins, sensor integration tools, advanced mapping plugins, communication enhancers, mission planning tools, and custom data visualization plugins."
        }
    ],
    
    "tactical": [
        {
            "q": "How do you create a 9-line MEDEVAC request in ATAK?",
            "a": "To create 9-line MEDEVAC: 1) Open the MEDEVAC tool, 2) Fill in all 9 lines (location, frequency, patients, etc.), 3) Set pickup zone marker, 4) Send to appropriate recipients, 5) Track status updates."
        },
        {
            "q": "How do you mark a target in ATAK?",
            "a": "To mark a target: 1) Use the marker tool, 2) Select 'Hostile' marker type, 3) Place at target location, 4) Add details (type, threat level, notes), 5) Share with team for situational awareness."
        },
        {
            "q": "How do you create a sector sketch in ATAK?",
            "a": "To create sector sketch: 1) Use drawing tools, 2) Mark key terrain features, 3) Add range rings and reference points, 4) Label sectors and boundaries, 5) Share with team for coordinated operations."
        }
    ],
    
    "troubleshooting": [
        {
            "q": "What do you do if ATAK won't connect to GPS?",
            "a": "GPS troubleshooting: 1) Check location permissions in Android settings, 2) Ensure GPS is enabled, 3) Move to an area with clear sky view, 4) Restart ATAK, 5) Check if mock locations are disabled."
        },
        {
            "q": "How do you fix network connection issues in ATAK?",
            "a": "Network troubleshooting: 1) Verify server address and port, 2) Check network connectivity (WiFi/cellular), 3) Confirm firewall settings, 4) Verify credentials, 5) Check server status with administrator."
        },
        {
            "q": "What do you do if markers aren't syncing?",
            "a": "Marker sync troubleshooting: 1) Check network connection, 2) Verify server connection status, 3) Check data permissions, 4) Restart ATAK, 5) Manually resend markers if needed."
        }
    ]
}

def generate_training_data() -> List[Dict]:
    """Generate comprehensive training dataset"""
    training_data = []
    
    # Add all base Q&A pairs
    for category, qa_pairs in ATAK_KNOWLEDGE.items():
        for qa in qa_pairs:
            training_data.append({
                "input": qa["q"],
                "output": qa["a"],
                "category": category
            })
    
    # Generate variations
    variations = []
    
    # Add conversational variations
    for item in training_data:
        # Casual variation
        variations.append({
            "input": f"Hey, {item['input'].lower()}",
            "output": item["output"],
            "category": item["category"]
        })
        
        # Question variation
        variations.append({
            "input": f"Can you explain {item['input'].lower()}",
            "output": f"Sure! {item['output']}",
            "category": item["category"]
        })
    
    training_data.extend(variations)
    
    # Add meta questions about ATAK
    meta_questions = [
        {
            "input": "What can you help me with regarding ATAK?",
            "output": "I can help you with ATAK basics, marker management, navigation, tools, communication, maps, settings, plugins, tactical operations, and troubleshooting. Just ask me anything about ATAK!",
            "category": "meta"
        },
        {
            "input": "Are you an ATAK expert?",
            "output": "Yes! I'm trained on ATAK documentation and can help you with all aspects of using ATAK, from basic operations to advanced tactical features.",
            "category": "meta"
        },
        {
            "input": "How do I get started with ATAK?",
            "output": "To get started with ATAK: 1) Install ATAK on your Android device, 2) Configure your callsign and location settings, 3) Connect to a TAK server if available, 4) Import offline maps for your area, 5) Practice with basic markers and navigation. Would you like details on any of these steps?",
            "category": "meta"
        }
    ]
    
    training_data.extend(meta_questions)
    
    return training_data

def save_training_data(data: List[Dict], output_file: str):
    """Save training data in JSONL format"""
    with open(output_file, 'w') as f:
        for item in data:
            f.write(json.dumps(item) + '\n')
    
    print(f"✅ Saved {len(data)} training examples to {output_file}")

def create_train_val_split(data: List[Dict], val_ratio: float = 0.1):
    """Split data into train and validation sets"""
    random.shuffle(data)
    split_idx = int(len(data) * (1 - val_ratio))
    
    train_data = data[:split_idx]
    val_data = data[split_idx:]
    
    return train_data, val_data

def main():
    print("=" * 70)
    print("🎯 ATAK Training Data Generator")
    print("=" * 70)
    
    # Generate data
    print("\n📊 Generating training data...")
    training_data = generate_training_data()
    
    print(f"✅ Generated {len(training_data)} total examples")
    
    # Show category breakdown
    categories = {}
    for item in training_data:
        cat = item.get("category", "unknown")
        categories[cat] = categories.get(cat, 0) + 1
    
    print("\n📈 Category Breakdown:")
    for cat, count in sorted(categories.items()):
        print(f"   • {cat}: {count} examples")
    
    # Split into train/val
    print("\n🔀 Splitting into train/validation sets...")
    train_data, val_data = create_train_val_split(training_data, val_ratio=0.15)
    
    print(f"   • Training: {len(train_data)} examples")
    print(f"   • Validation: {len(val_data)} examples")
    
    # Save files
    print("\n💾 Saving files...")
    save_training_data(train_data, "training/datasets/atak_train.jsonl")
    save_training_data(val_data, "training/datasets/atak_val.jsonl")
    
    # Save full dataset as JSON for reference
    with open("training/datasets/atak_full.json", 'w') as f:
        json.dump(training_data, f, indent=2)
    
    print("\n✅ Training data generation complete!")
    print("=" * 70)
    print("\nFiles created:")
    print("   • training/datasets/atak_train.jsonl")
    print("   • training/datasets/atak_val.jsonl")
    print("   • training/datasets/atak_full.json")
    print("=" * 70)

if __name__ == "__main__":
    main()
