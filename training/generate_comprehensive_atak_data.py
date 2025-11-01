#!/usr/bin/env python3
"""
Generate COMPREHENSIVE ATAK Training Data
Combines ALL sources: PDFs, Thor Q&A, Argonaut plugin, UAS tools, NoviTAK
"""

import json
import random
from typing import List, Dict

def load_thor_qa() -> List[Dict]:
    """Load existing Q&A from Thor"""
    try:
        with open("training/atak_source_data/thor_qa_pairs.json", 'r') as f:
            data = json.load(f)
        
        result = []
        for item in data:
            result.append({
                "input": item["question"],
                "output": item["answer"],
                "source": "thor_pdf",
                "page": item.get("page", "unknown")
            })
        
        print(f"✅ Loaded {len(result)} Q&A from Thor PDFs")
        return result
    except:
        print("⚠️  Could not load Thor Q&A")
        return []

def generate_argonaut_plugin_qa() -> List[Dict]:
    """Generate Argonaut Plugin Q&A for end users"""
    qa_pairs = [
        {
            "input": "What is the ATAK Argonaut Plugin?",
            "output": "The ATAK Argonaut Plugin is a custom ATAK plugin built using NoviTAK. It extends ATAK's capabilities with additional features for tactical operations and mission planning.",
            "source": "argonaut_plugin",
            "category": "plugin"
        },
        {
            "input": "How do I install the Argonaut Plugin on my Android device?",
            "output": "To install Argonaut Plugin: 1) Download the plugin APK, 2) Ensure ATAK is already installed on your device, 3) Install the plugin APK, 4) Open ATAK and the plugin will be automatically detected and loaded.",
            "source": "argonaut_plugin",
            "category": "installation"
        },
        {
            "input": "What is NoviTAK?",
            "output": "NoviTAK is a framework for building custom ATAK plugins. It provides tools and APIs that make it easier to extend ATAK's functionality with custom features.",
            "source": "argonaut_plugin",
            "category": "novitak"
        },
        {
            "input": "Do I need developer tools to use the Argonaut Plugin?",
            "output": "No, as an end user you only need: 1) An Android device, 2) ATAK installed, 3) The Argonaut Plugin APK. Developer tools (Android Studio, SDK) are only needed if you want to modify or develop the plugin.",
            "source": "argonaut_plugin",
            "category": "usage"
        },
        {
            "input": "Where can I find the Argonaut Plugin APK?",
            "output": "The Argonaut Plugin APK should be provided by your organization or administrator. It's a custom plugin that needs to be distributed through your team's channels.",
            "source": "argonaut_plugin",
            "category": "installation"
        },
        {
            "input": "What features does the Argonaut Plugin add to ATAK?",
            "output": "The Argonaut Plugin extends ATAK with custom tactical features, enhanced mission planning tools, and specialized data visualization capabilities tailored for specific operational needs.",
            "source": "argonaut_plugin",
            "category": "features"
        },
        {
            "input": "Can I use Argonaut Plugin with the civilian version of ATAK?",
            "output": "Yes, the Argonaut Plugin is designed to work with ATAK-CIV (civilian version). Make sure you have ATAK-CIV installed before installing the plugin.",
            "source": "argonaut_plugin",
            "category": "compatibility"
        },
        {
            "input": "How do I access Argonaut Plugin features in ATAK?",
            "output": "After installing the plugin, its features appear in ATAK's tools menu. Look for the Argonaut icon or menu items. The plugin integrates seamlessly with ATAK's existing interface.",
            "source": "argonaut_plugin",
            "category": "usage"
        },
        {
            "input": "What should I do if the Argonaut Plugin doesn't load?",
            "output": "If the plugin doesn't load: 1) Verify ATAK is installed and working, 2) Reinstall the plugin APK, 3) Check Android permissions for both ATAK and the plugin, 4) Restart ATAK, 5) Check with your administrator for the correct plugin version.",
            "source": "argonaut_plugin",
            "category": "troubleshooting"
        }
    ]
    
    print(f"✅ Generated {len(qa_pairs)} Argonaut Plugin Q&A")
    return qa_pairs

def generate_uas_tool_qa() -> List[Dict]:
    """Generate UAS (Unmanned Aerial System) Tool Q&A"""
    qa_pairs = [
        {
            "input": "What is the UAS Tool in ATAK?",
            "output": "The UAS Tool in ATAK provides drone/unmanned aerial system integration. It allows you to control, monitor, and receive data from drones directly within ATAK for enhanced situational awareness.",
            "source": "uas_tool",
            "category": "uas"
        },
        {
            "input": "How do I connect a drone to ATAK using the UAS Tool?",
            "output": "To connect a drone: 1) Open ATAK UAS Tool, 2) Select your drone type (DJI, Parrot, etc.), 3) Enable connection (WiFi or radio), 4) The drone will appear on the map with telemetry data, 5) You can now control and monitor the drone from ATAK.",
            "source": "uas_tool",
            "category": "uas_connection"
        },
        {
            "input": "What drone brands are supported by ATAK UAS Tool?",
            "output": "ATAK UAS Tool supports various drone platforms including DJI drones, Parrot drones, and drones using MAVLink protocol (PX4, ArduPilot). Specific support depends on your ATAK version and installed plugins.",
            "source": "uas_tool",
            "category": "uas_compatibility"
        },
        {
            "input": "Can I see live video from my drone in ATAK?",
            "output": "Yes, ATAK UAS Tool can display live video feed from compatible drones. The video appears in a window within ATAK, and you can also see the drone's position, altitude, battery, and other telemetry on the map.",
            "source": "uas_tool",
            "category": "uas_video"
        },
        {
            "input": "How do I plan a drone mission in ATAK?",
            "output": "To plan a drone mission: 1) Use ATAK's route planning tools to create waypoints, 2) Set altitude and speed for each waypoint, 3) Send the mission to the drone via UAS Tool, 4) The drone will execute the mission autonomously while sending telemetry back to ATAK.",
            "source": "uas_tool",
            "category": "uas_mission"
        },
        {
            "input": "What telemetry data can I see from my drone in ATAK?",
            "output": "ATAK displays comprehensive drone telemetry including: GPS position, altitude (AGL and MSL), speed, heading, battery level, signal strength, camera gimbal angle, flight mode, and distance from operator.",
            "source": "uas_tool",
            "category": "uas_telemetry"
        },
        {
            "input": "Can I control my drone directly from ATAK?",
            "output": "Yes, ATAK UAS Tool provides basic drone control: takeoff, land, return to home, and waypoint navigation. For advanced flight control, you may need to use the drone's dedicated controller alongside ATAK.",
            "source": "uas_tool",
            "category": "uas_control"
        },
        {
            "input": "How do I share drone video with my team in ATAK?",
            "output": "Drone video can be shared via ATAK's video streaming features. The video feed is broadcast over the TAK server network, allowing team members to view the live feed on their ATAK devices.",
            "source": "uas_tool",
            "category": "uas_sharing"
        },
        {
            "input": "What is MAVLink and how does it relate to ATAK?",
            "output": "MAVLink is a communication protocol for drones and ground control stations. ATAK supports MAVLink, allowing it to communicate with drones running PX4 or ArduPilot firmware. This enables ATAK to control and monitor a wide range of drone platforms.",
            "source": "uas_tool",
            "category": "uas_mavlink"
        },
        {
            "input": "Can I use ATAK with multiple drones simultaneously?",
            "output": "Yes, ATAK can track and manage multiple drones at once. Each drone appears as a separate icon on the map with its own telemetry. This is useful for coordinated multi-drone operations.",
            "source": "uas_tool",
            "category": "uas_multi"
        }
    ]
    
    print(f"✅ Generated {len(qa_pairs)} UAS Tool Q&A")
    return qa_pairs

def generate_novitak_qa() -> List[Dict]:
    """Generate NoviTAK Q&A"""
    qa_pairs = [
        {
            "input": "What is NoviTAK?",
            "output": "NoviTAK is a framework and set of tools for developing custom ATAK plugins. It simplifies plugin development by providing APIs, templates, and utilities that make it easier to extend ATAK's functionality.",
            "source": "novitak",
            "category": "novitak"
        },
        {
            "input": "Do I need to know NoviTAK to use ATAK plugins?",
            "output": "No, NoviTAK is a development framework. As an end user, you only need to install plugins built with NoviTAK. You don't need to know anything about NoviTAK itself to use the plugins.",
            "source": "novitak",
            "category": "novitak_usage"
        },
        {
            "input": "Are NoviTAK plugins compatible with standard ATAK?",
            "output": "Yes, plugins built with NoviTAK are standard ATAK plugins. They work with both ATAK-CIV (civilian) and ATAK-MIL (military) versions, following ATAK's plugin architecture.",
            "source": "novitak",
            "category": "novitak_compatibility"
        }
    ]
    
    print(f"✅ Generated {len(qa_pairs)} NoviTAK Q&A")
    return qa_pairs

def generate_advanced_atak_qa() -> List[Dict]:
    """Generate advanced ATAK end-user Q&A"""
    qa_pairs = [
        {
            "input": "How do I backup my ATAK data?",
            "output": "To backup ATAK data: 1) Go to Settings > Data Management, 2) Select 'Export Data Package', 3) Choose what to include (markers, routes, preferences), 4) Save the .zip file to external storage or cloud, 5) You can restore this data later using 'Import Data Package'.",
            "source": "advanced_atak",
            "category": "data_management"
        },
        {
            "input": "Can I use ATAK offline without internet?",
            "output": "Yes, ATAK works fully offline. Pre-download offline maps for your area, and ATAK will function without internet. You can still use GPS, create markers, plan routes, and use all local features. Network features (team tracking, chat) require connection to a TAK server.",
            "source": "advanced_atak",
            "category": "offline"
        },
        {
            "input": "How do I connect ATAK to a TAK server?",
            "output": "To connect to TAK server: 1) Go to Settings > Network Preferences, 2) Tap '+' to add server, 3) Enter server address (IP:port), 4) Add authentication (certificate or username/password), 5) Enable the connection, 6) ATAK will connect and sync with the server.",
            "source": "advanced_atak",
            "category": "networking"
        },
        {
            "input": "What's the difference between ATAK-CIV and ATAK-MIL?",
            "output": "ATAK-CIV is the civilian/public version, freely available. ATAK-MIL is the military version with additional classified features and encryption. Both share the same core functionality. Most users use ATAK-CIV.",
            "source": "advanced_atak",
            "category": "versions"
        },
        {
            "input": "How do I import KML files into ATAK?",
            "output": "To import KML: 1) Copy the .kml or .kmz file to your device, 2) In ATAK, go to Import Manager, 3) Select 'Local SD', 4) Navigate to your file, 5) Tap to import, 6) The KML data (markers, routes, overlays) will appear on your map.",
            "source": "advanced_atak",
            "category": "import_export"
        },
        {
            "input": "Can I use ATAK for search and rescue operations?",
            "output": "Yes, ATAK is excellent for search and rescue. Features include: team tracking, area search patterns, marker placement for clues/findings, route planning, offline maps, and communication. Many SAR teams use ATAK for coordination.",
            "source": "advanced_atak",
            "category": "use_cases"
        },
        {
            "input": "How do I share my location continuously with my team?",
            "output": "Your location is automatically shared when connected to a TAK server. Your team members see your position update in real-time on their maps. You can adjust update frequency in Settings > Device Preferences > Location.",
            "source": "advanced_atak",
            "category": "location_sharing"
        },
        {
            "input": "What Android version does ATAK require?",
            "output": "ATAK requires Android 5.0 (Lollipop) or higher. For best performance, Android 8.0 or newer is recommended. ATAK works on both phones and tablets.",
            "source": "advanced_atak",
            "category": "requirements"
        },
        {
            "input": "Can I use ATAK with external GPS devices?",
            "output": "Yes, ATAK supports external GPS via Bluetooth. This is useful for better accuracy or when using devices without built-in GPS. Pair the GPS device in Android settings, then select it in ATAK's location settings.",
            "source": "advanced_atak",
            "category": "gps"
        },
        {
            "input": "How do I create a geofence in ATAK?",
            "output": "To create a geofence: 1) Use the drawing tools to create a shape (circle, polygon), 2) Long-press the shape, 3) Select 'Set as Geofence', 4) Configure alerts (entry/exit notifications), 5) ATAK will alert you when markers or team members enter/exit the area.",
            "source": "advanced_atak",
            "category": "geofence"
        }
    ]
    
    print(f"✅ Generated {len(qa_pairs)} Advanced ATAK Q&A")
    return qa_pairs

def combine_all_sources() -> List[Dict]:
    """Combine all Q&A sources"""
    print("=" * 70)
    print("🎯 Combining ALL ATAK Knowledge Sources")
    print("=" * 70)
    print()
    
    all_data = []
    
    # Load Thor Q&A
    all_data.extend(load_thor_qa())
    
    # Load our basic generated Q&A
    try:
        with open("training/datasets/atak_full.json", 'r') as f:
            basic_qa = json.load(f)
        all_data.extend(basic_qa)
        print(f"✅ Loaded {len(basic_qa)} basic ATAK Q&A")
    except:
        print("⚠️  Could not load basic Q&A")
    
    # Add Argonaut Plugin Q&A
    all_data.extend(generate_argonaut_plugin_qa())
    
    # Add UAS Tool Q&A
    all_data.extend(generate_uas_tool_qa())
    
    # Add NoviTAK Q&A
    all_data.extend(generate_novitak_qa())
    
    # Add Advanced ATAK Q&A
    all_data.extend(generate_advanced_atak_qa())
    
    print()
    print(f"✅ TOTAL: {len(all_data)} comprehensive Q&A pairs")
    
    return all_data

def save_comprehensive_dataset(data: List[Dict]):
    """Save comprehensive dataset"""
    print()
    print("=" * 70)
    print("💾 Saving Comprehensive Dataset")
    print("=" * 70)
    
    # Shuffle
    random.shuffle(data)
    
    # Split 85/15
    split_idx = int(len(data) * 0.85)
    train_data = data[:split_idx]
    val_data = data[split_idx:]
    
    # Save JSONL
    with open("training/datasets/atak_comprehensive_train.jsonl", 'w') as f:
        for item in train_data:
            f.write(json.dumps(item) + '\n')
    
    with open("training/datasets/atak_comprehensive_val.jsonl", 'w') as f:
        for item in val_data:
            f.write(json.dumps(item) + '\n')
    
    # Save full JSON
    with open("training/datasets/atak_comprehensive_full.json", 'w') as f:
        json.dump(data, f, indent=2)
    
    print(f"✅ Train: {len(train_data)} examples")
    print(f"✅ Val: {len(val_data)} examples")
    print(f"✅ Total: {len(data)} examples")
    
    # Breakdown by source
    sources = {}
    categories = {}
    for item in data:
        source = item.get("source", "unknown")
        sources[source] = sources.get(source, 0) + 1
        
        cat = item.get("category", "unknown")
        categories[cat] = categories.get(cat, 0) + 1
    
    print()
    print("📊 Breakdown by source:")
    for source, count in sorted(sources.items()):
        print(f"   • {source}: {count} examples")
    
    print()
    print("📊 Top categories:")
    for cat, count in sorted(categories.items(), key=lambda x: x[1], reverse=True)[:10]:
        print(f"   • {cat}: {count} examples")

def main():
    print("=" * 70)
    print("🚀 COMPREHENSIVE ATAK Training Data Generation")
    print("=" * 70)
    print()
    print("Sources:")
    print("  • Thor PDF Q&A")
    print("  • Basic ATAK Q&A")
    print("  • Argonaut Plugin")
    print("  • UAS Tool")
    print("  • NoviTAK")
    print("  • Advanced ATAK Features")
    print("=" * 70)
    print()
    
    # Combine all
    data = combine_all_sources()
    
    # Save
    save_comprehensive_dataset(data)
    
    print()
    print("=" * 70)
    print("✅ COMPLETE!")
    print("=" * 70)
    print()
    print("Files created:")
    print("  • training/datasets/atak_comprehensive_train.jsonl")
    print("  • training/datasets/atak_comprehensive_val.jsonl")
    print("  • training/datasets/atak_comprehensive_full.json")
    print()
    print("Next: Deploy to Thor and train!")
    print("=" * 70)

if __name__ == "__main__":
    main()
