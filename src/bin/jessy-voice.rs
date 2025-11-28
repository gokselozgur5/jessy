//! JESSY Voice - Hands-free Voice Interaction
//! 
//! Continuously listens for voice, transcribes, processes, and speaks back.

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use rodio::{OutputStream, Sink, buffer::SamplesBuffer};
use jessy::sherpa::{SherpaRecognizer, SherpaTts};
use jessy::config::SystemConfig;
use jessy::processing::ConsciousnessOrchestrator;
use jessy::navigation::NavigationSystem;
use jessy::memory::MmapManager;
use jessy::llm::LLMConfig;

const SAMPLE_RATE: u32 = 16000;
const VAD_THRESHOLD: f32 = 0.015; // Adjust based on mic sensitivity
const SILENCE_DURATION_MS: u128 = 1500; // 1.5 seconds of silence to stop

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();
    
    // Graceful shutdown setup
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\n🛑 Stopping JESSY Voice...");
        r.store(false, Ordering::SeqCst);
    })?;

    println!("🧠 Initializing JESSY Voice System...");

    // 1. Initialize Consciousness System (Orchestrator)
    // ------------------------------------------------------------------
    let config = SystemConfig::from_env().unwrap_or_default();
    
    // Load dimensions
    let dimensions_path = if std::path::Path::new("data/cognitive_layers.json").exists() {
        "data/dimensions.json"
    } else {
        "data/dimensions.json"
    };
    
    let dimensions_data = std::fs::read_to_string(dimensions_path)
        .unwrap_or_else(|_| include_str!("../../data/dimensions.json").to_string());
        
    let registry = Arc::new(jessy::navigation::DimensionRegistry::load_dimensions(&dimensions_data)?);
    let memory = Arc::new(MmapManager::new(config.limits.memory_limit_mb)?);
    let navigation = Arc::new(NavigationSystem::new(registry.clone(), memory.clone())?);
    
    // Initialize stub layers for all dimensions (Fix for "Layer not found")
    println!("   - Creating placeholder layers for all dimensions");
    for dim_id in 1..=14 {
        let dimension_id = jessy::DimensionId(dim_id);
        if let Some(dim_meta) = registry.get_dimension(dimension_id) {
            if let Some(root_layer) = registry.get_root_layer(dimension_id) {
                let stub_content = format!(
                    "Dimension: {}\nFrequency: {:.2} Hz\nKeywords: {}\n",
                    dim_meta.name,
                    root_layer.frequency,
                    root_layer.keywords.join(", ")
                );
                
                if let Err(e) = memory.create_proto_dimension(dimension_id, stub_content.into_bytes()) {
                    eprintln!("⚠️  Warning: Failed to create stub for dimension {}: {}", dim_id, e);
                }
            }
        }
    }
    
    // LLM Setup
    let api_key = std::env::var("ANTHROPIC_API_KEY").unwrap_or_default();
    let llm_config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-3-haiku-20240307".to_string(), // Switch to Haiku for speed and availability
        api_key,
        timeout_secs: 30,
        max_retries: 3,
    };
    
    let mut orchestrator = ConsciousnessOrchestrator::with_llm(
        navigation,
        memory,
        llm_config
    )?;

    // 2. Initialize Audio Systems
    // ------------------------------------------------------------------
    println!("🎤 Initializing Sherpa STT/TTS...");
    let stt = SherpaRecognizer::new().expect("Failed to initialize STT");
    let tts = SherpaTts::new().expect("Failed to initialize TTS");

    println!("🔊 Initializing Audio Device...");
    let host = cpal::default_host();
    let input_device = host.default_input_device().expect("No input device found");
    
    println!("✅ JESSY Voice is ready! Start speaking...");
    println!("   (Silence for 1.5s triggers processing)");

    // Main Loop
    while running.load(Ordering::SeqCst) {
        // 3. Record Audio
        // --------------------------------------------------------------
        let samples = record_audio(&input_device, running.clone())?;
        
        if !running.load(Ordering::SeqCst) { break; }
        if samples.len() < (SAMPLE_RATE as usize) / 2 {
            continue; // Too short, ignore
        }

        // 4. Transcribe (STT)
        // --------------------------------------------------------------
        print!("Thinking... "); 
        use std::io::Write;
        std::io::stdout().flush()?;
        
        if let Some(text) = stt.decode(&samples) {
            let text = text.trim();
            if text.is_empty() {
                println!("(no speech detected)");
                continue;
            }
            
            println!("\n🗣️  You: {}", text);
            
            // 5. Process (Brain)
            // --------------------------------------------------------------
            let messages = vec![]; 
            
            match orchestrator.process(text, Some("voice-user"), messages).await {
                Ok(result) => {
                    let response = result.final_response;
                    println!("🤖 Jessy: {}", response);
                    
                    if !running.load(Ordering::SeqCst) { break; }

                    // 6. Speak (TTS)
                    // --------------------------------------------------------------
                    if let Some(audio_samples) = tts.generate(&response) {
                        play_audio(&audio_samples, running.clone());
                    } else {
                        eprintln!("❌ TTS failed to generate audio");
                    }
                }
                Err(e) => {
                    eprintln!("❌ Error processing query: {}", e);
                }
            }
        } else {
            println!("(STT returned null)");
        }
        
        println!("\n🎤 Listening...");
    }
    
    println!("👋 Goodbye!");
    Ok(())
}

fn record_audio(device: &cpal::Device, running: Arc<AtomicBool>) -> Result<Vec<f32>, Box<dyn std::error::Error>> {
    let config: cpal::StreamConfig = device.default_input_config()?.into();
    let err_fn = |err| eprintln!("an error occurred on stream: {}", err);
    
    let (tx, rx) = mpsc::channel();
    
    let stream_sample_rate = config.sample_rate.0;
    let target_sample_rate = SAMPLE_RATE;
    let channels = config.channels as usize;
    
    let stream = device.build_input_stream(
        &config,
        move |data: &[f32], _: &_| {
            for frame in data.chunks(channels) {
                let sample = frame.iter().sum::<f32>() / channels as f32;
                let _ = tx.send(sample);
            }
        },
        err_fn,
        None
    )?;

    stream.play()?;

    let mut buffer = Vec::new();
    let mut is_speaking = false;
    let mut last_speech = Instant::now();
    
    let window_size = (stream_sample_rate / 20) as usize; 
    let mut window = Vec::with_capacity(window_size);

    while running.load(Ordering::SeqCst) {
        while let Ok(sample) = rx.try_recv() {
            window.push(sample);
            if window.len() >= window_size {
                let rms = (window.iter().map(|x| x * x).sum::<f32>() / window.len() as f32).sqrt();
                
                if rms > VAD_THRESHOLD {
                    if !is_speaking {
                        print!("🎙️ "); 
                        use std::io::Write;
                        std::io::stdout().flush().unwrap();
                    }
                    is_speaking = true;
                    last_speech = Instant::now();
                } else if is_speaking {
                    if last_speech.elapsed().as_millis() > SILENCE_DURATION_MS {
                        buffer.extend_from_slice(&window);
                        return Ok(resample(&buffer, stream_sample_rate, target_sample_rate));
                    }
                }
                
                buffer.extend_from_slice(&window);
                window.clear();
            }
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    
    Ok(Vec::new())
}

fn resample(input: &[f32], input_rate: u32, target_rate: u32) -> Vec<f32> {
    if input_rate == target_rate {
        return input.to_vec();
    }
    
    let ratio = input_rate as f32 / target_rate as f32;
    let target_len = (input.len() as f32 / ratio) as usize;
    let mut output = Vec::with_capacity(target_len);
    
    for i in 0..target_len {
        let index = (i as f32 * ratio) as usize;
        if index < input.len() {
            output.push(input[index]);
        }
    }
    
    output
}

fn play_audio(samples: &[f32], running: Arc<AtomicBool>) {
    if !running.load(Ordering::SeqCst) { return; }
    
    let (_stream, stream_handle) = OutputStream::try_default().unwrap();
    let sink = Sink::try_new(&stream_handle).unwrap();
    
    let source = SamplesBuffer::new(1, 22050, samples);
    sink.append(source);
    
    // Wait nicely checking running state
    while !sink.empty() {
        if !running.load(Ordering::SeqCst) {
            sink.stop();
            break;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
}
