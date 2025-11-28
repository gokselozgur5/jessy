//! JESSY Voice - Cloud Hybrid Edition
//! 
//! STT: OpenAI Whisper (Multi-lingual, Mix)
//! Brain: Anthropic Claude (Fast)
//! TTS: OpenAI TTS (High Quality)

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::mpsc;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};
use rodio::{OutputStream, Sink, Decoder};
use jessy::config::SystemConfig;
use jessy::processing::ConsciousnessOrchestrator;
use jessy::navigation::NavigationSystem;
use jessy::memory::MmapManager;
use jessy::llm::LLMConfig;
use std::io::Cursor;
use hound::WavWriter;

const SAMPLE_RATE: u32 = 16000; // Whisper handles 16k well
const VAD_THRESHOLD: f32 = 0.01; // More sensitive (was 0.015)
const SILENCE_DURATION_MS: u128 = 2500; // Allow 2.5s pause (was 1.5s) 

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();
    
    // Check keys
    let anthropic_key = std::env::var("ANTHROPIC_API_KEY").unwrap_or_default();
    let openai_key = std::env::var("OPENAI_API_KEY").expect("❌ OPENAI_API_KEY must be set in .env for high-quality voice!");

    // Graceful shutdown
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\n🛑 Stopping JESSY Voice...");
        r.store(false, Ordering::SeqCst);
    })?;

    println!("🧠 Initializing JESSY Voice System (Cloud Edition)...");

    // 1. Initialize Consciousness
    let config = SystemConfig::from_env().unwrap_or_default();
    
    // Load dimensions stub logic
    let dimensions_data = include_str!("../../data/dimensions.json").to_string();
    let registry = Arc::new(jessy::navigation::DimensionRegistry::load_dimensions(&dimensions_data)?);
    let memory = Arc::new(MmapManager::new(config.limits.memory_limit_mb)?);
    let navigation = Arc::new(NavigationSystem::new(registry.clone(), memory.clone())?);
    
    // Initialize stubs
    for dim_id in 1..=14 {
        let dimension_id = jessy::DimensionId(dim_id);
        if let Some(dim_meta) = registry.get_dimension(dimension_id) {
            if let Some(root_layer) = registry.get_root_layer(dimension_id) {
                let stub = format!("Dimension: {}\nStub", dim_meta.name);
                let _ = memory.create_proto_dimension(dimension_id, stub.into_bytes());
            }
        }
    }
    
    let llm_config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-3-5-haiku-20241022".to_string(), 
        api_key: anthropic_key,
        timeout_secs: 30,
        max_retries: 3,
    };
    
    let mut orchestrator = ConsciousnessOrchestrator::with_llm(
        navigation,
        memory,
        llm_config
    )?;

    // Audio Setup
    println!("🔊 Initializing Audio Device...");
    let host = cpal::default_host();
    let input_device = host.default_input_device().expect("No input device found");
    let client = reqwest::Client::new();

    println!("✅ JESSY Cloud Voice is ready! (Turkish/English Mix Supported)");
    println!("   (Silence for 1.5s triggers processing)");

    while running.load(Ordering::SeqCst) {
        // 2. Record
        let samples = record_audio(&input_device, running.clone())?;
        if !running.load(Ordering::SeqCst) { break; }
        if samples.is_empty() { continue; }

        // 3. Transcribe (Whisper)
        print!("👂 Listening (Whisper)... "); 
        use std::io::Write;
        std::io::stdout().flush()?;
        
        // Convert to WAV bytes
        let wav_bytes = samples_to_wav(&samples);
        
        match transcribe_whisper(&client, &openai_key, wav_bytes).await {
            Ok(text) => {
                if text.trim().is_empty() { continue; }
                println!("\n🗣️  You: {}", text);
                
                // 4. Process
                let messages = vec![]; 
                match orchestrator.process(&text, Some("voice-user"), messages).await {
                    Ok(result) => {
                        let response = result.final_response;
                        println!("🤖 Jessy: {}", response);
                        
                        if !running.load(Ordering::SeqCst) { break; }

                        // 5. Speak (OpenAI TTS)
                        print!("🔊 Generating HD Audio... ");
                        std::io::stdout().flush()?;
                        match generate_tts_openai(&client, &openai_key, &response).await {
                            Ok(audio_data) => {
                                println!("Done.");
                                play_audio_stream(audio_data, running.clone());
                            },
                            Err(e) => eprintln!("\n❌ TTS Error: {}", e),
                        }
                    }
                    Err(e) => eprintln!("❌ Processing Error: {}", e),
                }
            },
            Err(e) => eprintln!("\n❌ Transcription Error: {}", e),
        }
        
        println!("\n🎤 Ready...");
    }
    
    Ok(())
}

// OpenAI Whisper API
async fn transcribe_whisper(client: &reqwest::Client, key: &str, wav_data: Vec<u8>) -> Result<String, Box<dyn std::error::Error>> {
    use reqwest::multipart;
    
    let part = multipart::Part::bytes(wav_data)
        .file_name("audio.wav")
        .mime_str("audio/wav")?;
        
    let form = multipart::Form::new()
        .part("file", part)
        .text("model", "whisper-1");

    let res = client.post("https://api.openai.com/v1/audio/transcriptions")
        .bearer_auth(key)
        .multipart(form)
        .send()
        .await?;

    if !res.status().is_success() {
        return Err(format!("API Error: {}", res.text().await?).into());
    }

    let json: serde_json::Value = res.json().await?;
    Ok(json["text"].as_str().unwrap_or("").to_string())
}

// OpenAI TTS API
async fn generate_tts_openai(client: &reqwest::Client, key: &str, text: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let voice = std::env::var("OPENAI_VOICE").unwrap_or_else(|_| "nova".to_string());
    
    let res = client.post("https://api.openai.com/v1/audio/speech")
        .bearer_auth(key)
        .json(&serde_json::json!({
            "model": "tts-1",
            "input": text,
            "voice": voice
        }))
        .send()
        .await?;

    if !res.status().is_success() {
        return Err(format!("API Error: {}", res.text().await?).into());
    }

    let bytes = res.bytes().await?;
    Ok(bytes.to_vec())
}

fn samples_to_wav(samples: &[f32]) -> Vec<u8> {
    let spec = hound::WavSpec {
        channels: 1,
        sample_rate: SAMPLE_RATE,
        bits_per_sample: 16,
        sample_format: hound::SampleFormat::Int,
    };
    
    let mut cursor = Cursor::new(Vec::new());
    {
        let mut writer = WavWriter::new(&mut cursor, spec).unwrap();
        for &sample in samples {
            let s = (sample * 32767.0) as i16;
            writer.write_sample(s).unwrap();
        }
        writer.finalize().unwrap();
    }
    cursor.into_inner()
}

fn record_audio(device: &cpal::Device, running: Arc<AtomicBool>) -> Result<Vec<f32>, Box<dyn std::error::Error>> {
    let config: cpal::StreamConfig = device.default_input_config()?.into();
    let (tx, rx) = mpsc::channel();
    
    let stream_sample_rate = config.sample_rate.0;
    let channels = config.channels as usize;
    
    let stream = device.build_input_stream(
        &config,
        move |data: &[f32], _: &_| {
            for frame in data.chunks(channels) {
                let sample = frame.iter().sum::<f32>() / channels as f32;
                let _ = tx.send(sample);
            }
        },
        |e| eprintln!("Stream error: {}", e),
        None
    )?;
    
    stream.play()?;
    
    let mut buffer = Vec::new();
    let mut is_speaking = false;
    let mut last_speech = Instant::now();
    let window_size = (stream_sample_rate / 20); // 50ms
    let mut window = Vec::with_capacity(window_size as usize);
    
    while running.load(Ordering::SeqCst) {
        while let Ok(sample) = rx.try_recv() {
            window.push(sample);
            if window.len() >= window_size as usize {
                let rms = (window.iter().map(|x| x * x).sum::<f32>() / window.len() as f32).sqrt();
                
                if rms > VAD_THRESHOLD {
                    if !is_speaking {
                        print!("🎙️ ");
                        use std::io::Write;
                        std::io::stdout().flush()?;
                        is_speaking = true;
                    }
                    last_speech = Instant::now();
                } else if is_speaking {
                    if last_speech.elapsed().as_millis() > SILENCE_DURATION_MS {
                        buffer.extend_from_slice(&window);
                        return Ok(resample(&buffer, stream_sample_rate, SAMPLE_RATE)); // Resample before returning
                    }
                }
                buffer.extend_from_slice(&window);
                window.clear();
            }
        }
        // Cap max duration to avoid memory boom (e.g. 30s)
        if buffer.len() > (SAMPLE_RATE as usize) * 30 { 
            eprintln!("\n⚠️ Max recording duration reached (30s). Processing audio.");
            return Ok(resample(&buffer, stream_sample_rate, SAMPLE_RATE)); // Resample and return
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    
    // If loop ends by Ctrl+C, process whatever was recorded
    Ok(resample(&buffer, stream_sample_rate, SAMPLE_RATE)) // Resample and return
}

// Basic nearest-neighbor/linear resampling
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
fn play_audio_stream(data: Vec<u8>, running: Arc<AtomicBool>) {
    if !running.load(Ordering::SeqCst) { return; }
    let (_stream, stream_handle) = OutputStream::try_default().unwrap();
    let sink = Sink::try_new(&stream_handle).unwrap();
    
    let cursor = Cursor::new(data);
    if let Ok(decoder) = Decoder::new(cursor) {
        sink.append(decoder);
        while !sink.empty() {
            if !running.load(Ordering::SeqCst) { sink.stop(); break; }
            std::thread::sleep(Duration::from_millis(10)); // Check more frequently
        }
    } else {
        eprintln!("Failed to decode MP3 data from OpenAI");
    }
}
