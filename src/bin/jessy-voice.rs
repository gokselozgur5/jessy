//! JESSY Voice - Push-to-Talk Edition
//! 
//! HOLD SPACEBAR to talk. Release to send. Press to interrupt.

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
use device_query::{DeviceQuery, DeviceState, Keycode};

const SAMPLE_RATE: u32 = 16000; 

// Global sink for interruption
lazy_static::lazy_static! {
    static ref GLOBAL_SINK: Arc<Mutex<Option<Sink>>> = Arc::new(Mutex::new(None));
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();
    
    let anthropic_key = std::env::var("ANTHROPIC_API_KEY").unwrap_or_default();
    let openai_key = std::env::var("OPENAI_API_KEY").expect("❌ OPENAI_API_KEY must be set in .env!");

    // Initialize Global Audio Output
    let (_stream, stream_handle) = OutputStream::try_default().unwrap();
    let sink = Sink::try_new(&stream_handle).unwrap();
    *GLOBAL_SINK.lock().unwrap() = Some(sink);

    println!("🧠 Initializing JESSY Push-to-Talk...");

    // 1. Initialize Consciousness
    let config = SystemConfig::from_env().unwrap_or_default();
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

    // Audio Input Setup
    println!("🔊 Initializing Audio Device...");
    let host = cpal::default_host();
    let input_device = host.default_input_device().expect("No input device found");
    let client = reqwest::Client::new();
    let device_state = DeviceState::new();

    println!("\n✅ JESSY PTT Ready!");
    println!("🔘 HOLD SPACEBAR to talk.");
    println!("   (Release to send. Press to interrupt.)\n");

    let mut last_space = false;

    loop {
        let keys = device_state.get_keys();
        let space_pressed = keys.contains(&Keycode::Space);

        if space_pressed && !last_space {
            // START RECORDING
            // 1. Interrupt playback
            if let Some(sink) = GLOBAL_SINK.lock().unwrap().as_ref() {
                if !sink.empty() {
                    sink.stop();
                    // Re-create sink because stop() kills it usually? 
                    // rodio::Sink::stop() clears queue but keeps sink alive?
                    // Actually stop() pauses. clear() clears.
                    // rodio::Sink doesn't have clear(). stop() is deprecated/pause.
                    // We will just create a new one next time or let it empty.
                    // Actually, creating a new Sink is safest for instant cut.
                    *GLOBAL_SINK.lock().unwrap() = Some(Sink::try_new(&stream_handle).unwrap());
                    println!("🛑 Interrupted.");
                }
            }

            println!("🔴 Recording... (Release SPACE to send)");
            
            match record_while_pressed(&input_device, &device_state) {
                Ok(samples) => {
                    if samples.len() < 8000 { // < 0.5s
                        println!("⚠️ Too short.");
                        last_space = false; // Reset state
                        // Wait until space release to avoid loop
                        while device_state.get_keys().contains(&Keycode::Space) {
                            std::thread::sleep(Duration::from_millis(10));
                        }
                        continue;
                    }

                    println!("📤 Processing...");
                    let wav_bytes = samples_to_wav(&samples);
                    
                    match transcribe_whisper(&client, &openai_key, wav_bytes).await {
                        Ok(text) => {
                            let text = text.trim();
                            if !text.is_empty() {
                                println!("🗣️  You: {}", text);
                                
                                // Process
                                let messages = vec![];
                                match orchestrator.process(text, Some("voice-user"), messages).await {
                                    Ok(result) => {
                                        let response = result.final_response;
                                        println!("🤖 Jessy: {}", response);
                                        
                                        // TTS
                                        match generate_tts_openai(&client, &openai_key, &response).await {
                                            Ok(audio_data) => {
                                                play_audio_blob(audio_data, &stream_handle);
                                            },
                                            Err(e) => eprintln!("❌ TTS Error: {}", e),
                                        }
                                    }
                                    Err(e) => eprintln!("❌ Brain Error: {}", e),
                                }
                            }
                        },
                        Err(e) => eprintln!("❌ STT Error: {}", e),
                    }
                },
                Err(e) => eprintln!("❌ Mic Error: {}", e),
            }
            
            println!("\n🔘 HOLD SPACEBAR to talk...");
        }
        
        last_space = space_pressed;
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn record_while_pressed(device: &cpal::Device, device_state: &DeviceState) -> Result<Vec<f32>, Box<dyn std::error::Error>> {
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
    
    // Record loop while space is held
    while device_state.get_keys().contains(&Keycode::Space) {
        while let Ok(sample) = rx.try_recv() {
            buffer.push(sample);
        }
        std::thread::sleep(Duration::from_millis(10));
    }
    
    // Capture remaining samples
    while let Ok(sample) = rx.try_recv() {
        buffer.push(sample);
    }
    
    Ok(resample(&buffer, stream_sample_rate, SAMPLE_RATE))
}

fn play_audio_blob(data: Vec<u8>, stream_handle: &rodio::OutputStreamHandle) {
    let cursor = Cursor::new(data);
    if let Ok(decoder) = Decoder::new(cursor) {
        // Use the global sink so we can interrupt it later
        let mut sink_lock = GLOBAL_SINK.lock().unwrap();
        // Replace with new sink to ensure clean state
        *sink_lock = Some(Sink::try_new(stream_handle).unwrap());
        
        if let Some(sink) = sink_lock.as_ref() {
            sink.append(decoder);
            sink.play(); // Ensure it plays
            // We DON'T sleep here. We return to main loop so we can check for Spacebar interrupt.
        }
    }
}

// ... (Reuse existing helpers: transcribe_whisper, generate_tts_openai, samples_to_wav, resample) ...
// I will include them to be safe as replace replaces the whole file content if I use write_file or big match
// Since I'm using `replace` with `new_string` and `old_string`, I need to be careful.
// I'll use `write_file` to overwrite the whole file cleanly with the new logic.

async fn transcribe_whisper(client: &reqwest::Client, key: &str, wav_data: Vec<u8>) -> Result<String, Box<dyn std::error::Error>> {
    use reqwest::multipart;
    let part = multipart::Part::bytes(wav_data).file_name("audio.wav").mime_str("audio/wav")?;
    let form = multipart::Form::new().part("file", part).text("model", "whisper-1");
    let res = client.post("https://api.openai.com/v1/audio/transcriptions").bearer_auth(key).multipart(form).send().await?;
    if !res.status().is_success() { return Err(format!("API Error: {}", res.text().await?).into()); }
    let json: serde_json::Value = res.json().await?;
    Ok(json["text"].as_str().unwrap_or("").to_string())
}

async fn generate_tts_openai(client: &reqwest::Client, key: &str, text: &str) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let voice = std::env::var("OPENAI_VOICE").unwrap_or_else(|_| "nova".to_string());
    let res = client.post("https://api.openai.com/v1/audio/speech").bearer_auth(key).json(&serde_json::json!({ "model": "tts-1", "input": text, "voice": voice })).send().await?;
    if !res.status().is_success() { return Err(format!("API Error: {}", res.text().await?).into()); }
    let bytes = res.bytes().await?;
    Ok(bytes.to_vec())
}

fn samples_to_wav(samples: &[f32]) -> Vec<u8> {
    let spec = hound::WavSpec { channels: 1, sample_rate: SAMPLE_RATE, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut cursor = Cursor::new(Vec::new());
    {
        let mut writer = WavWriter::new(&mut cursor, spec).unwrap();
        for &sample in samples { let s = (sample * 32767.0) as i16; writer.write_sample(s).unwrap(); } 
        writer.finalize().unwrap();
    }
    cursor.into_inner()
}

fn resample(input: &[f32], input_rate: u32, target_rate: u32) -> Vec<f32> {
    if input_rate == target_rate { return input.to_vec(); }
    let ratio = input_rate as f32 / target_rate as f32;
    let target_len = (input.len() as f32 / ratio) as usize;
    let mut output = Vec::with_capacity(target_len);
    for i in 0..target_len {
        let index = (i as f32 * ratio) as usize;
        if index < input.len() { output.push(input[index]); }
    }
    output
}
