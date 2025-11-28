//! JESSY Voice - Real-Time Pipeline Edition
//! 
//! Architecture:
//! 1. Record (PTT) -> Whisper API -> Text
//! 2. Anthropic LLM (Streaming) -> Text Chunks -> Sentence Builder
//! 3. Sentence Builder -> OpenAI TTS API -> Audio Blob -> Player
//! 
//! All stages overlap for minimum latency.

use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use std::sync::mpsc as std_mpsc;
use tokio::sync::mpsc as tokio_mpsc;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;
use rodio::{OutputStream, Sink, Decoder};
use jessy::navigation::DimensionSelector;
use jessy::llm::{LLMConfig, AnthropicProvider};
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

    println!("🧠 Initializing JESSY Real-Time Voice...");

    // Init components
    let selector = DimensionSelector::new(anthropic_key.clone());
    let llm_config = LLMConfig {
        provider: "anthropic".to_string(),
        model: "claude-3-5-haiku-20241022".to_string(), 
        api_key: anthropic_key.clone(),
        timeout_secs: 30,
        max_retries: 3,
    };
    let anthropic = Arc::new(AnthropicProvider::new(&llm_config)?);

    // Audio Input Setup
    println!("🔊 Initializing Audio Device...");
    let host = cpal::default_host();
    let input_device = host.default_input_device().expect("No input device found");
    let client = reqwest::Client::new();
    let device_state = DeviceState::new();

    // Graceful shutdown
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\n🛑 Stopping...");
        r.store(false, Ordering::SeqCst);
    })?;

    println!("\n✅ JESSY Real-Time Ready!");
    println!("🔘 HOLD SPACEBAR to talk. (Releasing sends)");
    println!("   (Pressing SPACE while talking interrupts)\n");

    let mut last_space = false;
    
    // Task handles for interruption
    let mut current_task: Option<tokio::task::JoinHandle<()>> = None;

    while running.load(Ordering::SeqCst) {
        let keys = device_state.get_keys();
        let space_pressed = keys.contains(&Keycode::Space);

        if space_pressed && !last_space {
            // INTERRUPT & RECORD
            
            // 1. Kill previous task
            if let Some(handle) = current_task.take() {
                handle.abort();
                println!("⚡ Task Aborted.");
            }
            
            // 2. Kill Audio
            {
                let mut sink_lock = GLOBAL_SINK.lock().unwrap();
                if let Some(sink) = sink_lock.as_ref() {
                    if !sink.empty() {
                        // Replace sink to clear queue instantly
                        *sink_lock = Some(Sink::try_new(&stream_handle).unwrap());
                        println!("🛑 Audio Stopped.");
                    }
                }
            }

            println!("🔴 Recording...");
            
            match record_while_pressed(&input_device, &device_state) {
                Ok(samples) => {
                    if samples.len() < 8000 {
                        println!("⚠️ Click (too short)");
                    } else {
                        println!("📤 Processing...");
                        let wav_bytes = samples_to_wav(&samples);
                        let openai_key_clone = openai_key.clone();
                        let client_clone = client.clone();
                        let anthropic_clone = anthropic.clone();
                        let selector_clone = selector.clone();
                        let stream_handle_clone = stream_handle.clone();
                        
                        // Spawn processing task
                        current_task = Some(tokio::spawn(async move {
                            if let Err(e) = process_and_stream(
                                client_clone, 
                                openai_key_clone, 
                                wav_bytes,
                                anthropic_clone,
                                selector_clone,
                                stream_handle_clone
                            ).await {
                                eprintln!("❌ Process Error: {}", e);
                            }
                        }));
                    }
                },
                Err(e) => eprintln!("❌ Mic Error: {}", e),
            }
            
            println!("\n🔘 HOLD SPACEBAR...");
            
            // Debounce
            while device_state.get_keys().contains(&Keycode::Space) {
                tokio::time::sleep(Duration::from_millis(10)).await;
            }
        }
        
        last_space = space_pressed;
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    
    Ok(())
}

// The Mega-Pipeline
async fn process_and_stream(
    client: reqwest::Client,
    openai_key: String,
    wav_bytes: Vec<u8>,
    anthropic: Arc<AnthropicProvider>,
    selector: DimensionSelector,
    stream_handle: rodio::OutputStreamHandle,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    
    // 1. STT
    let text = transcribe_whisper(&client, &openai_key, wav_bytes).await?;
    let text = text.trim();
    if text.is_empty() { return Ok(()); }
    println!("🗣️  You: {}", text);

    // 2. Setup Context (Fast)
    let selection = selector.select(text).await?;
    let system_prompt = jessy::api::chat::build_system_prompt(&selection, None);
    let user_prompt = format!("CURRENT QUERY: {}", text);

    // 3. Stream LLM -> TTS
    let (tx_text, mut rx_text) = tokio_mpsc::unbounded_channel::<String>();
    
    // Spawn LLM Streamer
    let anthropic_clone = anthropic.clone();
    let user_prompt_clone = user_prompt.clone();
    let system_prompt_clone = system_prompt.clone();
    
    tokio::spawn(async move {
        let _ = anthropic_clone.call_api_streaming(
            &user_prompt_clone,
            &system_prompt_clone,
            |chunk| {
                let _ = tx_text.send(chunk.to_string());
            }
        ).await;
    });

    // Sentence Accumulator & TTS Loop
    let mut buffer = String::new();
    println!("🤖 Jessy: ");
    
    while let Some(chunk) = rx_text.recv().await {
        print!("{}", chunk); // Print raw chunk as it comes
        use std::io::Write;
        std::io::stdout().flush().unwrap();
        
        buffer.push_str(&chunk);
        
        // Check for sentence end
        if let Some(idx) = find_sentence_end(&buffer) {
            let sentence = buffer[..=idx].to_string();
            let remainder = buffer[idx+1..].to_string();
            buffer = remainder;
            
            if sentence.trim().len() > 1 {
                // Generate TTS for this sentence concurrently
                // Wait... if we await here, we block reading new chunks?
                // No, we must spawn TTS generation or it blocks the text stream processing.
                // BUT we must play them in order.
                // Solution: We await TTS generation, but play audio async?
                // Or better: We fetch TTS, then play.
                // If we block here, the text accumulation pauses. That's actually fine, 
                // LLM is usually faster than TTS+Playback.
                // But we want to fetch NEXT sentence while PLAYING current.
                
                // Current flow: 
                // 1. Got sentence.
                // 2. Fetch Audio (takes ~500ms).
                // 3. Play Audio (takes ~3s).
                
                // If we await fetch, we delay next text processing by 500ms.
                // If we await playback, we delay by 3s!
                // play_audio_blob is now async but doesn't block? No, I changed it back?
                // I need a `play_audio_queue` logic.
                
                // Let's fetch TTS here (awaiting 500ms is acceptable delay for stream).
                // Then fire-and-forget playback to Global Sink (which queues automatically!)
                // Rodio sink queues automatically if we append.
                
                match generate_tts_openai(client.clone(), openai_key.clone(), sentence).await {
                    Ok(audio) => {
                        queue_audio_to_global(audio, &stream_handle);
                    },
                    Err(e) => eprintln!("\n❌ TTS Gen Error: {}", e),
                }
            }
        }
    }
    
    // Process remaining buffer
    if buffer.trim().len() > 0 {
        match generate_tts_openai(client.clone(), openai_key.clone(), buffer).await {
            Ok(audio) => queue_audio_to_global(audio, &stream_handle),
            Err(_) => {},
        }
    }
    
    println!("\n✅ Done.");
    Ok(())
}

fn queue_audio_to_global(data: Vec<u8>, stream_handle: &rodio::OutputStreamHandle) {
    let cursor = Cursor::new(data);
    if let Ok(decoder) = Decoder::new(cursor) {
        let mut sink_lock = GLOBAL_SINK.lock().unwrap();
        // Create sink if none (shouldn't happen if init correct)
        if sink_lock.is_none() {
            *sink_lock = Some(Sink::try_new(stream_handle).unwrap());
        }
        
        if let Some(sink) = sink_lock.as_ref() {
            sink.append(decoder);
            // sink.play() is implied if not paused.
            // Just appending queues it.
        }
    }
}

fn find_sentence_end(text: &str) -> Option<usize> {
    // Simple heuristic: . ! ? followed by space or end
    // Iterate chars
    let chars: Vec<(usize, char)> = text.char_indices().collect();
    for i in 0..chars.len() {
        let (_, c) = chars[i];
        if ".!?".contains(c) {
            // Check if next is space or end
            if i + 1 >= chars.len() || chars[i+1].1.is_whitespace() {
                return Some(chars[i].0);
            }
        }
    }
    None
}

// ... (Keep existing helpers: transcribe_whisper, generate_tts_openai, record_while_pressed, samples_to_wav, resample) ...

fn record_while_pressed(device: &cpal::Device, device_state: &DeviceState) -> Result<Vec<f32>, Box<dyn std::error::Error + Send + Sync>> {
    let config: cpal::StreamConfig = device.default_input_config()?.into();
    let (tx, rx) = std_mpsc::channel(); // Use std mpsc for cpal callback
    let stream_sample_rate = config.sample_rate.0;
    let channels = config.channels as usize;
    
    let stream = device.build_input_stream(
        &config,
        move |data: &[f32], _: &cpal::InputCallbackInfo| {
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
    while device_state.get_keys().contains(&Keycode::Space) {
        while let Ok(sample) = rx.try_recv() { buffer.push(sample); }
        std::thread::sleep(Duration::from_millis(10));
    }
    while let Ok(sample) = rx.try_recv() { buffer.push(sample); }
    
    Ok(resample(&buffer, stream_sample_rate, SAMPLE_RATE))
}

async fn transcribe_whisper(client: &reqwest::Client, key: &str, wav_data: Vec<u8>) -> Result<String, Box<dyn std::error::Error + Send + Sync>> {
    use reqwest::multipart;
    let part = multipart::Part::bytes(wav_data).file_name("audio.wav").mime_str("audio/wav").unwrap();
    let form = multipart::Form::new().part("file", part).text("model", "whisper-1");
    let res = client.post("https://api.openai.com/v1/audio/transcriptions").bearer_auth(key).multipart(form).send().await?;
    if !res.status().is_success() { return Err(format!("API Error: {}", res.text().await?).into()); }
    let json: serde_json::Value = res.json().await?;
    Ok(json["text"].as_str().unwrap_or("").to_string())
}

async fn generate_tts_openai(client: reqwest::Client, key: String, text: String) -> Result<Vec<u8>, Box<dyn std::error::Error + Send + Sync>> {
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
