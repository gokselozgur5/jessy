// src/sherpa/stt.rs
use std::ffi::CString;
use std::ptr;
use super::bindings::*;

pub struct SherpaRecognizer {
    recognizer: *const SherpaOnnxOnlineRecognizer,
    stream: *const SherpaOnnxOnlineStream,
}

unsafe impl Send for SherpaRecognizer {}
unsafe impl Sync for SherpaRecognizer {}

impl SherpaRecognizer {
    pub fn new() -> Option<Self> {
        unsafe {
            let mut config: SherpaOnnxOnlineRecognizerConfig = std::mem::zeroed();
            
            let base_path = std::env::current_dir().ok()?.join("native/sherpa/models/stt");
            let base_path_str = base_path.to_string_lossy();
            
            let encoder_path = CString::new(format!("{}/encoder-epoch-99-avg-1.onnx", base_path_str)).ok()?;
            let decoder_path = CString::new(format!("{}/decoder-epoch-99-avg-1.onnx", base_path_str)).ok()?;
            let joiner_path = CString::new(format!("{}/joiner-epoch-99-avg-1.onnx", base_path_str)).ok()?;
            let tokens_path = CString::new(format!("{}/tokens.txt", base_path_str)).ok()?;
            let decoding_method = CString::new("greedy_search").ok()?;
            let provider = CString::new("cpu").ok()?;
            let model_type = CString::new("zipformer").ok()?;

            config.model_config.transducer.encoder = encoder_path.as_ptr();
            config.model_config.transducer.decoder = decoder_path.as_ptr();
            config.model_config.transducer.joiner = joiner_path.as_ptr();
            config.model_config.tokens = tokens_path.as_ptr();
            config.model_config.num_threads = 1;
            config.model_config.debug = 0;
            config.model_config.provider = provider.as_ptr();
            config.model_config.model_type = model_type.as_ptr();
            config.decoding_method = decoding_method.as_ptr();
            config.max_active_paths = 4;
            config.feat_config.sample_rate = 16000;
            config.feat_config.feature_dim = 80;
            
            println!("Sherpa STT: Initializing...");
            let recognizer = SherpaOnnxCreateOnlineRecognizer(&config);
            
            if recognizer.is_null() {
                println!("Sherpa STT: Failed to create recognizer!");
                return None;
            }
            
            let stream = SherpaOnnxCreateOnlineStream(recognizer);
            if stream.is_null() {
                println!("Sherpa STT: Failed to create stream!");
                SherpaOnnxDestroyOnlineRecognizer(recognizer);
                return None;
            }
            
            println!("Sherpa STT: Ready!");
            Some(SherpaRecognizer { recognizer, stream })
        }
    }

    pub fn decode(&self, samples: &[f32]) -> Option<String> {
        unsafe {
            SherpaOnnxOnlineStreamAcceptWaveform(self.stream, 16000, samples.as_ptr(), samples.len() as i32);
            while SherpaOnnxIsOnlineStreamReady(self.recognizer, self.stream) == 1 {
                SherpaOnnxDecodeOnlineStream(self.recognizer, self.stream);
            }
            let result_ptr = SherpaOnnxGetOnlineStreamResult(self.recognizer, self.stream);
            if result_ptr.is_null() { return None; }
            
            let text_ptr = (*result_ptr).text;
            let result = if !text_ptr.is_null() {
                std::ffi::CStr::from_ptr(text_ptr).to_string_lossy().into_owned()
            } else { String::new() };
            
            SherpaOnnxDestroyOnlineRecognizerResult(result_ptr);
            
            if result.is_empty() { None } else { Some(result) }
        }
    }
}

impl Drop for SherpaRecognizer {
    fn drop(&mut self) {
        unsafe {
            if !self.stream.is_null() { SherpaOnnxDestroyOnlineStream(self.stream); }
            if !self.recognizer.is_null() { SherpaOnnxDestroyOnlineRecognizer(self.recognizer); }
        }
    }
}
