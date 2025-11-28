// src/sherpa/tts.rs
use std::ffi::CString;
use std::ptr;
use super::bindings::*;

pub struct SherpaTts {
    tts: *const SherpaOnnxOfflineTts,
}

unsafe impl Send for SherpaTts {}
unsafe impl Sync for SherpaTts {}

impl SherpaTts {
    pub fn new() -> Option<Self> {
        unsafe {
            let mut config: SherpaOnnxOfflineTtsConfig = std::mem::zeroed();
            let base_path = std::env::current_dir().ok()?.join("native/sherpa/models/tts");
            let base_path_str = base_path.to_string_lossy();
            
            let model = CString::new(format!("{}/en_US-amy-low.onnx", base_path_str)).ok()?;
            let tokens = CString::new(format!("{}/tokens.txt", base_path_str)).ok()?; 
            let data_dir = CString::new(format!("{}/espeak-ng-data", base_path_str)).ok()?;
            let provider = CString::new("cpu").ok()?;

            config.model.vits.model = model.as_ptr();
            config.model.vits.tokens = tokens.as_ptr();
            config.model.vits.data_dir = data_dir.as_ptr();
            config.model.provider = provider.as_ptr();
            config.model.num_threads = 1;
            config.model.debug = 0;
            
            println!("Sherpa TTS: Initializing...");
            let tts = SherpaOnnxCreateOfflineTts(&config);
            
            if tts.is_null() {
                println!("Sherpa TTS: Failed to create TTS engine!");
                return None;
            }
            
            println!("Sherpa TTS: Ready!");
            Some(SherpaTts { tts })
        }
    }

    pub fn generate(&self, text: &str) -> Option<Vec<f32>> {
        unsafe {
            let text_c = CString::new(text).ok()?;
            // Speed 0.9 (slower) might sound less "screechy"
            let audio = SherpaOnnxOfflineTtsGenerate(self.tts, text_c.as_ptr(), 0, 0.9);
            
            if audio.is_null() {
                return None;
            }
            
            let samples_ptr = (*audio).samples;
            let n_samples = (*audio).n;
            
            let mut samples = Vec::with_capacity(n_samples as usize);
            std::ptr::copy_nonoverlapping(samples_ptr, samples.as_mut_ptr(), n_samples as usize);
            samples.set_len(n_samples as usize);
            
            SherpaOnnxDestroyOfflineTtsGeneratedAudio(audio);
            
            Some(samples)
        }
    }
}

impl Drop for SherpaTts {
    fn drop(&mut self) {
        unsafe {
            if !self.tts.is_null() { SherpaOnnxDestroyOfflineTts(self.tts); }
        }
    }
}
