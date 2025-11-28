#[allow(non_snake_case)]
#[allow(non_camel_case_types)]
#[allow(non_upper_case_globals)]
#[allow(improper_ctypes)]
#[allow(dead_code)]
pub mod bindings {
    include!(concat!(env!("OUT_DIR"), "/sherpa_bindings.rs"));
}

pub mod stt;
pub mod tts;

pub use stt::SherpaRecognizer;
pub use tts::SherpaTts;
