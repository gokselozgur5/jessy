//! Foreign Function Interface (FFI) Layer
//!
//! This module provides C-compatible functions for interfacing with the
//! Rust consciousness core from other languages (primarily Go via CGO).
//!
//! # Safety
//!
//! All FFI functions are marked as `unsafe` or use `extern "C"` calling convention.
//! Callers must ensure:
//! - Valid pointers (non-null where required)
//! - Proper memory management (caller frees returned strings)
//! - Thread-safe usage (functions are thread-safe)
//!
//! # Memory Management
//!
//! - Rust allocates strings returned to caller
//! - Caller must call `consciousness_free_string()` to free
//! - Rust owns all internal state
//! - No shared mutable state across FFI boundary

pub mod types;
pub mod functions;
pub mod error;

pub use types::*;
pub use functions::*;
pub use error::*;
