extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    let manifest_dir = env::var("CARGO_MANIFEST_DIR").expect("CARGO_MANIFEST_DIR not set");
    let native_dir = PathBuf::from(manifest_dir).join("native");
    let sherpa_dir = native_dir.join("sherpa");
    let lib_dir = sherpa_dir.join("lib");
    let include_dir = sherpa_dir.join("include");
    let include_header = include_dir.join("c-api.h");

    println!("cargo:rustc-link-search={}", lib_dir.display());
    println!("cargo:rustc-link-lib=sherpa-onnx-c-api");
    println!("cargo:rustc-link-lib=onnxruntime.1.17.1");

    println!("cargo:rerun-if-changed={}", include_header.display());

    let bindings = bindgen::Builder::default()
        .header(include_header.to_string_lossy())
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("sherpa_bindings.rs"))
        .expect("Couldn't write bindings!");
}
