//! WebAssembly library entrypoint for the Marketplace Leptos application.
//!
//! The binary target is retained for native tooling. Trunk builds this library for
//! WASM and invokes the start function when the module is instantiated.

mod app;

#[cfg_attr(target_arch = "wasm32", wasm_bindgen::prelude::wasm_bindgen(start))]
pub fn start() {
    app::start();
}
