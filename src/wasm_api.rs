//! Future WebAssembly (WASM) interoperability boundary.
//!
//! This module serves as the dedicated architectural boundary between the pure-Rust physics
//! core and future WebAssembly / JavaScript / TypeScript bindings.
//!
//! ## Design Rules
//! - Core physics modules (`math`, `ball`, `table`, `collision`, `simulation`) must never
//!   depend on this module or on any browser/JS runtime types.
//! - Operational WebAssembly exports and `wasm-bindgen` adapters will be implemented in
//!   a dedicated later phase once the core physics engine is complete.

/// Version metadata descriptor exposed for future host environment inspection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WasmApiMetadata {
    /// Semantic version string of the `plazr-billiards` library.
    pub version: &'static str,
}

impl WasmApiMetadata {
    /// Returns the compile-time metadata descriptor for the crate.
    #[must_use]
    pub const fn current() -> Self {
        Self {
            version: env!("CARGO_PKG_VERSION"),
        }
    }
}
