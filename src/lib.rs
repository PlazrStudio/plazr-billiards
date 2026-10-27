//! # Plazr Billiards
//!
//! `plazr-billiards` is a standalone, open-source billiards physics simulation library
//! developed under the Plazr platform.
//!
//! ## Architectural Overview (Phase 1 Foundation)
//! The crate is organized into strictly decoupled modules separating mathematical primitives,
//! physical state representations, collision detection, simulation stepping, and future
//! WebAssembly bindings:
//!
//! - [`math`]: Foundational vector types and scalar constants for physics calculations.
//! - [`ball`]: Billiard ball state representation, physical properties, and identifiers.
//! - [`table`]: Table geometry, playing surface dimensions, cushions, and pocket definitions.
//! - [`collision`]: Discrete and continuous collision event structures and detection contracts.
//! - [`simulation`]: Top-level simulation world state and configuration.
//! - [`wasm_api`]: Boundary module reserved for future WebAssembly / JavaScript / TypeScript interoperability.

#![forbid(unsafe_code)]
#![warn(missing_docs, rust_2018_idioms)]

pub mod ball;
pub mod collision;
pub mod math;
pub mod simulation;
pub mod table;
pub mod wasm_api;
