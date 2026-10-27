//! # Plazr Billiards
//!
//! `plazr-billiards` is a standalone, open-source billiards physics simulation library
//! developed by **Plazr Studio**.
//!
//! ## Architectural Overview
//! The crate is organized into strictly decoupled modules separating mathematical primitives,
//! physical state representations, collision detection and impulse resolution, fixed-timestep
//! simulation stepping, and future WebAssembly bindings:
//!
//! - [`math`]: 2D/3D vector arithmetic, dot/cross products, distances, angles, reflections,
//!   safe normalization, and numerical sanitization against `NaN` and `Infinity`.
//! - [`ball`]: Billiard ball state representation (`Ball`, `BallId`, `BallSpecs`, `BallState`),
//!   motion integration, configurable deceleration, and shot impulse application.
//! - [`table`]: Table geometry (`Table`, `TableSpecs`), cushion normal definitions, and
//!   pocket capture zones (`Pocket`).
//! - [`collision`]: Ball-to-ball collision detection, mass- and restitution-aware impulse
//!   response, overlap separation, cushion rebounds, pocket drop detection, and analytic TOI queries.
//! - [`simulation`]: Top-level [`simulation::SimulationWorld`] coordinator and [`simulation::SimulationConfig`]
//!   driving fixed-timestep updates with adaptive sub-stepping.
//! - [`wasm_api`]: Boundary module reserved for future WebAssembly / JavaScript / TypeScript interoperability.
//!
//! ## Quick Example
//! ```rust
//! use plazr_billiards::ball::{Ball, BallId, BallSpecs};
//! use plazr_billiards::math::Vec2;
//! use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
//! use plazr_billiards::table::Table;
//!
//! let config = SimulationConfig::default();
//! let table = Table::standard_9ft_six_pocket();
//! let specs = BallSpecs::standard_pool();
//!
//! let cue_ball = Ball::new_stationary(BallId(0), specs, Vec2::new(0.65, 0.635));
//! let object_ball = Ball::new_stationary(BallId(1), specs, Vec2::new(1.25, 0.635));
//!
//! let mut world = SimulationWorld::new(config, table, vec![cue_ball, object_ball]);
//! world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 3.5);
//!
//! let (steps, events) = world.step_until_settled(2_000);
//! assert!(steps > 0);
//! assert!(!events.is_empty());
//! assert!(world.is_settled());
//! ```

#![forbid(unsafe_code)]
#![warn(missing_docs, rust_2018_idioms)]

pub mod ball;
pub mod collision;
pub mod math;
pub mod simulation;
pub mod table;
pub mod wasm_api;
