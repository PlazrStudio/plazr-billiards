//! Simulation state management and configuration.
//!
//! Coordinates the [`crate::table::Table`] and collection of [`crate::ball::Ball`] entities.
//! Time-stepping, numerical integration, friction, spin transitions, and collision loops
//! are intentionally omitted in Phase 1.

use crate::ball::Ball;
use crate::math::Scalar;
use crate::table::Table;

/// Configuration parameters controlling simulation stepping and tolerances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationConfig {
    /// Fixed time step duration in seconds.
    pub time_step: Scalar,
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            time_step: 1.0 / 240.0,
        }
    }
}

/// Top-level container holding the complete state of a billiards simulation instance.
#[derive(Debug, Clone, PartialEq)]
pub struct SimulationWorld {
    /// Configuration governing the simulation instance.
    pub config: SimulationConfig,
    /// Table geometry and surface specifications.
    pub table: Table,
    /// Active balls currently present in the simulation.
    pub balls: Vec<Ball>,
}

impl SimulationWorld {
    /// Creates a new simulation world with the given configuration, table, and initial balls.
    #[must_use]
    pub const fn new(config: SimulationConfig, table: Table, balls: Vec<Ball>) -> Self {
        Self {
            config,
            table,
            balls,
        }
    }
}
