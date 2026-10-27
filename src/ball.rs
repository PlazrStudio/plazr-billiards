//! Billiard ball representation and physical attributes.
//!
//! Defines the data structures holding the physical parameters and kinematic state
//! of an individual billiard ball without embedding motion integration logic (reserved for Phase 2).

use crate::math::{Scalar, Vec2, Vec3};

/// Unique identifier for a ball within a simulation instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BallId(pub u8);

/// Physical material and dimensional properties of a billiard ball.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallSpecs {
    /// Radius of the ball in meters.
    pub radius: Scalar,
    /// Mass of the ball in kilograms.
    pub mass: Scalar,
}

/// Kinematic state of a single billiard ball on the table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ball {
    /// Logical identifier of the ball (e.g., `0` for cue ball).
    pub id: BallId,
    /// Physical specifications (radius and mass).
    pub specs: BallSpecs,
    /// Planar center position on the table surface.
    pub position: Vec2,
    /// Linear velocity vector.
    pub velocity: Vec3,
    /// Angular velocity vector (spin) in radians per second.
    pub angular_velocity: Vec3,
}

impl Ball {
    /// Creates a stationary ball at the given planar position with the specified physical specs.
    #[must_use]
    pub const fn new_stationary(id: BallId, specs: BallSpecs, position: Vec2) -> Self {
        Self {
            id,
            specs,
            position,
            velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
        }
    }
}
