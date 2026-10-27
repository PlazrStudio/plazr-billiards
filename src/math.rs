//! Mathematical types and constants for the billiards physics engine.
//!
//! This module provides foundational 2D and 3D vector representations used across
//! [`crate::ball`], [`crate::table`], [`crate::collision`], and [`crate::simulation`].
//! Arithmetic operations and geometric solvers are scheduled for Phase 2.

/// Standard floating-point scalar type used throughout the physics engine.
pub type Scalar = f64;

/// A two-dimensional vector representing planar table coordinates or planar velocities.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    /// Coordinate or component along the longitudinal table axis (X).
    pub x: Scalar,
    /// Coordinate or component along the lateral table axis (Y).
    pub y: Scalar,
}

impl Vec2 {
    /// Zero vector constant `(0.0, 0.0)`.
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    /// Creates a new 2D vector from `x` and `y` components.
    #[must_use]
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        Self { x, y }
    }
}

/// A three-dimensional vector representing spatial coordinates, linear velocity, or angular velocity.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec3 {
    /// Component along the X-axis.
    pub x: Scalar,
    /// Component along the Y-axis.
    pub y: Scalar,
    /// Component along the Z-axis (vertical axis perpendicular to the table bed).
    pub z: Scalar,
}

impl Vec3 {
    /// Zero vector constant `(0.0, 0.0, 0.0)`.
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    /// Creates a new 3D vector from `x`, `y`, and `z` components.
    #[must_use]
    pub const fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self { x, y, z }
    }
}
