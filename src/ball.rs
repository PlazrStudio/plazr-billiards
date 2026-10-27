//! Billiard ball representation, kinematic state management, and motion integration.
//!
//! Defines the physical attributes (`BallSpecs`), activity state (`BallState`), and kinematic
//! state (`Ball`) of an individual billiard ball, including fixed-timestep linear motion
//! integration, configurable deceleration, shot impulse application, and stopping thresholds.

use crate::math::{sanitize_scalar, Scalar, Vec2, Vec3, EPSILON};

/// Unique identifier for a ball within a simulation instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct BallId(pub u8);

/// Physical material and dimensional properties of a billiard ball.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallSpecs {
    /// Radius of the ball in meters.
    pub radius: Scalar,
    /// Mass of the ball in kilograms.
    pub mass: Scalar,
}

impl BallSpecs {
    /// Standard WPA regulation pool ball radius (`0.028575` m, i.e., `2.25` inches diameter).
    pub const STANDARD_POOL_RADIUS: Scalar = 0.028_575;

    /// Standard WPA regulation pool ball mass (`0.170` kg, i.e., `6` oz).
    pub const STANDARD_POOL_MASS: Scalar = 0.170;

    /// Creates a new [`BallSpecs`] instance with the specified radius and mass.
    #[must_use]
    pub const fn new(radius: Scalar, mass: Scalar) -> Self {
        Self { radius, mass }
    }

    /// Returns standard regulation 9-foot pool ball specifications (`radius = 0.028575 m`, `mass = 0.170 kg`).
    #[must_use]
    pub const fn standard_pool() -> Self {
        Self {
            radius: Self::STANDARD_POOL_RADIUS,
            mass: Self::STANDARD_POOL_MASS,
        }
    }

    /// Returns `true` if both `radius` and `mass` are strictly positive finite numbers.
    #[must_use]
    #[inline]
    pub fn is_valid(self) -> bool {
        self.radius.is_finite()
            && self.mass.is_finite()
            && self.radius > EPSILON
            && self.mass > EPSILON
    }

    /// Computes the inverse mass (`1.0 / mass`) or returns `0.0` if the mass is invalid.
    #[must_use]
    #[inline]
    pub fn inverse_mass(self) -> Scalar {
        if self.is_valid() {
            1.0 / self.mass
        } else {
            0.0
        }
    }
}

impl Default for BallSpecs {
    fn default() -> Self {
        Self::standard_pool()
    }
}

/// Lifecycle and motion state of a ball within the simulation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BallState {
    /// The ball is on the playing surface and stationary.
    #[default]
    Stationary,
    /// The ball is on the playing surface and in motion.
    Moving,
    /// The ball has entered a pocket and is no longer active on the playing surface.
    Pocketed {
        /// Logical index of the pocket that captured the ball.
        pocket_index: u8,
    },
}

impl BallState {
    /// Returns `true` if the ball is still on the table bed (`Stationary` or `Moving`).
    #[must_use]
    #[inline]
    pub const fn is_on_table(self) -> bool {
        !matches!(self, Self::Pocketed { .. })
    }

    /// Returns `true` if the ball is currently moving on the table.
    #[must_use]
    #[inline]
    pub const fn is_moving(self) -> bool {
        matches!(self, Self::Moving)
    }

    /// Returns `true` if the ball has been pocketed.
    #[must_use]
    #[inline]
    pub const fn is_pocketed(self) -> bool {
        matches!(self, Self::Pocketed { .. })
    }
}

/// Kinematic state of a single billiard ball on the table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ball {
    /// Logical identifier of the ball (e.g., `0` for cue ball).
    pub id: BallId,
    /// Physical specifications (radius and mass).
    pub specs: BallSpecs,
    /// Planar center position on the table surface in meters.
    pub position: Vec2,
    /// Linear velocity vector in meters per second.
    pub velocity: Vec3,
    /// Angular velocity vector (spin) in radians per second.
    pub angular_velocity: Vec3,
    /// Current operational state of the ball inside the simulation.
    pub state: BallState,
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
            state: BallState::Stationary,
        }
    }

    /// Returns `true` if the ball is currently on the table surface (not pocketed).
    #[must_use]
    #[inline]
    pub const fn is_on_table(&self) -> bool {
        self.state.is_on_table()
    }

    /// Returns `true` if the ball is marked as [`BallState::Moving`].
    #[must_use]
    #[inline]
    pub const fn is_moving(&self) -> bool {
        self.state.is_moving()
    }

    /// Returns `true` if the ball is marked as [`BallState::Pocketed`].
    #[must_use]
    #[inline]
    pub const fn is_pocketed(&self) -> bool {
        self.state.is_pocketed()
    }

    /// Returns the planar `(x, y)` linear velocity of the ball.
    #[must_use]
    #[inline]
    pub const fn planar_velocity(&self) -> Vec2 {
        self.velocity.xy()
    }

    /// Sets the planar `(x, y)` linear velocity while zeroing any vertical `z` velocity component,
    /// and updates the ball state to [`BallState::Moving`] or [`BallState::Stationary`].
    pub fn set_planar_velocity(&mut self, planar_velocity: Vec2, stop_speed_threshold: Scalar) {
        if !self.is_on_table() {
            self.velocity = Vec3::ZERO;
            self.angular_velocity = Vec3::ZERO;
            return;
        }
        let clean = planar_velocity.sanitized();
        let threshold = sanitize_scalar(stop_speed_threshold, 0.0).max(0.0);
        if clean.length() <= threshold {
            self.stop();
        } else {
            self.velocity = Vec3::from_vec2(clean, 0.0);
            self.state = BallState::Moving;
        }
    }

    /// Computes the planar speed (magnitude of `velocity.xy()`) in meters per second.
    #[must_use]
    #[inline]
    pub fn speed(&self) -> Scalar {
        self.velocity.xy().length()
    }

    /// Computes the planar kinetic energy `0.5 * m * v²` in joules.
    #[must_use]
    #[inline]
    pub fn kinetic_energy(&self) -> Scalar {
        if !self.is_on_table() || !self.specs.is_valid() {
            return 0.0;
        }
        0.5 * self.specs.mass * self.velocity.xy().length_squared()
    }

    /// Immediately brings the ball to rest on the table (unless already pocketed).
    pub fn stop(&mut self) {
        self.velocity = Vec3::ZERO;
        self.angular_velocity = Vec3::ZERO;
        if self.is_on_table() {
            self.state = BallState::Stationary;
        }
    }

    /// Marks the ball as pocketed in the specified `pocket_index` and zeroes its velocities.
    pub fn mark_pocketed(&mut self, pocket_index: u8) {
        self.velocity = Vec3::ZERO;
        self.angular_velocity = Vec3::ZERO;
        self.state = BallState::Pocketed { pocket_index };
    }

    /// Sanitizes position and velocities against `NaN` or `Infinity` and synchronizes [`Ball::state`]
    /// against `stop_speed_threshold`.
    pub fn sanitize_and_sync_state(&mut self, stop_speed_threshold: Scalar) {
        self.position = self.position.sanitized();
        self.velocity = self.velocity.sanitized();
        self.angular_velocity = self.angular_velocity.sanitized();

        if !self.is_on_table() {
            self.velocity = Vec3::ZERO;
            self.angular_velocity = Vec3::ZERO;
            return;
        }

        let threshold = sanitize_scalar(stop_speed_threshold, 0.0).max(0.0);
        let planar_speed = self.speed();
        let angular_speed = self.angular_velocity.length();

        if planar_speed <= threshold && angular_speed <= threshold {
            self.stop();
        } else {
            if planar_speed <= threshold {
                self.velocity = Vec3::ZERO;
            } else {
                // Keep planar motion strictly on the table bed (z = 0).
                self.velocity.z = 0.0;
            }
            if angular_speed <= threshold {
                self.angular_velocity = Vec3::ZERO;
            }
            self.state = if self.velocity.xy().length() > threshold
                || self.angular_velocity.length() > threshold
            {
                BallState::Moving
            } else {
                BallState::Stationary
            };
        }
    }

    /// Applies a planar cue strike or impulse in the given `direction` with magnitude `force_or_impulse`
    /// (interpreted as linear momentum impulse `N·s`, producing `Δv = impulse / mass`).
    ///
    /// Returns `true` if the strike was valid and applied, or `false` if the ball is pocketed,
    /// has invalid mass, or the direction/impulse parameters are invalid.
    pub fn apply_impulse(
        &mut self,
        direction: Vec2,
        impulse: Scalar,
        max_speed: Scalar,
        stop_speed_threshold: Scalar,
    ) -> bool {
        if !self.is_on_table() || !self.specs.is_valid() {
            return false;
        }
        if !impulse.is_finite() || impulse <= EPSILON {
            return false;
        }
        let Some(unit_dir) = direction.try_normalize() else {
            return false;
        };

        let delta_v = unit_dir.scale(impulse * self.specs.inverse_mass());
        let mut new_planar_v = (self.velocity.xy() + delta_v).sanitized();
        if max_speed.is_finite() && max_speed > EPSILON {
            new_planar_v = new_planar_v.clamp_length(max_speed);
        }
        self.set_planar_velocity(new_planar_v, stop_speed_threshold);
        self.is_moving()
    }

    /// Advances the ball's planar position and updates its velocity over `dt` seconds
    /// using constant linear deceleration (`linear_drag` in `m/s²`) and angular deceleration
    /// (`angular_drag` in `rad/s²`).
    ///
    /// When the ball's speed drops at or below `stop_speed_threshold`, it comes to a complete stop.
    pub fn integrate_motion(
        &mut self,
        dt: Scalar,
        linear_drag: Scalar,
        angular_drag: Scalar,
        stop_speed_threshold: Scalar,
    ) {
        if !self.is_on_table() || !dt.is_finite() || dt <= 0.0 {
            return;
        }

        self.sanitize_and_sync_state(stop_speed_threshold);
        if !self.is_moving() {
            return;
        }

        let decel = sanitize_scalar(linear_drag, 0.0).max(0.0);
        let stop_threshold = sanitize_scalar(stop_speed_threshold, 0.0).max(0.0);

        let v0 = self.velocity.xy();
        let speed0 = v0.length();

        if speed0 > stop_threshold {
            let dir = v0.normalize_or_zero();
            // Time required to decelerate to 0 under constant deceleration
            let decel_delta = decel * dt;
            let new_speed = (speed0 - decel_delta).max(0.0);

            // Average speed over the interval for second-order accurate constant-deceleration displacement
            let effective_dt = if decel > EPSILON && speed0 < decel_delta {
                speed0 / decel
            } else {
                dt
            };
            let avg_speed = 0.5 * (speed0 + new_speed);
            self.position = (self.position + dir.scale(avg_speed * effective_dt)).sanitized();

            if new_speed <= stop_threshold {
                self.velocity = Vec3::ZERO;
            } else {
                self.velocity = Vec3::from_vec2(dir.scale(new_speed), 0.0);
            }
        } else {
            self.velocity = Vec3::ZERO;
        }

        // Apply angular deceleration if spin is present
        let ang_speed0 = self.angular_velocity.length();
        let ang_decel = sanitize_scalar(angular_drag, 0.0).max(0.0);
        if ang_speed0 > stop_threshold {
            let new_ang_speed = (ang_speed0 - ang_decel * dt).max(0.0);
            if new_ang_speed <= stop_threshold {
                self.angular_velocity = Vec3::ZERO;
            } else {
                self.angular_velocity = self
                    .angular_velocity
                    .normalize_or_zero()
                    .scale(new_ang_speed);
            }
        } else {
            self.angular_velocity = Vec3::ZERO;
        }

        self.sanitize_and_sync_state(stop_threshold);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ball_creation_and_impulse() {
        let mut ball =
            Ball::new_stationary(BallId(0), BallSpecs::standard_pool(), Vec2::new(0.5, 0.5));
        assert!(ball.is_on_table());
        assert!(!ball.is_moving());
        assert_eq!(ball.speed(), 0.0);

        // Apply impulse along +X: impulse = 0.34 N*s on 0.17 kg ball => v = 2.0 m/s
        let applied = ball.apply_impulse(Vec2::new(1.0, 0.0), 0.34, 20.0, 1e-3);
        assert!(applied);
        assert!(ball.is_moving());
        assert!((ball.speed() - 2.0).abs() < 1e-9);
    }

    #[test]
    fn ball_motion_integration_and_stopping() {
        let mut ball = Ball::new_stationary(BallId(1), BallSpecs::standard_pool(), Vec2::ZERO);
        ball.set_planar_velocity(Vec2::new(1.0, 0.0), 0.01);

        // Decelerate at 0.5 m/s^2 for 1.0 second in 100 steps of 0.01s
        for _ in 0..100 {
            ball.integrate_motion(0.01, 0.5, 0.5, 0.01);
        }
        assert!(ball.is_moving());
        assert!((ball.speed() - 0.5).abs() < 1e-6);
        assert!((ball.position.x - 0.75).abs() < 1e-4);

        // Continue another 1.0 second until ball stops completely
        for _ in 0..100 {
            ball.integrate_motion(0.01, 0.5, 0.5, 0.01);
        }
        assert!(!ball.is_moving());
        assert_eq!(ball.state, BallState::Stationary);
        assert_eq!(ball.speed(), 0.0);
        assert!((ball.position.x - 1.0).abs() < 1e-2);
    }
}
