//! Mathematical types, constants, and vector operations for the billiards physics engine.
//!
//! This module provides foundational 2D and 3D vector representations (`Vec2`, `Vec3`),
//! standard operator implementations, geometric utilities (length, distance, direction,
//! angles, and reflections), and safe numerical validation helpers to prevent `NaN` or
//! infinite values from corrupting simulation state.

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// Standard floating-point scalar type used throughout the physics engine.
pub type Scalar = f64;

/// Default numerical tolerance for near-zero length, normalization, and parallel-vector checks.
pub const EPSILON: Scalar = 1e-9;

/// Checks whether a scalar value is finite (neither `NaN` nor infinite).
#[must_use]
#[inline]
pub fn is_valid_scalar(value: Scalar) -> bool {
    value.is_finite()
}

/// Replaces a non-finite scalar (`NaN` or `±Infinity`) with `fallback`.
#[must_use]
#[inline]
pub fn sanitize_scalar(value: Scalar, fallback: Scalar) -> Scalar {
    if value.is_finite() {
        value
    } else if fallback.is_finite() {
        fallback
    } else {
        0.0
    }
}

/// Clamps a scalar to `[min, max]` while returning `fallback` if `value` is non-finite.
#[must_use]
#[inline]
pub fn clamp_scalar(value: Scalar, min: Scalar, max: Scalar, fallback: Scalar) -> Scalar {
    if !value.is_finite() || !min.is_finite() || !max.is_finite() || min > max {
        return sanitize_scalar(fallback, 0.0);
    }
    value.clamp(min, max)
}

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

    /// Unit vector along the positive X-axis `(1.0, 0.0)`.
    pub const UNIT_X: Self = Self { x: 1.0, y: 0.0 };

    /// Unit vector along the positive Y-axis `(0.0, 1.0)`.
    pub const UNIT_Y: Self = Self { x: 0.0, y: 1.0 };

    /// Creates a new 2D vector from `x` and `y` components.
    #[must_use]
    #[inline]
    pub const fn new(x: Scalar, y: Scalar) -> Self {
        Self { x, y }
    }

    /// Creates a unit 2D vector from an angle in radians measured from the positive X-axis.
    ///
    /// Returns [`Vec2::ZERO`] if `angle_rad` is not finite.
    #[must_use]
    #[inline]
    pub fn from_angle(angle_rad: Scalar) -> Self {
        if !angle_rad.is_finite() {
            return Self::ZERO;
        }
        Self {
            x: angle_rad.cos(),
            y: angle_rad.sin(),
        }
    }

    /// Expands this 2D planar vector into a 3D vector with the specified `z` component.
    #[must_use]
    #[inline]
    pub const fn to_vec3(self, z: Scalar) -> Vec3 {
        Vec3::new(self.x, self.y, z)
    }

    /// Returns `true` if both `x` and `y` components are finite numbers (`!is_nan()` and `!is_infinite()`).
    #[must_use]
    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }

    /// Returns a sanitized copy of the vector, replacing the vector with [`Vec2::ZERO`]
    /// if either component is `NaN` or infinite.
    #[must_use]
    #[inline]
    pub fn sanitized(self) -> Self {
        if self.is_finite() {
            self
        } else {
            Self::ZERO
        }
    }

    /// Returns the component-wise addition of `self` and `other`.
    #[must_use]
    #[inline]
    pub fn add_vec(self, other: Self) -> Self {
        self + other
    }

    /// Returns the component-wise subtraction `self - other`.
    #[must_use]
    #[inline]
    pub fn sub_vec(self, other: Self) -> Self {
        self - other
    }

    /// Multiplies the vector by a scalar factor.
    ///
    /// If the scalar or vector is non-finite, returns [`Vec2::ZERO`].
    #[must_use]
    #[inline]
    pub fn scale(self, factor: Scalar) -> Self {
        if !self.is_finite() || !factor.is_finite() {
            return Self::ZERO;
        }
        let result = Self::new(self.x * factor, self.y * factor);
        result.sanitized()
    }

    /// Safely divides the vector by a scalar divisor.
    ///
    /// Returns `None` if `divisor` is non-finite or its absolute value is less than or equal to [`EPSILON`],
    /// or if the resulting components overflow to infinity.
    #[must_use]
    #[inline]
    pub fn checked_div(self, divisor: Scalar) -> Option<Self> {
        if !self.is_finite() || !divisor.is_finite() || divisor.abs() <= EPSILON {
            return None;
        }
        let out = Self::new(self.x / divisor, self.y / divisor);
        if out.is_finite() {
            Some(out)
        } else {
            None
        }
    }

    /// Computes the dot product `self · other`.
    ///
    /// Returns `0.0` if either input vector contains non-finite values or if the product overflows.
    #[must_use]
    #[inline]
    pub fn dot(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        sanitize_scalar(self.x * other.x + self.y * other.y, 0.0)
    }

    /// Computes the 2D scalar cross product (z-component of the 3D cross product, `x1*y2 - y1*x2`).
    ///
    /// Returns `0.0` if either input vector contains non-finite values.
    #[must_use]
    #[inline]
    pub fn cross(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        sanitize_scalar(self.x * other.y - self.y * other.x, 0.0)
    }

    /// Computes the squared Euclidean length of the vector (`x² + y²`).
    #[must_use]
    #[inline]
    pub fn length_squared(self) -> Scalar {
        self.dot(self)
    }

    /// Computes the Euclidean length (magnitude) of the vector.
    ///
    /// Uses `hypot` for resistance against intermediate overflow/underflow.
    /// Returns `0.0` if the vector is non-finite.
    #[must_use]
    #[inline]
    pub fn length(self) -> Scalar {
        if !self.is_finite() {
            return 0.0;
        }
        sanitize_scalar(self.x.hypot(self.y), 0.0)
    }

    /// Computes the squared Euclidean distance between `self` and `other`.
    #[must_use]
    #[inline]
    pub fn distance_squared(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        (other - self).length_squared()
    }

    /// Computes the Euclidean distance between `self` and `other`.
    #[must_use]
    #[inline]
    pub fn distance(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        (other - self).length()
    }

    /// Attempts to normalize the vector to unit length, returning `None` if the vector
    /// is non-finite or has length less than or equal to [`EPSILON`].
    #[must_use]
    #[inline]
    pub fn try_normalize(self) -> Option<Self> {
        let len = self.length();
        if len <= EPSILON {
            None
        } else {
            let unit = Self::new(self.x / len, self.y / len);
            if unit.is_finite() {
                Some(unit)
            } else {
                None
            }
        }
    }

    /// Normalizes the vector to unit length, returning [`Vec2::ZERO`] if its length is
    /// less than or equal to [`EPSILON`] or if its components are non-finite.
    #[must_use]
    #[inline]
    pub fn normalize_or_zero(self) -> Self {
        self.try_normalize().unwrap_or(Self::ZERO)
    }

    /// Alias for [`Vec2::normalize_or_zero`] providing safe normalization without panicking or producing `NaN`.
    #[must_use]
    #[inline]
    pub fn normalize(self) -> Self {
        self.normalize_or_zero()
    }

    /// Computes the unit direction vector pointing from `self` toward `target`.
    ///
    /// Returns [`Vec2::ZERO`] if the two points coincide within [`EPSILON`] or contain non-finite coordinates.
    #[must_use]
    #[inline]
    pub fn direction_to(self, target: Self) -> Self {
        if !self.is_finite() || !target.is_finite() {
            return Self::ZERO;
        }
        (target - self).normalize_or_zero()
    }

    /// Returns the polar angle of this vector in radians in the range `[-π, π]` relative to `+X`.
    ///
    /// Returns `0.0` if the vector is zero or non-finite.
    #[must_use]
    #[inline]
    pub fn angle(self) -> Scalar {
        if !self.is_finite() || self.length_squared() <= EPSILON * EPSILON {
            return 0.0;
        }
        self.y.atan2(self.x)
    }

    /// Computes the unsigned angle in radians `[0, π]` between `self` and `other`.
    ///
    /// Returns `None` if either vector has near-zero length or contains non-finite values.
    #[must_use]
    #[inline]
    pub fn angle_between(self, other: Self) -> Option<Scalar> {
        let u = self.try_normalize()?;
        let v = other.try_normalize()?;
        let cos_theta = u.dot(v).clamp(-1.0, 1.0);
        Some(cos_theta.acos())
    }

    /// Computes the signed angle in radians `[-π, π]` from `self` to `other`.
    ///
    /// Returns `None` if either vector has near-zero length or contains non-finite values.
    #[must_use]
    #[inline]
    pub fn signed_angle_to(self, other: Self) -> Option<Scalar> {
        let u = self.try_normalize()?;
        let v = other.try_normalize()?;
        Some(u.cross(v).atan2(u.dot(v)))
    }

    /// Rotates the vector counter-clockwise by `angle_rad` radians.
    #[must_use]
    #[inline]
    pub fn rotate(self, angle_rad: Scalar) -> Self {
        if !self.is_finite() || !angle_rad.is_finite() {
            return Self::ZERO;
        }
        let (sin_a, cos_a) = angle_rad.sin_cos();
        Self::new(
            self.x * cos_a - self.y * sin_a,
            self.x * sin_a + self.y * cos_a,
        )
        .sanitized()
    }

    /// Returns a vector perpendicular to `self` rotated 90 degrees counter-clockwise `(-y, x)`.
    #[must_use]
    #[inline]
    pub fn perp(self) -> Self {
        if !self.is_finite() {
            return Self::ZERO;
        }
        Self::new(-self.y, self.x)
    }

    /// Projects `self` onto `onto`.
    ///
    /// Returns [`Vec2::ZERO`] if `onto` has near-zero length or if either vector is non-finite.
    #[must_use]
    #[inline]
    pub fn project_onto(self, onto: Self) -> Self {
        if let Some(dir) = onto.try_normalize() {
            dir.scale(self.dot(dir))
        } else {
            Self::ZERO
        }
    }

    /// Reflects `self` (treated as an incident vector) across a surface with the given `normal`.
    ///
    /// Uses the reflection formula `v - 2 * (v · n̂) * n̂`. Automatically normalizes `normal`.
    /// If `normal` is invalid or zero-length, returns `self.sanitized()`.
    #[must_use]
    #[inline]
    pub fn reflect(self, normal: Self) -> Self {
        if !self.is_finite() {
            return Self::ZERO;
        }
        if let Some(n) = normal.try_normalize() {
            (self - n.scale(2.0 * self.dot(n))).sanitized()
        } else {
            self
        }
    }

    /// Linearly interpolates between `self` and `other` by parameter `t`.
    #[must_use]
    #[inline]
    pub fn lerp(self, other: Self, t: Scalar) -> Self {
        if !self.is_finite() || !other.is_finite() || !t.is_finite() {
            return Self::ZERO;
        }
        (self.scale(1.0 - t) + other.scale(t)).sanitized()
    }

    /// Checks whether `self` and `other` are component-wise equal within `tolerance`.
    #[must_use]
    #[inline]
    pub fn approx_eq(self, other: Self, tolerance: Scalar) -> bool {
        if !self.is_finite() || !other.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
            return false;
        }
        (self.x - other.x).abs() <= tolerance && (self.y - other.y).abs() <= tolerance
    }

    /// Clamps the magnitude of the vector to at most `max_length`.
    #[must_use]
    #[inline]
    pub fn clamp_length(self, max_length: Scalar) -> Self {
        if !self.is_finite() || !max_length.is_finite() || max_length <= 0.0 {
            return Self::ZERO;
        }
        let len = self.length();
        if len > max_length && len > EPSILON {
            self.scale(max_length / len)
        } else {
            self
        }
    }
}

impl Add for Vec2 {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl Sub for Vec2 {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl Mul<Scalar> for Vec2 {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Scalar) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

impl Mul<Vec2> for Scalar {
    type Output = Vec2;

    #[inline]
    fn mul(self, rhs: Vec2) -> Self::Output {
        Vec2::new(self * rhs.x, self * rhs.y)
    }
}

impl MulAssign<Scalar> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, rhs: Scalar) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl Div<Scalar> for Vec2 {
    type Output = Self;

    /// Divides the vector by `rhs`. Returns [`Vec2::ZERO`] if `rhs` is zero or non-finite
    /// to prevent `NaN` or `Infinity` propagation.
    #[inline]
    fn div(self, rhs: Scalar) -> Self::Output {
        self.checked_div(rhs).unwrap_or(Self::ZERO)
    }
}

impl DivAssign<Scalar> for Vec2 {
    #[inline]
    fn div_assign(&mut self, rhs: Scalar) {
        *self = *self / rhs;
    }
}

impl Neg for Vec2 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y)
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

    /// Unit vector along the positive X-axis `(1.0, 0.0, 0.0)`.
    pub const UNIT_X: Self = Self {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    /// Unit vector along the positive Y-axis `(0.0, 1.0, 0.0)`.
    pub const UNIT_Y: Self = Self {
        x: 0.0,
        y: 1.0,
        z: 0.0,
    };

    /// Unit vector along the positive Z-axis `(0.0, 0.0, 1.0)`.
    pub const UNIT_Z: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 1.0,
    };

    /// Creates a new 3D vector from `x`, `y`, and `z` components.
    #[must_use]
    #[inline]
    pub const fn new(x: Scalar, y: Scalar, z: Scalar) -> Self {
        Self { x, y, z }
    }

    /// Constructs a 3D vector from a planar [`Vec2`] and a `z` component.
    #[must_use]
    #[inline]
    pub const fn from_vec2(planar: Vec2, z: Scalar) -> Self {
        Self {
            x: planar.x,
            y: planar.y,
            z,
        }
    }

    /// Extracts the planar `(x, y)` projection of this 3D vector as a [`Vec2`].
    #[must_use]
    #[inline]
    pub const fn xy(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }

    /// Returns `true` if `x`, `y`, and `z` components are all finite numbers.
    #[must_use]
    #[inline]
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Returns a sanitized copy of the vector, replacing it with [`Vec3::ZERO`] if any component is non-finite.
    #[must_use]
    #[inline]
    pub fn sanitized(self) -> Self {
        if self.is_finite() {
            self
        } else {
            Self::ZERO
        }
    }

    /// Multiplies the vector by a scalar factor, returning [`Vec3::ZERO`] on non-finite input.
    #[must_use]
    #[inline]
    pub fn scale(self, factor: Scalar) -> Self {
        if !self.is_finite() || !factor.is_finite() {
            return Self::ZERO;
        }
        Self::new(self.x * factor, self.y * factor, self.z * factor).sanitized()
    }

    /// Safely divides the vector by a scalar divisor.
    ///
    /// Returns `None` if `divisor` is non-finite or its magnitude is less than or equal to [`EPSILON`].
    #[must_use]
    #[inline]
    pub fn checked_div(self, divisor: Scalar) -> Option<Self> {
        if !self.is_finite() || !divisor.is_finite() || divisor.abs() <= EPSILON {
            return None;
        }
        let out = Self::new(self.x / divisor, self.y / divisor, self.z / divisor);
        if out.is_finite() {
            Some(out)
        } else {
            None
        }
    }

    /// Computes the 3D dot product `self · other`.
    ///
    /// Returns `0.0` if either vector contains non-finite values.
    #[must_use]
    #[inline]
    pub fn dot(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        sanitize_scalar(self.x * other.x + self.y * other.y + self.z * other.z, 0.0)
    }

    /// Computes the 3D right-handed cross product `self × other`.
    ///
    /// Returns [`Vec3::ZERO`] if either vector contains non-finite values.
    #[must_use]
    #[inline]
    pub fn cross(self, other: Self) -> Self {
        if !self.is_finite() || !other.is_finite() {
            return Self::ZERO;
        }
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
        .sanitized()
    }

    /// Computes the squared Euclidean length (`x² + y² + z²`).
    #[must_use]
    #[inline]
    pub fn length_squared(self) -> Scalar {
        self.dot(self)
    }

    /// Computes the Euclidean length (magnitude) of the 3D vector.
    ///
    /// Returns `0.0` if any component is non-finite.
    #[must_use]
    #[inline]
    pub fn length(self) -> Scalar {
        if !self.is_finite() {
            return 0.0;
        }
        sanitize_scalar(self.x.hypot(self.y).hypot(self.z), 0.0)
    }

    /// Computes the squared Euclidean distance between `self` and `other`.
    #[must_use]
    #[inline]
    pub fn distance_squared(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        (other - self).length_squared()
    }

    /// Computes the Euclidean distance between `self` and `other`.
    #[must_use]
    #[inline]
    pub fn distance(self, other: Self) -> Scalar {
        if !self.is_finite() || !other.is_finite() {
            return 0.0;
        }
        (other - self).length()
    }

    /// Attempts to normalize the vector to unit length, returning `None` if the vector
    /// is non-finite or has length less than or equal to [`EPSILON`].
    #[must_use]
    #[inline]
    pub fn try_normalize(self) -> Option<Self> {
        let len = self.length();
        if len <= EPSILON {
            None
        } else {
            let unit = Self::new(self.x / len, self.y / len, self.z / len);
            if unit.is_finite() {
                Some(unit)
            } else {
                None
            }
        }
    }

    /// Normalizes the vector to unit length, returning [`Vec3::ZERO`] if its length is
    /// less than or equal to [`EPSILON`] or if its components are non-finite.
    #[must_use]
    #[inline]
    pub fn normalize_or_zero(self) -> Self {
        self.try_normalize().unwrap_or(Self::ZERO)
    }

    /// Alias for [`Vec3::normalize_or_zero`] providing safe normalization without panicking or producing `NaN`.
    #[must_use]
    #[inline]
    pub fn normalize(self) -> Self {
        self.normalize_or_zero()
    }

    /// Computes the unit direction vector pointing from `self` toward `target`.
    ///
    /// Returns [`Vec3::ZERO`] if the two points coincide within [`EPSILON`] or contain non-finite coordinates.
    #[must_use]
    #[inline]
    pub fn direction_to(self, target: Self) -> Self {
        if !self.is_finite() || !target.is_finite() {
            return Self::ZERO;
        }
        (target - self).normalize_or_zero()
    }

    /// Computes the unsigned angle in radians `[0, π]` between `self` and `other`.
    ///
    /// Returns `None` if either vector has near-zero length or contains non-finite values.
    #[must_use]
    #[inline]
    pub fn angle_between(self, other: Self) -> Option<Scalar> {
        let u = self.try_normalize()?;
        let v = other.try_normalize()?;
        let cos_theta = u.dot(v).clamp(-1.0, 1.0);
        Some(cos_theta.acos())
    }

    /// Projects `self` onto `onto`.
    #[must_use]
    #[inline]
    pub fn project_onto(self, onto: Self) -> Self {
        if let Some(dir) = onto.try_normalize() {
            dir.scale(self.dot(dir))
        } else {
            Self::ZERO
        }
    }

    /// Reflects `self` (treated as an incident vector) across a surface with the given 3D `normal`.
    #[must_use]
    #[inline]
    pub fn reflect(self, normal: Self) -> Self {
        if !self.is_finite() {
            return Self::ZERO;
        }
        if let Some(n) = normal.try_normalize() {
            (self - n.scale(2.0 * self.dot(n))).sanitized()
        } else {
            self
        }
    }

    /// Linearly interpolates between `self` and `other` by parameter `t`.
    #[must_use]
    #[inline]
    pub fn lerp(self, other: Self, t: Scalar) -> Self {
        if !self.is_finite() || !other.is_finite() || !t.is_finite() {
            return Self::ZERO;
        }
        (self.scale(1.0 - t) + other.scale(t)).sanitized()
    }

    /// Checks whether `self` and `other` are component-wise equal within `tolerance`.
    #[must_use]
    #[inline]
    pub fn approx_eq(self, other: Self, tolerance: Scalar) -> bool {
        if !self.is_finite() || !other.is_finite() || !tolerance.is_finite() || tolerance < 0.0 {
            return false;
        }
        (self.x - other.x).abs() <= tolerance
            && (self.y - other.y).abs() <= tolerance
            && (self.z - other.z).abs() <= tolerance
    }
}

impl Add for Vec3 {
    type Output = Self;

    #[inline]
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;

    #[inline]
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Mul<Scalar> for Vec3 {
    type Output = Self;

    #[inline]
    fn mul(self, rhs: Scalar) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl Mul<Vec3> for Scalar {
    type Output = Vec3;

    #[inline]
    fn mul(self, rhs: Vec3) -> Self::Output {
        Vec3::new(self * rhs.x, self * rhs.y, self * rhs.z)
    }
}

impl MulAssign<Scalar> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, rhs: Scalar) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div<Scalar> for Vec3 {
    type Output = Self;

    /// Divides the vector by `rhs`. Returns [`Vec3::ZERO`] if `rhs` is zero or non-finite
    /// to prevent `NaN` or `Infinity` propagation.
    #[inline]
    fn div(self, rhs: Scalar) -> Self::Output {
        self.checked_div(rhs).unwrap_or(Self::ZERO)
    }
}

impl DivAssign<Scalar> for Vec3 {
    #[inline]
    fn div_assign(&mut self, rhs: Scalar) {
        *self = *self / rhs;
    }
}

impl Neg for Vec3 {
    type Output = Self;

    #[inline]
    fn neg(self) -> Self::Output {
        Self::new(-self.x, -self.y, -self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    #[test]
    fn vec2_basic_arithmetic_and_operators() {
        let a = Vec2::new(3.0, -4.0);
        let b = Vec2::new(1.5, 2.5);

        assert!((a + b).approx_eq(Vec2::new(4.5, -1.5), 1e-12));
        assert!((a - b).approx_eq(Vec2::new(1.5, -6.5), 1e-12));
        assert!((a * 2.0).approx_eq(Vec2::new(6.0, -8.0), 1e-12));
        assert!((2.0 * a).approx_eq(Vec2::new(6.0, -8.0), 1e-12));
        assert!((a / 2.0).approx_eq(Vec2::new(1.5, -2.0), 1e-12));
        assert!((-a).approx_eq(Vec2::new(-3.0, 4.0), 1e-12));

        let mut c = a;
        c += b;
        c -= Vec2::new(0.5, 0.5);
        c *= 2.0;
        c /= 4.0;
        assert!(c.approx_eq(Vec2::new(2.0, -1.0), 1e-12));
    }

    #[test]
    fn vec3_basic_arithmetic_and_operators() {
        let a = Vec3::new(2.0, -3.0, 6.0);
        let b = Vec3::new(1.0, 4.0, -2.0);

        assert!((a + b).approx_eq(Vec3::new(3.0, 1.0, 4.0), 1e-12));
        assert!((a - b).approx_eq(Vec3::new(1.0, -7.0, 8.0), 1e-12));
        assert!((a * 3.0).approx_eq(Vec3::new(6.0, -9.0, 18.0), 1e-12));
        assert!((3.0 * a).approx_eq(Vec3::new(6.0, -9.0, 18.0), 1e-12));
        assert!((a / 2.0).approx_eq(Vec3::new(1.0, -1.5, 3.0), 1e-12));
        assert!((-a).approx_eq(Vec3::new(-2.0, 3.0, -6.0), 1e-12));
    }

    #[test]
    fn dot_and_cross_products() {
        let u2 = Vec2::new(2.0, 3.0);
        let v2 = Vec2::new(-4.0, 5.0);
        assert!((u2.dot(v2) - 7.0).abs() < 1e-12);
        assert!((u2.cross(v2) - 22.0).abs() < 1e-12);

        let x = Vec3::UNIT_X;
        let y = Vec3::UNIT_Y;
        let z = Vec3::UNIT_Z;
        assert_eq!(x.dot(y), 0.0);
        assert!(x.cross(y).approx_eq(z, 1e-12));
        assert!(y.cross(x).approx_eq(-z, 1e-12));
    }

    #[test]
    fn length_distance_direction_and_safe_normalization() {
        let v = Vec2::new(3.0, 4.0);
        assert!((v.length() - 5.0).abs() < 1e-12);
        assert!((v.length_squared() - 25.0).abs() < 1e-12);

        let unit = v.normalize();
        assert!((unit.length() - 1.0).abs() < 1e-12);
        assert!(unit.approx_eq(Vec2::new(0.6, 0.8), 1e-12));

        assert_eq!(Vec2::ZERO.try_normalize(), None);
        assert_eq!(Vec2::ZERO.normalize(), Vec2::ZERO);
        assert_eq!(Vec3::ZERO.try_normalize(), None);
        assert_eq!(Vec3::ZERO.normalize(), Vec3::ZERO);

        let p1 = Vec2::new(1.0, 2.0);
        let p2 = Vec2::new(4.0, 6.0);
        assert!((p1.distance(p2) - 5.0).abs() < 1e-12);
        assert!(p1.direction_to(p2).approx_eq(Vec2::new(0.6, 0.8), 1e-12));
        assert_eq!(p1.direction_to(p1), Vec2::ZERO);
    }

    #[test]
    fn angles_rotations_and_reflections() {
        let right = Vec2::UNIT_X;
        let up = Vec2::UNIT_Y;
        assert!((right.angle_between(up).unwrap() - FRAC_PI_2).abs() < 1e-12);
        assert!((right.signed_angle_to(up).unwrap() - FRAC_PI_2).abs() < 1e-12);
        assert!((up.signed_angle_to(right).unwrap() + FRAC_PI_2).abs() < 1e-12);

        let rotated = right.rotate(FRAC_PI_4);
        let expected = Vec2::from_angle(FRAC_PI_4);
        assert!(rotated.approx_eq(expected, 1e-12));

        let incident = Vec2::new(2.0, -1.0);
        let normal = Vec2::new(0.0, 1.0);
        let reflected = incident.reflect(normal);
        assert!(reflected.approx_eq(Vec2::new(2.0, 1.0), 1e-12));

        let incident_3d = Vec3::new(1.0, -2.0, 0.5);
        let normal_3d = Vec3::new(0.0, 1.0, 0.0);
        assert!(incident_3d
            .reflect(normal_3d)
            .approx_eq(Vec3::new(1.0, 2.0, 0.5), 1e-12));
        assert!((Vec2::new(-1.0, 0.0).angle() - PI).abs() < 1e-12);
    }

    #[test]
    fn non_finite_and_zero_division_safety() {
        let nan_v2 = Vec2::new(Scalar::NAN, 1.0);
        let inf_v2 = Vec2::new(Scalar::INFINITY, -2.0);
        let valid_v2 = Vec2::new(2.0, 3.0);

        assert!(!nan_v2.is_finite());
        assert!(!inf_v2.is_finite());
        assert_eq!(nan_v2.sanitized(), Vec2::ZERO);
        assert_eq!(inf_v2.normalize(), Vec2::ZERO);
        assert_eq!(valid_v2 / 0.0, Vec2::ZERO);
        assert_eq!(valid_v2 / Scalar::NAN, Vec2::ZERO);
        assert_eq!(valid_v2.checked_div(0.0), None);
        assert_eq!(nan_v2.dot(valid_v2), 0.0);
        assert_eq!(nan_v2.distance(valid_v2), 0.0);

        let nan_v3 = Vec3::new(1.0, Scalar::NAN, 0.0);
        let valid_v3 = Vec3::new(1.0, 2.0, 3.0);
        assert!(!nan_v3.is_finite());
        assert_eq!(nan_v3.sanitized(), Vec3::ZERO);
        assert_eq!(valid_v3 / 0.0, Vec3::ZERO);
        assert_eq!(nan_v3.cross(valid_v3), Vec3::ZERO);
    }
}
