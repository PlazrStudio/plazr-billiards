//! Billiard table geometry, boundary limits, and pocket specifications.
//!
//! Represents the structural dimensions and static physical parameters of the playing surface,
//! cushions, and pockets. The coordinate system places `(0.0, 0.0)` at the bottom-left corner
//! of the playable bed (`x` in `[0.0, length]`, `y` in `[0.0, width]`).
//!
//! ## Scope & Documented Limitations of the Current Table & Pocket Model
//! - **Circular Pocket Capture & Mouth Clearance Zones**: Pockets are modeled as planar circular
//!   drop zones (`Pocket { center, radius }`) surrounded by a configurable mouth clearance zone
//!   (`Pocket::DEFAULT_MOUTH_FACTOR`). When a ball is within a pocket's mouth opening and moving
//!   toward the pocket center, cushion reflections are suppressed so corner and side shots enter
//!   the pocket smoothly without bouncing off an imaginary rail across the pocket opening.
//! - **Not Yet Modeled**: Angled bevels/facings of physical pocket jaws (which can cause rattle
//!   rejection on fast rail shots), vertical ball drop trajectories inside the pocket basket, and
//!   3D cushion nose profile contact height (`cushion_height` spin coupling) are deferred to a
//!   future phase.

use crate::math::{Scalar, Vec2, EPSILON};

/// Logical cushion rail indices around a rectangular billiard table bed.
pub mod cushion_index {
    /// Left rail (`x = 0.0`, inward normal `+X`).
    pub const LEFT: u8 = 0;
    /// Right rail (`x = length`, inward normal `-X`).
    pub const RIGHT: u8 = 1;
    /// Bottom rail (`y = 0.0`, inward normal `+Y`).
    pub const BOTTOM: u8 = 2;
    /// Top rail (`y = width`, inward normal `-Y`).
    pub const TOP: u8 = 3;
}

/// Dimensions and surface characteristics of a billiard table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TableSpecs {
    /// Playable length of the table bed (nose-to-nose) along the X-axis in meters.
    pub length: Scalar,
    /// Playable width of the table bed (nose-to-nose) along the Y-axis in meters.
    pub width: Scalar,
    /// Height of the cushion contact nose above the table bed in meters.
    pub cushion_height: Scalar,
}

impl TableSpecs {
    /// Standard WPA 9-foot pool table playable length (`2.54` m, `100` inches).
    pub const STANDARD_9FT_LENGTH: Scalar = 2.54;

    /// Standard WPA 9-foot pool table playable width (`1.27` m, `50` inches).
    pub const STANDARD_9FT_WIDTH: Scalar = 1.27;

    /// Standard pool cushion nose height (`0.0365` m, ~63.5% of standard ball diameter).
    pub const STANDARD_CUSHION_HEIGHT: Scalar = 0.0365;

    /// Creates a new [`TableSpecs`] with the given dimensions.
    #[must_use]
    pub const fn new(length: Scalar, width: Scalar, cushion_height: Scalar) -> Self {
        Self {
            length,
            width,
            cushion_height,
        }
    }

    /// Returns standard 9-foot regulation pool table dimensions (`2.54 m × 1.27 m`).
    #[must_use]
    pub const fn standard_9ft() -> Self {
        Self {
            length: Self::STANDARD_9FT_LENGTH,
            width: Self::STANDARD_9FT_WIDTH,
            cushion_height: Self::STANDARD_CUSHION_HEIGHT,
        }
    }

    /// Returns `true` if `length`, `width`, and `cushion_height` are finite positive numbers.
    #[must_use]
    #[inline]
    pub fn is_valid(self) -> bool {
        self.length.is_finite()
            && self.width.is_finite()
            && self.cushion_height.is_finite()
            && self.length > EPSILON
            && self.width > EPSILON
            && self.cushion_height >= 0.0
    }
}

impl Default for TableSpecs {
    fn default() -> Self {
        Self::standard_9ft()
    }
}

/// Descriptor for a single table pocket opening.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pocket {
    /// Planar center coordinate of the pocket drop zone.
    pub center: Vec2,
    /// Effective capture radius of the pocket in meters.
    pub radius: Scalar,
}

impl Pocket {
    /// Default multiplier (`1.45`) applied to `radius` to define the pocket mouth opening
    /// along adjacent cushion rails where balls heading into the pocket are not reflected.
    pub const DEFAULT_MOUTH_FACTOR: Scalar = 1.45;

    /// Creates a new [`Pocket`] with the given `center` coordinate and capture `radius`.
    #[must_use]
    pub const fn new(center: Vec2, radius: Scalar) -> Self {
        Self { center, radius }
    }

    /// Returns `true` if a ball whose center is at `ball_position` falls within the pocket's
    /// effective capture radius (`distance(ball_position, center) <= radius`).
    #[must_use]
    #[inline]
    pub fn contains_point(&self, ball_position: Vec2) -> bool {
        if !self.center.is_finite()
            || !ball_position.is_finite()
            || !self.radius.is_finite()
            || self.radius <= 0.0
        {
            return false;
        }
        self.center.distance_squared(ball_position) <= self.radius * self.radius
    }

    /// Returns `true` if a ball at `ball_position` with planar `velocity` is inside this pocket's
    /// mouth clearance zone (`radius * mouth_factor`) and moving toward the pocket center.
    #[must_use]
    #[inline]
    pub fn is_entering_mouth(
        &self,
        ball_position: Vec2,
        velocity: Vec2,
        mouth_factor: Scalar,
    ) -> bool {
        if !self.center.is_finite()
            || !ball_position.is_finite()
            || !velocity.is_finite()
            || !self.radius.is_finite()
            || self.radius <= 0.0
            || !mouth_factor.is_finite()
            || mouth_factor <= 0.0
        {
            return false;
        }
        let mouth_radius = self.radius * mouth_factor;
        let to_pocket = self.center - ball_position;
        if to_pocket.length_squared() > mouth_radius * mouth_radius {
            return false;
        }
        to_pocket.dot(velocity) > EPSILON
    }
}

/// Complete static representation of a billiard table.
#[derive(Debug, Clone, PartialEq)]
pub struct Table {
    /// Dimensional and surface parameters of the table.
    pub specs: TableSpecs,
    /// Collection of pockets configured for this table (empty for carom tables).
    pub pockets: Vec<Pocket>,
}

impl Table {
    /// Constructs a new [`Table`] with the provided specifications and pocket definitions.
    #[must_use]
    pub const fn new(specs: TableSpecs, pockets: Vec<Pocket>) -> Self {
        Self { specs, pockets }
    }

    /// Creates a standard 9-foot 6-pocket pool table with corner and side pockets positioned
    /// along the playing surface boundaries (`x` in `[0.0, 2.54]`, `y` in `[0.0, 1.27]`).
    #[must_use]
    pub fn standard_9ft_six_pocket() -> Self {
        let specs = TableSpecs::standard_9ft();
        let corner_radius = 0.060;
        let side_radius = 0.065;
        let half_length = specs.length * 0.5;

        let pockets = vec![
            // Bottom-left, Bottom-middle, Bottom-right
            Pocket::new(Vec2::new(0.0, 0.0), corner_radius),
            Pocket::new(Vec2::new(half_length, 0.0), side_radius),
            Pocket::new(Vec2::new(specs.length, 0.0), corner_radius),
            // Top-left, Top-middle, Top-right
            Pocket::new(Vec2::new(0.0, specs.width), corner_radius),
            Pocket::new(Vec2::new(half_length, specs.width), side_radius),
            Pocket::new(Vec2::new(specs.length, specs.width), corner_radius),
        ];

        Self { specs, pockets }
    }

    /// Creates a pocketless carom billiard table with the given specifications.
    #[must_use]
    pub const fn pocketless(specs: TableSpecs) -> Self {
        Self {
            specs,
            pockets: Vec::new(),
        }
    }

    /// Returns the planar center coordinate of the playing bed `(length / 2, width / 2)`.
    #[must_use]
    #[inline]
    pub fn center(&self) -> Vec2 {
        Vec2::new(self.specs.length * 0.5, self.specs.width * 0.5)
    }

    /// Returns the inward unit normal vector for a given logical cushion index (`0..=3`).
    #[must_use]
    #[inline]
    pub const fn cushion_normal(cushion_idx: u8) -> Option<Vec2> {
        match cushion_idx {
            cushion_index::LEFT => Some(Vec2::UNIT_X),
            cushion_index::RIGHT => Some(Vec2::new(-1.0, 0.0)),
            cushion_index::BOTTOM => Some(Vec2::UNIT_Y),
            cushion_index::TOP => Some(Vec2::new(0.0, -1.0)),
            _ => None,
        }
    }

    /// Checks whether a ball at `position` is captured by any pocket on the table, returning
    /// the logical pocket index (`0..pockets.len()`) of the nearest capturing pocket.
    #[must_use]
    pub fn find_capturing_pocket(&self, position: Vec2) -> Option<u8> {
        if !position.is_finite() {
            return None;
        }
        let mut best: Option<(u8, Scalar)> = None;
        for (idx, pocket) in self.pockets.iter().enumerate() {
            if pocket.contains_point(position) {
                if let Ok(idx_u8) = u8::try_from(idx) {
                    let dist_sq = pocket.center.distance_squared(position);
                    if best.map_or(true, |(_, best_d)| dist_sq < best_d) {
                        best = Some((idx_u8, dist_sq));
                    }
                }
            }
        }
        best.map(|(idx, _)| idx)
    }

    /// Returns `true` if `position` is within the mouth zone of any pocket (`radius * mouth_factor`),
    /// where the cushion rail opens into the pocket jaws instead of rebounding the ball.
    #[must_use]
    pub fn is_near_pocket_opening(&self, position: Vec2, mouth_factor: Scalar) -> bool {
        if !position.is_finite() || !mouth_factor.is_finite() || mouth_factor <= 0.0 {
            return false;
        }
        for pocket in &self.pockets {
            let effective = pocket.radius * mouth_factor;
            if pocket.center.distance_squared(position) <= effective * effective {
                return true;
            }
        }
        false
    }

    /// Returns `true` if a ball at `position` moving with `velocity` is entering the mouth opening
    /// of any pocket on the table, meaning adjacent cushion rails should not bounce it away before
    /// it reaches the pocket capture zone.
    #[must_use]
    pub fn is_entering_any_pocket_mouth(&self, position: Vec2, velocity: Vec2) -> bool {
        self.pockets
            .iter()
            .any(|p| p.is_entering_mouth(position, velocity, Pocket::DEFAULT_MOUTH_FACTOR))
    }
}

impl Default for Table {
    fn default() -> Self {
        Self::standard_9ft_six_pocket()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pocket_capture_and_mouth_clearance_near_corners() {
        let table = Table::standard_9ft_six_pocket();
        assert_eq!(table.find_capturing_pocket(Vec2::new(0.02, 0.02)), Some(0));
        assert_eq!(table.find_capturing_pocket(Vec2::new(1.27, 0.03)), Some(1));
        assert_eq!(table.find_capturing_pocket(Vec2::new(0.5, 0.5)), None);

        let pos = Vec2::new(0.055, 0.0285);
        let vel = Vec2::new(-1.0, -0.5);
        assert!(table.is_entering_any_pocket_mouth(pos, vel));
        assert!(!table.is_entering_any_pocket_mouth(pos, -vel));
    }
}
