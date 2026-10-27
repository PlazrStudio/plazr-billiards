//! Billiard table geometry and surface specifications.
//!
//! Represents the structural dimensions and static physical parameters of the playing surface,
//! cushions, and pockets. Collision resolution against cushions and pockets is deferred to Phase 2+.

use crate::math::{Scalar, Vec2};

/// Dimensions and surface characteristics of a billiard table.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TableSpecs {
    /// Playable length of the table bed (nose-to-nose) in meters.
    pub length: Scalar,
    /// Playable width of the table bed (nose-to-nose) in meters.
    pub width: Scalar,
    /// Height of the cushion contact nose above the table bed in meters.
    pub cushion_height: Scalar,
}

/// Descriptor for a single table pocket opening.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pocket {
    /// Planar center coordinate of the pocket drop zone.
    pub center: Vec2,
    /// Effective capture radius of the pocket in meters.
    pub radius: Scalar,
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
}
