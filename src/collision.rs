//! Collision event definitions and detection contracts.
//!
//! Defines the structural types representing potential and resolved contacts between
//! balls, cushions, and pockets. Actual collision detection and impulse solver algorithms
//! are scheduled for Phase 2+.

use crate::ball::BallId;
use crate::math::Scalar;

/// Classification of participants involved in a collision event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CollisionKind {
    /// Contact between two distinct billiard balls.
    BallToBall {
        /// First ball involved in the contact.
        first: BallId,
        /// Second ball involved in the contact.
        second: BallId,
    },
    /// Contact between a billiard ball and a table cushion rail.
    BallToCushion {
        /// Ball striking the cushion.
        ball: BallId,
        /// Logical index of the cushion segment.
        cushion_index: u8,
    },
    /// Entry of a billiard ball into a pocket zone.
    BallToPocket {
        /// Ball entering the pocket.
        ball: BallId,
        /// Logical index of the pocket.
        pocket_index: u8,
    },
}

/// Record of a single collision event occurring at a specific time offset within a simulation step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionEvent {
    /// Time of impact relative to the start of the current simulation step, in seconds.
    pub time_of_impact: Scalar,
    /// Type and participants of the collision.
    pub kind: CollisionKind,
}
