//! Collision detection, Continuous Collision Detection (CCD), impulse resolution,
//! cushion rebounds, and pocket capture logic.
//!
//! This module implements:
//! - Analytic Continuous Collision Detection (CCD) Time of Impact (TOI) queries for
//!   ball-to-ball (`compute_ball_ball_toi`), ball-to-cushion (`compute_ball_cushion_toi`),
//!   and ball-to-pocket (`compute_ball_pocket_toi`) trajectories.
//! - Discrete and continuous contact manifold evaluation (`detect_ball_ball_contact`) with
//!   deterministic fallback normals for touching or coincident-center balls.
//! - Mass- and restitution-aware normal impulse resolution (`resolve_ball_ball_collision`)
//!   that conserves linear momentum (`m₁v₁ + m₂v₂`) and, for perfectly elastic (`e = 1.0`)
//!   contacts, kinetic energy (`½m₁v₁² + ½m₂v₂²`).
//! - Multi-pass positional overlap separation (`resolve_ball_overlap`) weighted by inverse mass
//!   with optional slop tolerance so touching balls in tight racks or Newton's cradle lines
//!   exchange impulses cleanly without artificial positional drift.
//! - Pocket mouth clearance during cushion checks (`resolve_ball_cushion_collisions`) so shots
//!   aimed into corner or side pockets near rail intersections enter the pocket rather than
//!   bouncing off an imaginary rail across the pocket mouth.
//!
//! ## Scope & Documented Limitations of the Current Collision Model
//! 1. **Piecewise-Linear CCD Within Sub-steps**: Within each fixed timestep (or adaptive sub-step),
//!    analytic TOI queries assume constant velocity over the sub-step interval `Δt` before applying
//!    deceleration over the sub-step segment. When multiple collisions occur within a single fixed
//!    step, the simulation advances to the earliest TOI up to a configurable event budget and then
//!    runs multi-pass Gauss-Seidel impulse/overlap resolution for simultaneous clusters.
//! 2. **Frictionless Tangential Contact & Simplified Cushion/Pocket Geometry**: Ball-to-ball and
//!    ball-to-cushion collisions currently resolve normal linear impulses with configurable
//!    restitution (`ball_restitution`, `cushion_restitution`). Throw (tangential friction during
//!    impact), 3D spin transfer (`angular_velocity` coupling into cushion rebound angles or
//!    rolling/sliding transitions), and angled pocket jaw segments are not yet modeled.

use crate::ball::{Ball, BallId};
use crate::math::{clamp_scalar, sanitize_scalar, Scalar, Vec2, EPSILON};
use crate::table::{cushion_index, Pocket, Table};

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
        /// Logical index of the cushion segment (`0` = Left, `1` = Right, `2` = Bottom, `3` = Top).
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

/// Record of a single collision event occurring at a time offset within a simulation step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CollisionEvent {
    /// Time of impact relative to the start of the current simulation step (or multi-step run),
    /// in seconds.
    pub time_of_impact: Scalar,
    /// Type and participants of the collision.
    pub kind: CollisionKind,
}

/// Detailed geometric contact manifold between two overlapping or touching balls.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BallContact {
    /// Unit contact normal pointing from ball `a` toward ball `b`.
    pub normal: Vec2,
    /// Center-to-center distance between the two balls in meters.
    pub distance: Scalar,
    /// Penetration depth `(r_a + r_b) - distance` in meters (`>= 0.0`).
    pub penetration: Scalar,
}

/// Small geometric contact tolerance (`1e-7` m) allowing balls resting in contact (such as a tight
/// rack or Newton's cradle chain) to be detected as touching without artificial overlap jitter.
pub const CONTACT_TOLERANCE: Scalar = 1e-7;

/// Detects whether two active balls on the table are touching or overlapping within [`CONTACT_TOLERANCE`].
///
/// Returns `Some(BallContact)` if both balls are on the table, have valid specifications,
/// and their center-to-center distance is less than or equal to `r_a + r_b + CONTACT_TOLERANCE`.
/// When the two ball centers coincide (`distance <= EPSILON`), the fallback normal is oriented
/// along `(v_a - v_b).try_normalize()` (or `+X` if stationary) so that an approaching relative
/// velocity yields `(v_b - v_a) · normal < 0` and triggers a physical collision impulse.
#[must_use]
pub fn detect_ball_ball_contact(ball_a: &Ball, ball_b: &Ball) -> Option<BallContact> {
    if !ball_a.is_on_table()
        || !ball_b.is_on_table()
        || !ball_a.specs.is_valid()
        || !ball_b.specs.is_valid()
        || !ball_a.position.is_finite()
        || !ball_b.position.is_finite()
    {
        return None;
    }

    let combined_radius = ball_a.specs.radius + ball_b.specs.radius;
    let max_contact_dist = combined_radius + CONTACT_TOLERANCE;
    let delta = ball_b.position - ball_a.position;
    let dist_sq = delta.length_squared();

    if dist_sq > max_contact_dist * max_contact_dist {
        return None;
    }

    let distance = dist_sq.sqrt();
    let normal = if distance > EPSILON {
        delta / distance
    } else {
        // Fallback deterministic separation axis when two ball centers coincide:
        // orient normal along (v_a - v_b) so that relative velocity (v_b - v_a) · normal < 0,
        // ensuring an approaching velocity produces a physical collision impulse.
        let approach_axis = ball_a.planar_velocity() - ball_b.planar_velocity();
        approach_axis.try_normalize().unwrap_or(Vec2::UNIT_X)
    };

    Some(BallContact {
        normal,
        distance,
        penetration: (combined_radius - distance).max(0.0),
    })
}

/// Computes the exact analytic Time of Impact (TOI) within `[0.0, max_dt]` for two balls moving
/// at constant linear velocities before considering deceleration.
///
/// - Returns `Some(0.0)` if the balls are already touching/overlapping AND approaching each other
///   (or coincident with non-zero relative velocity).
/// - Returns `Some(t)` if they come into contact at `0.0 <= t <= max_dt`.
/// - Returns `None` if they are moving apart, stationary, or do not collide within `max_dt`.
#[must_use]
pub fn compute_ball_ball_toi(ball_a: &Ball, ball_b: &Ball, max_dt: Scalar) -> Option<Scalar> {
    if !ball_a.is_on_table()
        || !ball_b.is_on_table()
        || !ball_a.specs.is_valid()
        || !ball_b.specs.is_valid()
        || !max_dt.is_finite()
        || max_dt < 0.0
    {
        return None;
    }

    let r = ball_a.specs.radius + ball_b.specs.radius;
    let dp = ball_b.position - ball_a.position;
    let dv = ball_b.planar_velocity() - ball_a.planar_velocity();
    if !dp.is_finite() || !dv.is_finite() {
        return None;
    }

    let a = dv.length_squared();
    let b = dp.dot(dv);
    let c = dp.length_squared() - r * r;

    // Already touching or overlapping
    if c <= CONTACT_TOLERANCE * r {
        if b < -EPSILON || (dp.length_squared() <= EPSILON * EPSILON && a > EPSILON) {
            return Some(0.0);
        }
        return None;
    }

    if a <= EPSILON || b >= 0.0 {
        return None;
    }

    let discriminant = b * b - a * c;
    if discriminant < 0.0 {
        return None;
    }

    let toi = (-b - discriminant.sqrt()) / a;
    if toi >= 0.0 && toi <= max_dt {
        Some(toi)
    } else {
        None
    }
}

/// Computes the earliest analytic Time of Impact (TOI) within `[0.0, max_dt]` at which an active
/// `ball` moving at constant velocity enters any pocket capture radius on `table`.
///
/// Returns `Some((toi, pocket_index))` if the ball's center enters a pocket within `max_dt`.
#[must_use]
pub fn compute_ball_pocket_toi(ball: &Ball, table: &Table, max_dt: Scalar) -> Option<(Scalar, u8)> {
    if !ball.is_on_table() || !ball.position.is_finite() || !max_dt.is_finite() || max_dt < 0.0 {
        return None;
    }

    // Immediate capture check at t = 0
    if let Some(pocket_idx) = table.find_capturing_pocket(ball.position) {
        return Some((0.0, pocket_idx));
    }

    let v = ball.planar_velocity();
    let a = v.length_squared();
    if a <= EPSILON {
        return None;
    }

    let mut earliest: Option<(Scalar, u8)> = None;
    for (idx, pocket) in table.pockets.iter().enumerate() {
        let Ok(pocket_idx) = u8::try_from(idx) else {
            continue;
        };
        if !pocket.center.is_finite() || !pocket.radius.is_finite() || pocket.radius <= 0.0 {
            continue;
        }
        let dp = ball.position - pocket.center;
        let b = dp.dot(v);
        let c = dp.length_squared() - pocket.radius * pocket.radius;

        if c <= 0.0 {
            return Some((0.0, pocket_idx));
        }
        if b >= 0.0 {
            continue;
        }
        let disc = b * b - a * c;
        if disc < 0.0 {
            continue;
        }
        let toi = (-b - disc.sqrt()) / a;
        if (0.0..=max_dt).contains(&toi) && earliest.map_or(true, |(best_t, _)| toi < best_t) {
            earliest = Some((toi, pocket_idx));
        }
    }

    earliest
}

/// Computes the earliest analytic Time of Impact (TOI) within `[0.0, max_dt]` at which an active
/// `ball` strikes one of the four rectangular cushion rails of `table`, ignoring rails where the
/// impact point falls inside a pocket mouth opening that the ball is entering.
///
/// Returns `Some((toi, cushion_index))` if a cushion impact occurs within `max_dt`.
#[must_use]
pub fn compute_ball_cushion_toi(
    ball: &Ball,
    table: &Table,
    max_dt: Scalar,
) -> Option<(Scalar, u8)> {
    if !ball.is_on_table()
        || !ball.specs.is_valid()
        || !table.specs.is_valid()
        || !max_dt.is_finite()
        || max_dt < 0.0
    {
        return None;
    }

    let r = ball.specs.radius;
    let length = table.specs.length;
    let width = table.specs.width;
    if length <= 2.0 * r || width <= 2.0 * r {
        return None;
    }

    let pos = ball.position;
    let v = ball.planar_velocity();
    if !pos.is_finite() || !v.is_finite() {
        return None;
    }

    let mut earliest: Option<(Scalar, u8)> = None;

    let mut consider = |toi: Scalar, cushion_idx: u8| {
        if !(0.0..=max_dt).contains(&toi) {
            return;
        }
        let impact_pos = pos + v.scale(toi);
        // Suppress rail collision if the ball is entering a pocket mouth or already inside a pocket
        if table.find_capturing_pocket(impact_pos).is_some()
            || table.is_entering_any_pocket_mouth(impact_pos, v)
        {
            return;
        }
        if earliest.map_or(true, |(best_t, _)| toi < best_t) {
            earliest = Some((toi, cushion_idx));
        }
    };

    // Left rail (x = r, moving -X)
    if v.x < -EPSILON {
        let toi = if pos.x <= r { 0.0 } else { (r - pos.x) / v.x };
        consider(toi, cushion_index::LEFT);
    } else if v.x > EPSILON {
        // Right rail (x = length - r, moving +X)
        let target_x = length - r;
        let toi = if pos.x >= target_x {
            0.0
        } else {
            (target_x - pos.x) / v.x
        };
        consider(toi, cushion_index::RIGHT);
    }

    // Bottom rail (y = r, moving -Y)
    if v.y < -EPSILON {
        let toi = if pos.y <= r { 0.0 } else { (r - pos.y) / v.y };
        consider(toi, cushion_index::BOTTOM);
    } else if v.y > EPSILON {
        // Top rail (y = width - r, moving +Y)
        let target_y = width - r;
        let toi = if pos.y >= target_y {
            0.0
        } else {
            (target_y - pos.y) / v.y
        };
        consider(toi, cushion_index::TOP);
    }

    earliest
}

/// Resolves positional overlap (penetration) between two balls weighted by their inverse masses.
///
/// Returns `true` if a positional correction was applied.
pub fn resolve_ball_overlap(ball_a: &mut Ball, ball_b: &mut Ball) -> bool {
    let Some(contact) = detect_ball_ball_contact(ball_a, ball_b) else {
        return false;
    };

    if contact.penetration <= CONTACT_TOLERANCE {
        return false;
    }

    let inv_mass_a = ball_a.specs.inverse_mass();
    let inv_mass_b = ball_b.specs.inverse_mass();
    let total_inv_mass = inv_mass_a + inv_mass_b;

    if total_inv_mass <= EPSILON {
        return false;
    }

    // Separate to exact touching distance plus a minimal epsilon (5e-9 m) inside CONTACT_TOLERANCE
    let correction_magnitude = (contact.penetration + 5e-9) / total_inv_mass;
    let correction = contact.normal.scale(correction_magnitude);

    ball_a.position = (ball_a.position - correction.scale(inv_mass_a)).sanitized();
    ball_b.position = (ball_b.position + correction.scale(inv_mass_b)).sanitized();
    true
}

/// Resolves both positional overlap and elastic/inelastic normal velocity impulse between two balls.
///
/// - Accounts for each ball's mass (`ball.specs.mass`) and the coefficient of restitution (`restitution` in `[0.0, 1.0]`).
/// - Separates overlapping balls proportionally to their inverse masses.
/// - Updates `BallState` according to `stop_speed_threshold`.
///
/// Returns `true` if a collision contact occurred (either impulse exchange or overlap separation).
pub fn resolve_ball_ball_collision(
    ball_a: &mut Ball,
    ball_b: &mut Ball,
    restitution: Scalar,
    stop_speed_threshold: Scalar,
) -> bool {
    let Some(contact) = detect_ball_ball_contact(ball_a, ball_b) else {
        return false;
    };

    // Separate any geometric overlap beyond CONTACT_TOLERANCE
    let separated = resolve_ball_overlap(ball_a, ball_b);

    let inv_mass_a = ball_a.specs.inverse_mass();
    let inv_mass_b = ball_b.specs.inverse_mass();
    let total_inv_mass = inv_mass_a + inv_mass_b;
    if total_inv_mass <= EPSILON {
        return false;
    }

    let va = ball_a.planar_velocity();
    let vb = ball_b.planar_velocity();
    let relative_velocity = vb - va;
    let velocity_along_normal = relative_velocity.dot(contact.normal);

    // Apply normal impulse if the balls are approaching each other
    if velocity_along_normal < -EPSILON {
        let e = clamp_scalar(restitution, 0.0, 1.0, 0.95);
        let impulse_scalar = -(1.0 + e) * velocity_along_normal / total_inv_mass;
        let impulse = contact.normal.scale(impulse_scalar);

        let new_va = (va - impulse.scale(inv_mass_a)).sanitized();
        let new_vb = (vb + impulse.scale(inv_mass_b)).sanitized();

        ball_a.set_planar_velocity(new_va, stop_speed_threshold);
        ball_b.set_planar_velocity(new_vb, stop_speed_threshold);
        return true;
    }

    separated && contact.penetration > 1e-6
}

/// Checks whether an active ball has entered any pocket on the `table`.
///
/// If captured, marks the ball as [`crate::ball::BallState::Pocketed`], zeroes its velocity,
/// and returns the corresponding [`CollisionKind::BallToPocket`].
pub fn check_and_resolve_pocket(ball: &mut Ball, table: &Table) -> Option<CollisionKind> {
    if !ball.is_on_table() || !ball.position.is_finite() {
        return None;
    }

    if let Some(pocket_index) = table.find_capturing_pocket(ball.position) {
        ball.mark_pocketed(pocket_index);
        return Some(CollisionKind::BallToPocket {
            ball: ball.id,
            pocket_index,
        });
    }

    None
}

/// Checks and resolves collisions between an active `ball` and the rectangular cushion rails of `table`.
///
/// - Checks whether the ball is first captured by a pocket so pocketed balls do not bounce off rails.
/// - Suppresses cushion bounce if the ball is inside a pocket mouth opening (`Pocket::DEFAULT_MOUTH_FACTOR`)
///   and moving toward the pocket center, allowing corner and side pocket shots near rail intersections
///   to enter the pocket naturally.
/// - Reflects normal velocity using `cushion_restitution` (`e_c` in `[0.0, 1.0]`) and clamps the ball
///   inside the playable table boundaries `[radius, length - radius]` and `[radius, width - radius]`.
/// - Appends any generated [`CollisionEvent`] records (timestamped at `time_offset`) to `events_out`.
pub fn resolve_ball_cushion_collisions(
    ball: &mut Ball,
    table: &Table,
    cushion_restitution: Scalar,
    stop_speed_threshold: Scalar,
    time_offset: Scalar,
    events_out: &mut Vec<CollisionEvent>,
) {
    if !ball.is_on_table() || !ball.specs.is_valid() || !table.specs.is_valid() {
        return;
    }

    // 1. Check pocket capture first before enforcing cushion boundaries
    if let Some(pocket_kind) = check_and_resolve_pocket(ball, table) {
        events_out.push(CollisionEvent {
            time_of_impact: sanitize_scalar(time_offset, 0.0).max(0.0),
            kind: pocket_kind,
        });
        return;
    }

    let r = ball.specs.radius;
    let length = table.specs.length;
    let width = table.specs.width;

    // Guard against degenerate tables smaller than a single ball diameter
    if length <= 2.0 * r || width <= 2.0 * r {
        return;
    }

    let mut v = ball.planar_velocity();

    // 2. If the ball is inside a pocket mouth zone and moving toward that pocket, let it continue
    //    into the pocket rather than clamping/reflecting it off the rail across the pocket mouth.
    for (idx, pocket) in table.pockets.iter().enumerate() {
        if pocket.is_entering_mouth(ball.position, v, Pocket::DEFAULT_MOUTH_FACTOR) {
            // Check if a short projection along its current velocity enters the pocket capture zone
            let to_pocket = pocket.center - ball.position;
            let dist_to_center = to_pocket.length();
            if dist_to_center <= pocket.radius {
                if let Ok(pocket_index) = u8::try_from(idx) {
                    ball.mark_pocketed(pocket_index);
                    events_out.push(CollisionEvent {
                        time_of_impact: sanitize_scalar(time_offset, 0.0).max(0.0),
                        kind: CollisionKind::BallToPocket {
                            ball: ball.id,
                            pocket_index,
                        },
                    });
                }
            }
            return;
        }
    }

    let e = clamp_scalar(cushion_restitution, 0.0, 1.0, 0.80);
    let toi = sanitize_scalar(time_offset, 0.0).max(0.0);
    let mut hit_cushion = false;

    // Left cushion (x = 0, normal = +X)
    if ball.position.x <= r + CONTACT_TOLERANCE {
        if ball.position.x < r {
            ball.position.x = r;
        }
        if v.x < -EPSILON {
            v.x = -v.x * e;
            hit_cushion = true;
            events_out.push(CollisionEvent {
                time_of_impact: toi,
                kind: CollisionKind::BallToCushion {
                    ball: ball.id,
                    cushion_index: cushion_index::LEFT,
                },
            });
        }
    } else if ball.position.x >= length - r - CONTACT_TOLERANCE {
        // Right cushion (x = length, normal = -X)
        if ball.position.x > length - r {
            ball.position.x = length - r;
        }
        if v.x > EPSILON {
            v.x = -v.x * e;
            hit_cushion = true;
            events_out.push(CollisionEvent {
                time_of_impact: toi,
                kind: CollisionKind::BallToCushion {
                    ball: ball.id,
                    cushion_index: cushion_index::RIGHT,
                },
            });
        }
    }

    // Bottom cushion (y = 0, normal = +Y)
    if ball.position.y <= r + CONTACT_TOLERANCE {
        if ball.position.y < r {
            ball.position.y = r;
        }
        if v.y < -EPSILON {
            v.y = -v.y * e;
            hit_cushion = true;
            events_out.push(CollisionEvent {
                time_of_impact: toi,
                kind: CollisionKind::BallToCushion {
                    ball: ball.id,
                    cushion_index: cushion_index::BOTTOM,
                },
            });
        }
    } else if ball.position.y >= width - r - CONTACT_TOLERANCE {
        // Top cushion (y = width, normal = -Y)
        if ball.position.y > width - r {
            ball.position.y = width - r;
        }
        if v.y > EPSILON {
            v.y = -v.y * e;
            hit_cushion = true;
            events_out.push(CollisionEvent {
                time_of_impact: toi,
                kind: CollisionKind::BallToCushion {
                    ball: ball.id,
                    cushion_index: cushion_index::TOP,
                },
            });
        }
    }

    if hit_cushion {
        ball.set_planar_velocity(v, stop_speed_threshold);
    }
}

/// Computes the recommended number of sub-steps for a simulation step of duration `dt`
/// so that the fastest moving ball travels at most `0.45 * min_radius` per sub-step,
/// capped by `max_substeps`.
///
/// Note: When combined with Continuous Collision Detection (CCD) in `SimulationWorld`,
/// analytic TOI sweeps prevent tunneling even before sub-division, while adaptive sub-steps
/// bound the piecewise-linear deceleration approximation error.
#[must_use]
pub fn compute_adaptive_substeps(balls: &[Ball], dt: Scalar, max_substeps: u32) -> u32 {
    let cap = max_substeps.max(1);
    if !dt.is_finite() || dt <= EPSILON {
        return 1;
    }

    let mut max_speed: Scalar = 0.0;
    let mut min_radius: Scalar = Scalar::INFINITY;

    for ball in balls {
        if ball.is_on_table() && ball.specs.is_valid() {
            let s = ball.speed();
            if s > max_speed {
                max_speed = s;
            }
            if ball.specs.radius < min_radius {
                min_radius = ball.specs.radius;
            }
        }
    }

    if max_speed <= EPSILON || !min_radius.is_finite() || min_radius <= EPSILON {
        return 1;
    }

    let max_safe_displacement = (min_radius * 0.45).max(1e-4);
    let total_displacement = max_speed * dt;
    let needed = (total_displacement / max_safe_displacement).ceil();

    if !needed.is_finite() || needed <= 1.0 {
        1
    } else if needed >= Scalar::from(cap) {
        cap
    } else {
        needed as u32
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ball::BallSpecs;
    use crate::table::TableSpecs;

    #[test]
    fn head_on_equal_mass_elastic_collision_exchanges_velocities_and_conserves_energy() {
        let specs = BallSpecs::standard_pool();
        let r = specs.radius;
        let mut a = Ball::new_stationary(BallId(0), specs, Vec2::new(1.0, 0.5));
        let mut b = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0 + 2.0 * r, 0.5));
        a.set_planar_velocity(Vec2::new(4.0, 0.0), 1e-4);

        let initial_ke = a.kinetic_energy() + b.kinetic_energy();
        let initial_px =
            a.specs.mass * a.planar_velocity().x + b.specs.mass * b.planar_velocity().x;

        let collided = resolve_ball_ball_collision(&mut a, &mut b, 1.0, 1e-4);
        assert!(collided);
        assert!(a.speed() < 1e-9);
        assert!((b.planar_velocity().x - 4.0).abs() < 1e-9);

        let final_ke = a.kinetic_energy() + b.kinetic_energy();
        let final_px = a.specs.mass * a.planar_velocity().x + b.specs.mass * b.planar_velocity().x;
        assert!((initial_ke - final_ke).abs() < 1e-12);
        assert!((initial_px - final_px).abs() < 1e-12);
    }

    #[test]
    fn different_mass_and_restitution_collision() {
        let heavy_specs = BallSpecs::new(0.03, 0.30);
        let light_specs = BallSpecs::new(0.03, 0.10);
        let mut heavy = Ball::new_stationary(BallId(0), heavy_specs, Vec2::new(1.0, 0.5));
        let mut light = Ball::new_stationary(BallId(1), light_specs, Vec2::new(1.059, 0.5));
        heavy.set_planar_velocity(Vec2::new(2.0, 0.0), 1e-4);

        let collided = resolve_ball_ball_collision(&mut heavy, &mut light, 0.8, 1e-4);
        assert!(collided);
        let final_momentum = heavy.specs.mass * heavy.planar_velocity().x
            + light.specs.mass * light.planar_velocity().x;
        assert!((final_momentum - 0.60).abs() < 1e-9);
        assert!(light.planar_velocity().x > heavy.planar_velocity().x);
    }

    #[test]
    fn coincident_centers_approaching_velocity_produces_impulse() {
        let specs = BallSpecs::standard_pool();
        let same_pos = Vec2::new(1.0, 0.635);
        let mut a = Ball::new_stationary(BallId(0), specs, same_pos);
        let mut b = Ball::new_stationary(BallId(1), specs, same_pos);
        a.set_planar_velocity(Vec2::new(3.0, 0.0), 1e-4);

        let collided = resolve_ball_ball_collision(&mut a, &mut b, 1.0, 1e-4);
        assert!(collided);
        assert!(a.speed() < 1e-4);
        assert!((b.planar_velocity().x - 3.0).abs() < 1e-6);
        assert!(b.position.x > a.position.x);
    }

    #[test]
    fn adaptive_substeps_respects_max_cap() {
        let specs = BallSpecs::standard_pool();
        let mut fast_ball = Ball::new_stationary(BallId(0), specs, Vec2::new(0.5, 0.5));
        fast_ball.set_planar_velocity(Vec2::new(50.0, 0.0), 1e-4);

        let capped = compute_adaptive_substeps(&[fast_ball], 1.0 / 60.0, 8);
        assert_eq!(capped, 8);
        let min_cap = compute_adaptive_substeps(&[fast_ball], 1.0 / 60.0, 0);
        assert_eq!(min_cap, 1);
    }

    #[test]
    fn cushion_rebound_and_pocket_capture() {
        let table = Table::pocketless(TableSpecs::standard_9ft());
        let specs = BallSpecs::standard_pool();
        let mut ball = Ball::new_stationary(BallId(0), specs, Vec2::new(0.01, 0.6));
        ball.set_planar_velocity(Vec2::new(-3.0, 1.0), 1e-4);

        let mut events = Vec::new();
        resolve_ball_cushion_collisions(&mut ball, &table, 0.8, 1e-4, 0.0, &mut events);

        assert_eq!(events.len(), 1);
        assert_eq!(
            events[0].kind,
            CollisionKind::BallToCushion {
                ball: BallId(0),
                cushion_index: cushion_index::LEFT,
            }
        );
        assert!((ball.position.x - specs.radius).abs() < 1e-9);
        assert!((ball.planar_velocity().x - 2.4).abs() < 1e-6);
        assert!((ball.planar_velocity().y - 1.0).abs() < 1e-6);

        // Pocket capture test
        let pocket_table = Table::standard_9ft_six_pocket();
        let mut pocket_ball = Ball::new_stationary(BallId(2), specs, Vec2::new(0.02, 0.02));
        pocket_ball.set_planar_velocity(Vec2::new(-1.0, -1.0), 1e-4);
        let mut pocket_events = Vec::new();
        resolve_ball_cushion_collisions(
            &mut pocket_ball,
            &pocket_table,
            0.8,
            1e-4,
            0.0,
            &mut pocket_events,
        );
        assert!(pocket_ball.is_pocketed());
        assert_eq!(
            pocket_events[0].kind,
            CollisionKind::BallToPocket {
                ball: BallId(2),
                pocket_index: 0,
            }
        );
    }
}
