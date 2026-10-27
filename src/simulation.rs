//! Simulation state management, fixed-timestep stepping, Continuous Collision Detection (CCD),
//! and shot execution.
//!
//! Coordinates the [`crate::table::Table`] and collection of [`crate::ball::Ball`] entities,
//! advancing motion using a fixed timestep (`time_step`) that combines adaptive sub-stepping
//! with analytic earliest-Time-of-Impact (TOI) Continuous Collision Detection and multi-pass
//! impulse/overlap resolution. Completely decoupled from rendering frameworks, browser APIs,
//! and external game platforms.

use crate::ball::{Ball, BallId};
use crate::collision::{
    compute_adaptive_substeps, compute_ball_ball_toi, compute_ball_cushion_toi,
    compute_ball_pocket_toi, detect_ball_ball_contact, resolve_ball_cushion_collisions,
    resolve_ball_overlap, CollisionEvent, CollisionKind,
};
use crate::math::{clamp_scalar, sanitize_scalar, Scalar, Vec2, EPSILON};
use crate::table::Table;

/// Maximum number of earliest-TOI CCD sub-events processed within a single sub-step before
/// falling back to direct discrete resolution of any remaining duration.
const MAX_CCD_EVENTS_PER_SUBSTEP: usize = 16;

/// Number of Gauss-Seidel pairwise solver passes executed at each collision instant to
/// propagate impulses through touching multi-ball chains (e.g., Newton's cradle or tight racks).
const SOLVER_PASSES_PER_CONTACT: usize = 6;

/// Configuration parameters controlling simulation stepping, damping, restitution, and tolerances.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SimulationConfig {
    /// Fixed time step duration in seconds (default: `1.0 / 240.0`).
    pub time_step: Scalar,
    /// Constant planar linear deceleration (rolling/sliding resistance) in `m/s²` (default: `0.25`).
    pub linear_drag: Scalar,
    /// Constant angular deceleration in `rad/s²` (default: `0.50`).
    pub angular_drag: Scalar,
    /// Speed threshold in `m/s` at or below which a moving ball snaps to a full stop (default: `1e-3`).
    pub stop_speed_threshold: Scalar,
    /// Coefficient of restitution (`e` in `[0.0, 1.0]`) for ball-to-ball collisions (default: `0.94`).
    pub ball_restitution: Scalar,
    /// Coefficient of restitution (`e` in `[0.0, 1.0]`) for ball-to-cushion collisions (default: `0.80`).
    pub cushion_restitution: Scalar,
    /// Maximum allowed linear ball speed in `m/s` to guard against extreme inputs (default: `25.0`).
    pub max_ball_speed: Scalar,
    /// Maximum number of adaptive sub-steps per fixed step (default: `32`).
    pub max_substeps: u32,
}

impl SimulationConfig {
    /// Returns a sanitized copy of the configuration with valid positive time steps and clamped coefficients.
    #[must_use]
    pub fn sanitized(self) -> Self {
        let time_step = if self.time_step.is_finite() && self.time_step > EPSILON {
            self.time_step
        } else {
            1.0 / 240.0
        };
        let linear_drag = sanitize_scalar(self.linear_drag, 0.25).max(0.0);
        let angular_drag = sanitize_scalar(self.angular_drag, 0.50).max(0.0);
        // Allow zero stop_speed_threshold only up to a minimal floor (1e-7 m/s) to prevent infinite creep
        let stop_speed_threshold = sanitize_scalar(self.stop_speed_threshold, 1e-3).max(1e-7);
        let ball_restitution = if self.ball_restitution.is_finite() {
            self.ball_restitution.clamp(0.0, 1.0)
        } else {
            0.94
        };
        let cushion_restitution = if self.cushion_restitution.is_finite() {
            self.cushion_restitution.clamp(0.0, 1.0)
        } else {
            0.80
        };
        let max_ball_speed = if self.max_ball_speed.is_finite() && self.max_ball_speed > 0.1 {
            self.max_ball_speed
        } else {
            25.0
        };
        let max_substeps = self.max_substeps.clamp(1, 256);

        Self {
            time_step,
            linear_drag,
            angular_drag,
            stop_speed_threshold,
            ball_restitution,
            cushion_restitution,
            max_ball_speed,
            max_substeps,
        }
    }
}

impl Default for SimulationConfig {
    fn default() -> Self {
        Self {
            time_step: 1.0 / 240.0,
            linear_drag: 0.25,
            angular_drag: 0.50,
            stop_speed_threshold: 1e-3,
            ball_restitution: 0.94,
            cushion_restitution: 0.80,
            max_ball_speed: 25.0,
            max_substeps: 32,
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
    /// Balls tracked by the simulation instance (including active and pocketed balls).
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

    /// Looks up a ball by its logical [`BallId`].
    #[must_use]
    pub fn ball(&self, id: BallId) -> Option<&Ball> {
        self.balls.iter().find(|b| b.id == id)
    }

    /// Looks up a mutable reference to a ball by its logical [`BallId`].
    pub fn ball_mut(&mut self, id: BallId) -> Option<&mut Ball> {
        self.balls.iter_mut().find(|b| b.id == id)
    }

    /// Returns `true` if all balls on the table have come to a complete stop (`Stationary` or `Pocketed`).
    #[must_use]
    pub fn is_settled(&self) -> bool {
        self.balls.iter().all(|b| !b.is_moving())
    }

    /// Returns the number of balls currently on the table surface (not pocketed).
    #[must_use]
    pub fn active_ball_count(&self) -> usize {
        self.balls.iter().filter(|b| b.is_on_table()).count()
    }

    /// Returns the number of balls that have been pocketed.
    #[must_use]
    pub fn pocketed_ball_count(&self) -> usize {
        self.balls.iter().filter(|b| b.is_pocketed()).count()
    }

    /// Computes the total planar kinetic energy (`∑ ½ m v²`) of all active balls on the table in joules.
    #[must_use]
    pub fn total_kinetic_energy(&self) -> Scalar {
        self.balls
            .iter()
            .filter(|b| b.is_on_table())
            .map(Ball::kinetic_energy)
            .sum()
    }

    /// Computes the total planar linear momentum vector (`∑ m v`) of all active balls on the table in `kg·m/s`.
    #[must_use]
    pub fn total_linear_momentum(&self) -> Vec2 {
        let mut p = Vec2::ZERO;
        for ball in &self.balls {
            if ball.is_on_table() && ball.specs.is_valid() {
                p += ball.planar_velocity().scale(ball.specs.mass);
            }
        }
        p.sanitized()
    }

    /// Executes a cue strike on the target ball (`ball_id`) in the planar `direction` with
    /// the given linear impulse `force` (`N·s`, producing `Δv = force / mass`).
    ///
    /// Returns `true` if the target ball exists on the table and the shot parameters were valid.
    pub fn strike_ball(&mut self, ball_id: BallId, direction: Vec2, force: Scalar) -> bool {
        let cfg = self.config.sanitized();
        let Some(ball) = self.ball_mut(ball_id) else {
            return false;
        };
        ball.apply_impulse(
            direction,
            force,
            cfg.max_ball_speed,
            cfg.stop_speed_threshold,
        )
    }

    /// Directly assigns an initial planar shot speed (`speed` in `m/s`) along `direction`
    /// to the target ball (`ball_id`).
    ///
    /// Returns `true` if the ball exists on the table and was set into motion.
    pub fn strike_ball_with_speed(
        &mut self,
        ball_id: BallId,
        direction: Vec2,
        speed: Scalar,
    ) -> bool {
        let cfg = self.config.sanitized();
        let Some(unit_dir) = direction.try_normalize() else {
            return false;
        };
        if !speed.is_finite() || speed <= cfg.stop_speed_threshold {
            return false;
        }
        let clamped_speed = speed.min(cfg.max_ball_speed);
        let Some(ball) = self.ball_mut(ball_id) else {
            return false;
        };
        if !ball.is_on_table() {
            return false;
        }
        ball.set_planar_velocity(unit_dir.scale(clamped_speed), cfg.stop_speed_threshold);
        ball.is_moving()
    }

    /// Advances the simulation by a single fixed timestep (`self.config.sanitized().time_step`)
    /// and returns all collision and pocket events detected during the step.
    pub fn step(&mut self) -> Vec<CollisionEvent> {
        let cfg = self.config.sanitized();
        self.config = cfg;
        self.step_with_dt(cfg.time_step)
    }

    /// Advances the simulation by `steps` fixed timesteps, returning all collision events
    /// accumulated across the steps with `time_of_impact` offsets measured from the start
    /// of the `step_n` call.
    pub fn step_n(&mut self, steps: usize) -> Vec<CollisionEvent> {
        let mut all_events = Vec::new();
        self.config = self.config.sanitized();
        let step_dt = self.config.time_step;

        for step_idx in 0..steps {
            let time_base = (step_idx as Scalar) * step_dt;
            let mut step_events = self.step();
            for ev in &mut step_events {
                ev.time_of_impact += time_base;
            }
            all_events.extend(step_events);
        }
        all_events
    }

    /// Runs fixed timesteps up to `max_steps` until all balls come to rest (`is_settled()`),
    /// returning the number of steps executed and the list of all collision events with
    /// `time_of_impact` offsets measured from the start of the call.
    pub fn step_until_settled(&mut self, max_steps: usize) -> (usize, Vec<CollisionEvent>) {
        let mut all_events = Vec::new();
        let mut executed = 0;
        self.config = self.config.sanitized();
        let step_dt = self.config.time_step;

        for step_idx in 0..max_steps {
            if self.is_settled() {
                break;
            }
            let time_base = (step_idx as Scalar) * step_dt;
            let mut step_events = self.step();
            for ev in &mut step_events {
                ev.time_of_impact += time_base;
            }
            all_events.extend(step_events);
            executed += 1;
        }

        (executed, all_events)
    }

    /// Finds the earliest analytic Time of Impact (TOI) within `[0.0, max_dt]` across all
    /// ball-to-ball, ball-to-pocket, and ball-to-cushion pairs.
    #[must_use]
    fn find_earliest_toi(&self, max_dt: Scalar) -> Option<Scalar> {
        let mut earliest: Option<Scalar> = None;
        let count = self.balls.len();

        let mut update_min = |t: Scalar| {
            if (0.0..=max_dt).contains(&t) && earliest.map_or(true, |best| t < best) {
                earliest = Some(t);
            }
        };

        for i in 0..count {
            let ball_i = &self.balls[i];
            if !ball_i.is_on_table() {
                continue;
            }
            if let Some((t_pocket, _)) = compute_ball_pocket_toi(ball_i, &self.table, max_dt) {
                update_min(t_pocket);
            }
            if let Some((t_cushion, _)) = compute_ball_cushion_toi(ball_i, &self.table, max_dt) {
                update_min(t_cushion);
            }
            for j in (i + 1)..count {
                let ball_j = &self.balls[j];
                if !ball_j.is_on_table() {
                    continue;
                }
                if let Some(t_ball) = compute_ball_ball_toi(ball_i, ball_j, max_dt) {
                    update_min(t_ball);
                }
            }
        }

        earliest
    }

    /// Resolves all active pocket captures, cushion contacts, and multi-pass ball-to-ball
    /// collisions at the current simulation instant `current_time`.
    fn resolve_contacts_at_instant(
        &mut self,
        cfg: &SimulationConfig,
        current_time: Scalar,
        events: &mut Vec<CollisionEvent>,
    ) {
        // 1. Resolve pocket captures and cushion rebounds
        for ball in &mut self.balls {
            resolve_ball_cushion_collisions(
                ball,
                &self.table,
                cfg.cushion_restitution,
                cfg.stop_speed_threshold,
                current_time,
                events,
            );
        }

        // 2. Multi-pass coupled contact solver (Delassus operator / LCP) ensuring:
        //    - Exact linear momentum conservation (∑ m v = const).
        //    - Exact kinetic energy conservation in elastic collisions (e = 1.0, ∑ ½ m v² = const).
        //    - Complete symmetry in simultaneous multi-ball cluster collisions (e.g. cue ball hitting 2 balls).
        //    - Order-independence across permutations of balls in the array.
        //    - Clean sequential impulse transfer through touching chains (e.g. 3-ball Newton's cradle).
        let ball_count = self.balls.len();
        for _pass in 0..SOLVER_PASSES_PER_CONTACT {
            let mut any_resolved_in_pass = false;

            // Collect all contacting pairs that are approaching each other
            struct ApproachingContact {
                i_idx: usize,
                j_idx: usize,
                id_i: BallId,
                id_j: BallId,
                normal: Vec2, // Oriented from ball i to ball j
                v_rel_normal: Scalar,
            }

            let mut active_contacts = Vec::new();
            for i in 0..ball_count {
                for j in (i + 1)..ball_count {
                    let ball_a = &self.balls[i];
                    let ball_b = &self.balls[j];
                    if !ball_a.is_on_table() || !ball_b.is_on_table() {
                        continue;
                    }
                    if let Some(contact) = detect_ball_ball_contact(ball_a, ball_b) {
                        let inv_m_a = ball_a.specs.inverse_mass();
                        let inv_m_b = ball_b.specs.inverse_mass();
                        if inv_m_a + inv_m_b > EPSILON {
                            // Orient canonically from smaller BallId to larger BallId for order-independence
                            let (i_idx, j_idx, id_i, id_j, normal) = if ball_a.id < ball_b.id {
                                (i, j, ball_a.id, ball_b.id, contact.normal)
                            } else {
                                (j, i, ball_b.id, ball_a.id, -contact.normal)
                            };

                            let va = self.balls[i_idx].planar_velocity();
                            let vb = self.balls[j_idx].planar_velocity();
                            let relative_velocity = vb - va;
                            let v_rel_normal = relative_velocity.dot(normal);

                            if v_rel_normal < -EPSILON {
                                active_contacts.push(ApproachingContact {
                                    i_idx,
                                    j_idx,
                                    id_i,
                                    id_j,
                                    normal,
                                    v_rel_normal,
                                });
                            }
                        }
                    }
                }
            }

            // Sort contacts canonically by (id_i, id_j) so iteration order is completely independent
            // of how balls are ordered in self.balls
            active_contacts.sort_unstable_by_key(|c| (c.id_i, c.id_j));

            let m = active_contacts.len();
            if m > 0 {
                any_resolved_in_pass = true;
                let e = clamp_scalar(cfg.ball_restitution, 0.0, 1.0, 0.95);
                let mut lambdas = vec![0.0; m];

                if m == 1 {
                    let c = &active_contacts[0];
                    let inv_i = self.balls[c.i_idx].specs.inverse_mass();
                    let inv_j = self.balls[c.j_idx].specs.inverse_mass();
                    let total_inv = inv_i + inv_j;
                    if total_inv > EPSILON {
                        lambdas[0] = -(1.0 + e) * c.v_rel_normal / total_inv;
                    }
                } else if m == 2 {
                    // Analytical solution of the 2x2 coupled Delassus system
                    let c0 = &active_contacts[0];
                    let c1 = &active_contacts[1];

                    let inv_i0 = self.balls[c0.i_idx].specs.inverse_mass();
                    let inv_j0 = self.balls[c0.j_idx].specs.inverse_mass();
                    let a00 = inv_i0 + inv_j0;

                    let inv_i1 = self.balls[c1.i_idx].specs.inverse_mass();
                    let inv_j1 = self.balls[c1.j_idx].specs.inverse_mass();
                    let a11 = inv_i1 + inv_j1;

                    // Compute coupling cross-term A[0][1] = A[1][0]
                    let dot = c1.normal.dot(c0.normal);
                    let mut a01 = 0.0;
                    if c0.j_idx == c1.j_idx {
                        a01 += inv_j0 * dot;
                    }
                    if c0.j_idx == c1.i_idx {
                        a01 -= inv_j0 * dot;
                    }
                    if c0.i_idx == c1.j_idx {
                        a01 -= inv_i0 * dot;
                    }
                    if c0.i_idx == c1.i_idx {
                        a01 += inv_i0 * dot;
                    }
                    let a10 = a01;

                    let b0 = -(1.0 + e) * c0.v_rel_normal;
                    let b1 = -(1.0 + e) * c1.v_rel_normal;

                    let det = a00 * a11 - a01 * a10;
                    if det > EPSILON {
                        let l0 = (a11 * b0 - a01 * b1) / det;
                        let l1 = (-a10 * b0 + a00 * b1) / det;
                        if l0 >= 0.0 && l1 >= 0.0 {
                            lambdas[0] = l0;
                            lambdas[1] = l1;
                        } else if l0 < 0.0 && l1 >= 0.0 {
                            lambdas[0] = 0.0;
                            lambdas[1] = (b1 / a11).max(0.0);
                        } else if l1 < 0.0 && l0 >= 0.0 {
                            lambdas[0] = (b0 / a00).max(0.0);
                            lambdas[1] = 0.0;
                        }
                    } else if a00 > EPSILON && a11 > EPSILON {
                        lambdas[0] = (b0 / a00).max(0.0);
                        lambdas[1] = (b1 / a11).max(0.0);
                    }
                } else {
                    // General M-contact Projected Gauss-Seidel solver for M >= 3
                    let mut a_diag = vec![0.0; m];
                    let mut a_matrix = vec![0.0; m * m];
                    let mut b_vec = vec![0.0; m];

                    for r in 0..m {
                        let cr = &active_contacts[r];
                        let inv_ir = self.balls[cr.i_idx].specs.inverse_mass();
                        let inv_jr = self.balls[cr.j_idx].specs.inverse_mass();
                        a_diag[r] = inv_ir + inv_jr;
                        a_matrix[r * m + r] = a_diag[r];
                        b_vec[r] = -(1.0 + e) * cr.v_rel_normal;

                        for col in (r + 1)..m {
                            let c_col = &active_contacts[col];
                            let dot = c_col.normal.dot(cr.normal);
                            let mut cross = 0.0;
                            if cr.j_idx == c_col.j_idx {
                                cross += inv_jr * dot;
                            }
                            if cr.j_idx == c_col.i_idx {
                                cross -= inv_jr * dot;
                            }
                            if cr.i_idx == c_col.j_idx {
                                cross -= inv_ir * dot;
                            }
                            if cr.i_idx == c_col.i_idx {
                                cross += inv_ir * dot;
                            }
                            a_matrix[r * m + col] = cross;
                            a_matrix[col * m + r] = cross;
                        }
                    }

                    for _ in 0..50 {
                        let mut max_diff: Scalar = 0.0;
                        for k in 0..m {
                            if a_diag[k] <= EPSILON {
                                continue;
                            }
                            let mut sum_other = 0.0;
                            for j in 0..m {
                                if j != k {
                                    sum_other += a_matrix[k * m + j] * lambdas[j];
                                }
                            }
                            let new_val = ((b_vec[k] - sum_other) / a_diag[k]).max(0.0);
                            let diff = (new_val - lambdas[k]).abs();
                            if diff > max_diff {
                                max_diff = diff;
                            }
                            lambdas[k] = new_val;
                        }
                        if max_diff < 1e-12 {
                            break;
                        }
                    }
                }

                // Apply computed normal impulses to the balls
                let mut delta_v = vec![Vec2::ZERO; ball_count];
                for (k, c) in active_contacts.iter().enumerate() {
                    let lambda = lambdas[k];
                    if lambda > 1e-9 {
                        let imp = c.normal.scale(lambda);
                        let inv_i = self.balls[c.i_idx].specs.inverse_mass();
                        let inv_j = self.balls[c.j_idx].specs.inverse_mass();
                        delta_v[c.i_idx] -= imp.scale(inv_i);
                        delta_v[c.j_idx] += imp.scale(inv_j);

                        let id_a = c.id_i;
                        let id_b = c.id_j;
                        let duplicate = events.iter().rev().take(ball_count * 2).any(|ev| {
                            (ev.time_of_impact - current_time).abs() <= 1e-7
                                && matches!(
                                    ev.kind,
                                    CollisionKind::BallToBall { first, second }
                                        if (first == id_a && second == id_b)
                                            || (first == id_b && second == id_a)
                                )
                        });
                        if !duplicate {
                            events.push(CollisionEvent {
                                time_of_impact: current_time,
                                kind: CollisionKind::BallToBall {
                                    first: id_a,
                                    second: id_b,
                                },
                            });
                        }
                    }
                }

                for (i, &dv) in delta_v.iter().enumerate().take(ball_count) {
                    if dv != Vec2::ZERO {
                        let new_v = (self.balls[i].planar_velocity() + dv).sanitized();
                        self.balls[i].set_planar_velocity(new_v, cfg.stop_speed_threshold);
                    }
                }
            }

            // Separate geometric overlaps beyond CONTACT_TOLERANCE canonically
            let mut overlap_pairs = Vec::new();
            for i in 0..ball_count {
                for j in (i + 1)..ball_count {
                    if detect_ball_ball_contact(&self.balls[i], &self.balls[j]).is_some() {
                        let id_a = self.balls[i].id;
                        let id_b = self.balls[j].id;
                        let (first, second) = if id_a < id_b { (i, j) } else { (j, i) };
                        overlap_pairs.push((
                            self.balls[first].id,
                            self.balls[second].id,
                            first,
                            second,
                        ));
                    }
                }
            }
            overlap_pairs.sort_unstable_by_key(|&(id_a, id_b, _, _)| (id_a, id_b));
            for (_, _, first, second) in overlap_pairs {
                let (left_idx, right_idx) = if first < second {
                    (first, second)
                } else {
                    (second, first)
                };
                let (left, right) = self.balls.split_at_mut(right_idx);
                if resolve_ball_overlap(&mut left[left_idx], &mut right[0]) {
                    any_resolved_in_pass = true;
                }
            }

            if !any_resolved_in_pass {
                break;
            }
        }

        // 3. Re-verify cushion/pocket boundaries after ball-to-ball separation
        for ball in &mut self.balls {
            resolve_ball_cushion_collisions(
                ball,
                &self.table,
                cfg.cushion_restitution,
                cfg.stop_speed_threshold,
                current_time,
                events,
            );
            ball.sanitize_and_sync_state(cfg.stop_speed_threshold);
        }
    }

    /// Internal stepper advancing the simulation over an arbitrary positive duration `dt`
    /// using adaptive sub-stepping combined with earliest-TOI Continuous Collision Detection (CCD).
    fn step_with_dt(&mut self, dt: Scalar) -> Vec<CollisionEvent> {
        let mut events = Vec::new();
        if !dt.is_finite() || dt <= EPSILON {
            return events;
        }

        let cfg = self.config.sanitized();

        // Sanitize all balls prior to stepping
        for ball in &mut self.balls {
            ball.sanitize_and_sync_state(cfg.stop_speed_threshold);
            if ball.speed() > cfg.max_ball_speed {
                let clamped = ball.planar_velocity().clamp_length(cfg.max_ball_speed);
                ball.set_planar_velocity(clamped, cfg.stop_speed_threshold);
            }
        }

        let substeps = compute_adaptive_substeps(&self.balls, dt, cfg.max_substeps);
        let sub_dt = dt / Scalar::from(substeps);

        for sub_idx in 0..substeps {
            let sub_start_time = Scalar::from(sub_idx) * sub_dt;
            let mut remaining = sub_dt;
            let mut elapsed_in_sub = 0.0;
            let mut ccd_iter = 0;

            while remaining > EPSILON && ccd_iter < MAX_CCD_EVENTS_PER_SUBSTEP {
                ccd_iter += 1;

                if let Some(toi) = self.find_earliest_toi(remaining) {
                    // Advance motion up to the exact Time of Impact (plus a tiny 1e-9 s nudge if toi > 0)
                    let advance_dt = if toi > 0.0 {
                        (toi + 1e-9).min(remaining)
                    } else {
                        0.0
                    };

                    if advance_dt > 0.0 {
                        for ball in &mut self.balls {
                            ball.integrate_motion(
                                advance_dt,
                                cfg.linear_drag,
                                cfg.angular_drag,
                                cfg.stop_speed_threshold,
                            );
                        }
                        elapsed_in_sub += advance_dt;
                        remaining = (remaining - advance_dt).max(0.0);
                    }

                    let event_time = sub_start_time + elapsed_in_sub;
                    let events_before = events.len();
                    self.resolve_contacts_at_instant(&cfg, event_time, &mut events);

                    // If toi == 0.0 and no new contact was resolved, advance by a small slice to guarantee progress
                    if advance_dt == 0.0 && events.len() == events_before {
                        let fallback_slice = remaining.min(sub_dt * 0.25).max(1e-7);
                        for ball in &mut self.balls {
                            ball.integrate_motion(
                                fallback_slice,
                                cfg.linear_drag,
                                cfg.angular_drag,
                                cfg.stop_speed_threshold,
                            );
                        }
                        elapsed_in_sub += fallback_slice;
                        remaining = (remaining - fallback_slice).max(0.0);
                        self.resolve_contacts_at_instant(
                            &cfg,
                            sub_start_time + elapsed_in_sub,
                            &mut events,
                        );
                    }
                } else {
                    // No collision within `remaining`; integrate the rest of the sub-step directly
                    for ball in &mut self.balls {
                        ball.integrate_motion(
                            remaining,
                            cfg.linear_drag,
                            cfg.angular_drag,
                            cfg.stop_speed_threshold,
                        );
                    }
                    elapsed_in_sub += remaining;
                    remaining = 0.0;
                    self.resolve_contacts_at_instant(
                        &cfg,
                        sub_start_time + elapsed_in_sub,
                        &mut events,
                    );
                }
            }

            // If the per-substep CCD iteration cap was reached with time still remaining, integrate it cleanly
            if remaining > EPSILON {
                for ball in &mut self.balls {
                    ball.integrate_motion(
                        remaining,
                        cfg.linear_drag,
                        cfg.angular_drag,
                        cfg.stop_speed_threshold,
                    );
                }
                self.resolve_contacts_at_instant(&cfg, sub_start_time + sub_dt, &mut events);
            }
        }

        events
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ball::{BallSpecs, BallState};

    #[test]
    fn strike_and_settle_single_ball() {
        let config = SimulationConfig::default();
        let table = Table::pocketless(crate::table::TableSpecs::standard_9ft());
        let cue = Ball::new_stationary(BallId(0), BallSpecs::standard_pool(), Vec2::new(0.5, 0.6));
        let mut world = SimulationWorld::new(config, table, vec![cue]);

        assert!(world.is_settled());
        assert!(world.strike_ball(BallId(0), Vec2::UNIT_X, 0.17)); // ~1.0 m/s
        assert!(!world.is_settled());

        let (steps, _events) = world.step_until_settled(2000);
        assert!(steps > 0 && steps < 2000);
        assert!(world.is_settled());
        let b = world.ball(BallId(0)).unwrap();
        assert_eq!(b.state, BallState::Stationary);
        assert!(b.position.x > 0.5);
    }

    #[test]
    fn ccd_prevents_tunneling_even_when_max_substeps_is_one() {
        let config = SimulationConfig {
            time_step: 1.0 / 30.0, // Coarse 33.3 ms step
            max_ball_speed: 100.0,
            max_substeps: 1, // Force single substep so only analytic CCD can catch the impact
            linear_drag: 0.0,
            ball_restitution: 1.0,
            ..SimulationConfig::default()
        };
        let table = Table::pocketless(crate::table::TableSpecs::standard_9ft());
        let specs = BallSpecs::standard_pool();

        let cue = Ball::new_stationary(BallId(0), specs, Vec2::new(0.3, 0.635));
        let target = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0, 0.635));
        let mut world = SimulationWorld::new(config, table, vec![cue, target]);

        // At 60 m/s over 1/30 s, displacement is 2.0 m (> 34x ball diameter!)
        assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 60.0));
        let events = world.step();

        assert!(events.iter().any(|e| matches!(
            e.kind,
            CollisionKind::BallToBall {
                first: BallId(0),
                second: BallId(1),
            }
        )));
        assert!(world.ball(BallId(1)).unwrap().planar_velocity().x > 10.0);
    }

    #[test]
    fn invalid_time_step_config_produces_finite_event_offsets() {
        let invalid_config = SimulationConfig {
            time_step: Scalar::NAN,
            ..SimulationConfig::default()
        };
        let table = Table::pocketless(crate::table::TableSpecs::standard_9ft());
        let specs = BallSpecs::standard_pool();
        let cue = Ball::new_stationary(BallId(0), specs, Vec2::new(0.05, 0.635));
        let mut world = SimulationWorld::new(invalid_config, table, vec![cue]);

        assert!(world.strike_ball_with_speed(BallId(0), Vec2::new(-1.0, 0.0), 4.0));
        let events = world.step_n(5);
        assert!(!events.is_empty());
        for ev in &events {
            assert!(ev.time_of_impact.is_finite());
            assert!(ev.time_of_impact > 0.0);
        }
    }
}
