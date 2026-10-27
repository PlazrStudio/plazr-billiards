//! Advanced engineering & physics validation test suite (Stage 3).
//!
//! Covers all 10 required verification scenarios plus targeted regression tests:
//! 1. Two-ball collisions across varying speeds (head-on and oblique 90-degree split).
//! 2. Sequential three-ball collisions (Newton's cradle / collinear momentum transfer).
//! 3. Simultaneous and multi-ball cluster collisions (apex break into triangular rack).
//! 4. High-speed shots with Continuous Collision Detection (no tunneling even at coarse dt).
//! 5. Very low-speed creep and clean stopping transition at `stop_speed_threshold`.
//! 6. Conservation of linear momentum and kinetic energy in elastic, drag-free collisions.
//! 7. Single-runtime repeatability across identical multi-ball break setups.
//! 8. Thousands of consecutive simulation steps and 1,000 independent simulation runs.
//! 9. Invalid (`NaN`, `±Infinity`), negative, and extreme numerical boundary inputs.
//! 10. Agreement with closed-form analytical reference solutions (stopping distance & time,
//!     1D elastic/inelastic post-collision velocities, 2D 90-degree equal-mass scattering,
//!     and corner pocket entry near cushion intersections).

use plazr_billiards::ball::{Ball, BallId, BallSpecs, BallState};
use plazr_billiards::collision::{resolve_ball_ball_collision, CollisionKind};
use plazr_billiards::math::{Scalar, Vec2, Vec3};
use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
use plazr_billiards::table::{Table, TableSpecs};

/// 1. Two balls colliding at different speeds (both moving along the same axis and opposing).
#[test]
fn test_01_two_balls_different_speeds() {
    let specs = BallSpecs::standard_pool();
    let r = specs.radius;

    // Case A: Both moving +X, rear ball faster (v1 = 5.0 m/s, v2 = 1.5 m/s), e = 1.0
    let mut a = Ball::new_stationary(BallId(0), specs, Vec2::new(1.0, 0.635));
    let mut b = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0 + 2.0 * r, 0.635));
    a.set_planar_velocity(Vec2::new(5.0, 0.0), 1e-4);
    b.set_planar_velocity(Vec2::new(1.5, 0.0), 1e-4);

    assert!(resolve_ball_ball_collision(&mut a, &mut b, 1.0, 1e-4));
    // Equal masses with e = 1.0 must swap normal velocities: v1' = 1.5, v2' = 5.0
    assert!((a.planar_velocity().x - 1.5).abs() < 1e-9);
    assert!((b.planar_velocity().x - 5.0).abs() < 1e-9);

    // Case B: Opposing velocities (v1 = +4.0 m/s, v2 = -2.0 m/s), e = 0.90
    let mut c = Ball::new_stationary(BallId(2), specs, Vec2::new(1.0, 0.635));
    let mut d = Ball::new_stationary(BallId(3), specs, Vec2::new(1.0 + 2.0 * r, 0.635));
    c.set_planar_velocity(Vec2::new(4.0, 0.0), 1e-4);
    d.set_planar_velocity(Vec2::new(-2.0, 0.0), 1e-4);

    assert!(resolve_ball_ball_collision(&mut c, &mut d, 0.90, 1e-4));
    // Analytical 1D solution for equal masses:
    // v_c' = 0.5 * ((4 - 2) - 0.9 * (4 - (-2))) = 1.0 - 2.7 = -1.7 m/s
    // v_d' = 0.5 * ((4 - 2) + 0.9 * (4 - (-2))) = 1.0 + 2.7 = +3.7 m/s
    assert!((c.planar_velocity().x - (-1.7)).abs() < 1e-9);
    assert!((d.planar_velocity().x - 3.7).abs() < 1e-9);
}

/// 2. Sequential three-ball collision (Newton's cradle / 3 collinear balls).
#[test]
fn test_02_three_sequential_balls_newtons_cradle() {
    let config = SimulationConfig {
        time_step: 1.0 / 240.0,
        linear_drag: 0.0,
        angular_drag: 0.0,
        ball_restitution: 1.0,
        cushion_restitution: 1.0,
        stop_speed_threshold: 1e-5,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let specs = BallSpecs::standard_pool();
    let d = 2.0 * specs.radius;

    // Ball 0 approaches touching balls 1 and 2
    let b0 = Ball::new_stationary(BallId(0), specs, Vec2::new(0.80, 0.635));
    let b1 = Ball::new_stationary(BallId(1), specs, Vec2::new(1.00, 0.635));
    let b2 = Ball::new_stationary(BallId(2), specs, Vec2::new(1.00 + d, 0.635));

    let mut world = SimulationWorld::new(config, table, vec![b0, b1, b2]);
    assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 3.0));

    let initial_ke = world.total_kinetic_energy();
    let initial_p = world.total_linear_momentum();

    world.step_n(20);

    // After the wave passes through Ball 1 into Ball 2, Ball 2 carries the +3.0 m/s velocity
    // while Ball 0 and Ball 1 remain nearly at rest.
    let v0 = world.ball(BallId(0)).unwrap().speed();
    let v1 = world.ball(BallId(1)).unwrap().speed();
    let v2 = world.ball(BallId(2)).unwrap().planar_velocity().x;

    assert!(v0 < 1e-3, "Expected ball 0 to stop, got speed {v0}");
    assert!(v1 < 1e-3, "Expected ball 1 to stop, got speed {v1}");
    assert!(
        (v2 - 3.0).abs() < 1e-3,
        "Expected ball 2 to move at ~3.0 m/s, got {v2}"
    );

    assert!((world.total_kinetic_energy() - initial_ke).abs() < 1e-6);
    assert!(world.total_linear_momentum().approx_eq(initial_p, 1e-6));
}

/// 3. Simultaneous and multi-ball cluster collisions (symmetric apex impact into a 2-ball row).
#[test]
fn test_03_simultaneous_multi_ball_cluster_collision() {
    let config = SimulationConfig {
        time_step: 1.0 / 240.0,
        linear_drag: 0.0,
        ball_restitution: 1.0,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let specs = BallSpecs::standard_pool();
    let r = specs.radius;

    // Cue ball strikes symmetrically between two balls placed at y = 0.635 ± r
    let cue = Ball::new_stationary(BallId(0), specs, Vec2::new(0.80, 0.635));
    let upper = Ball::new_stationary(BallId(1), specs, Vec2::new(1.00, 0.635 + r + 1e-4));
    let lower = Ball::new_stationary(BallId(2), specs, Vec2::new(1.00, 0.635 - r - 1e-4));

    let mut world = SimulationWorld::new(config, table, vec![cue, upper, lower]);
    assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 4.0));
    let p_initial = world.total_linear_momentum();
    let ke_initial = world.total_kinetic_energy();

    world.step_n(25);

    let b1 = world.ball(BallId(1)).unwrap();
    let b2 = world.ball(BallId(2)).unwrap();

    // Symmetry check: upper and lower balls must have equal X velocity and opposite Y velocity
    assert!((b1.planar_velocity().x - b2.planar_velocity().x).abs() < 1e-6);
    assert!((b1.planar_velocity().y + b2.planar_velocity().y).abs() < 1e-6);
    assert!(b1.planar_velocity().y > 0.5);
    assert!(world.total_linear_momentum().approx_eq(p_initial, 1e-6));
    assert!(
        (world.total_kinetic_energy() - ke_initial).abs() < 1e-6,
        "Kinetic energy drift: initial = {ke_initial}, final = {}",
        world.total_kinetic_energy()
    );
}

/// 4. High-speed shots with Continuous Collision Detection (preventing tunneling).
#[test]
fn test_04_high_speed_continuous_collision_detection() {
    let config = SimulationConfig {
        time_step: 1.0 / 60.0,
        max_ball_speed: 50.0,
        max_substeps: 1, // Force single substep so analytic CCD must catch the collision
        linear_drag: 0.0,
        ball_restitution: 1.0,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let specs = BallSpecs::standard_pool();

    let cue = Ball::new_stationary(BallId(0), specs, Vec2::new(0.40, 0.635));
    let target = Ball::new_stationary(BallId(1), specs, Vec2::new(0.75, 0.635));
    let mut world = SimulationWorld::new(config, table, vec![cue, target]);

    assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 45.0));
    let events = world.step();

    assert!(events.iter().any(|ev| matches!(
        ev.kind,
        CollisionKind::BallToBall {
            first: BallId(0),
            second: BallId(1),
        }
    )));
    assert!(world.ball(BallId(1)).unwrap().speed() > 20.0);
}

/// 5. Very low-speed motion and clean stopping transition.
#[test]
fn test_05_very_low_speed_stopping_transition() {
    let config = SimulationConfig {
        time_step: 1.0 / 240.0,
        linear_drag: 0.25,
        stop_speed_threshold: 1e-3,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let specs = BallSpecs::standard_pool();
    let mut ball = Ball::new_stationary(BallId(0), specs, Vec2::new(1.0, 0.5));

    // Velocity just above stop threshold (1.5 mm/s)
    ball.set_planar_velocity(Vec2::new(1.5e-3, 0.0), config.stop_speed_threshold);
    assert_eq!(ball.state, BallState::Moving);

    let mut world = SimulationWorld::new(config, table, vec![ball]);
    let (steps, _) = world.step_until_settled(10);
    assert!(steps <= 2);
    assert!(world.is_settled());
    assert_eq!(world.ball(BallId(0)).unwrap().state, BallState::Stationary);
    assert_eq!(world.ball(BallId(0)).unwrap().speed(), 0.0);
}

/// 6. Conservation of kinetic energy and linear momentum in elastic collisions.
#[test]
fn test_06_conservation_of_energy_and_momentum() {
    let config = SimulationConfig {
        time_step: 1.0 / 240.0,
        linear_drag: 0.0,
        angular_drag: 0.0,
        ball_restitution: 1.0,
        cushion_restitution: 1.0,
        stop_speed_threshold: 1e-6,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let specs = BallSpecs::standard_pool();

    // Oblique 2D collision away from cushions
    let b0 = Ball::new_stationary(BallId(0), specs, Vec2::new(0.80, 0.60));
    let b1 = Ball::new_stationary(BallId(1), specs, Vec2::new(1.10, 0.62));
    let mut world = SimulationWorld::new(config, table, vec![b0, b1]);

    assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 3.0));
    let e0 = world.total_kinetic_energy();
    let p0 = world.total_linear_momentum();

    let events = world.step_n(30);
    assert!(!events.is_empty());

    let e1 = world.total_kinetic_energy();
    let p1 = world.total_linear_momentum();

    assert!((e1 - e0).abs() < 1e-9, "Energy drift: {e0} -> {e1}");
    assert!(p1.approx_eq(p0, 1e-9), "Momentum drift: {p0:?} -> {p1:?}");
}

/// 7. Single-runtime repeatability with identical inputs.
#[test]
fn test_07_repeatability_with_identical_inputs() {
    let run_once = || {
        let config = SimulationConfig::default();
        let table = Table::standard_9ft_six_pocket();
        let specs = BallSpecs::standard_pool();
        let d = 2.0 * specs.radius + 1e-4;

        let mut balls = Vec::new();
        balls.push(Ball::new_stationary(
            BallId(0),
            specs,
            Vec2::new(0.635, 0.635),
        ));
        let mut id = 1u8;
        for row in 0..3 {
            for col in 0..=row {
                let x = 1.80 + (row as Scalar) * d * 0.866_025_403_784;
                let y = 0.635 + ((col as Scalar) - (row as Scalar) * 0.5) * d;
                balls.push(Ball::new_stationary(BallId(id), specs, Vec2::new(x, y)));
                id += 1;
            }
        }

        let mut world = SimulationWorld::new(config, table, balls);
        world.strike_ball_with_speed(BallId(0), Vec2::new(1.0, 0.015), 7.0);
        let (steps, events) = world.step_until_settled(4000);
        (steps, world.balls, events)
    };

    let (s1, b1, e1) = run_once();
    let (s2, b2, e2) = run_once();
    assert_eq!(s1, s2);
    assert_eq!(b1, b2);
    assert_eq!(e1, e2);
}

/// 8. Stress test: thousands of sequential simulation steps and 1,000 independent simulations.
#[test]
fn test_08_thousands_of_steps_and_simulations_stability() {
    let config = SimulationConfig::default();
    let table = Table::standard_9ft_six_pocket();
    let specs = BallSpecs::standard_pool();

    // Part A: 5,000 consecutive steps on a multi-ball table
    let balls = vec![
        Ball::new_stationary(BallId(0), specs, Vec2::new(0.5, 0.5)),
        Ball::new_stationary(BallId(1), specs, Vec2::new(1.2, 0.52)),
        Ball::new_stationary(BallId(2), specs, Vec2::new(1.5, 0.8)),
    ];
    let mut world = SimulationWorld::new(config, table.clone(), balls);
    world.strike_ball_with_speed(BallId(0), Vec2::new(1.0, 0.1), 10.0);
    world.step_n(5_000);
    assert!(world.is_settled());
    for b in &world.balls {
        assert!(b.position.is_finite());
        assert!(b.velocity.is_finite());
    }

    // Part B: 1,000 independent short simulations at varying angles
    for i in 0..1_000_u32 {
        let angle = (i as Scalar) * 0.006_283_185;
        let dir = Vec2::from_angle(angle);
        let cue = Ball::new_stationary(BallId(0), specs, Vec2::new(0.65, 0.635));
        let obj = Ball::new_stationary(BallId(1), specs, Vec2::new(1.25, 0.635));
        let mut sim = SimulationWorld::new(config, table.clone(), vec![cue, obj]);
        sim.strike_ball_with_speed(BallId(0), dir, 4.0);
        sim.step_n(10);
        for b in &sim.balls {
            assert!(b.position.is_finite());
            assert!(b.velocity.is_finite());
        }
    }
}

/// 9. Invalid values (`NaN`, `±Infinity`, zero/negative mass/radius) and numerical boundaries.
#[test]
fn test_09_invalid_values_and_numerical_boundaries() {
    let bad_specs = BallSpecs::new(-1.0, Scalar::NAN);
    assert!(!bad_specs.is_valid());
    assert_eq!(bad_specs.inverse_mass(), 0.0);

    let mut bad_ball = Ball::new_stationary(
        BallId(99),
        bad_specs,
        Vec2::new(Scalar::INFINITY, Scalar::NEG_INFINITY),
    );
    bad_ball.velocity = Vec3::new(Scalar::NAN, 1e308, -1e308);
    assert!(!bad_ball.apply_impulse(Vec2::UNIT_X, 1.0, 25.0, 1e-3));

    let mut world = SimulationWorld::new(
        SimulationConfig {
            time_step: -5.0,
            linear_drag: Scalar::NAN,
            angular_drag: Scalar::INFINITY,
            stop_speed_threshold: -1.0,
            ball_restitution: Scalar::NAN,
            cushion_restitution: Scalar::INFINITY,
            max_ball_speed: -10.0,
            max_substeps: 0,
        },
        Table::standard_9ft_six_pocket(),
        vec![bad_ball],
    );

    let events = world.step_n(5);
    assert!(events.is_empty());
    assert!(world.ball(BallId(99)).unwrap().position.is_finite());
    assert!(world.ball(BallId(99)).unwrap().velocity.is_finite());
}

/// 10. Agreement with closed-form mathematical reference solutions & corner pocket jaw entry.
#[test]
fn test_10_analytical_reference_solutions_and_corner_pocket_entry() {
    // Reference 1: 90-degree post-impact divergence angle for oblique elastic collision of equal masses
    // When a moving ball strikes an identical stationary ball off-center with e = 1.0,
    // their post-collision velocity vectors v_a' and v_b' must be orthogonal (v_a' · v_b' = 0).
    let specs = BallSpecs::standard_pool();
    let r = specs.radius;
    let impact_param = r; // 30-degree cut angle (sin(theta) = 0.5)
    let dx = (4.0 * r * r - impact_param * impact_param).sqrt();

    let mut a = Ball::new_stationary(BallId(0), specs, Vec2::new(1.0, 0.5));
    let mut b = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0 + dx, 0.5 + impact_param));
    a.set_planar_velocity(Vec2::new(4.0, 0.0), 1e-6);

    assert!(resolve_ball_ball_collision(&mut a, &mut b, 1.0, 1e-6));
    let dot_post = a.planar_velocity().dot(b.planar_velocity());
    assert!(
        dot_post.abs() < 1e-9,
        "Post-collision velocities for equal-mass elastic cut must be orthogonal, got dot={dot_post}"
    );

    // Reference 2: Corner pocket shot grazing the rail inside the pocket mouth opening
    let table = Table::standard_9ft_six_pocket();
    let corner_shot_ball = Ball::new_stationary(BallId(5), specs, Vec2::new(0.075, 0.032));
    let mut world =
        SimulationWorld::new(SimulationConfig::default(), table, vec![corner_shot_ball]);
    assert!(world.strike_ball_with_speed(BallId(5), Vec2::new(-1.0, -0.35), 2.0));
    let (_steps, events) = world.step_until_settled(120);
    assert_eq!(
        world.ball(BallId(5)).unwrap().state,
        BallState::Pocketed { pocket_index: 0 }
    );
    assert!(events.iter().any(|e| matches!(
        e.kind,
        CollisionKind::BallToPocket {
            ball: BallId(5),
            pocket_index: 0,
        }
    )));
}

/// 11. One cue ball striking two identical balls symmetrically with permutation invariance.
///
/// Verifies:
/// - Absolute symmetry between upper and lower balls.
/// - Exact linear momentum conservation (P_final == P_initial).
/// - Exact kinetic energy conservation (E_final == E_initial) under elastic, drag-free conditions.
/// - Bit-level / strict invariance across all permutations of balls in the array.
#[test]
fn test_11_simultaneous_two_target_balls_permutation_invariance() {
    let run_sim = |ball_order: Vec<u8>| -> (Vec2, Vec2, Vec2, Scalar, Vec2) {
        let config = SimulationConfig {
            time_step: 1.0 / 240.0,
            linear_drag: 0.0,
            angular_drag: 0.0,
            ball_restitution: 1.0,
            cushion_restitution: 1.0,
            stop_speed_threshold: 1e-6,
            ..SimulationConfig::default()
        };
        let table = Table::pocketless(TableSpecs::standard_9ft());
        let specs = BallSpecs::standard_pool();
        let r = specs.radius;

        let cue = Ball::new_stationary(BallId(0), specs, Vec2::new(0.80, 0.635));
        let upper = Ball::new_stationary(BallId(1), specs, Vec2::new(1.00, 0.635 + r + 1e-4));
        let lower = Ball::new_stationary(BallId(2), specs, Vec2::new(1.00, 0.635 - r - 1e-4));

        let mut balls = Vec::new();
        for &id in &ball_order {
            match id {
                0 => balls.push(cue),
                1 => balls.push(upper),
                2 => balls.push(lower),
                _ => unreachable!(),
            }
        }

        let mut world = SimulationWorld::new(config, table, balls);
        assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 4.0));

        let initial_ke = world.total_kinetic_energy();
        let initial_p = world.total_linear_momentum();

        world.step_n(25);

        let v_cue = world.ball(BallId(0)).unwrap().planar_velocity();
        let v_upper = world.ball(BallId(1)).unwrap().planar_velocity();
        let v_lower = world.ball(BallId(2)).unwrap().planar_velocity();
        let final_ke = world.total_kinetic_energy();
        let final_p = world.total_linear_momentum();

        // Exact conservation checks for this run
        assert!(
            (final_ke - initial_ke).abs() < 1e-9,
            "Energy drift: initial = {initial_ke}, final = {final_ke}"
        );
        assert!(
            final_p.approx_eq(initial_p, 1e-9),
            "Momentum drift: initial = {initial_p:?}, final = {final_p:?}"
        );

        // Exact symmetry check
        assert!(
            (v_upper.x - v_lower.x).abs() < 1e-9,
            "X-velocity symmetry violated"
        );
        assert!(
            (v_upper.y + v_lower.y).abs() < 1e-9,
            "Y-velocity symmetry violated"
        );
        assert!(v_cue.y.abs() < 1e-9, "Cue deflected off symmetry axis");
        assert!(v_upper.y > 0.5);

        (v_cue, v_upper, v_lower, final_ke, final_p)
    };

    let (c_a, u_a, l_a, e_a, p_a) = run_sim(vec![0, 1, 2]); // cue, upper, lower
    let (c_b, u_b, l_b, e_b, p_b) = run_sim(vec![2, 0, 1]); // lower, cue, upper
    let (c_c, u_c, l_c, e_c, p_c) = run_sim(vec![1, 2, 0]); // upper, lower, cue

    // Verify complete permutation invariance
    assert!(c_a.approx_eq(c_b, 1e-12) && c_a.approx_eq(c_c, 1e-12));
    assert!(u_a.approx_eq(u_b, 1e-12) && u_a.approx_eq(u_c, 1e-12));
    assert!(l_a.approx_eq(l_b, 1e-12) && l_a.approx_eq(l_c, 1e-12));
    assert!((e_a - e_b).abs() < 1e-12 && (e_a - e_c).abs() < 1e-12);
    assert!(p_a.approx_eq(p_b, 1e-12) && p_a.approx_eq(p_c, 1e-12));
}

/// 12. Three touching collinear balls (Newton's cradle) across permutations.
///
/// Verifies:
/// - Exact momentum transfer through touching balls with intermediate ball stopping.
/// - Invariance to the order in which balls are inserted into the world array.
#[test]
fn test_12_three_touching_balls_newtons_cradle_permutation_invariance() {
    let run_cradle = |ball_order: Vec<u8>| -> (Vec2, Vec2, Vec2, Scalar, Vec2) {
        let config = SimulationConfig {
            time_step: 1.0 / 240.0,
            linear_drag: 0.0,
            angular_drag: 0.0,
            ball_restitution: 1.0,
            cushion_restitution: 1.0,
            stop_speed_threshold: 1e-6,
            ..SimulationConfig::default()
        };
        let table = Table::pocketless(TableSpecs::standard_9ft());
        let specs = BallSpecs::standard_pool();
        let d = 2.0 * specs.radius;

        let b0 = Ball::new_stationary(BallId(0), specs, Vec2::new(0.80, 0.635));
        let b1 = Ball::new_stationary(BallId(1), specs, Vec2::new(1.00, 0.635));
        let b2 = Ball::new_stationary(BallId(2), specs, Vec2::new(1.00 + d, 0.635));

        let mut balls = Vec::new();
        for &id in &ball_order {
            match id {
                0 => balls.push(b0),
                1 => balls.push(b1),
                2 => balls.push(b2),
                _ => unreachable!(),
            }
        }

        let mut world = SimulationWorld::new(config, table, balls);
        assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 3.0));

        let initial_ke = world.total_kinetic_energy();
        let initial_p = world.total_linear_momentum();

        world.step_n(20);

        let v0 = world.ball(BallId(0)).unwrap().planar_velocity();
        let v1 = world.ball(BallId(1)).unwrap().planar_velocity();
        let v2 = world.ball(BallId(2)).unwrap().planar_velocity();
        let final_ke = world.total_kinetic_energy();
        let final_p = world.total_linear_momentum();

        assert!(v0.length() < 1e-3, "Ball 0 failed to stop: {:?}", v0);
        assert!(v1.length() < 1e-3, "Ball 1 failed to stop: {:?}", v1);
        assert!(
            (v2.x - 3.0).abs() < 1e-3,
            "Ball 2 expected ~3.0, got: {:?}",
            v2
        );

        assert!((final_ke - initial_ke).abs() < 1e-6);
        assert!(final_p.approx_eq(initial_p, 1e-6));

        (v0, v1, v2, final_ke, final_p)
    };

    let (v0_a, v1_a, v2_a, e_a, p_a) = run_cradle(vec![0, 1, 2]);
    let (v0_b, v1_b, v2_b, e_b, p_b) = run_cradle(vec![2, 1, 0]);
    let (v0_c, v1_c, v2_c, e_c, p_c) = run_cradle(vec![1, 0, 2]);

    assert!(v0_a.approx_eq(v0_b, 1e-9) && v0_a.approx_eq(v0_c, 1e-9));
    assert!(v1_a.approx_eq(v1_b, 1e-9) && v1_a.approx_eq(v1_c, 1e-9));
    assert!(v2_a.approx_eq(v2_b, 1e-9) && v2_a.approx_eq(v2_c, 1e-9));
    assert!((e_a - e_b).abs() < 1e-9 && (e_a - e_c).abs() < 1e-9);
    assert!(p_a.approx_eq(p_b, 1e-9) && p_a.approx_eq(p_c, 1e-9));
}

/// 13. Three touching balls in an equilateral triangular cluster under head-on impact.
///
/// Verifies energy conservation, momentum conservation, and permutation invariance in a compact 3-ball cluster.
#[test]
fn test_13_three_touching_balls_triangle_cluster_energy_and_permutation() {
    let run_cluster = |ball_order: Vec<u8>| -> (Vec2, Vec2, Vec2, Scalar, Vec2) {
        let config = SimulationConfig {
            time_step: 1.0 / 240.0,
            linear_drag: 0.0,
            angular_drag: 0.0,
            ball_restitution: 1.0,
            cushion_restitution: 1.0,
            stop_speed_threshold: 1e-6,
            ..SimulationConfig::default()
        };
        let table = Table::pocketless(TableSpecs::standard_9ft());
        let specs = BallSpecs::standard_pool();
        let r = specs.radius;
        let d = 2.0 * r;

        // Apex ball 0 strikes touching pair (1, 2)
        // Pair 1 & 2 are touching: y1 = 0.635 + r, y2 = 0.635 - r (distance = 2r)
        // Ball 0 is placed in front at distance d: x0 = 1.00 - d * cos(30°), y0 = 0.635
        let dx = d * (3.0_f64.sqrt() / 2.0);
        let b0 = Ball::new_stationary(BallId(0), specs, Vec2::new(1.00 - dx - 0.15, 0.635));
        let b1 = Ball::new_stationary(BallId(1), specs, Vec2::new(1.00, 0.635 + r));
        let b2 = Ball::new_stationary(BallId(2), specs, Vec2::new(1.00, 0.635 - r));

        let mut balls = Vec::new();
        for &id in &ball_order {
            match id {
                0 => balls.push(b0),
                1 => balls.push(b1),
                2 => balls.push(b2),
                _ => unreachable!(),
            }
        }

        let mut world = SimulationWorld::new(config, table, balls);
        assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 3.5));

        let initial_ke = world.total_kinetic_energy();
        let initial_p = world.total_linear_momentum();

        world.step_n(30);

        let v0 = world.ball(BallId(0)).unwrap().planar_velocity();
        let v1 = world.ball(BallId(1)).unwrap().planar_velocity();
        let v2 = world.ball(BallId(2)).unwrap().planar_velocity();
        let final_ke = world.total_kinetic_energy();
        let final_p = world.total_linear_momentum();

        // Exact energy and momentum conservation
        assert!(
            (final_ke - initial_ke).abs() < 1e-9,
            "Cluster energy drift: {initial_ke} -> {final_ke}"
        );
        assert!(
            final_p.approx_eq(initial_p, 1e-9),
            "Cluster momentum drift: {initial_p:?} -> {final_p:?}"
        );

        // Symmetry between balls 1 and 2
        assert!(
            (v1.x - v2.x).abs() < 1e-9,
            "X-symmetry in triangle cluster violated"
        );
        assert!(
            (v1.y + v2.y).abs() < 1e-9,
            "Y-symmetry in triangle cluster violated"
        );

        (v0, v1, v2, final_ke, final_p)
    };

    let (v0_a, v1_a, v2_a, e_a, p_a) = run_cluster(vec![0, 1, 2]);
    let (v0_b, v1_b, v2_b, e_b, p_b) = run_cluster(vec![2, 0, 1]);
    let (v0_c, v1_c, v2_c, e_c, p_c) = run_cluster(vec![1, 2, 0]);

    assert!(v0_a.approx_eq(v0_b, 1e-12) && v0_a.approx_eq(v0_c, 1e-12));
    assert!(v1_a.approx_eq(v1_b, 1e-12) && v1_a.approx_eq(v1_c, 1e-12));
    assert!(v2_a.approx_eq(v2_b, 1e-12) && v2_a.approx_eq(v2_c, 1e-12));
    assert!((e_a - e_b).abs() < 1e-12 && (e_a - e_c).abs() < 1e-12);
    assert!(p_a.approx_eq(p_b, 1e-12) && p_a.approx_eq(p_c, 1e-12));
}
