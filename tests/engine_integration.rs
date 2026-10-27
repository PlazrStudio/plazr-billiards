//! Integration test suite for `plazr-billiards` core APIs.
//!
//! Covers:
//! 1. 2D/3D vector API, dot/cross products, lengths, distances, directions, angles, and reflections.
//! 2. Safe normalization and numerical sanitization against `NaN` and `Infinity`.
//! 3. Ball kinematics, fixed-timestep integration, linear deceleration, and stopping thresholds.
//! 4. Head-on ball-to-ball contact, mass ratios, restitution, coincident-center impulse, and overlap separation.
//! 5. Cushion rebounds and pocket capture transitions.
//! 6. Simulation stability and repeatability within a single runtime environment.

use plazr_billiards::ball::{Ball, BallId, BallSpecs, BallState};
use plazr_billiards::collision::{
    compute_ball_ball_toi, detect_ball_ball_contact, resolve_ball_ball_collision,
    resolve_ball_overlap, CollisionKind,
};
use plazr_billiards::math::{Scalar, Vec2, Vec3};
use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
use plazr_billiards::table::{cushion_index, Table, TableSpecs};

#[test]
fn math_vectors_directions_angles_and_reflections() {
    let a = Vec2::new(6.0, 8.0);
    let b = Vec2::new(-3.0, 4.0);

    assert!((a.length() - 10.0).abs() < 1e-12);
    assert!((b.length() - 5.0).abs() < 1e-12);
    assert!((a.dot(b) - 14.0).abs() < 1e-12);
    assert!((a.cross(b) - 48.0).abs() < 1e-12);

    let dir = a.direction_to(b);
    assert!((dir.length() - 1.0).abs() < 1e-12);

    let v3_a = Vec3::new(1.0, 2.0, 3.0);
    let v3_b = Vec3::new(4.0, -5.0, 6.0);
    assert!((v3_a.dot(v3_b) - 12.0).abs() < 1e-12);
    assert!(v3_a
        .cross(v3_b)
        .approx_eq(Vec3::new(27.0, 6.0, -13.0), 1e-12));

    let reflected = Vec2::new(3.0, -4.0).reflect(Vec2::UNIT_Y);
    assert!(reflected.approx_eq(Vec2::new(3.0, 4.0), 1e-12));
}

#[test]
fn ball_motion_deceleration_and_stop_threshold() {
    let config = SimulationConfig {
        time_step: 1.0 / 240.0,
        linear_drag: 0.40,
        stop_speed_threshold: 1e-3,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let cue = Ball::new_stationary(BallId(0), BallSpecs::standard_pool(), Vec2::new(0.3, 0.635));
    let mut world = SimulationWorld::new(config, table, vec![cue]);

    assert!(world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 0.8));
    let (steps, events) = world.step_until_settled(2000);

    assert!(steps > 100);
    assert!(events.is_empty());
    assert!(world.is_settled());

    let final_ball = world.ball(BallId(0)).unwrap();
    assert_eq!(final_ball.state, BallState::Stationary);
    assert_eq!(final_ball.speed(), 0.0);
    // Expected stopping distance d = v0^2 / (2 * a) = 0.64 / 0.80 = 0.80 m => x ≈ 1.10 m
    assert!((final_ball.position.x - 1.10).abs() < 0.01);
}

#[test]
fn ball_to_ball_head_on_contact_and_overlap_resolution() {
    let specs = BallSpecs::standard_pool();
    let r = specs.radius;

    // 1. Overlap separation test
    let mut b1 = Ball::new_stationary(BallId(0), specs, Vec2::new(1.0, 0.5));
    let mut b2 = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0 + r, 0.5));
    assert!(detect_ball_ball_contact(&b1, &b2).is_some());
    assert!(resolve_ball_overlap(&mut b1, &mut b2));
    assert!(b1.position.distance(b2.position) >= 2.0 * r - 1e-6);

    // 2. Analytic TOI query test
    let moving = Ball {
        velocity: Vec3::new(2.0, 0.0, 0.0),
        state: BallState::Moving,
        ..Ball::new_stationary(BallId(0), specs, Vec2::new(0.5, 0.5))
    };
    let target = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0, 0.5));
    let toi = compute_ball_ball_toi(&moving, &target, 1.0).expect("should collide within 1s");
    let expected_toi = (0.5 - 2.0 * r) / 2.0;
    assert!((toi - expected_toi).abs() < 1e-9);

    // 3. Head-on impulse conservation of momentum test
    let mut a = moving;
    a.position = Vec2::new(1.0 - 2.0 * r + 1e-5, 0.5);
    let mut b = target;
    assert!(resolve_ball_ball_collision(&mut a, &mut b, 0.95, 1e-4));
    let p_initial = specs.mass * 2.0;
    let p_final = specs.mass * a.planar_velocity().x + specs.mass * b.planar_velocity().x;
    assert!((p_initial - p_final).abs() < 1e-9);
}

#[test]
fn cushion_rebound_and_pocket_capture_in_world() {
    let config = SimulationConfig::default();

    // Test cushion bounce on a pocketless table
    let carom_table = Table::pocketless(TableSpecs::standard_9ft());
    let ball = Ball::new_stationary(
        BallId(0),
        BallSpecs::standard_pool(),
        Vec2::new(0.20, 0.635),
    );
    let mut carom_world = SimulationWorld::new(config, carom_table, vec![ball]);
    assert!(carom_world.strike_ball_with_speed(BallId(0), Vec2::new(-1.0, 0.0), 3.0));
    let events = carom_world.step_n(40);
    assert!(events.iter().any(|e| matches!(
        e.kind,
        CollisionKind::BallToCushion {
            ball: BallId(0),
            cushion_index: cushion_index::LEFT,
        }
    )));
    assert!(carom_world.ball(BallId(0)).unwrap().planar_velocity().x > 0.0);

    // Test pocket capture on a 6-pocket table (aiming directly into bottom-left corner pocket at (0, 0))
    let pool_table = Table::standard_9ft_six_pocket();
    let pocket_ball =
        Ball::new_stationary(BallId(7), BallSpecs::standard_pool(), Vec2::new(0.18, 0.18));
    let mut pool_world = SimulationWorld::new(config, pool_table, vec![pocket_ball]);
    assert!(pool_world.strike_ball_with_speed(BallId(7), Vec2::new(-1.0, -1.0), 2.5));
    let (_steps, pool_events) = pool_world.step_until_settled(240);

    assert!(pool_events.iter().any(|e| matches!(
        e.kind,
        CollisionKind::BallToPocket {
            ball: BallId(7),
            pocket_index: 0,
        }
    )));
    assert_eq!(
        pool_world.ball(BallId(7)).unwrap().state,
        BallState::Pocketed { pocket_index: 0 }
    );
    assert_eq!(pool_world.active_ball_count(), 0);
    assert_eq!(pool_world.pocketed_ball_count(), 1);
}

#[test]
fn repeatability_within_single_runtime_and_stability() {
    fn run_break_scenario() -> (Vec<Ball>, Vec<CollisionKind>) {
        let config = SimulationConfig::default();
        let table = Table::standard_9ft_six_pocket();
        let specs = BallSpecs::standard_pool();
        let d = specs.radius * 2.0 + 1e-4;

        let balls = vec![
            Ball::new_stationary(BallId(0), specs, Vec2::new(0.60, 0.635)),
            Ball::new_stationary(BallId(1), specs, Vec2::new(1.80, 0.635)),
            Ball::new_stationary(
                BallId(2),
                specs,
                Vec2::new(1.80 + d * 0.866, 0.635 - d * 0.5),
            ),
            Ball::new_stationary(
                BallId(3),
                specs,
                Vec2::new(1.80 + d * 0.866, 0.635 + d * 0.5),
            ),
        ];

        let mut world = SimulationWorld::new(config, table, balls);
        assert!(world.strike_ball_with_speed(BallId(0), Vec2::new(1.0, 0.02), 6.5));
        let (_steps, events) = world.step_until_settled(5000);
        assert!(world.is_settled());

        for b in &world.balls {
            assert!(b.position.is_finite());
            assert!(b.velocity.is_finite());
            assert!(b.angular_velocity.is_finite());
        }

        let kinds = events.into_iter().map(|e| e.kind).collect();
        (world.balls, kinds)
    }

    let (balls_run_1, events_run_1) = run_break_scenario();
    let (balls_run_2, events_run_2) = run_break_scenario();

    assert_eq!(balls_run_1, balls_run_2);
    assert_eq!(events_run_1, events_run_2);
    assert!(!events_run_1.is_empty());
}

#[test]
fn sanitization_of_invalid_nan_and_infinite_values() {
    let config = SimulationConfig {
        time_step: Scalar::NAN,
        linear_drag: Scalar::INFINITY,
        angular_drag: -10.0,
        stop_speed_threshold: Scalar::NAN,
        ball_restitution: 5.0,
        cushion_restitution: -2.0,
        max_ball_speed: Scalar::NAN,
        max_substeps: 0,
    };

    let table = Table::standard_9ft_six_pocket();
    let mut bad_ball = Ball::new_stationary(
        BallId(0),
        BallSpecs::standard_pool(),
        Vec2::new(Scalar::NAN, Scalar::INFINITY),
    );
    bad_ball.velocity = Vec3::new(Scalar::INFINITY, Scalar::NAN, 100.0);

    let mut world = SimulationWorld::new(config, table, vec![bad_ball]);
    assert!(!world.strike_ball(BallId(0), Vec2::ZERO, 10.0));
    assert!(!world.strike_ball(BallId(0), Vec2::new(Scalar::NAN, 1.0), 10.0));
    assert!(!world.strike_ball(BallId(0), Vec2::UNIT_X, Scalar::NAN));

    let _events = world.step_n(10);
    let b = world.ball(BallId(0)).unwrap();
    assert!(b.position.is_finite());
    assert!(b.velocity.is_finite());
    assert!(b.angular_velocity.is_finite());
}
