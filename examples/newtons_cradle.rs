//! Example: 3-ball collinear momentum transfer (Newton's cradle).
//!
//! Run with: `cargo run --example newtons_cradle`

use plazr_billiards::ball::{Ball, BallId, BallSpecs};
use plazr_billiards::math::Vec2;
use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
use plazr_billiards::table::{Table, TableSpecs};

fn main() {
    let config = SimulationConfig {
        linear_drag: 0.0,
        angular_drag: 0.0,
        ball_restitution: 1.0,
        ..SimulationConfig::default()
    };
    let table = Table::pocketless(TableSpecs::standard_9ft());
    let specs = BallSpecs::standard_pool();
    let diameter = 2.0 * specs.radius;

    let balls = vec![
        Ball::new_stationary(BallId(0), specs, Vec2::new(0.80, 0.635)),
        Ball::new_stationary(BallId(1), specs, Vec2::new(1.10, 0.635)),
        Ball::new_stationary(BallId(2), specs, Vec2::new(1.10 + diameter, 0.635)),
    ];

    let mut world = SimulationWorld::new(config, table, balls);
    world.strike_ball_with_speed(BallId(0), Vec2::UNIT_X, 3.0);

    println!("Initial KE: {:.6} J", world.total_kinetic_energy());
    let events = world.step_n(30);
    println!("Events recorded: {}", events.len());
    for b in &world.balls {
        println!(
            "Ball {:?}: pos=({:.4}, {:.4}), vel=({:.4}, {:.4})",
            b.id,
            b.position.x,
            b.position.y,
            b.planar_velocity().x,
            b.planar_velocity().y
        );
    }
    println!("Final KE:   {:.6} J", world.total_kinetic_energy());
}
