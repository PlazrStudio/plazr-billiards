//! Example: Standard 16-ball break shot (cue ball + 15-ball rack) on a 9-foot 6-pocket table.
//!
//! Run with: `cargo run --example sixteen_ball_break`

use plazr_billiards::ball::{Ball, BallId, BallSpecs};
use plazr_billiards::math::Vec2;
use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
use plazr_billiards::table::Table;

fn main() {
    let config = SimulationConfig::default();
    let table = Table::standard_9ft_six_pocket();
    let specs = BallSpecs::standard_pool();
    let d = 2.0 * specs.radius + 1e-4;

    let mut balls = Vec::with_capacity(16);
    balls.push(Ball::new_stationary(
        BallId(0),
        specs,
        Vec2::new(0.635, 0.635),
    ));

    let mut id = 1u8;
    for row in 0..5 {
        for col in 0..=row {
            let x = 1.905 + (row as f64) * d * 0.866_025_403_784;
            let y = 0.635 + ((col as f64) - (row as f64) * 0.5) * d;
            balls.push(Ball::new_stationary(BallId(id), specs, Vec2::new(x, y)));
            id += 1;
        }
    }

    let mut world = SimulationWorld::new(config, table, balls);
    world.strike_ball_with_speed(BallId(0), Vec2::new(1.0, 0.012), 8.0);

    let (steps, events) = world.step_until_settled(6_000);
    println!(
        "16-ball break settled in {steps} steps ({:.2}s simulated time).",
        (steps as f64) * world.config.time_step
    );
    println!(
        "Total events: {}, Active balls on table: {}, Pocketed balls: {}",
        events.len(),
        world.active_ball_count(),
        world.pocketed_ball_count()
    );
}
