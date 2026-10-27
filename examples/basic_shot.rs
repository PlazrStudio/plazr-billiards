//! Runnable example demonstrating a basic cue shot, ball-to-ball collision, and settling.
//!
//! Run with: `cargo run --example basic_shot`

use plazr_billiards::ball::{Ball, BallId, BallSpecs};
use plazr_billiards::collision::CollisionKind;
use plazr_billiards::math::Vec2;
use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
use plazr_billiards::table::Table;

fn main() {
    let config = SimulationConfig::default();
    let table = Table::standard_9ft_six_pocket();
    let specs = BallSpecs::standard_pool();

    let cue_ball = Ball::new_stationary(BallId(0), specs, Vec2::new(0.65, 0.635));
    let object_ball = Ball::new_stationary(BallId(1), specs, Vec2::new(1.25, 0.635));

    let mut world = SimulationWorld::new(config, table, vec![cue_ball, object_ball]);

    let struck = world.strike_ball(BallId(0), Vec2::new(1.0, 0.0), 0.68);
    assert!(struck, "Failed to strike cue ball");

    let (steps, events) = world.step_until_settled(5_000);
    println!(
        "Simulation settled in {steps} steps with {} events:",
        events.len()
    );

    for event in &events {
        match event.kind {
            CollisionKind::BallToBall { first, second } => {
                println!(
                    "  [{:.4}s] BallToBall: {:?} <-> {:?}",
                    event.time_of_impact, first, second
                );
            }
            CollisionKind::BallToCushion {
                ball,
                cushion_index,
            } => {
                println!(
                    "  [{:.4}s] BallToCushion: {:?} hit rail {}",
                    event.time_of_impact, ball, cushion_index
                );
            }
            CollisionKind::BallToPocket { ball, pocket_index } => {
                println!(
                    "  [{:.4}s] BallToPocket: {:?} dropped into pocket {}",
                    event.time_of_impact, ball, pocket_index
                );
            }
        }
    }

    for ball in &world.balls {
        println!(
            "Final state of {:?}: pos=({:.4}, {:.4}), state={:?}",
            ball.id, ball.position.x, ball.position.y, ball.state
        );
    }
}
