//! Reproducible performance and memory benchmark harness (`cargo bench --bench simulation_bench`).
//!
//! Uses only the Rust standard library (`std::time::Instant` and `std::mem::size_of`) so it
//! compiles and runs without external benchmark crate dependencies.

use std::mem::size_of;
use std::time::Instant;

use plazr_billiards::ball::{Ball, BallId, BallSpecs};
use plazr_billiards::collision::resolve_ball_ball_collision;
use plazr_billiards::math::Vec2;
use plazr_billiards::simulation::{SimulationConfig, SimulationWorld};
use plazr_billiards::table::Table;

fn build_sixteen_ball_world() -> SimulationWorld {
    let config = SimulationConfig::default();
    let table = Table::standard_9ft_six_pocket();
    let specs = BallSpecs::standard_pool();
    let d = 2.0 * specs.radius + 1e-4;

    let mut balls = Vec::with_capacity(16);
    // Cue ball at head spot
    balls.push(Ball::new_stationary(
        BallId(0),
        specs,
        Vec2::new(0.635, 0.635),
    ));

    // 15-ball triangle rack at foot spot (5 rows: 1 + 2 + 3 + 4 + 5 = 15)
    let mut id = 1u8;
    for row in 0..5 {
        for col in 0..=row {
            let x = 1.905 + (row as f64) * d * 0.866_025_403_784;
            let y = 0.635 + ((col as f64) - (row as f64) * 0.5) * d;
            balls.push(Ball::new_stationary(BallId(id), specs, Vec2::new(x, y)));
            id += 1;
        }
    }

    SimulationWorld::new(config, table, balls)
}

fn main() {
    println!("=== Plazr Billiards Benchmark Suite ===");
    println!("Memory footprint of core structures:");
    println!(
        "  size_of::<Ball>()            = {} bytes",
        size_of::<Ball>()
    );
    println!(
        "  size_of::<Table>()           = {} bytes (stack) + {} pockets",
        size_of::<Table>(),
        6
    );
    println!("  size_of::<SimulationWorld>() = {} bytes (stack) + Vec<Ball> heap ({} bytes for 16 balls)",
        size_of::<SimulationWorld>(),
        16 * size_of::<Ball>()
    );

    // 1. Pairwise collision solver micro-benchmark
    let specs = BallSpecs::standard_pool();
    let r = specs.radius;
    let iterations = 100_000_u32;
    let t0 = Instant::now();
    for _ in 0..iterations {
        let mut a = Ball::new_stationary(BallId(0), specs, Vec2::new(1.0, 0.5));
        let mut b = Ball::new_stationary(BallId(1), specs, Vec2::new(1.0 + 2.0 * r - 1e-5, 0.5));
        a.set_planar_velocity(Vec2::new(3.5, 0.2), 1e-4);
        let _ = resolve_ball_ball_collision(&mut a, &mut b, 0.94, 1e-4);
    }
    let elapsed_collision = t0.elapsed();
    println!(
        "1. Pairwise collision resolution ({iterations} calls): {:?} ({:.1} ns/call)",
        elapsed_collision,
        elapsed_collision.as_nanos() as f64 / f64::from(iterations)
    );

    // 2. Single fixed-step cost on an active 16-ball table
    let mut world_16 = build_sixteen_ball_world();
    world_16.strike_ball_with_speed(BallId(0), Vec2::new(1.0, 0.01), 7.5);
    let step_count = 1_000_usize;
    let t1 = Instant::now();
    let events = world_16.step_n(step_count);
    let elapsed_steps = t1.elapsed();
    println!(
        "2. 16-ball simulation ({step_count} steps, {} events): {:?} ({:.2} us/step)",
        events.len(),
        elapsed_steps,
        elapsed_steps.as_secs_f64() * 1e6 / (step_count as f64)
    );

    // 3. Full 16-ball break until settled
    let mut break_world = build_sixteen_ball_world();
    break_world.strike_ball_with_speed(BallId(0), Vec2::new(1.0, 0.015), 8.0);
    let t2 = Instant::now();
    let (settled_steps, break_events) = break_world.step_until_settled(6_000);
    let elapsed_break = t2.elapsed();
    println!(
        "3. Full 16-ball break until settled ({settled_steps} steps, {} events): {:?}",
        break_events.len(),
        elapsed_break
    );

    // 4. 1,000 independent 16-ball break simulations (100 steps each = 100,000 world steps)
    let sim_runs = 1_000_u32;
    let t3 = Instant::now();
    let mut total_events = 0usize;
    for i in 0..sim_runs {
        let mut w = build_sixteen_ball_world();
        let dy = ((i % 20) as f64 - 10.0) * 0.002;
        w.strike_ball_with_speed(BallId(0), Vec2::new(1.0, dy), 7.0);
        total_events += w.step_n(100).len();
    }
    let elapsed_batch = t3.elapsed();
    println!(
        "4. {sim_runs} independent 16-ball simulations (100 steps each, {total_events} total events): {:?} ({:.2} ms/sim)",
        elapsed_batch,
        elapsed_batch.as_secs_f64() * 1e3 / f64::from(sim_runs)
    );
}
