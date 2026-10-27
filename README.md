# Plazr Billiards

A billiards physics library written in Rust.
Built at Plazr Studio.

This project handles the physics side of billiards: moving balls, detecting collisions, cushion rebounds, and pockets.

There’s no renderer or game UI included. You can bring your own.

## Current state

The core simulation is working, but it’s still early.

Here’s what’s implemented so far:

* 2D ball movement with fixed time steps
* Ball-to-ball and cushion collisions
* Continuous collision detection (CCD)
* Handling of simultaneous ball contacts
* Basic friction and stopping behavior
* Simplified pockets and table geometry
* Tests for momentum, kinetic energy, and collision edge cases

The engine runs independently of any browser or game framework.

## Getting started

Clone the repository:

```bash
git clone https://github.com/PlazrStudio/plazr-billiards.git
cd plazr-billiards
```

Run the tests:

```bash
cargo test
```

Or try one of the examples:

```bash
cargo run --example basic_shot
cargo run --example newtons_cradle
cargo run --example sixteen_ball_break
```

That’s enough to see what the engine currently does.

## Things we’re still working on

The physics isn’t finished.

Spin and cloth interaction are still simplified. Pocket geometry could use more work, particularly around the jaws.

There’s also no working JavaScript or WebAssembly API yet. That’s planned, but we’re focusing on getting the Rust side right first.

The simulation has automated tests, but passing those tests doesn’t mean every possible collision is physically accurate.

## Why build this?

We wanted a billiards simulation that could be developed and tested without tying it to a particular game.

Keeping the physics separate makes it easier to experiment, measure behavior, and fix things without dealing with rendering or networking code.

It’s also a good excuse to spend too much time figuring out why three balls won’t collide properly.

## Contributing

Issues and pull requests are welcome.

If you find a physics bug, please include the initial ball positions, velocities, and the result you expected.

A small reproducible example is much more useful than a screenshot of a ball doing something weird.

## License

The repository currently includes MIT and Apache-2.0 license files. Publishing to crates.io is not enabled yet.

---

Plazr Studio · plazr.net
