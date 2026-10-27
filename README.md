# Plazr Billiards

A Rust library for billiards physics, developed by [Plazr Studio](https://plazr.net).

We're building a physics engine that developers can use without bringing along an entire game. The goal is to keep the simulation separate from graphics, networking, and any particular game framework.

**This project is still in its early stages.** The current code sets up the library's structure and data types. It doesn't simulate ball movement or collisions yet.

## Why this project exists

Plazr Billiards started as a way to explore billiards physics in a standalone, reusable library. It's a separate project, built independently of the commercial Plazr game engine.

We're starting with Rust and plan to add WebAssembly support so the same simulation can eventually be used from JavaScript and TypeScript. There's no browser integration yet.

## What's here today

The first version lays out the pieces we'll build on:

- `math` — basic 2D and 3D vector types.
- `ball` — ball identity, physical properties, position, and velocity data.
- `table` — table dimensions and pocket definitions.
- `collision` — types for describing contact events.
- `simulation` — configuration and a container for the table and balls.
- `wasm_api` — a place for future WebAssembly bindings.

These modules define the data model. The physics algorithms still need to be written and tested.

## What's next

We're working toward these milestones, roughly in this order:

1. Vector operations and basic geometry.
2. Time-stepped ball movement.
3. Ball-to-ball and ball-to-cushion collisions.
4. Friction, rolling, sliding, and spin.
5. Tests for numerical stability and repeatable results.
6. WebAssembly bindings and a small browser example.

The order may change as we learn more. We'd rather ship something correct and understandable than rush to a feature checklist.

## Building from source

You'll need Rust 1.75 or newer. From the repository root, run:

```bash
cargo check
cargo build
cargo test
cargo fmt --check
```

At this stage, `cargo test` does not exercise a working physics engine. Simulation tests will be added alongside the features they cover.

## Contributing

Feedback on the design, data types, and planned API is welcome. The project is young, so public interfaces may change as the simulation takes shape. If you'd like to make a larger contribution, please discuss the approach in an issue first.

## License

A license hasn't been selected yet. Until a license is added, please don't assume the code is available for reuse or redistribution. We're reviewing the options in [LICENSING_OPTIONS.md](LICENSING_OPTIONS.md) before the public release.

---

Made at **Plazr Studio** · [plazr.net](https://plazr.net)
