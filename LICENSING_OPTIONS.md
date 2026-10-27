# Open-Source Licensing Options for Plazr Billiards

In accordance with the Phase 1 specification, **no final license has been automatically selected or applied** (`Cargo.toml` leaves the `license` field unset and `publish = false` until you confirm your choice).

Below is a concise comparison of the standard open-source licensing options recommended for a Rust + WebAssembly library in the Plazr ecosystem:

| License Option | SPDX Identifier | Key Characteristics | Best Suited When |
| :--- | :--- | :--- | :--- |
| **Dual MIT OR Apache-2.0** *(Rust Ecosystem Standard)* | `MIT OR Apache-2.0` | Permissive adoption like MIT, combined with Apache 2.0's explicit patent grant and contributor patent retaliation clauses. | You want maximum compatibility with the Rust ecosystem (`crates.io` convention) while protecting Plazr against patent litigation from contributors/users. |
| **MIT License** | `MIT` | Short, permissive license allowing commercial and closed-source use with minimal attribution requirements. | You want the simplest possible permissive license with zero friction for JavaScript/TypeScript and WebAssembly consumers. |
| **Apache License 2.0** | `Apache-2.0` | Permissive license with explicit patent protection and clear requirements for documenting modifications. | You want strong corporate/patent protection in a single license file. |
| **Mozilla Public License 2.0** *(File-Level Copyleft)* | `MPL-2.0` | Weak copyleft: modifications to `plazr-billiards` source files must remain open-source, but the library can be linked into proprietary games or commercial platforms (including Plazr) without open-sourcing the host application. | You want to ensure community improvements to the physics engine itself are shared back, while still allowing proprietary games (including Plazr) to embed the library cleanly. |

## How to Finalize Your Choice

Once you select a license:
1. Add `license = "<SPDX-Identifier>"` to `Cargo.toml`.
2. Add the corresponding `LICENSE` (or `LICENSE-MIT` and `LICENSE-APACHE`) file(s) to the repository root.
3. Update the Licensing section in `README.md`.
