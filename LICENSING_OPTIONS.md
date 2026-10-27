# Open-Source Licensing Configuration for Plazr Billiards

The proposed license configuration for `plazr-billiards` is the standard Rust ecosystem dual license:

- **SPDX Identifier**: `MIT OR Apache-2.0`
- **Publisher / Project Attribution**: `Plazr Studio`
- **Included License Texts**:
  - [`LICENSE-MIT`](./LICENSE-MIT)
  - [`LICENSE-APACHE`](./LICENSE-APACHE)

## Current Pre-Release Status

`Cargo.toml` is configured with `license = "MIT OR Apache-2.0"` and `publish = false`. Final public release under this license remains subject to explicit approval by the project owner prior to setting `publish = true` or publishing to `crates.io`.

## Summary of Candidate Options

| License Option | SPDX Identifier | Key Characteristics | Status |
| :--- | :--- | :--- | :--- |
| **Dual MIT OR Apache-2.0** *(Rust Ecosystem Standard)* | `MIT OR Apache-2.0` | Permissive adoption like MIT, combined with Apache 2.0's explicit patent grant and contributor patent retaliation clauses. | **Prepared in repository** (`LICENSE-MIT` + `LICENSE-APACHE`), pending owner final sign-off before public release. |
| **MIT License** | `MIT` | Short, permissive license allowing commercial and closed-source use with minimal attribution requirements. | Covered as the first option of the dual license. |
| **Apache License 2.0** | `Apache-2.0` | Permissive license with explicit patent protection and clear requirements for documenting modifications. | Covered as the second option of the dual license. |
| **Mozilla Public License 2.0** *(File-Level Copyleft)* | `MPL-2.0` | Weak copyleft: modifications to `plazr-billiards` source files must remain open-source, while allowing linking into proprietary applications. | Alternative option if file-level copyleft is preferred before release. |
