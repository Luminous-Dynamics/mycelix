# Terra Preta lab core observation operators v0.1

**Status:** synthetic-only Rust reference for two saturated 1-D laboratory core tests.

## Operators

### Constant head

Darcy's law for a uniform saturated specimen:

`Q = K A Δh / L`, so `K = Q L / (A Δh)`.

### Falling head

From standpipe continuity coupled to Darcy flow:

`K = (a L / (A t)) ln(h0/h1)`.

The implementation evaluates the logarithm as `ln_1p((h0-h1)/h1)` for stability when the two heads are close. Its forward map is `h1 = h0 exp(-K A t/(a L))`.

All input geometry and measurements are SI. The report records the protocol, physical core support, geometry, K estimate, forward reconstruction error and idealized assumptions.

## Why this matters

Two core specimens with the same latent conductivity but different diameter or length do not produce the same measured flow. The geometry and protocol therefore belong in the observation operator; Ksat should not be treated as a raw, protocol-free label. The measurement contract still preserves the source method, units, sample ID and study/site groups.

## Explicit exclusions

These equations do not represent tension infiltrometers, Guelph/Aardvark permeameters, unsaturated transient redistribution, anisotropy, sidewall leakage, or heterogeneous core-scale flow. Those require separate observation operators and cannot be admitted by relabelling the protocol to constant/falling head.

## Run

```bash
cargo fmt --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml -- \
  research/terra-preta/tools/lab-core-observation-reference/fixtures/core-observations.json \
  /tmp/terra-core-observations.json
```
