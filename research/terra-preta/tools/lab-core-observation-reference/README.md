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

All input geometry and measurements are SI. The report records the protocol, original method label, specimen preparation, physical core support, geometry, K estimate, forward reconstruction error and idealized assumptions. Every observation requires a record ID and canonical physical-sample group ID so repeated observations can remain grouped in downstream validation.

## Why this matters

Two core specimens with the same latent conductivity but different diameter or length do not produce the same measured flow. The geometry and protocol therefore belong in the observation operator; Ksat should not be treated as a raw, protocol-free label. The measurement contract still preserves the source method, units, sample ID and study/site groups.

## Provenance and validation

The implementation accepts SI-valued geometry and measurement fields. Dataset adapters must retain the original units and source strings in the broader observation contract before converting to SI; this small solver is not an importer and does not infer preparation or units. The specimen preparation field is preserved verbatim and is not inferred from the protocol.

The test suite includes analytic constant- and falling-head round trips, multiple K scales, near-equal head differences, geometry scaling, nonphysical inputs, and missing source-method metadata.

## Explicit exclusions

These equations do not represent tension infiltrometers, Guelph/Aardvark permeameters, unsaturated transient redistribution, anisotropy, sidewall leakage, or heterogeneous core-scale flow. Those require separate observation operators and cannot be admitted by relabelling the protocol to constant/falling head.

## Local observation Jacobians

Each receipt now includes a local sensitivity record with a shared parameter identity, observed-quantity identity, observed value, derivative with respect to log hydraulic conductivity, derivative units, and dimensionless log-elasticity.

For constant-head flow:
`Q(K) = K A \Delta h / L`, so `dQ/d(log K) = Q` and `d(log Q)/d(log K) = 1`.

For falling-head final level:
`h_1(K) = h_0 exp(-K A t/(a L))`, so `dh_1/d(log K) = h_1 log(h_1/h_0)` and the log-elasticity is `log(h_1/h_0)`.

The two analytic Jacobians are checked against central finite differences in log-K coordinates over multiple conductivity scales. They are local derivatives of the declared idealized lab operators; no noise model or Fisher information is fabricated when instrument uncertainty is absent from the input.

## Run

```bash
cargo fmt --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/lab-core-observation-reference/Cargo.toml -- \
  research/terra-preta/tools/lab-core-observation-reference/fixtures/core-observations.json \
  /tmp/terra-core-observations.json
```
