# CT-informed macropore–matrix exchange reference

Synthetic-only reference kernel for the CT-informed transient exchange term discussed by Chaif et al. (2025), DOI [10.1016/j.jhydrol.2025.133791](https://doi.org/10.1016/j.jhydrol.2025.133791).

## Dimensional contract

- (a_{wetted}): specific wetted interface area, m² interface per m³ bulk soil.
- (j_{macro}): signed interfacial exchange flux density, m/s.
- (r_{exchange}=a_{wetted}j_{macro}): volumetric exchange-rate density, 1/s.
- (dV_{exchange}=r_{exchange}V_{bulk}dt): water volume transferred, m³.

Positive transfer is macropore → matrix; negative transfer reverses direction. A transfer is subtracted from one compartment and added to the other. The reference fixture's flux density is explicitly synthetic; the cited paper derives its flux-density term from a horizontal Richards-equation representation of the matrix, so a future physical integration must preserve that model/provenance distinction.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/ct-exchange-reference/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/ct-exchange-reference/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/ct-exchange-reference/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/ct-exchange-reference/Cargo.toml -- \
  research/terra-preta/tools/ct-exchange-reference/fixtures/synthetic-ct-exchange.json \
  /tmp/ct-exchange-reference-report.json
```

## Scope

The reference compares time-varying wetted interface × measured/synthetic macropore flux with a fixed-initial-interface ablation. It uses transparent trapezoidal integration of endpoint rate densities, splits sign reversals at their zero crossing, and rejects donor overdraw, receiver overfill and gaps beyond the declared maximum.

It does **not** implement CT image processing, Richards flow, a dual-permeability solver, root dynamics, tracer transport, state/parameter assimilation, or the complete model in Chaif et al. (2025). It is a manufactured dimensional/conservation oracle for future coupled implementations, not a calibrated or validated soil model.
