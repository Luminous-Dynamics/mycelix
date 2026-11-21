# Equivalent-soil-mass carbon stock auditor

Standalone Rust auditor for synthetic layered-core SOC stock calculations.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/esm-carbon-stock-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/esm-carbon-stock-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/esm-carbon-stock-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/esm-carbon-stock-audit/Cargo.toml -- \
  research/terra-preta/tools/esm-carbon-stock-audit/fixtures/synthetic-compaction-only.json \
  /tmp/esm-carbon-stock-report.json
```

## Equations

Per-layer fine-earth mass (Mg ha⁻¹):

`fine-earth BD (g cm⁻³) × layer thickness (cm) × 100 × (1 − coarse-fragment volume fraction)`

Per-layer SOC stock (Mg C ha⁻¹):

`fine-earth mass × SOC concentration (g C kg⁻¹) / 1000`

- Fixed-depth stock sums carbon over the complete measured profile.
- ESM stock integrates the layerwise SOC concentration until the supplied, predeclared fine-earth reference mass is reached.
- If the reference mass exceeds any measured core's profile mass, the audit rejects the scenario; it never extrapolates beyond measured support.

## Boundaries

All values are manufactured. Cores are nested spatial subsamples of one experimental unit, not independent treatment replicates. V0.1 does not propagate spatial, analytical, layer, reference-mass or temporal covariance; it does not estimate a causal treatment effect or validate a carbon-credit methodology. Fixed-depth and ESM values are reported separately.
