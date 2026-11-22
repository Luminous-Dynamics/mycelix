# SOC stock / equivalent soil mass auditor

Standalone Rust auditor for synthetic layer-resolved SOC stock calculations.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/soc-stock-esm-audit/Cargo.toml -- \
  research/terra-preta/tools/soc-stock-esm-audit/fixtures/synthetic-compaction-esm.json \
  /tmp/soc-stock-esm-report.json
```

## Input basis

Each layer declares total dry-soil mass per area (including coarse fragments), a coarse-fragment mass fraction, and SOC concentration on a fine-earth basis.

`fine-earth mass = total dry-soil mass × (1 - coarse-fragment mass fraction)`

The coarse-fragment fraction must be mass-based. A volume fraction is not interchangeable. Carbon data must be organic carbon or total carbon corrected for inorganic carbon.

## Outputs

For each core profile:
- fixed-depth fine-earth mass and SOC stock;
- observed profile fine-earth mass;
- ESM stock at the single declared reference fine-earth mass;
- whether the ESM target cuts through a layer.

For each measurement round:
- mean and sample standard deviation across cores for FD and ESM separately.

Across rounds:
- descriptive first-to-last difference for FD and ESM.

Core dispersion is not a treatment confidence interval. Cores do not create new treatment-level independent experimental units.

## Boundaries

- The fixed depth must match a measured layer boundary.
- All profiles / rounds must share the same layer-depth boundaries.
- ESM must be fully supported by observed fine-earth mass; there is no extrapolation.
- Partial-layer ESM assumes the supplied layer concentration is constant within that layer.
- No lab/core/covariance uncertainty is propagated.
- The fixture is synthetic-only and makes no MRV, credit, sequestration or intervention-effect claim.
