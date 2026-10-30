# Elemental nutrient boundary-budget auditor

A standalone Rust CLI that audits a single element-specific, period-specific nutrient budget for a declared soil–crop control volume.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/nutrient-budget-audit/Cargo.toml
cargo fmt --manifest-path research/terra-preta/tools/nutrient-budget-audit/Cargo.toml -- --check
cargo clippy --manifest-path research/terra-preta/tools/nutrient-budget-audit/Cargo.toml --all-targets -- -D warnings
cargo run --release --manifest-path research/terra-preta/tools/nutrient-budget-audit/Cargo.toml -- \
  research/terra-preta/tools/nutrient-budget-audit/fixtures/synthetic-n-budget.json \
  /tmp/nutrient-budget-report.json
```

Validate inputs and reports against:
- [nutrient-budget-input.schema.json](./nutrient-budget-input.schema.json)
- [nutrient-budget-audit-report.schema.json](./nutrient-budget-audit-report.schema.json)

## Budget semantics

`residual = external_inputs - external_outputs - (storage_end - storage_start)`

The basis is kg of elemental N, P or K per hectare. Do not supply oxide mass (P2O5/K2O), product mass, or concentration without a declared conversion performed and recorded upstream.

The control volume normally includes soil, standing biomass and retained residues. The boundary description must say what is included. Root uptake, mineralization, immobilization, sorption/desorption and residue return are internal transfers when both source and destination remain inside the control volume; they are not additional external inputs. Harvest removed, leaching across the declared lower boundary, runoff, erosion and gaseous export are boundary outputs where relevant.

Storage pools must be mutually exclusive and have exactly matching IDs/compartments at the start and end of the period. Extractable/available nutrient indicators are reported separately and are **never added** to stock pools.

## Uncertainty status

Every measured input/output/stock should declare standard uncertainty. If any term omits it, the tool reports the residual but marks the budget `unqualified_missing_uncertainty`.

When all are present, the tool sums standard uncertainties linearly as a conservative bound under unspecified covariance. This is not a confidence interval and does not account for systematic bias or omitted fluxes. A residual within this bound is an arithmetic consistency screen only—not proof that the system boundary is complete or the nutrient measurements are valid.

## Deliberate limits

- One element, experimental unit, control volume and period per input.
- No auto conversion, missing-flux imputation, pool repair, or residual forcing.
- No causal effect estimation or treatment aggregation.
- No judgement that more available nutrient, lower leaching, or a smaller residual is automatically agronomically beneficial.
- No product safety, trial, regulatory, fertilizer-substitution or field efficacy determination.

The fixture is synthetic software-test data only; it is not a measurement or agronomic result.
