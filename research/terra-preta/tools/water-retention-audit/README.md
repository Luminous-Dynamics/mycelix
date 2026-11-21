# Soil-water retention curve audit

A standalone Rust CLI that converts one measured desorption curve into an auditable volumetric-water representation and computes a study-defined available-water interval.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/water-retention-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/water-retention-audit/Cargo.toml --all-targets -- -D warnings
cargo run --release --manifest-path research/terra-preta/tools/water-retention-audit/Cargo.toml -- \
  research/terra-preta/tools/water-retention-audit/fixtures/synthetic-curve-input.json \
  /tmp/water-retention-audit.json
```

Validate the input and report with the committed Draft 2020-12 schemas:
- [curve-input.schema.json](./curve-input.schema.json)
- [water-retention-audit-report.schema.json](./water-retention-audit-report.schema.json)

The committed fixture is synthetic software-test input only; it is not measured soil data and must not be cited as a research observation.

## Measurement contract

- Each input is one sample/curve. Independent specimens and replicates remain separate; the tool does not average them.
- Matric potential is signed in kPa and must be zero or negative. Suction is reported as its non-negative magnitude.
- Water content basis is explicit: volumetric fraction (m³/m³), or gravimetric kg water per kg dry soil. Gravimetric conversion requires declared dry bulk density and water density:
  `theta_v = theta_g * rho_bulk / rho_water`.
- Field-capacity and permanent-wilting-point matric potentials are explicit negative-kPa inputs. The tool does not impose a universal field-capacity pressure or assume that a conventional threshold is appropriate to every crop, soil, or trial.
- Targets must lie within measured curve support. Interpolation is piecewise linear in `log10(suction_kPa)`; the report identifies the method and exact bracketing observations. Extrapolation is rejected.
- Desorption measurements are sorted by suction. A water-content increase with increasing suction beyond the preregistered tolerance fails closed. Smaller violations are retained unchanged and reported as warnings; no smoothing or isotonic repair is applied.
- Available-water fraction is `theta_v(field_capacity) - theta_v(wilting_point)`. Layer storage is reported only when the input explicitly supplies layer depth: fraction × depth (mm). This is not a full root-zone water balance.
- The input SHA-256, normalized observations, measurement method, targets, interpolation provenance, and warnings are included in the report.

## Deliberate limits

This tool audits arithmetic and measurement structure for one curve. It does not certify ISO compliance, infer field capacity from drainage dynamics, determine plant-specific stress thresholds, estimate measurement uncertainty, aggregate replicates, fit van Genuchten/Brooks–Corey models, estimate treatment effects, predict crop yield, establish biochar efficacy/safety, or authorize material production/application.

Measurement uncertainty is marked `not_propagated`; do not interpret a precise numeric output as precise science. Replicate-level and laboratory-method uncertainty must be addressed in the study analysis. A schema-valid report is not proof that the sample, method, or identifiers are valid.
