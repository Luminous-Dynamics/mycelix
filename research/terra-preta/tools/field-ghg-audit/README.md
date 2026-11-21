# Field GHG observation auditor

A standalone Rust CLI for the **synthetic-only v0.1 field greenhouse-gas observation envelope**.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/field-ghg-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/field-ghg-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/field-ghg-audit/Cargo.toml --all-targets -- -D warnings
cargo run --release --manifest-path research/terra-preta/tools/field-ghg-audit/Cargo.toml -- \
  research/terra-preta/fixtures/synthetic-field-ghg-observations.json \
  /tmp/field-ghg-audit-report.json
```

## What it checks

- Explicit synthetic-only envelope marker and non-empty observation set.
- Required object members and unknown-field rejection.
- Unique observation IDs.
- Gas species / mass basis / flux unit consistency, including N2O versus N2O-N.
- Explicit positive-to-atmosphere sign convention; negative flux remains permitted.
- Chamber geometry and closure duration are finite and positive.
- RFC3339 timestamp validity and observation time bounded by the stated chamber measurement interval.
- SHA-256 source digest shape.
- Uncertainty status and unit consistency.
- Detection-limit value, unit and basis.
- QC, instrument calibration and uncertainty warnings.

## Important limitations

This v0.1 auditor **does not integrate flux observations over a season**. Chamber closure time is the concentration-measurement window used to estimate a flux rate; it is not the time interval over which that flux represents the field. Seasonal integration needs a separate time-series/coverage contract with explicit sampling schedule, event windows, interpolation policy, maximum allowed observation gap and treatment of censored/non-detect observations.

It does not estimate treatment effects, infer missing fluxes, qualify an instrument, establish field efficacy, perform net carbon accounting, certify a material or authorize a physical trial. It refuses non-synthetic inputs to avoid an accidental claim of empirical qualification.

The fixture includes an intentionally flagged record with unavailable uncertainty. The report therefore demonstrates that structural validity and qualification status are separate: a parseable record collection can still be unqualified.

A SHA-256 digest provides integrity binding only; it does not establish truth, calibration, laboratory competence or source authority.
