# Building measurement condition applicability oracle

This is a synthetic, contract-conformance slice under Mycelix #6065. It composes with the existing instrument passport and does not change passport v0.1.

## What it checks

- Six condition-profile families: thermal, optical PM, gas, water flow, electrical and acoustics.
- Exact registry digest, profile/channel/measurand, firmware/configuration and date applicability.
- Required numeric/categorical influence variables, uncertainty bounds and evidence state.
- Sensor-local geometry/location and temporal coverage using the exact stream/clock/boot-epoch fields expected by Mycelix #6054.
- Correction method/revision/digest/receipt and exactly-once application where a synthetic profile requires correction.
- 43 frozen negative/known-answer cases and strict closed JSON Schemas.

## Evidence ceiling

Every profile and positive condition observation is `synthetic_reference_only`. No record claims to be an independent measurement, current calibration, verified installation or real rights grant. The sole positive disposition is `SYNTHETIC_CONDITION_CONTRACT_VALIDATION_ONLY`; it is not instrument admission, physical review eligibility or a service-quality result.

Real condition applicability must be backed by owner-verified source observations, current calibration and installation records, exact stream/timebase receipts, validated methods/corrections and purpose-specific uncertainty. Mycelix #6048 owns uncertainty computation, #6054 owns stream/timebase, #6042 owns descendant invalidation, and #6047 owns digital calibration-certificate verification.

## Run

```sh
cargo test --locked --manifest-path tools/build-measurement-conditions/Cargo.toml
cargo run --locked --manifest-path tools/build-measurement-conditions/Cargo.toml
```

The hosted workflow uses an offline locked test/Clippy/CLI sequence after `cargo fetch --locked`. Queued, skipped or pending is not PASS.
