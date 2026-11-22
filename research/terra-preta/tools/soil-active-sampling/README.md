# Soil active-sensing benchmark

Standalone Rust task/evaluator adapter for Symthaea's generic active-sensing planner.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/soil-active-sampling/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/soil-active-sampling/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/soil-active-sampling/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/soil-active-sampling/Cargo.toml -- \
  research/terra-preta/tools/soil-active-sampling/fixtures/short-rewetting-pulse.json \
  /tmp/soil-active-report.json
```

## Architecture

- The synthetic generator creates exact minute-resolution latent forcing and response.
- The task's public contract contains a finite horizon, sample budget, detection threshold and maximum integration gap.
- The candidate-policy callback receives only current time, public contract and past noisy observations.
- Fixed-cadence and rewetting-triggered policies are implemented as baselines.
- The evaluator samples the high-rate truth exactly at requested timestamps under a deterministic sensor-noise model.
- Hidden event onset, width, amplitude, exact pulse integral and future values are used only by the evaluator/report path.

The public and hidden structures are separate Rust types, and no candidate callback accepts hidden truth. However, v0.1 runs candidate and evaluator in one process; this is not an adversarial or OS-level isolation boundary. A confirmatory benchmark must move oracle ownership into a separately permissioned process.

## Metrics

- pulse detection and detection delay;
- sampled observations in the hidden event window;
- peak estimate/error;
- sampled-event trapezoidal integral/error, with an explicit abstention when fewer than two event samples exist or a gap exceeds the configured maximum;
- maximum sampling gap;
- sample count, synthetic sensor energy and synthetic wake energy;
- false-positive detection on a no-pulse control.

## Important limits

All event trajectories and sensor costs are manufactured. The noise generator is deterministic and illustrative, not a calibrated instrument model. The candidate/evaluator split is a typed API convention, not a security boundary. Compute, transmission, memory writes, chamber/lab cost, field operations and real sensor energy are not measured. No emission, field-resilience or agronomic claim follows from this benchmark.
