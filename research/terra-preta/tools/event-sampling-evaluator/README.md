# Known-truth event sampling evaluator

Standalone Rust evaluator for **synthetic sampling schedules**. It does not choose a schedule or operate a sensor. The latent high-rate stream is joined to a schedule only after the schedule's decision trace has passed the no-future-observation check.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/event-sampling-evaluator/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/event-sampling-evaluator/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/event-sampling-evaluator/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/event-sampling-evaluator/Cargo.toml -- \
  research/terra-preta/tools/event-sampling-evaluator/fixtures/synthetic-short-pulse.json \
  /tmp/event-sampling-report.json
```

## Evaluated arms

- Fixed 6-hour cadence: deliberately misses a short pulse between boundary samples.
- Adaptive-from-past: each next sample decision references only the previous observation.
- 1-hour high-rate oracle: evaluation ceiling only, never a deployable candidate.

## Metrics

Pulse capture / miss, sampled peak, peak magnitude and timing error, integrated absolute anomaly only when boundaries and maximum gap qualify, sample count and energy. A missed pulse is not assigned zero flux. Under-resolved integration is null with a reason.

## Hard boundaries

- No policy may reference an observation it has not yet sampled.
- Every requested time must be present in the source truth grid.
- No interpolation is used to manufacture an unsampled peak.
- Integration is refused when a boundary is missing or the largest event-window sample gap exceeds the contract.
- A schedule that exceeds the sample or energy budget is marked as over budget.
- All source values are synthetic; this does not validate a real field sensor, seasonal flux, or adaptive policy efficacy.
