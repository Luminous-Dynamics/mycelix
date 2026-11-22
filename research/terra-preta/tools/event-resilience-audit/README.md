# Event-resolved soil resilience auditor

Standalone Rust auditor for a **single synthetic event in one experimental unit**. It reports descriptive peak response, recovery and pulse burden, without treating repeated observations as independent treatment replicates.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/event-resilience-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/event-resilience-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/event-resilience-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/event-resilience-audit/Cargo.toml -- \
  research/terra-preta/tools/event-resilience-audit/fixtures/synthetic-rewetting-event.json \
  /tmp/event-resilience-report.json
```

## Metrics

- Pre-event baseline mean.
- Maximum absolute stress-window departure, signed departure and timing.
- Post-stress recovery mean and departure.
- Descriptive recovery fraction, which is not clamped and may be negative after overshoot.
- Recovery time only when a tolerance band and consecutive-observation requirement are explicitly declared.
- Piecewise-linear positive, negative, signed and absolute anomaly integrals, with exact splitting at baseline crossings; only when event-start and recovery-end observations exist and every phase gap is within the configured maximum.

## Qualification boundaries

- If any phase has inadequate observation count, or any within-phase observation gap exceeds the configured maximum, response metrics abstain.
- Missing event boundaries or excessive gaps block pulse integration; the auditor does not interpolate through them.
- No recovery threshold means no recovery-time estimate.
- Serial covariance, sensor drift, shared calibration and uncertainty propagation are not implemented in v0.1. The report states this explicitly.
- One time series is not a causal treatment effect, and repeated readings are not independent experimental replicates.
- The fixture is synthetic-only. It does not qualify field resilience, GHG emissions, nutrient flux or any intervention.
