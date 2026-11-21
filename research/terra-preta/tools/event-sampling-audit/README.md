# Event-sampling observability evaluator

Evaluator-only Rust tool to measure whether candidate sampling schedules observe known synthetic soil pulses. It deliberately keeps the dense truth trace separate from the candidate schedule.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/event-sampling-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/event-sampling-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/event-sampling-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/event-sampling-audit/Cargo.toml -- \
  research/terra-preta/tools/event-sampling-audit/fixtures/synthetic-event-truth.json \
  research/terra-preta/tools/event-sampling-audit/fixtures/synthetic-candidate-schedule.json \
  /tmp/event-sampling-report.json
```

## Important boundary

The truth file is evaluator-only. A real candidate must never receive the event labels or dense unsampled trace. Candidate observations must be exact source timestamps and values; interpolation is forbidden. This is a manufactured high-rate source stream, not a real sensor record or validated sensor model. Resource figures are synthetic accounting inputs, not measured energy.
