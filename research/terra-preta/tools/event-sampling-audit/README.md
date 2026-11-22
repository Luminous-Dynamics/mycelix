# Event-sampling observability evaluator

Evaluator-only Rust tool to measure whether candidate sampling schedules observe known synthetic soil pulses. Candidate action traces contain timestamps and declared prior-observation IDs only; the evaluator joins response values from its hidden truth file after schedule submission.

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

## Policy bake-off fixtures

- `synthetic-fixed-30m-schedule.json`: 49 samples, misses both short pulses.
- `synthetic-candidate-schedule.json`: 54 samples, captures the wider pulse and misses the narrower one.
- `synthetic-fixed-5m-schedule.json`: 289 samples, captures both pulses but exceeds the 60-sample budget.

All three use the same evaluator-only truth file. These demonstrate the recall / resource trade-off in a manufactured example, not real-world optimality.

## Candidate schedule / decision trace

Each candidate action records the time the decision was made, the requested sample timestamp, and the IDs of previously sampled observations visible to that decision. Validation rejects unknown/future observation references and decisions made before an observation was acquired. Candidate files contain no response values; the evaluator binds each requested timestamp to its exact source value after the trace is frozen.

## Important boundary

The offline CLI reads both truth and the frozen schedule, so it cannot itself prove policy-process isolation. It reports `policy_evaluator_isolation_status=not_enforced_by_offline_evaluator`. Production qualification requires a separate sandboxed policy run that receives only allowed context and past sensor observations; the evaluator should only receive the signed completed schedule after the policy process exits. No hidden response value is interpolated. This is a manufactured high-rate source stream, not a real sensor record or validated sensor model. Resource figures are synthetic accounting inputs, not measured energy.
