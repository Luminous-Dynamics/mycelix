# Active event-sampling schedule evaluator

Standalone Rust evaluator for **frozen synthetic sampling traces**. Policy generation is owned by Symthaea EDGE-ACTIVE-SENSING-001 #9817. This tool checks what each policy actually sampled, what it missed, and what resource vector it consumed.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/active-sampling-eval/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/active-sampling-eval/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/active-sampling-eval/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/active-sampling-eval/Cargo.toml -- \
  research/terra-preta/tools/active-sampling-eval/fixtures/synthetic-active-sampling.json \
  /tmp/active-sampling-report.json
```

## What it evaluates

- Fixed sparse schedule, context-triggered burst schedule, and a hindsight-only upper bound.
- Each frozen policy trace is replayed against both a short-pulse truth case and a no-pulse case with the same public context stream.
- The evaluator computes target response at actual acquisition time after sensor latency.
- It reports capture, detection delay, peak magnitude/timing error, exact-boundary pulse integral (when available), false burst samples, energy, assay cost and communication bytes.
- Target/context counts, request chronology, action budgets, target rate, latency, public-context references and no-overwrite output are enforced.

## Important boundaries

- This code evaluates schedules; it does not generate a policy.
- The evaluator input includes hidden truth and must never be mounted into the candidate-policy process in confirmatory use.
- The `hindsight_oracle` is non-deployable and is only an upper-bound reference.
- Sparse historical data cannot be used to invent counterfactual observations at unsampled times.
- Pulse integrals are anomaly unit-hours, not seasonal gas mass unless a future qualified measurement model and physical-unit conversion explicitly establish that.
- Resource dimensions remain separate. Do not collapse energy, assay cost, latency and bytes into one score without a declared utility.
- All fixture values and schedules are manufactured, not actual Symthaea measurements or field results.
