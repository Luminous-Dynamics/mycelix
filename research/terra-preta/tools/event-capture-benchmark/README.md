# Known-truth soil event-capture benchmark

Rust evaluator for submitted event-sampling schedules. The policy schedule and evaluator-only dense truth are separate inputs.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/event-capture-benchmark/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/event-capture-benchmark/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/event-capture-benchmark/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/event-capture-benchmark/Cargo.toml -- \
  research/terra-preta/tools/event-capture-benchmark/fixtures/policy-submission.json \
  research/terra-preta/tools/event-capture-benchmark/fixtures/evaluator-truth.json \
  /tmp/event-capture-report.json
```

## Separation

- `policy-submission.json` contains policy identity/digests and requested timestamps only. No sampled response values, forcing values, event labels or latent truth.
- `evaluator-truth.json` is a separate evaluator-only input with a manufactured high-rate latent stream and event boundaries.
- The CLI accepts these as separate paths, computes separate SHA-256 digests, and scores only exact timestamp matches.
- A real policy runner must be sandboxed so its process cannot read evaluator truth. Separate CLI arguments/schema do not alone prove truth-blind execution.

## Metrics

For each submitted policy and each event:
- event detected / missed, first detection time and delay;
- true vs sampled peak magnitude/time and absolute peak error;
- true vs sampled integrated absolute anomaly and absolute/relative error;
- event sample count, largest sampled gap and explicit integral qualification status;
- per-policy sample count/cost, budget, out-of-event samples and false triggers.

No aggregate policy score is emitted. The evaluator does not execute an optimiser, and the static schedules in the fixture are not live Symthaea policies.

## Fail-closed behaviour

- Unknown policy fields are rejected by the schema and Rust deserializer.
- Times must be strictly ordered and match the truth grid exactly.
- No nearest-neighbour or interpolated sample values are generated.
- An event integral requires both event boundaries and all gaps within the declared maximum.
- Output uses create-new semantics and cannot overwrite an existing report.
- The truth stream and policies are manufactured. No empirical pulses or emission factors are encoded.
