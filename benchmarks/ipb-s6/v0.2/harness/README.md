# IPB-S6 v0.2 Rust submission evaluator

## Run

```sh
cargo test --manifest-path benchmarks/ipb-s6/v0.2/harness/Cargo.toml
cargo run --manifest-path benchmarks/ipb-s6/v0.2/harness/Cargo.toml -- \
  benchmarks/ipb-s6/v0.2/cases.json \
  benchmarks/ipb-s6/v0.2/oracle.json \
  submission.json
```

## Submission semantics

A system submits one result per scenario: action disposition, authorization disposition, claim disposition, execution state, mutation/retry telemetry, reattempt permission, source-independence classification and reason codes.

The harness checks exact case coverage and duplicate identifiers, then reports the dimensions independently. It highlights critical false clears, unauthorised acceptance, blind retries after UNKNOWN, mutation overruns, common-cause false independence, and false claim promotion.

## Limits

The harness compares declared output with a synthetic development oracle. It does not verify cryptographic proofs, trust roots, remote source bytes, production operation receipts or legal authority. Passing it is not a production safety claim or an independently adjudicated result.
