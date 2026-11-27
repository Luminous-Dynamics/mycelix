# aero-adapter-replay-benchmark

Pairwise replay comparison for content-verified AeroCommons adapter runs.

## Run

```sh
cargo fmt --manifest-path tools/aero-adapter-replay-benchmark/Cargo.toml -- --check
cargo test --manifest-path tools/aero-adapter-replay-benchmark/Cargo.toml
cargo clippy --manifest-path tools/aero-adapter-replay-benchmark/Cargo.toml --all-targets -- -D warnings
```

## API

- `compare_verified_runs` verifies both complete receipt payload sets using #6354 before classification.
- `ReplayDisposition` separates exact byte replay, stable normalized assertion bytes with raw output drift, normalized assertion byte drift, incomparable contexts and ineligible runs.
- `ReplayPanelSummary` preserves every offered comparison in denominator counts.

Different executable digests are incomparable same-toolchain replay contexts. They belong in a separate differential interoperability campaign with #6349 and an independent oracle.

## Claim limit

This is not a converter, runner, sandbox, signature verifier, semantic evaluator or production acceptance gate. Output drift is not automatically semantic loss; byte identity is not semantic correctness.
