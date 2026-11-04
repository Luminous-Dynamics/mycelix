# South African Payment Fraud Benchmark Validator

Effects-disabled Rust validator for the source-bound benchmark manifest.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-fraud-benchmark-validator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-fraud-benchmark-validator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-fraud-benchmark-validator/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/sa-payment-fraud-benchmark-validator/Cargo.toml --all-targets -- -D warnings
```

## Scope

Checks schema conformance, source/cohort/metric identities, source references, time bounds, ratio denominators, claim eligibility bounds, explicit SynSEPA row-count discrepancy, and frozen evaluation contract rules. It rejects random-row-only evaluation, post-outcome leakage, cross-jurisdiction extrapolation and a single composite score.

It cannot ingest or access real participant data, train/evaluate a fraud model, block a payment, determine legal eligibility, or claim production SOTA performance. Its baseline cohort numbers are public comparator snapshots with strict scope tags.
