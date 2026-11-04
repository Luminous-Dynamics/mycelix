# South African Payment Fraud Signal Evaluator

Independent, effects-disabled Rust reference evaluator for the synthetic federated signal contract.

```sh
cargo fmt --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --self-test
```

The evaluator validates source/profile references, subject-reference privacy, purpose/audience scope, evidence independence, typed status transitions, expiry, correction, withdrawal, appeal state, disclosure minimisation and receiving-participant decision boundaries.

It does not score customers, infer criminality, query a live bank, create or update a national fraud registry, disclose customer data, or make payment decisions.

All fixtures use synthetic participant and subject tokens. PASS is limited to the checked-in schema and semantic corpus.
