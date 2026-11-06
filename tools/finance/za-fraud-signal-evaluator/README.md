# South African Fraud-Signal Exchange Evaluator

Independent, effects-disabled Rust reference evaluator for the synthetic fraud-signal exchange corpus.

```sh
cargo fmt --manifest-path tools/finance/za-fraud-signal-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/za-fraud-signal-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/za-fraud-signal-evaluator/Cargo.toml -- --self-test
```

The evaluator validates the Draft 2020-12 schema and semantic boundaries for strategic aggregation, operational signal authority/purpose/audience, token scope, provenance, epistemic status, recipient-use limits, expiry, disputes and correction propagation.

It does not verify the underlying legal basis, cryptographic signature, truth of an allegation or recipient's real-world governance. It does not contact banks or SARB, move funds, disclose personal information, create risk scores or trigger account/payment actions.

All fixtures are synthetic. A PASS is not a legal opinion, regulatory safe harbour or production-compliance claim.
