# South African Payment Fraud-Signal Evaluator

Independent Rust reference evaluator for the synthetic payment fraud-signal admission corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml --all-targets -- -D warnings
```

The evaluator checks Draft 2020-12 structure, source/profile references, synthetic key-verification receipts, issuer scope, recipient and purpose admission, pairwise subject binding, retention, dispute/correction state, evidence independence, special-information review fixtures and permitted action boundaries.

## Security and legal boundary

This crate does not perform production signature verification, identify real people, establish a lawful basis, send data to a participant, or take an action against a customer. The checked-in verification and legal-review receipts are synthetic test evidence only. A real implementation must compose #5777/#5778, #5215/#5220, #5798, #5392 and #2437, and receive participant-specific South African privacy/regulatory review.

## Claim ceiling

A passing corpus shows that this exact evaluator rejects the seeded synthetic cases. It does not demonstrate fraud-detection accuracy, POPIA or FSR Act compliance, a live NPU service, SARB endorsement, a valid real-world data-sharing agreement or a safe adverse-action system.
