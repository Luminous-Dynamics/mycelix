# South African Payment Recourse Evaluator

Independent, effects-disabled Rust reference evaluator for the South African payment recourse corpus.

## Run

From the repository root:

```sh
cargo fmt --manifest-path tools/finance/sa-payment-recourse-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-recourse-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-recourse-evaluator/Cargo.toml -- --self-test
```

The evaluator validates Draft 2020-12 schema conformance and semantic invariants. It does not contact banks, submit claims, disclose customer data, initiate transfers or make legal determinations.

## Core invariant

A recall request, acknowledgement, trace, funds location or hold does not increase recovered value. Only a sequenced `funds_returned` event with valid actor authority, an independent return receipt and independent ledger reconciliation can do so.

All values use exact integer minor units. A returned amount cannot exceed the original claim. Partial returns preserve residual loss. Closed cases require an explicit disposition.

## Claim ceiling

The checked-in cases are synthetic. A passing evaluator demonstrates that seeded false-recovery, authority, evidence, temporal and case-accounting mutations are rejected. It does not establish a bank's actual remedy, a user's legal entitlement, a real recovery capability or safe real-money operation.
