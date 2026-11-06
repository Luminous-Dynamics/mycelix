# Agentic Payment Delegation Evaluator

Effects-disabled Rust reference evaluator for #5851.

It validates the Draft 2020-12 schema and deterministic constraints over opaque QIE and exact ActionRequest references. It does not define those generic objects and cannot dispatch payments, use credentials or change a budget ledger.

## Run

```sh
cargo fmt --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml -- --check
cargo test --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml
cargo run --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml -- --self-test
```

A green focused workflow proves only the synthetic corpus's stated invariants.
