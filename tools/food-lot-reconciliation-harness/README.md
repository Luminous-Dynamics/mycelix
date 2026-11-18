# Food Lot Reconciliation Harness

This minimal Cargo package imports the exact tracked production source file at:

`mycelix-commons/zomes/food-distribution/integrity/src/food_lot_reconciliation.rs`

It runs the module's in-source unit tests without resolving the complete Holochain workspace.

## Run

```sh
cargo fmt --manifest-path tools/food-lot-reconciliation-harness/Cargo.toml --all -- --check
cargo test --manifest-path tools/food-lot-reconciliation-harness/Cargo.toml
cargo clippy --manifest-path tools/food-lot-reconciliation-harness/Cargo.toml --all-targets -- -D warnings
```

## Claim ceiling

A passing run verifies the pure Rust request/report/reconciliation source and its included tests at the exact checkout. It does not compile the Holochain integrity zome in its production target, verify coordinator retrieval, authenticate DHT action authors, implement private local storage/consent, establish physical inventory or food safety, or qualify an appliance.
