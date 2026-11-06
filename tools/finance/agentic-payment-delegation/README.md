# Agentic Payment Delegation Evaluator

Effects-disabled Rust reference evaluator for #5851.

It validates a Draft 2020-12 corpus of 32 synthetic cases across four source-bound policy/comparator profiles. It checks payment-specific constraints over opaque QIE and exact ActionRequest references, atomic reservation event accounting, amount/count/concurrency budgets, known fee/FX caps, revocation and non-expanding sub-delegation.

It does not define or authorize generic QIE, Intent, ActionRequest, principal authority, credentials or payment dispatch. It cannot contact banks, spend funds, issue credentials, or mutate a production budget store.

## Run

```sh
cargo fmt --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml -- --check
cargo test --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml
cargo run --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/agentic-payment-delegation/Cargo.toml --all-targets -- -D warnings
```

Unknown fee/FX data requires terms requalification; it is not a user-confirmation problem. Unknown payment outcomes retain reservations until independent no-effect reconciliation. Returns do not replenish mandate budget by default. Revocation blocks future actions but cannot reverse an already dispatched payment.

The corpus's event sequence is a deterministic reference linearization order, not evidence of production distributed atomicity. A live implementation requires a durable serializable/compare-and-swap or equivalent fencing contract and separate qualification of time, principal approval, QIE, exact ActionRequest and provider evidence.

A passing focused workflow proves only the seeded synthetic invariants; it does not prove legal mandate validity, provider acceptance, cryptographic authentication, production atomicity or real-money safety.
