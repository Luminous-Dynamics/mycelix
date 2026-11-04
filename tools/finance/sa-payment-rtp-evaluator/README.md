# South African Request-to-Pay Evaluator

Independent, effects-disabled Rust reference evaluator for the temporal Request-to-Pay corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-rtp-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-rtp-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-rtp-evaluator/Cargo.toml -- --self-test
```

The evaluator validates the Draft 2020-12 schema, source/profile references, request status transitions, immutable request digest binding, payer authority, expiry/cancellation/decline handling, and single-instruction semantics.

It cannot submit requests, call banks, create payment instructions in a live system, disclose customer data or move money.

## Non-negotiable boundary

A request being created, delivered, viewed or accepted is not proof of settlement. A valid acceptance may authorise one payment instruction. Provider execution, recipient availability, settlement and finality remain separate evidence states.

The Capitec current product-page snapshot and older February 2025 explainer are preserved as separate temporal observations because their published amount limits differ. The evaluator does not guess an effective date.

## Claim ceiling

All checked-in cases are synthetic. A passing evaluator proves only that the reference contract rejects seeded stale-consent, replay, expiry, profile and status-promotion mutations. It does not establish a live PayShap Request integration or a current provider guarantee.