# South African ShapID Alias Lifecycle Evaluator

Independent, effects-disabled Rust reference evaluator for the alias lifecycle corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-shapid-alias-lifecycle-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-shapid-alias-lifecycle-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-shapid-alias-lifecycle-evaluator/Cargo.toml -- --self-test
```

## Core rules

A phone number, SIM-control event, ShapID registration, bank-account binding, resolver response and payer confirmation are different facts. The evaluator requires provider-owned registration, identity/account binding and a fresh directory result for the exact alias generation and beneficiary commitment.

A number change, number release/reassignment, SIM-swap risk signal, revocation, account closure, stale directory epoch, conflicting cross-bank default or material display mismatch must not be silently treated as an active binding. Re-registration uses a new generation and new evidence. Old bindings remain immutable.

The corpus preserves ICASA's July 2026 Fourth Amendment announcement separately from the 29 October 2026 Fifth Amendment draft. The draft's proposed biometric checks are not treated as binding law. Unknown cooling-off intervals remain unknown.

## Claim ceiling

All cases are synthetic. A passing evaluator proves only that the reference contract rejects seeded lifecycle, generation, currentness and identity-confusion errors. It does not establish telecom number ownership, a bank account's legal owner, live PayShap directory status, provider SLA or a real payment integration. Dispatch is permanently disabled.
