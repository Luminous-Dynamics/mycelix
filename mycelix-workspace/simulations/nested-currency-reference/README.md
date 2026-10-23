# Nested currency reference model

This standalone, dependency-free crate is a **reference model**, not a Holochain zome, payment rail, cryptographic authorization implementation, or production ledger.

It models four narrow properties:

1. Instrument IDs are authoritative; ticker collisions do not alias instruments.
2. Delegated authority binds issuer, delegate, instrument, action, profile revision, expiry, and an optional amount ceiling.
3. Community mutual-credit updates are atomic at the model boundary, replay-idempotent, and preserve zero net position per instrument.
4. Federation clearing exposure is isolated per agreement, binds instrument and agreement revision, and permits existing obligations to settle after new obligations are paused.

It does **not** prove real-world service delivery, legal status, collateral/reserves, key security, Holochain DHT integrity, multi-agent race safety, external settlement finality, or that every production code path enforces these rules.

## Run locally

From the repository root, with a stable Rust toolchain installed:

```sh
cargo fmt --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml -- --check
cargo test --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml
```

The test corpus contains deterministic negative controls plus a million-operation conservation stress test and 100,000 retry checks. These are **targets encoded as tests**, not evidence of a passing run until the exact committed revision has been compiled and executed.

## Next qualification stages

- Add proptest or equivalent generated operation sequences with reproducible seeds.
- Add multi-agent tests for concurrent submissions, replay, reordering, and partial-write recovery.
- Bind all operations to canonical serialized profile/policy digests and consumed authorization IDs.
- Integrate only after the existing Finance author-binding, typed-amount, and settlement/finality requirements qualify.
