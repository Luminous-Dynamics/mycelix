# Nested currency reference model

This standalone, dependency-free crate is a **reference model**, not a Holochain zome, payment rail, cryptographic authorization implementation, or production ledger.

It models these narrow properties:

1. Instrument IDs are authoritative; ticker collisions do not alias instruments or balances.
2. Authorization and mutual-credit mutations resolve profiles from a canonical registry rather than accepting arbitrary caller-supplied profiles; denomination identity and scale cannot silently change for an instrument ID, and same-revision policy mutations are rejected.
3. Delegated authority binds issuer, delegate, instrument, action, profile revision, expiry, and an amount ceiling; missing or negative amounts fail closed.
4. Community mutual-credit updates are atomic at the model boundary, replay-idempotent, and preserve zero net position per instrument.
5. Federation clearing enforces per-edge and per-instrument aggregate community gross-exposure limits; unlike instrument units are not added together as raw numbers. Agreements bind instrument/revision and allow already-incurred obligations to settle after new obligations are paused.

It does **not** prove real-world service delivery, legal status, collateral/reserves, key security, Holochain DHT integrity, multi-agent race safety, external settlement finality, or that every production code path enforces these rules.

## Run locally

From the repository root, with a stable Rust toolchain installed:

```sh
cargo fmt --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml -- --check
cargo test --manifest-path mycelix-workspace/simulations/nested-currency-reference/Cargo.toml
```

The test corpus contains deterministic negative controls plus a million-operation conservation stress test and 100,000 retry checks. It includes profile-registry binding, missing/negative amount caps, stale revision rejection, aggregate exposure caps, failed-operation state preservation, ticker-collision isolation, and settlement-after-pause semantics. These are **targets encoded as tests**, not evidence of a passing run until the exact committed revision has been compiled and executed.

## Next qualification stages

- Add proptest or equivalent generated operation sequences with reproducible seeds.
- Add multi-agent tests for concurrent submissions, replay, reordering, and partial-write recovery.
- Bind all operations to canonical serialized profile/policy digests and consumed authorization IDs.
- Extend profile-version tests into signed policy-digest and authorized migration receipts; the in-memory snapshot is not a cryptographic authority proof.
- Integrate only after the existing Finance author-binding, typed-amount, and settlement/finality requirements qualify.
