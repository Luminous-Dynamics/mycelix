# Farmer journal failure-injection harness

This standalone Rust crate is a **reference test harness**, not a browser/IndexedDB journal and not a production remote dispatcher. It exercises write-before-send and unknown-outcome rules with a deterministic fake durable store and fake remote.

## Run

From the repository root:

```sh
cargo test --manifest-path research/farmer-ux-005-journal-harness/Cargo.toml
cargo run --manifest-path research/farmer-ux-005-journal-harness/Cargo.toml -- --root .
```

A reviewed and committed `Cargo.lock` is still required before this harness can be called reproducible or used as a locked merge gate; the current draft workflow deliberately uses exact top-level dependency versions but does not yet claim transitive lockfile reproducibility.

## Contracts exercised

- The code distinguishes eligibility to start a new attempt from actual network-send authorization: a successful, verified durable Submitted marker is required before any fake remote send.
- Duplicate operation IDs do not trigger another send, and a previously fixed operation ID cannot be silently rebound to a new target scope.
- Durable Prepared and Submitted writes must succeed before any fake remote send.
- Failed Submitted commit means no send.
- A lost or malformed response becomes OutcomeUnknown, even if the fake remote may have applied the request.
- On restart, Submitted recovers as OutcomeUnknown and is not dispatch-eligible.
- Only typed authoritative responses produce Confirmed or ExplicitlyRejected.
- Result-persistence failure after a remote response recovers as OutcomeUnknown.
- Corrupt or unknown-version journal bytes are not treated as proof that nothing was sent.
- Duplicate operation IDs and selected-site retargeting fail closed.
- The 16-case JSON Schema Draft 2020-12 corpus and state mapping are validated, including no-auto-replay and reconciliation invariants.
- The report hashes exact fixture/schema bytes and carries an explicit claim ceiling.

## Limits

Passing tests prove only this fake model and committed fixture consistency. They do not prove browser storage transaction durability, real Holochain behavior, server idempotency, provider receipt, UI usability, agricultural advice quality, or field impact. The shared Hearth action lifecycle is open/unmerged; this harness is a separate oracle until an approved shared API exists.
