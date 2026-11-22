# PCOE Assurance Kernel

This is a dependency-free Rust reference model for the first executable slice of the Proof-Carrying Obligation Execution benchmark.

## Run

```sh
cargo fmt --check --manifest-path Cargo.toml
cargo test --locked --manifest-path Cargo.toml
```

## Implemented reference invariants

- Authority is checked against principal, actor, action, resource, amount, expiry and a current revocation epoch. BR3 or otherwise high-impact actions require a recorded human disposition.
- A simulated or producer-asserted quantity cannot be promoted to metered production.
- Meter interval, sequence, calibration coverage, rated capacity and integer Wh arithmetic are checked.
- Verifier attestations bind the exact claim and evidence digests. A separate witness check rejects producer self-verification, duplicate verifier identities and overlapping declared material failure domains.
- Milestone acceptance binds the exact obligation, contract digest, milestone digest and required evidence set.
- Offer fills use expected revisions, conservation, request-digest idempotency and replay receipts.
- Collateral identities are consumed once in the reference ledger.
- Payment intents distinguish in-flight, indeterminate, paid-final, rejected and manual-attested outcomes. Unknown outcomes block retry; final receipts must authenticate and match the exact obligation, parties, amount, currency, idempotency key and request digest.
- External receipt identifiers cannot settle two intents.
- The complete 39-scenario corpus is checked for ID presence and uniqueness; an explicit coverage map marks cases implemented, partial/adapter-dependent or not yet implemented.

## Deliberate limits

This model is not a production adapter and does not implement cryptographic signature verification, credentials, legal authority, revocation distribution, physical metrology, distributed transactions, atomic Holochain validation, a bank/payment rail, regulator rules, or full PCOE qualification. The `authentication_verified` receipt flag is a test seam representing an upstream authentication result, not cryptography. The witness failure-domain sets are declarations supplied by an external qualification process, not proof that dependencies are truly independent.

In-memory serialization of a method call is not a substitute for atomic integrity-level uniqueness and concurrent-agent tests. All distributed / field behaviors remain separate qualification gates.

## Scenario coverage

The source defines `SCENARIO_COVERAGE` for all 39 scenario IDs. It deliberately does not report full implementation. The Rust test only verifies that every mapped ID exists in the JSON corpus and that the corpus has 39 unique entries. Mapping status reflects this reference kernel slice, not the complete four-system architecture.
