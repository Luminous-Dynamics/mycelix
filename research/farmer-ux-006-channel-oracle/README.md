# FARMER-UX-006 channel continuity oracle

A standalone Rust reference verifier for the synthetic cross-channel contract at the repository root. It is deliberately not a production adapter, service evaluator, durable journal, Holochain zome or identity/consent provider.

## Run

From the Mycelix repository root:

```sh
cargo test --manifest-path research/farmer-ux-006-channel-oracle/Cargo.toml
cargo run --manifest-path research/farmer-ux-006-channel-oracle/Cargo.toml -- --root . --report-json research/farmer-ux-006-channel-oracle/target/channel-oracle-report.json
```

The focused workflow runs both commands and uploads the JSON report.

## What is executable here

- Full JSON Schema Draft 2020-12 schema meta-validation and instance validation of the 17-case FUXC corpus and the synthetic relay-envelope example.
- Semantic guards for every frozen case: continuity, event replay vs equivocation, actor/recorder attribution, consent revocation, source/time provenance, unknown outcomes, correction history, no-provider outcomes, farmer acceptance, disputes and denominator preservation.
- A deliberately small reference ledger: exact same-event replay is idempotent; same event ID with a changed digest or request binding is a conflict and cannot overwrite the first-seen event; separate request references are not fuzzy-merged.
- Fail-closed promotion rules for the high-risk transitions covered in the contract.
- 10,000 deterministic schedules per verifier run. Each schedule reorders eight events and injects one exact replay plus one same-ID/different-digest conflict. The seed and shuffle algorithm are fixed; no randomness dependency is used.
- Mutation tests that remove required conflict, relay, consent and unknown-outcome protections and require the semantic verifier to reject the mutated corpus.
- A JSON report with SHA-256 digests of source fixtures and schemas, schema-engine results, semantic errors, schedule count and an explicit claim ceiling.

## Interpret results honestly

A passing model proves only the code and fixtures in this crate. It does not prove browser/IndexedDB durability, server-side idempotency, real actor identity or consent, secure field relaying, provider receipt, bookings, service fulfilment, farmer comprehension, accessibility conformance, or agricultural benefit. The schedule generator is a model-based stress test—not a statistically representative field study or exhaustive proof of all event histories.

The contract and this verifier remain a stacked draft. Transitive dependency lockfile reproducibility is not qualified until a reviewed `Cargo.lock` is committed. The model must be checked against the exact reviewed shared journal/service-evaluator contracts before production integration.
