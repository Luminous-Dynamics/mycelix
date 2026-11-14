# Institutional Assurance Exchange v0.2 — end-to-end vertical slice

**Status:** synthetic development fixture. No legal, title, land-use, credential, transaction or production claim is made.

This version implements the bounded scope in Symthaea issue #12019: brief → candidate requirements → human-approved baseline → owner-native claim references → authority/evidence evaluation → decision packet → bitemporal replay after correction → unknown-operation reconciliation.

## Ownership boundaries

- Symthaea: candidate interpretation and requirement-source spans.
- Human governance: acceptance of the baseline.
- Mycelix: claim/evidence references, authority evaluation, decision lifecycle and replay packet.
- Domain systems: authoritative title, custody and land-use facts.
- External operation provider: execution state/receipt, never inferred from client timeout.

## Scenario

A synthetic stewardship trust considers a commercial lease for one property. The brief includes a mandatory title-holder check, a conditional land-use-variance requirement, a preferred condition inspection, and an unresolved continuity sentence.

A land-use correction is effective from 15 January 2026 but is only observed on 5 February 2026. The same effective-time query therefore has different valid historical knowledge views. A lease-submission operation timed out after a possible mutation and remains UNKNOWN.

## Run

```sh
cargo test --manifest-path benchmarks/institutional-assurance-exchange/v0.2/harness/Cargo.toml
cargo run --manifest-path benchmarks/institutional-assurance-exchange/v0.2/harness/Cargo.toml -- \
  benchmarks/institutional-assurance-exchange/v0.2/fixture.json \
  benchmarks/institutional-assurance-exchange/v0.2/oracle.json \
  benchmarks/institutional-assurance-exchange/v0.2/examples/reference-submission.json
```

The evaluator compares submitted decisions with a separate oracle, checks temporal selection and authority status from the fixture, enforces claim competence and cross-reference invariants, detects blind retries from the full event history, and emits SHA-256 receipts for the exact fixture, oracle, submission and evaluator executable.

All timestamps are canonical UTC ISO 8601 strings ending in `Z`. The evaluator's lexical comparison is valid only for this canonical format.

## Claim ceiling

The digest fields are explicitly synthetic placeholders and are not cryptographically verified. The baseline approval is a synthetic human-record reference; the evaluator does not authenticate the human or legal effect of approval. The oracle is not independently adjudicated. A passing synthetic test is not operational/legal qualification.


See [Research baseline](./RESEARCH_BASELINE_2026-11-14.md) for current SABench, TEMPO, requirements-quality and NIST critical-infrastructure references. The vertical-slice evaluator now audits normalized possible-write traces against explicit action/purpose/actor/asset/field authorization boundaries.


## Exact-operation finality boundary

The fixture contains a separate synthetic human approval for the exact lease operation, distinct from the human acceptance of the intent baseline. The operation projection binds logical operation ID, canonical/action digests (explicit synthetic placeholders), action, principal, target, purpose, destination/audience, resource revision and idempotency identity.

See [LISE mapping](./LISE_MAPPING.md). IAE owns the intent/claim/evidence packet; LISE owns exact-operation finality and retry/reconciliation semantics. This branch validates the mapping structurally but does not yet call the separate LISE Rust crate because that crate remains on its own draft PR.

