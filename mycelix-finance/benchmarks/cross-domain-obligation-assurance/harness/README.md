# PCOE Assurance Kernel

This is a dependency-free Rust reference model for the first executable slice of the Proof-Carrying Obligation Execution benchmark.

## Run

```sh
cargo fmt --check --manifest-path Cargo.toml
cargo test --locked --manifest-path Cargo.toml
```

## Implemented reference invariants

- Authority is checked against a distinct principal and actor, grant ID, task, audience, action, resource, amount, expiry and a current revocation epoch. BR3 or otherwise high-impact actions require an allow-listed, distinct human approver, an approval bound to the exact request digest, an approval freshness window, and an upstream-authentication test seam. Direct child delegation is checked for attenuation across principal, task, audience, action, resource, amount, validity interval, revocation epoch, one-time constraints and human-approval policy. One-time grant consumption is modeled separately.
- A simulated or producer-asserted quantity cannot be promoted to metered production.
- Meter interval, sequence identity, calibration coverage, expected measurement boundary, policy-bounded tolerance, rated capacity, non-negative time and integer Wh arithmetic are checked.
- Verifier attestations bind the exact claim and evidence digests and carry their witness set. Validation requires the attesting verifier to be a member of a minimum two-witness set and rejects producer self-verification, duplicate verifier identities and overlapping declared material failure domains.
- Milestone acceptance binds the exact obligation, contract digest, milestone digest and required evidence set.
- Offer fills use expected revisions, conservation, request-digest idempotency and replay receipts.
- Collateral identities are consumed once in the reference ledger.
- Payment intents distinguish in-flight, indeterminate, paid-final, rejected and manual-attested outcomes. Unknown outcomes block retry; final receipts must authenticate and match the exact obligation, parties, amount, currency, idempotency key and request digest.
- External receipt identifiers cannot settle two intents.
- Energy attributes are a separate ledger projection from physical generation and power delivery. Eligible generation is bound to a registry/product code and exact measurement boundary; issuance is denomination-bound and cannot exceed verified eligible generation; ownership transfers are distinct from terminal retirement.
- Deterministic stress tests exercise offer-fill conservation and EAC issuance/transfer/retirement over thousands of operations.
- Four separate corpora are checked for ID presence and uniqueness: 39 core PCOE scenarios, 10 NIST/OWASP-aligned agent-authority scenarios, 10 SCITT/metrology/market-adapter scenarios, and 12 EAC anti-double-counting scenarios. Every suite has an explicit coverage map that distinguishes implemented reference tests from adapter-dependent and not-yet-implemented cases.

## Deliberate limits

This model is not a production adapter and does not implement cryptographic signature verification, credential verification, legal authority, multi-hop cryptographic delegation-chain verification, revocation distribution, continuous monitoring, independent immutable logging, physical metrology, distributed transactions, atomic Holochain validation, a bank/payment rail, regulator rules, or full PCOE qualification. The `authentication_verified` receipt flag and EAC generation-verifier qualification fields are test seams representing upstream authentication/accreditation results, not cryptography or an accredited EAC registry. The witness failure-domain sets are declarations supplied by an external qualification process, not proof that dependencies are truly independent.

In-memory serialization of a method call is not a substitute for atomic integrity-level uniqueness and concurrent-agent tests. All distributed / field behaviors remain separate qualification gates.

## Scenario coverage

The source defines `SCENARIO_COVERAGE` for 39 core PCOE IDs, `AGENT_SCENARIO_COVERAGE` for ten NIST/OWASP agent-authority IDs, `TRANSPARENCY_METROLOGY_SCENARIO_COVERAGE` for ten SCITT/metrology/market-adapter IDs, and `ENERGY_ATTRIBUTE_SCENARIO_COVERAGE` for twelve EAC anti-double-counting IDs. Rust tests verify the JSON inventories and unique IDs. Transparency/metrology/market cases remain adapter/field targets; cross-registry EAC issuance, time matching, storage provenance, certificate corrections and full claim-evidence provenance are likewise not fully implemented. Mapping status reflects this reference kernel slice, not the complete four-system architecture.
