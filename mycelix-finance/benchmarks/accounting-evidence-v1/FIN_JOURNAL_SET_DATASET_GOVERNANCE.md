# FIN-JOURNAL-SET dataset governance

## Immutable releases

- **v1.0.0:** 25 synthetic period journal-set cases, v1.0.0 candidate schema, public oracle and oracle schema.
- **v1.1.0:** 26 synthetic cases with version-specific candidate/oracle schemas. It adds FJS-026: two individually under-limit partial reversals (1,000 + 1,001 minor units) exceed an original 2,000-unit posting in aggregate. The verifier now tracks cumulative remaining reversal capacity per original journal.

Both releases are regression/conformance suites, not blind model holdouts, professional accounting datasets or benchmark leaderboards. Keep all v1.0.0 bytes and hashes immutable. New cases or semantic changes require a new dataset version.

All entities, journals, events, account names, unit profiles, commitments and amounts are synthetic. No raw bank payload, customer record, real ledger or account credential is included.

Candidate inputs and public reference oracles are separate artifacts. Public reference oracles must never be mounted into candidate environments when measuring model performance. A future blind benchmark requires a separate access-controlled evaluator and separately frozen hidden cases.

Fixture digest tokens are synthetic tokens used to exercise identity/replay logic. They are not SHA-256 hashes of raw journal source bytes. The verifier hashes exact corpus/schema/oracle/verifier source bytes for its report, but does not create or authenticate upstream journal artifacts.

Release rules:
1. Preserve every released schema, candidate and public-oracle byte sequence.
2. Add cases or semantic changes only in a new dataset version.
3. Include positive and negative controls for each invariant.
4. Keep exact expected outcomes in the oracle only.
5. Have a qualified accounting reviewer adjudicate ambiguous posting/correction policy cases before creating a private holdout.
6. Never describe a public-oracle PASS as blind evaluation, audit sufficiency or production close assurance.
