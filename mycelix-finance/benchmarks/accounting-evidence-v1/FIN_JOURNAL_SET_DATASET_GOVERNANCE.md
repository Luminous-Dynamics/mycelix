# FIN-JOURNAL-SET dataset governance

v1.0.0 contains 25 synthetic period journal-set cases and a public reference oracle. It is a regression/conformance suite, not a blind model holdout, professional accounting dataset or benchmark leaderboard.

All entities, journals, events, account names, unit profiles, commitments and amounts are synthetic. No raw bank payload, customer record, real ledger or account credential is included.

The candidate-visible input and public oracle are separate files. The public oracle must never be mounted into a candidate runtime when measuring model performance. A future blind benchmark requires a separate access-controlled evaluator and separately frozen hidden cases.

Fixture digest tokens are synthetic tokens used to exercise identity/replay logic. They are not SHA-256 hashes of raw journal source bytes. The verifier hashes the exact corpus/schema/oracle/verifier source bytes for its run receipt, but it does not create or authenticate upstream journal artifacts.

Release rules:
1. Preserve v1.0.0 schema, candidate bytes and oracle bytes once frozen.
2. Add cases or semantic changes only in a new dataset version.
3. Include positive and negative controls for each invariant.
4. Keep exact expected outcomes in the oracle only.
5. Have a qualified accounting reviewer adjudicate ambiguous posting/correction policy cases before creating a private holdout.
6. Never describe a public-oracle PASS as blind evaluation, audit sufficiency or production close assurance.
