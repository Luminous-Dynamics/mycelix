# FIN-ASSURE-PROCEDURE dataset governance

## Immutable releases

- **v1.0.0:** 18 synthetic cases with its v1.0.0 candidate schema and public reference oracle.
- **v1.1.0:** 22 synthetic cases with version-specific candidate and oracle schemas. Adds explicit reporting-entity binding, procedure-purpose binding, pair-level stand-back receipts, multi-pair coverage and controls for purpose mismatch, cross-entity evidence and analytical threshold exceedance.
- **v1.2.0:** 25 synthetic cases with version-specific candidate, oracle, input schema and report schema. Removes the global `reliability_status` scalar; adds independent `accuracy`, `completeness`, `authenticity`, `absence_of_bias` and `credibility` fields, plus a pair-specific `required_evidence_attributes` list. Adds a multi-pair same-evidence case and separate negative controls for limited absence-of-bias and unqualified credibility.

Each release is a regression/conformance corpus, not a blind holdout, professional audit dataset or benchmark leaderboard. Released schemas, candidates and public-oracle artifacts remain immutable and replayable. New cases or semantic changes require a new version. The v1.2.0 report schema is versioned separately; the verifier's default report schema is the v1.2.0 snapshot.

All entities, risks, documents, origin groups and periods are synthetic. No client data or audit workpapers are included. Public reference oracles must not be mounted in a candidate environment when measuring model performance. A separate access-controlled evaluator is required for blind candidate comparison.

The fixture set encodes machine-checkable evidence-purpose, entity/period, population, denominator, origin-independence, attribute-specific reliability, contradiction, procedure-state, stale-workpaper and human-judgment boundaries. It does not encode the full IAASB exposure drafts or claim compliance with current/proposed auditing standards.

Dataset extension rules:
1. Preserve released schema, candidate, public-oracle and report-schema bytes and hashes.
2. Add positive and negative controls for every new invariant.
3. Keep candidate inputs, public reference outcomes and any future blind evaluator physically separate.
4. Keep standards mapping and expert adjudication separate from deterministic fixture correctness.
5. Report pair-level outcomes; do not let a case-level aggregate hide a blocked risk/assertion pair.
6. Do not infer real-world audit sufficiency from these synthetic cases.
