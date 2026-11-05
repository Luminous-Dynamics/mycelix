# FIN-ASSURE-PROCEDURE dataset governance

## Immutable releases

- **v1.0.0:** 18 synthetic cases with its v1.0.0 candidate schema and public reference oracle.
- **v1.1.0:** 22 synthetic cases with version-specific candidate and oracle schemas. It adds explicit reporting-entity binding to evidence, explicit procedure-purpose binding, per-risk/assertion pair results in verifier receipts, a multi-pair case, and negative controls for purpose mismatch, wrong-entity evidence and above-threshold analytical differences.

Each release is a regression/conformance corpus, not a blind holdout, professional audit dataset or benchmark leaderboard. v1.0.0 schema and oracle artifacts remain immutable and replayable. New cases or semantic changes require a new version.

All entities, risks, documents, origin groups and periods are synthetic. No client data or audit workpapers are included. Public reference oracles must not be mounted in a candidate environment when measuring model performance. A separate access-controlled evaluator is required for blind candidate comparison.

The fixture set encodes machine-checkable evidence-purpose, entity/period, population, denominator, origin-independence, contradiction, procedure-state, stale-workpaper and human-judgment boundaries. It does not encode the full ISA exposure drafts or claim compliance with current/proposed auditing standards.

Dataset extension rules:
1. Preserve released schema, candidate and public-oracle bytes and hashes.
2. Add positive and negative controls for every new invariant.
3. Keep candidate inputs, public reference outcomes and any future blind evaluator physically separate.
4. Keep standards mapping and expert adjudication separate from deterministic fixture correctness.
5. Report pair-level outcomes; do not let a case-level aggregate hide a blocked risk/assertion pair.
6. Do not infer real-world audit sufficiency from these synthetic cases.
