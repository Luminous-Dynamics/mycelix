# FIN-ACC assertion–procedure–population contract v1

**Parent:** FIN-ACC-EVIDENCE-001 / #5798  
**Seed interpretation:** #5800 / PR #5800  
**Benchmark:** FIN-ACC-ASSERTION-PURPOSE v1.0.0

## Paradigm

An evidence item has no universal "trust score." It is qualified only relative to a named assertion, intended procedure, entity, reporting period, policy frontier and evidence population. A document may support a supplier's billing claim while being insufficient for receipt, rights/obligations, valuation, cutoff, authorization or settlement finality.

This contract makes that distinction machine-readable. It is an evidence-evaluation input and reference outcome format; it is not an audit procedure or assurance conclusion.

## Contract shape

Each candidate-visible task binds:

- assertion kind and stable assertion ID;
- entity and reporting period/cutoff;
- intended procedure and explicit required obligations;
- population scope and completeness state;
- typed evidence items, each preserving source reference, origin group, evidence class, polarity, purpose-fit, source authentication, source authority, temporal fit and transformation state;
- candidate claims with explicit evidence references;
- known limitations.

The evaluator reports one of:
- `supported_within_declared_scope`;
- `contradicted`;
- `insufficient_evidence`;
- `indeterminate`;
- `not_applicable_with_rationale`;
- `needs_authorized_review`.

No universal score, confidence scalar or LLM explanation can upgrade a result. Supporting and contradictory evidence are both preserved.

## Evidence-origin independence

Two artifacts are not independent merely because they have different filenames or formats. Evidence is grouped by `origin_group_ref`, representing a documented upstream origin. A GL export and a management report from the same ledger origin count once. A screenshot and parsed report from one bank export session count once. Independence assumptions require professional review in real engagements.

## Separate dimensions

- **Purpose fit:** relevance to the exact assertion/procedure.
- **Source authentication:** whether the source/channel identity is qualified.
- **Issuer authority:** whether that origin is authorized for the claimed fact.
- **Temporal fit:** whether the evidence is relevant to the reporting window/cutoff.
- **Population completeness:** whether the expected universe is independently scoped and accounted for.
- **Transformation lineage:** whether original evidence is retained or a derived representation has a reproducible lineage.
- **Contradictions:** qualified, relevant adverse evidence is preserved and prevents a positive disposition.
- **Authorization:** approvals are a separate obligation; a cryptographically intact approval record does not prove the approver's delegation.

The reference evaluator is deliberately conservative. Missing or unqualified obligations cause abstention/review, not a best-effort positive result.

## v1.0.0 corpus

Twenty-one synthetic cases cover:
1. multi-origin occurrence support;
2. invoice-only occurrence insufficiency;
3. mirrored evidence not independent;
4. external bank contradiction;
5. unknown source authentication and issuer authority;
6. wrong-entity obligation contradiction;
7. stale evidence;
8. cutoff contradiction;
9. unauthorized approval;
10. invoice vs inventory valuation-purpose mismatch;
11. reports derived from the same GL origin;
12. management representation contradicted externally;
13. OCR without transformation lineage;
14. a model claim citing a nonexistent evidence reference;
15. invoice irrelevance to title/rights;
16. explicit not-applicable rationale;
17. incomplete expected population;
18. source relevant to amount but not existence;
19. explicitly unqualified source authentication;
20. two invoice copies from the same origin;\n21. duplicate evidence identifiers across different source references.

The oracle is published for transparent public regression. It is not a blind model evaluation set. Candidate-visible inputs and reference outcomes are in separate files/directories to preserve a future split boundary, but a public oracle is still learnable and must never be described as hidden.

## Qualification gates

- Zero qualified contradictory evidence silently represented as supported.
- Zero same-origin mirrors counted as independent corroboration.
- Zero missing source population represented as complete when completeness is a required obligation.
- Zero irrelevant source promoted to purpose-fit by model explanation.
- Zero unknown or unqualified authentication promoted to qualified.
- Zero model claim accepted when its source reference does not exist.
- Every result binds procedure/entity/period/policy and evidence references.
- Every material assertion has an explicit terminal disposition; "no result" is not a pass.

Report purpose-fit precision/recall, false contradiction rate, independent-origin misclassification, completeness false assurance, transformation-lineage failures, appropriate abstention/review, and per-disposition calibration. Do not collapse the metrics into one weighted score.

## Claim ceiling

Passing this public synthetic corpus establishes only that the deterministic reference evaluator implements these declared rules on these cases. It does not establish ISA compliance, audit evidence sufficiency, audit quality, legal truth, source completeness in a real entity, fraud-free operation or an audit opinion. The IAASB ISA 330/500/520 revisions discussed in 2026 remain exposure drafts; no final compliance claim follows from this work.
