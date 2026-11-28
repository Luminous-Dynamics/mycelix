# Craft Evidence-Preserving Multi-Resolution Projection Protocol V0.1

**Tracking:** #6836. **Foundation:** #6825 temporal/DJCC protocol and #6833 DBC oracle.

## Objective

Enable analysts to move from municipal portfolio to episode, object, event, relationship claim, evidence artifact and decision receipt without silently losing provenance, time semantics, uncertainty, conflicts, authority or outcome limitations.

A projection is a derived view with a content-addressed transformation receipt. It is not another event log, evidence store, temporal database, policy engine or action ledger.

## Research baseline and novelty boundary

Object-centric process mining already formalizes drill-down, roll-up, unfold and fold operations over OCEL. A 6 October 2026 paper evaluates these operations on four years of educational OCEL data and public BPIC-derived multi-million-event logs, and uses temporal Event Knowledge Graphs to investigate changing group membership. Generic multi-resolution OCPM and temporal relationship visualization are therefore not sufficient novelty claims.

A 2026 object-centric process management manifesto identifies dynamic object relations, missing objects, granularity mismatch and conformance cost as open issues. The strategic contribution here is to preserve **decision evidence semantics** through transformations.

TempBench (CIKM 2026) provides 8,710 temporal KG questions with per-question gold supporting subgraphs and typed same-subject/stale-fact negatives. A 2026 survey on temporal KG causal reasoning highlights the mismatch between temporal association/ranking metrics and verifiable causal validity, evidence grounding and robustness to uncertain timestamps.

References:
- Khayatbashi et al., *Advancing object-centric process mining with multi-dimensional data operations*, 6 October 2026: https://link.springer.com/article/10.1007/s10115-026-02904-0
- *Object-centric process management: A research manifesto* (2026): https://doi.org/10.1016/j.is.2026.102728
- TempBench, CIKM 2026 proceedings: https://www.sigweb.org/toc/cikm26.html
- *Deciphering Causal Reasoning in Temporal Knowledge Graphs: A Survey* (2026): https://www.scitepress.org/Papers/2026/151735/151735.pdf

## Projection tower

1. **Raw assertion:** source artifacts, events, object changes, qualified relations, temporal claims, mutations and trust/admission receipts.
2. **Operational object:** episodes, complaints, assets, zones, crews, permits, work orders, observations, samples, lab results and outcome claims.
3. **Process/episode:** lifecycle, causal event frontier, progress antichain, owner-universe state, evidence dependencies, common causes and action coverage.
4. **Decision:** exact DBC, policy/authority receipts, claim ceilings, conflicts, unknowns and abstention.
5. **Portfolio:** episode outcomes, recurrence, service availability, resource use, cost, delay, access and geographic distribution.

Every view is a query/projection over owner receipts at an explicit valid time and accepted causal frontier. A query returning rows does not prove that the source universe is complete.

## Transformation receipt

Every transformation emits a versioned, content-addressed receipt binding:
- receipt ID, schema/profile, source/target view and exact input receipt IDs/schemas/digests;
- output view digest and canonicalization version;
- algorithm/build digest and parameters;
- valid-time query and accepted causal-frontier receipt;
- progress-antichain receipt and separate owner/source-universe completeness disposition;
- inclusion/exclusion predicates, grouping keys and aggregation semantics;
- retained, summarized, omitted, rejected and unknown claims;
- source-to-output lineage for every retained assertion and aggregate;
- relationship cardinality/conflict mapping;
- common-cause/evidence-dependency propagation;
- correction, revocation, expiry and impact references;
- DJCC six-axis claim ceiling before and after transformation;
- declared reversibility and inverse/lineage reference, where supported.

### Reversible and lossy views

- **Exact reversible:** inverse reconstructs the normalized source inventory and receipt within the named profile.
- **Profile-bound reversible:** inverse reconstructs declared profile semantics, with explicit excluded extensions or unsupported shapes.
- **Lossy analytical:** intentionally drops or summarizes information; the receipt lists all omissions and downgrades dependent claims.

A lossy view cannot be called reversible because its row count or final answer appears unchanged. Round-trip parity is only claimed against the declared semantic inventory and fixtures.

### Monotone claim ceilings

For Authentication, Authority, Completeness, Effect, Independence and Truth, a transformation may preserve, narrow, mark NotEvaluated or reject. It cannot widen a claim merely because aggregation succeeded.

A valid event frontier does not prove source-universe completeness. A stable progress antichain is stable only relative to its declared owner universe. Two visible relationships do not prove independent evidence. Plan coverage and execution do not prove physical effect. Process conformance does not prove truth. Portfolio averages cannot erase unsafe episodes.

## Evidence-subgraph retrieval

Each benchmark query freezes a gold support subgraph:
- claim nodes and typed relationship edges;
- exact source references/digests;
- valid-time intervals and knowledge-frontier inclusion;
- required authority/policy claims;
- common-cause dependencies and admissibility status;
- stale, revoked, conflicting and same-subject distractor negatives.

The system returns the candidate support subgraph used for a decision. Score that graph, not only the final answer string.

Metrics:
- evidence-subgraph precision/recall;
- temporal validity at the named valid-time and causal frontier;
- source-attribution precision/recall;
- chain consistency across multi-hop support;
- stale-fact rejection;
- six-axis claim-ceiling non-widening;
- false absence and false completeness;
- reviewer traceability to exact source subgraphs and replay receipts.

TempBench's typed negative and gold-subgraph pattern is a design input, not an operational ground-truth source for water-service decisions.

## Frozen known-answer seeds

The companion JSON file contains 12 specification-only seeds. They are not executed benchmark results.

- PROJ-001: exact reversible fold/unfold preserves normalized inventory and receipt.
- PROJ-002: lossy roll-up declares omitted object history, downgrades dependent claims and preserves source lineage.
- PROJ-003: appending a future correction/revocation leaves earlier frontier receipts byte-identical.
- PROJ-004: current and expired assertions share subject/predicate; only the assertion valid at both frontiers is retrieved.
- PROJ-005: wrong-zone, wrong-type, wrong-predicate or wrong-authority-scope distractors are excluded.
- PROJ-006: observations sharing a calibration/sample ancestor retain the common-cause edge and cannot become independent.
- PROJ-007: every retained aggregate maps to exact source claim/artifact digests; fabricated lineage fails.
- PROJ-008: an unsafe episode cannot disappear behind a safe portfolio average.
- PROJ-009: relationship-cardinality conflict is retained, not silently resolved.
- PROJ-010: a transformation receipt from another schema/profile is rejected.
- PROJ-011: revocation changes only later projections; old frontier remains stable.
- PROJ-012: incomplete source universe does not imply false absence or completeness.

Every negative case needs positive controls. A system that rejects every query must not score well. Gold graphs and outcomes must be independently authored/adjudicated, not produced by the candidate evaluator.

## Targets and reporting

Targets, not results:
- 100% frozen known-answer agreement;
- zero silent loss of declared OCEL entities, attributes, qualified relations or relation mutations;
- zero historical receipt changes at unchanged frontier;
- zero false-positive support edges on typed stale/distractor negatives;
- 100% exact source attribution for retained facts;
- zero claim-ceiling widening;
- zero unreported information loss in lossy views;
- 100% canonical parity for transformations explicitly declared reversible.

Report evidence-subgraph precision/recall, temporal validity, chain consistency, lineage coverage, false absence/completeness/independence/false-safe, reviewer traceability, p50/p95/p99 latency, RSS, input/output bytes and transformation size separately. Do not collapse safety, authority and completeness failures into one weighted score.

## Scale and comparators

Use 1K for developer tests, 100K for integration, 1M for practical large logs, public BPIC-derived multi-million-event OCELs on isolated hardware, and 10M synthetic events for stress-only. Report relation fan-out and object topology alongside event counts.

Compare:
- PM4Py and published OCEL multi-dimensional operations for OCPM transforms/conformance;
- temporal Event Knowledge Graph baselines for changing object relations;
- TempBench-like gold-subgraph retrieval with typed stale negatives;
- XTDB only on matched valid/system-time query profiles;
- current Rust whole-document implementation as reference before #6803 streaming/external-sort work.

Measure parse, transform, lineage mapping, retrieval, receipt generation, drill-down and portfolio aggregation separately. Generic temporal-graph link-prediction data are not evidence of decision correctness.

## Implementation sequence

1. Freeze JSON Schema and canonical transformation receipt.
2. Extend #6825 temporal/DJCC fixtures with the 12 projection KAT seeds.
3. Implement an independent reversible/lossy transformation oracle.
4. Add gold support subgraphs and typed stale/distractor negatives.
5. Compose projection receipts into DBC; reuse temporal, trust, authority and outcome owners.
6. Compare whole-document and streaming implementations under #6803 for exact semantic parity.
7. Only then consider production adapters or consented, independently adjudicated field evaluation.

## Claim ceiling

This protocol does not prove source authenticity, complete source universe, process safety, evidence independence, water-quality validity, physical repair effectiveness or geographic equity. Those remain owner- and field-specific qualifications.
