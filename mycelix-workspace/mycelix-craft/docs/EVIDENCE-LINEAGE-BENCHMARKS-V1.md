# Craft Evidence Lineage V1 — Benchmark Protocol

- **Work item:** CRAFT-EVIDENCE-LINEAGE-001 (#6761)
- **Status:** Proposed qualification protocol; no field measurements yet
- **Version:** 1.0-draft
- **Date:** 2026-11-26

## 1. Research question

Can explicit, digest-pinned dependency graphs reduce stale evidence and authority reuse after upstream change, while avoiding unnecessary invalidation of independent evidence?

The unit of evaluation is not merely a changed document. It is the downstream artifact set whose validity depends on the exact version, authority, scope or measurement that changed.

## 2. Industry baseline and fair comparison

Requirements-management products already provide structured traceability, baselines and change-impact workflows. IBM describes DOORS Next as supporting requirement capture, traceability and change analysis; its requirements-management product supports linked artifacts and change management. These capabilities are the baseline, not a novelty claim.

Compare:
1. **No propagation:** plain links; reviewers must discover impacts manually.
2. **TTL-only:** time-based expiry without dependency identity or change-event propagation.
3. **Manual change-impact review:** domain experts inspect a conventional traceability graph.
4. **Configured commercial workflow:** IBM DOORS Next / Engineering Requirements Management and Jama Connect using their supported traceability, baseline, review and change-impact capabilities.
5. **Event-sourced baseline:** append-only change events with a manually authored dependency graph.
6. **Evidence Lineage V1:** exact artifact revisions, typed dependency relations, explicit impact policies and deterministic transitive evaluation.

Do not assume an enterprise product cannot implement dependency invalidation. Compare documented configurations and integration effort, and separate native features from custom extensions. Where product semantics differ, label the comparison non-equivalent rather than assigning a zero.

## 3. Frozen synthetic conformance corpus

Build a deterministic corpus with:
- source correction and supersession;
- authority revocation and jurisdiction change;
- expiring or withdrawn credentials;
- participant consent withdrawal;
- policy and standard revision;
- measurement correction or instrument calibration failure;
- a claim support result whose underlying source changed;
- a decision whose atomic claims remain supported but whose material-element closure is no longer complete;
- a project requirement whose originating need was amended;
- independent observations that must remain valid;
- informational links that must not affect validity;
- forks, duplicate events, replay, missing dependencies, mismatched digests and cycles.

Publish schema, fixed seeds, generator version and immutable input digests. Keep hidden prospective cases to measure benchmark contamination.

## 4. Matched evaluation design

Run all systems on identical artifact graphs, change events, actor/authority metadata, time horizon and review budget. The host pre-validates event identity and authority for all systems; the evaluation isolates impact propagation, not authentication.

For every episode:
1. Publish the initial graph and expected artifact-version references.
2. Apply an exact event stream.
3. Compare predicted affected artifacts and states against an independent graph oracle.
4. Record false negatives, false positives and explanation paths.
5. Introduce a replacement revision and verify that the prior revision remains stale while the new revision becomes current only with fresh dependency pins.
6. Replay the same events and test conflicting event-ID/sequence reuse.
7. Repeat with event order permutations that preserve sequence numbers.

Use at least two independent reviewers for real-world dependency semantics. Preserve disagreement about whether an edge should be revalidation, invalidation or informational; do not tune the evaluator to force one preferred policy.

## 5. Metrics and proposed qualification targets

| Metric | Definition | Proposed target |
| --- | --- | --- |
| Critical stale-dependency escape | Critical dependent remains Current after a declared invalidating change | 0 on frozen adversarial vectors |
| Impact recall | Fraction of oracle-affected artifacts identified | 100% on frozen graph vectors |
| False invalidation | Independent or informational-only artifact invalidated | 0 on frozen vectors |
| Digest binding | Mismatched source/target revision accepted | 0 |
| Revocation propagation | Declared invalidating descendants remain usable | 0 |
| Event replay | Exact duplicate changes state or impact twice | 0 |
| Event-ID conflict | Reused identifier with different content accepted | 0 |
| Sequence collision | Two distinct admitted events share one sequence | 0 |
| Cycle safety | Cyclic graph accepted | 0 across all 4,096 four-node configurations |
| Deterministic replay | Independent evaluator emits the same ordered state/impact packet | 100% |
| Recovery safety | Old artifact becomes Current without a new version and fresh dependencies | 0 |
| Reviewer burden | Minutes required to validate edge policy and impacted artifact set | report by domain |
| Operational delay | Unnecessary work pause caused by false impact | report, do not optimize at safety expense |

The zero-error targets are conformance requirements for the frozen corpus, not guarantees of zero real-world risk.

## 6. Ablations

Measure the contribution of each mechanism separately:
- remove exact digests but preserve artifact IDs;
- remove revision pins;
- remove typed dependency relations;
- collapse all impact policies into one default;
- disable transitive propagation;
- disable event replay protection;
- treat all graph links as authorization;
- allow old artifacts to be manually marked current;
- replace event streams with TTL-only freshness.

This identifies which safeguards improve recall and which create avoidable false invalidation.

## 7. Statistical and operational protocol

- Separate edge-policy annotation from evaluator scoring.
- Cluster confidence intervals by source document, project and institution.
- Report high-consequence, multilingual and jurisdiction-specific strata.
- Report false positives and false negatives separately; do not hide them in one score.
- Record graph size, degree distribution, event count, depth, branch count, runtime and memory.
- Report throughput and p50/p95 latency only after correctness gates pass.
- Keep an out-of-time holdout and a prospective pilot corpus.
- Publish all exclusions, disputed annotations and non-equivalent baseline features.

## 8. Better-than-SOTA strategy

Do not claim superiority because the implementation has a formal graph model. Demonstrate improvement on matched tasks:
1. eliminate stale critical dependencies escaping detection;
2. preserve independent observations and informational-only links;
3. propagate revocation/correction across multi-hop project chains;
4. reject revision, digest, event replay and cycle attacks;
5. produce deterministic results independently replayed by another implementation;
6. reduce human change-impact review time without increasing false invalidation;
7. prove that revalidation creates a new version rather than rewriting history.

This is the measurable wedge: **evidence-aware change impact**, not a replacement for established requirements-management or participatory-democracy platforms.
