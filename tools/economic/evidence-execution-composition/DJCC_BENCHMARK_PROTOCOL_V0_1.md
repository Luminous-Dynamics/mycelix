# DJCC v0.1 Benchmark Protocol

**Status:** research protocol; proposed targets, not measured results  
**Parent:** PR #6822; benchmark programme issue #6848  
**Rule:** correctness and claim-scope qualification precede performance claims.

## 1. What this benchmark is—and is not

DJCC is a composition verifier. It does not replace an authenticated query engine, temporal database, policy evaluator, transparency service, authorization kernel, execution system, or outcome adjudicator. The benchmark therefore has two separate tracks:

1. **Component track:** compare each owned primitive with the strongest relevant implementation under the same task, inputs, threat model, and hardware.
2. **Composition track:** compare complete decision evidence with and without DJCC, holding the underlying component outputs and trust assumptions constant.

Never compare a specialist query proof against an end-to-end decision certificate and call either globally faster or more secure. A result is comparable only when task semantics, completeness requirements, trust roots, update model, and measured work are aligned.

## 2. Paradigm shift: benchmark assurance, not a single score

The primary output is an **assurance-cost frontier**: for each declared claim profile, report the resources and human effort needed to reach each disposition, including when the verifier correctly abstains.

A single weighted score is prohibited. It could conceal a false-safe result behind high throughput. Publish a vector:

- **Safety:** false allows, false absence, false completeness, false authority, false effect closure, false independence.
- **Correctness:** exact replay parity, positive-control acceptance, negative-control rejection/abstention, canonicalization parity.
- **Locality:** correction-impact recall and precision, unaffected-certificate stability, dependency-cone size.
- **Efficiency:** p50/p95/p99 latency, CPU time, allocations, peak RSS, storage, proof/certificate bytes, IO and network amplification.
- **Operations:** cache hit rate, full/incremental work ratio, recovery time, unresolved OutcomeUnknown duration, operator interventions.
- **Human cost:** review minutes, evidence requests, escalation rate, and reviewer agreement in blinded trials.

Report each profile independently. NotEvaluated, UnsupportedByProfile, and Indeterminate are valid outcomes, not failures to be hidden or coerced into a binary allow/deny.

## 3. Baseline matrix

| Capability | Strong comparison points | Fair comparison |
|---|---|---|
| Authenticated temporal/range/aggregate queries | VeriTS; VTRQ; RFC 9162 Merkle inclusion/consistency; SCITT VDS profiles | Same query, stream/trajectory, coverage and adversary; measure proof generation and verification separately |
| Policy semantics and authorization | Cedar formal specification, Lean model, production Rust implementation, differential/property testing | Same policy/entity/request corpus; compare decisions, counterexamples, validation and evaluation latency |
| Temporal history and bitemporal replay | SQL Server system-versioned temporal tables; XTDB and PostgreSQL/temporal extensions | Same valid-time/system-time query and correction history; do not treat ordinary row history as authenticated evidence |
| Incremental recomputation | DBSP / Differential Dataflow; Rete-style incremental rules; full recomputation | Same query graph and delta stream; exact output parity first, then cost at each delta/fan-out point |
| Transparency and receipts | RFC 9943 SCITT architecture; RFC 9942 COSE Receipts; RFC 9162; current SCITT MMR and CCF profiles | Same signed statements, verifier-pinned checkpoints, inclusion/consistency task and trust assumptions |
| End-to-end composition | Baseline pipeline assembled from the above owners without DJCC composition checks | Same owner outputs, scope, frontier, policy epoch, action denominator and outcome observations |
| Agent authorization and collateral effects | AuthorizationBench; SABench; Mycelix #6170 matched subset | Same executable task, authorized/out-of-scope paths and trace oracle; report goal completion, authorization compliance and unauthorized effects separately |

## 3A. Existing Mycelix owners and matched benchmark lanes

DJCC should compose existing owners rather than clone their semantics:

- [#6170 LARP agent-authorization/effect benchmark](https://github.com/Luminous-Dynamics/mycelix/issues/6170) owns agent task/authorization/effect matched evaluation, with separate task success, authority and realized-effect outcomes. Reuse its matched Cedar/OPA and held-out action-effect methodology; do not claim DJCC improves the agent-authorization rate.
- [#6189 effect-boundary interposition](https://github.com/Luminous-Dynamics/mycelix/issues/6189) is the enforcement-time control. DJCC is a post-hoc composition verifier and must not be described as blocking effects merely because its certificate rejects them.
- [#830 AGENT-010 offline independent reconstruction](https://github.com/Luminous-Dynamics/mycelix/issues/830) is the natural consumer for a bounded, content-addressed evidence package and dimension-preserving verification report. Integration must preserve its rule that a report is not a runtime authority capability.
- [#6816 execution transcript](https://github.com/Luminous-Dynamics/mycelix/pull/6816) owns state/proof/frontier replay and unknown-outcome reconciliation; [#6819 immutable action denominator](https://github.com/Luminous-Dynamics/mycelix/pull/6819) owns the canonical authority-bound action set. DJCC consumes their versioned outputs and exact digests; it does not replace their domain invariants.
- [#6530 SCITT adapter](https://github.com/Luminous-Dynamics/mycelix/issues/6530), [#6810 checkpoint trust bridge](https://github.com/Luminous-Dynamics/mycelix/issues/6810), and [#5942 split-view defense](https://github.com/Luminous-Dynamics/mycelix/issues/5942) remain the owners for receipt, checkpoint and witness trust.

The benchmark should include **integration rows** that use the exact owner outputs from these tracks. A mismatch between a valid action approval and a different executed operation is a DJCC composition failure only when each owner’s own local checks pass but the cross-owner join fails. A local owner's own failure remains attributed to that owner. This prevents double-counting bugs and prevents DJCC from claiming to own controls it does not enforce.

### Research snapshot (28 November 2026)

- **SCITT:** RFC 9943 is a Proposed Standard architecture for trustworthy and transparent digital supply chains; RFC 9942 defines COSE Receipts. The SCITT MMR receipt profile draft-ietf-scitt-receipts-mmr-profile-00 was published 21 November 2026 and is an Internet-Draft, not an RFC. Treat draft fields and APIs as versioned inputs, not stable normative guarantees.
- **Receipt and action-join caution:** the MMR profile's published text says that a non-empty inclusion path binds the node index, but an empty path (a peak) needs a verifier-held tree size and accumulator to pin the index; its detached payload is recomputed from the proof. An independent, informational SCITT decision-provenance Internet-Draft (-03, 17 November 2026; not a WG standard or endorsement) also documents why a receipt alone may not authenticate every tree-size/index field, and why approval, exact action parameters, occurrence identity, target observation and independently trusted outcome must be joined field-by-field. Treat it as a useful adversarial-design input, not normative authority.
- **VeriTS:** the 28 August 2026 arXiv paper reports more than two orders of magnitude better verification efficiency for windowed aggregation versus per-record proofs and range-query proofs up to 14.5x smaller. These are paper-reported component results, not DJCC results; reproduce only on matching workloads.
- **VTRQ:** the 21 August 2026 paper targets verifiable spatio-temporal trajectory range queries with separate spatial and temporal authenticated structures. Use it for the query primitive, not as an end-to-end decision-composition baseline.
- **Cedar:** its verification-guided development combines executable formal models/proofs with differential random and property-based tests against production Rust. DJCC should adopt that discipline for its own composition model rather than duplicate Cedar's policy engine.
- **DBSP:** the 2025 VLDB Journal work addresses automatic incremental view maintenance for rich query languages. Treat incremental replay as a measured optimization over a full-recompute oracle, not as inherently superior.
- **AuthorizationBench:** its October 2026 arXiv study uses 286 executable tasks and 6,864 trajectories across model/harness configurations; it reports a 19.2% out-of-authorization rate (10.5–26.6% across configurations). The key result for DJCC is methodological: task completion is not authorization compliance.
- **SABench:** its October 2026 arXiv study uses 201 tasks and reports collateral-damage side effects in 15.2% of 2,211 model-task pairs; 12.8% of goal-satisfied runs still include unauthorized writes. This supports separate outcome axes and an effect-observation oracle, not direct comparison with DJCC certificate verification.

Sources:
- RFC 9943 / SCITT status: https://datatracker.ietf.org/doc/html/rfc9943 and https://datatracker.ietf.org/group/scitt/documents/
- RFC 9942: https://datatracker.ietf.org/doc/html/rfc9942
- SCITT MMR profile: https://datatracker.ietf.org/doc/html/draft-ietf-scitt-receipts-mmr-profile-00
- SCITT decision-provenance discussion draft (informational, independent): https://rsync.nic.funet.fi/pub/doc/netinfo/internet-drafts/draft-lemonnier-scitt-decision-provenance-03.html
- VeriTS: https://arxiv.org/abs/2608.28318
- VTRQ: https://arxiv.org/abs/2608.21314
- Cedar specification and differential testing: https://github.com/cedar-policy/cedar-spec
- Cedar verification-guided development: https://arxiv.org/abs/2407.01688
- DBSP: https://doi.org/10.1007/s00778-025-00922-y
- AuthorizationBench: https://arxiv.org/abs/2610.27548
- SABench: https://arxiv.org/abs/2610.34167
- SQL Server temporal tables: https://learn.microsoft.com/en-us/sql/relational-databases/tables/temporal/overview

These references are comparison inputs, not endorsements, certifications, or claims of feature parity.

## 4. Frozen workload dimensions

Every corpus release must contain fixture bytes, a manifest, canonical digests, a schema version, and an independently authored expected result. Symbolic fixture references do not count as executable fixtures.

### Scale

- Owners: 1, 10, 100, 1,000, 10,000.
- Events: 1, 10, 100, 1,000, 10,000, 100,000, 1M, 10M.
- Dependency edges: 1, 10, 100, 1,000, 10,000, 100,000, 1M.
- Evidence channels: 2, 4, 8, 16, 64, 256, 1,024.
- Failure domains: 2, 4, 8, 16, 64, 256, 1,024.

### Change and adversary

- Delta fraction: 0%, 0.01%, 0.1%, 1%, 5%, 25%, 50%, 100%.
- Correction position: newest, median, oldest; before and after accepted frontier.
- Dependency fan-out: 1, 2, 4, 8, 16, 64, 256.
- Common-cause density: 0%, 1%, 10%, 25%, 50%, 75%, 100%.
- Failure-domain overlap, selective disclosure, withheld owners, split views, checkpoint rollback, key rotation, profile substitution, action-set mutation, and OutcomeUnknown are explicit independent axes.
- Include clean positive controls, single-fault negatives, compound faults, and metamorphic variants.

Use a pairwise covering design for broad performance exploration, then exhaustive combinations for safety-critical interactions. Never infer a safety property from a statistical sample.

## 5. Minimum adversarial corpus

The initial frozen corpus must cover at least these 30 classes. Every class needs a distinct executable positive-control fixture and a distinct executable hard-negative fixture, an explicit mutation operator, expected stage/root dispositions, and claim ceilings. A qualification corpus therefore contains at least 60 executable fixture cases, zero symbolic fixtures, a frozen fixture-set digest, and a separately versioned/digested independent oracle. The result record must enumerate the class IDs and enumerate the exact same class set for positive and hard-negative controls; counts alone are insufficient.

1. Valid positive decision.
2. Unauthenticated source or caller-selected root.
3. False issuer/key/key-epoch binding.
4. Selectively withheld issuer statements.
5. Hidden owner or uncovered owner prefix.
6. Backdated correction after accepted frontier.
7. Query omission or range-boundary omission.
8. Semantic projection substitution or normalization drift.
9. Policy epoch/evaluator build substitution.
10. Authorization or legal-authority substitution.
11. Same cardinality, altered action-set membership.
12. Unauthorized denominator amendment.
13. Execution replay or duplicate effect.
14. OutcomeUnknown followed by a new mutation before reconciliation.
15. Provider completion without independent outcome evidence.
16. Shared operator/KMS/admin domain across purportedly independent channels.
17. Incomplete lineage or unclassified common ancestor.
18. Detached MMR payload mismatch.
19. RFC 9162 / MMR profile or proof-type substitution.
20. MMR inclusion proof with ambiguous/unpinned tree size or leaf index.
21. Empty/invalid proof accepted through permissive decoding.
22. Checkpoint rollback or split view.
23. Ambiguous canonical preimage / missing domain separation.
24. Auditor identity, credential, or key-epoch substitution.
25. Correction-impact omission of a dependent decision.
26. Correction-impact overreach into unrelated decisions.
27. Historical replay leakage from later policy or trust state.
28. Compound fault: one valid proof plus one incomplete owner universe plus one shared failure domain.
29. MMR peak inclusion with an empty path but no verifier-pinned tree size/accumulator.
30. Locally valid approval and execution receipts whose authenticated action/amount/target/occurrence joins disagree.

A corpus entry is not executable until the actual bytes and digest are checked into the fixture corpus and consumed by both the candidate and independent oracle. Keep expected results outside the candidate implementation.

## 6. Correctness gates (hard release gates)

Before any speedup claim:

- 100% positive-control acceptance under the declared profile.
- 100% hard-negative rejection or explicit abstention; zero high-consequence false allows.
- 100% kill rate for defined safety-critical mutation operators.
- Zero historical frontier leakage.
- Zero false absence or completeness promotion without authenticated owner-universe closure.
- Zero claim-ceiling widening.
- Zero action-denominator membership substitution or unauthorized amendment.
- Zero independence promotion with incomplete lineage or a material unresolved common cause.
- Zero same-scope mutation after OutcomeUnknown until exact reconciliation.
- 100% correction-impact recall; report precision separately and improve it without reducing recall.
- 100% cross-language canonicalization parity for every frozen known-answer vector.
- Full replay and incremental replay must produce identical canonical outputs/dispositions for the same accepted frontier.

Mutation score is meaningful only when the operator set is frozen and reviewed. Report surviving mutants by operator and consequence; do not average a critical survivor away.

## 7. Performance hypotheses (not promises)

Only after correctness parity, test these hypotheses:

- At 1% sparse changes, incremental requalification may reduce p95 CPU and wall time by at least 5x versus full recomputation.
- For sparse corrections, the recomputed evidence cone should scale with the affected dependency closure rather than the total historical event count.
- Shared-lineage analysis should scale near O(V + E + C) for V failure-domain nodes, E edges and C evidence channels, without allocating a dense C×C pair matrix.
- Cache reuse should depend on exact digest/profile/frontier identity; stale cache entries must never widen claims.
- Proof/certificate bytes and verifier cost should remain sublinear in unrelated history for the targeted sparse-query workloads, subject to the actual ADS profile and update model.

These are research hypotheses. Publish failures and workload boundaries; do not tune fixtures to make the hypotheses pass.

## 8. Reproducibility and reporting

For each benchmark cell, publish:

- source commit SHA, base SHA, clean/dirty status, corpus and profile digests;
- compiler/toolchain versions, flags, dependency lockfiles and build provenance;
- OS/kernel, CPU, memory, storage and network characteristics;
- fixed random seeds, warm-up policy, trial count and raw per-trial outputs;
- p50/p95/p99 with confidence intervals, not only best runs;
- full and incremental work counters, proof bytes, allocations, RSS, IO/network bytes;
- verifier disposition and all claim ceilings;
- command lines, stdout/stderr, exit status and artifact digests.

Use at least 30 timed trials per stable cell after warm-up. Preserve raw data. A benchmark result without enough metadata to reproduce it is an observation, not a qualification.

## 9. Decision rule for a DJCC advantage

A defensible advantage exists only when DJCC provides an additional verified claim or reduces end-to-end cost **without** weakening the specialist component's guarantees.

The first publishable result should be an assurance-cost frontier, not a universal "beats SOTA" score:

1. Same decision, same sources, same accepted frontier, same owner universe, same policy and authority assumptions.
2. Specialist components run unchanged in both arms.
3. Baseline arm trusts component outputs as ordinarily integrated.
4. DJCC arm verifies exact producer/consumer digests, profile claim requirements, frontier, failure domains and root claim ceiling.
5. Measure false-safe/abstention behavior and marginal resource/human cost.
6. Show at least one adversarial case where a locally valid specialist receipt or policy result is insufficient to justify the composed decision—and demonstrate that DJCC rejects or abstains for the correct reason.
7. Demonstrate zero regression on all positive controls.

This is the research claim worth pursuing: **a decision can be locally well-formed at every component and still be globally unjustified; DJCC makes that gap explicit, testable and measurable.**
