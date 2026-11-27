# Water Service Repair cross-core benchmark protocol V1

Status: proposed qualification protocol with a four-test executable slice. No field pilot, real service outcome, or comparative performance result is claimed.

## 1. Research question

Can a frozen water-service episode be translated through the six authoritative owner contracts without treating a provider status, missing evidence, mirrored sources, or stale observations as proof of physical restoration or permission to close the incident?

The composition target is:

`episode → Need Intake → Project Formation → Claim Support → Impact Qualification → Evidence Lineage → Decision Closure → independent oracle`

Every owner result remains visible, including abstention and safety-gate dispositions. The cross-core receipt binds each owner result to its source revision, packet digest and evaluator revision. It is an audit artifact, not an authority token.

## 2. Baselines and comparators

Compare semantic responsibilities separately.

| Comparator | What it does well | What must be matched before comparison |
|---|---|---|
| Conventional CMMS / work-order workflow | Assignment, work packages, job status, contractor completion, SLA and maintenance history | Same work order, authorization denominator, evidence freshness and explicit difference between reported completion and observed restoration |
| ITSM / incident-management workflow | Incident lifecycle, routing, escalation, change records and response-time reporting | Same incident events, ownership, escalation rules and closure definition |
| Event-sourced / append-only journal | Replayable event history and state reconstruction | Same correction, revocation, identity, idempotency and historical replay semantics |
| OCEL 2.1 / process-mining workflow | Object-centric event extraction, process discovery and conformance/deviation analysis | Same normalized events, object identities, source timestamps and provenance |
| Human-reviewed source packet | Independent domain judgement against source material | Blinded, pre-adjudicated outcomes, reviewer qualifications and recorded reasons |
| Mycelix six-owner composition | Typed responsibility boundaries, evidence support, impact sufficiency, lineage invalidation and conclusion-relative closure | Exact same source packets, revisions, authority scopes and expected dispositions |

A conventional workflow may be operationally excellent without claiming cryptographic or semantic completeness. Do not penalize it for failing obligations it was not designed to claim; report the obligation gap instead. Conversely, do not let Mycelix win by treating an owner abstention as success or by lowering the comparator's burden.

Useful external workload references:
- OCEL's deterministic synthetic relational databases provide six source-system scenarios with canonical OCEL references, including duplicates, corrections, retries, historical rows, projections and provenance. Preserve the exact package checksum and seed; the reference OCEL is an external target, not a proof of Mycelix's own extraction correctness.
- OCEL process-log collections include small, medium and large logs; use a small workload for iteration and a larger workload for scale only after semantics are frozen.
- The AWWA Utility Benchmarking Program publishes water/wastewater performance indicators stratified by utility type, size and region. Use these indicators for outcome reporting, not as a substitute for adjudicating the correctness of evidence decisions.

## 3. Frozen corpus and executable coverage

The source fixture bundle contains 16 known-answer synthetic WSR episodes and 20 proposed metamorphic transformations.

Current executable slice:
- WSR-001: provider reports completion; current pressure restoration and water-quality evidence are absent. Must remain NotEligible; Decision Closure remains OpenMissingElements.
- WSR-004: current pressure and water-quality evidence is supported. The episode may reach EligibleForLocalReview, while one required material-element mapping review remains absent; Decision Closure returns MappingReviewRequired and never Closed.
- Source expiry propagates NeedsRevalidation to dependent claim and decision artifacts.
- Two observations sharing one independence group do not satisfy a two-independent-source rule.

The four tests call all six public owner evaluators. The service-level oracle is separately authored and does not call the owner evaluators. This slice is not the full WSR benchmark: the remaining 14 known-answer cases and other metamorphic relations are not yet run through the adapters.

## 4. Correctness gates

Correctness is a hard gate. A single high-consequence false-safe outcome blocks qualification.

Proposed gates (targets, not achieved results):
1. 16/16 exact known-answer dispositions against an independently adjudicated oracle.
2. 20/20 registered high-risk metamorphic transformations produce the expected change or preserve-the-same result.
3. Zero provider-reported-complete-only cases treated as physical restoration or closure.
4. Zero stale, expired, revoked, wrong-zone, wrong-context, wrong-jurisdiction or wrong-revision evidence accepted as current support.
5. Zero false NotAffected / false independent classifications after a relevant high-consequence mutation.
6. Zero mirrored or common-cause evidence counted as independent corroboration.
7. Zero denominator expansion, shrinking or substitution after authorization.
8. 100% targeted revalidation recall for high-consequence changes on the frozen oracle; precision and false holds reported separately.
9. 100% deterministic replay of exact frozen inputs, toolchain, owner revisions and adapter build.
10. Zero auto-closed service episodes. “Eligible for local review” is the ceiling of this reference slice, not a permission to execute work or release water.

For every mutation, record exact field-level transformation, source and target packet digests, risk tier, owner outputs, expected oracle output, whether the mutation is applicable, and a classification of detected / equivalent / invalid / survived. Do not report a mutation score until all registered mutations were executed.

## 5. Outcome metrics (separate from evaluator correctness)

Report outcomes independently; do not collapse them into one weighted score:
- customer minutes off supply and interruption duration;
- interruption count and repeat-failure rate;
- hours of supply per day / continuity;
- pressure-deficit exposure by zone and duration;
- water-quality sample pass rate and monitoring coverage;
- response time, repair time and verified restoration time as separate clocks;
- alternative-supply volume, duration and adequacy;
- customer complaints and water-quality complaints;
- distributional impact by relevant service area/population denominator;
- worker safety events, near misses and qualification/permit gate failures.

The exact operational definitions, sample sizes, time windows, missing-data policy and denominator are part of each benchmark revision. Averages must not hide prolonged outages, repeat bursts, localized water-quality risks or unequal impacts. Industry water-utility indicator frameworks track continuity, interruption count/duration, pressure, water-quality monitoring and response/restoration time; reuse their definitions only where units and populations match.

## 6. Performance and resource metrics

Measure only after correctness passes:
- per-owner evaluation latency and end-to-end cross-core latency p50/p95/p99;
- packet construction, canonicalization, digest and receipt-generation time;
- wall time and CPU per mutation;
- peak memory and bytes per episode / per owner receipt;
- dependency graph size, change fan-out and lineage invalidation cost;
- number of owners evaluated, proofs/receipts examined, and reasons for requalification;
- human-review queue volume, time-to-review and false-hold rate;
- cold/warm throughput under fixed concurrency;
- incremental re-evaluation cost against full recomputation.

Freeze the CPU, RAM, OS/kernel, toolchain, lockfile, source commit SHAs, case digests, evaluator/adapter revisions, concurrency, warm-up, and measurement seed. Publish raw machine-readable results and run medians/p95, not only a headline average.

## 7. SOTA comparison strategy

A credible improvement is not simply faster workflow automation. Mycelix's differentiator must be measured as *decision quality under incomplete, corrected, correlated and adversarial evidence*.

Compare:
- ordinary CMMS/ITSM status closure;
- event-sourcing / replay baseline;
- process-mining extraction and conformance baseline;
- human-adjudicated source packet;
- Mycelix composed owner evaluators.

For each, publish a capability matrix stating whether it can express:
- exact scope and immutable source revisions;
- distinction between provider report and independently observed restoration;
- explicit uncertainty / abstention;
- correction and revocation lineage;
- common-cause independence;
- immutable authorized denominator;
- safety-critical capability and permit blockers;
- historical replay under the same accepted evidence frontier;
- independently reviewable conclusion mappings.

If a comparator cannot express one of these, mark it “not represented” and test the nearest native behavior. Never claim it failed the semantic test if its interface has no such claim.

## 8. Field-pilot boundaries

All current fixtures are synthetic. A real municipal pilot requires appropriate authority and community approval, privacy/data-sharing agreements, worker-safety controls, operational and water-quality criteria approved by qualified professionals, independent evaluation, and an explicit rollback/incident process.

A synthetic correctness pass is not legal compliance, water-quality certification, field readiness, or causal evidence that service outcomes improved.
