# Joint Disruption Stress v1 — benchmark contract

**Status:** deterministic effects-disabled prototype. Authored tests and target metrics are not qualification results.  
**Tracking:** [PROC-RESILIENCE-002 #5169](https://github.com/Luminous-Dynamics/mycelix/issues/5169).  
**Implementation:** [joint_disruption.rs](https://github.com/Luminous-Dynamics/mycelix/blob/feat/procurement-joint-disruption-stress-v1/mycelix-finance/tools/economic-coordination-reference/src/joint_disruption.rs).

## Paradigm shift

A portfolio is not resilient because every request looks feasible in isolation. Supplier or shared-pool outages must be applied to the entire request portfolio, followed by one bounded joint re-optimization under the surviving offer set. This evaluator retains each scenario's Pareto frontier, including one shortage axis per request, instead of hiding who goes unserved inside a weighted average.

Robust optimization and disruption-aware supplier selection are established research areas. The intended Mycelix contribution is a reproducible, bounded, fail-closed stress evaluator around the existing joint allocator, with explicit common failure domains and exact-head evidence—not a novelty claim for robust optimization.

## Input and output contract

Inputs:
- one frozen `JointOptimizationProblem`;
- a scenario-set ID and caller-supplied digest;
- 1–32 named scenarios, each with a distinct ID and caller-supplied manifest digest;
- explicit failed supplier IDs and failed shared-capacity-pool IDs per scenario;
- an aggregate search-node cap in addition to the allocator's per-scenario cap.

For every scenario, all offers from every failed supplier or failed pool are removed before re-optimization, and every capability mapping referencing a removed offer is removed too. The original problem is validated before any failure filtering, so a malformed record cannot be hidden by a scenario. Scenario and failure-domain input order is canonicalized.

A successful report includes a complete frontier for every scenario and total visited-node count. Any invalid scenario, unknown failure domain, underlying optimizer error, arithmetic overflow, or budget exhaustion returns an error; no partial report is returned. The wrapper does not authenticate caller-supplied IDs/digests or verify the provenance of disruption claims.

## Non-goals and claim boundary

Scenarios are unweighted. A list of 32 scenarios is not a probability distribution and does not provide expected shortage, expected cost, availability, or real-world risk estimates. Do not duplicate a scenario to express probability. The evaluator optimizes after a declared scenario has been applied; it does not choose ex-ante backup commitments, estimate scenario likelihoods, predict correlated failures from telemetry, or authorize procurement.

The output is a bounded exact result only within the finite allocation model and scenario catalogue. Search exhaustion is an error, not a complete frontier. An observed scenario being absent from the catalogue is not evidence it cannot occur.

## Correctness targets

| Measure | Target |
|---|---:|
| Agreement with an independent exhaustive toy oracle | 100% |
| Assignments from a declared failed supplier | 0 |
| Assignments from a declared failed capacity pool | 0 |
| Shared-pool overcommitments per scenario | 0 |
| Unknown failure domains silently ignored | 0 |
| Duplicate scenario identities accepted | 0 |
| Partial frontiers returned on budget exhaustion | 0 |
| Invented scenario probabilities or expected values | 0 |
| Input-order permutations changing canonical output | 0 |
| Stable golden fixture IDs | At least 48 |

Correctness targets are hard gates. No benchmark should claim they passed until an exact-head run reports a completed passing test result and its source SHA is recorded.

## Performance evaluation plan

Use the same frozen corpus and separately report request count, offer count, mapping count, scenario count (1, 4, 8, 16, 32), per-scenario and aggregate node budgets, visited nodes, wall-clock duration, output frontier sizes, excluded-offer counts, and corpus/manifest digests. Measure release and debug profiles separately if both are relevant. Record exact commit SHA, Rust version, host/runner class, and CI run URL.

Do not publish a speedup or scaling claim until those measurements exist. The current search remains bounded enumeration; this slice does not introduce Benders decomposition, column generation, or claim polynomial-time scaling.

## Baselines

1. No-failure control scenario compared directly with the existing joint allocator.
2. Independent per-request outage analysis (diagnostic only) to expose false capacity duplication; it is not an acceptable implementation baseline.
3. The portfolio-wide joint scenario evaluator.
4. Where applicable, a separately implemented exhaustive hand-model oracle on tiny binary allocation instances.

Measure exact frontier agreement, missed or extra allocations, per-request shortages, failed-domain leakage, shared-pool cap violations, order invariance, total nodes, and runtime. Never use a single average-shortage score as a replacement for per-request axes.

## Research references

- Dai, Zou & Zhang (2026), *A two-stage robust optimization approach for resilient supplier selection and order allocation under disruption risks*: https://doi.org/10.1016/j.ijpe.2026.110060
- Li, Doan & Branke (2026), *Resilient supplier selection and order allocation under disruption risk over a Wasserstein ambiguity*: https://doi.org/10.1016/j.ijpe.2026.110160
- Open Contracting Data Standard: cross-stage public-contract data standard; not an execution or e-procurement engine: https://www.open-contracting.org/data-standard/
