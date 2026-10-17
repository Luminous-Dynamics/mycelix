# Exact Joint Allocation v1 — benchmark and claim contract

**Status:** bounded exhaustive reference implementation; targets proposed, results unqualified.  
**Tracking issue:** [PROC-JOINT-EXACT-001 #5151](https://github.com/Luminous-Dynamics/mycelix/issues/5151).  
**Feasibility dependency:** [joint-capacity checker PR #5150](https://github.com/Luminous-Dynamics/mycelix/pull/5150).

## Design shift

Do not produce one arbitrary procurement winner. Compute the complete supported Pareto frontier over:
1. total landed allocation cost in one declared common currency, and
2. a distinct shortage quantity for every request.

Shortages with unlike units are never added. A plan that costs less but leaves a different request short is an explicit trade-off, not a value to erase with an uncalibrated fairness weight. This is a decision-support frontier, not authority to order, reserve or pay.

## Implementation contract

The reference enumerates integer quantities over request/offer pairs explicitly qualified by a capability mapping. Each complete candidate is checked by the shared joint feasibility evaluator for demand, budget, offer limit, shared capacity, mapping evidence, unit/currency compatibility, deadlines and aggregate order lots.

- Request cohorts and all offers in the optimizer input must use one currency. Mixed-currency optimization fails closed; FX normalization requires a later versioned policy and evidence.
- Requests and offers are canonically ordered by stable IDs. Equal objective vectors keep one deterministic lexicographically smallest assignment representative.
- The report gives evaluator profile, manifest digest, currency, search-node count, feasible/infeasible leaf counts and frontier.
- The search-node limit is explicit, nonzero and capped at 500,000 nodes in v1.
- Search-bound exhaustion returns an error containing the visited-node count and limit. No partial frontier is returned.
- This is exact only for a completed search over this discrete model; it does not establish physical truth, supplier reliability or real-world optimality beyond the modeled inputs.
- This first version enumerates quantity assignments and validates at leaves. It is intentionally bounded rather than pretending to solve arbitrarily large instances.

## Correctness gates

| Measure | Acceptance target |
|---|---:|
| Objective-frontier agreement with an independent exact oracle on supported small instances | 100% |
| Demand / offer / shared-capacity / budget / lot / deadline / capability violations accepted | 0 |
| Mixed-currency scalar comparisons without a versioned FX policy | 0 |
| Input-vector permutations that alter canonical output | 0 |
| Search exhaustion labelled complete or optimal | 0 |
| Feasible/infeasible leaf-count mismatch against the independent oracle | 0 |
| Stable hand-authored fixtures | At least 40 |
| Deterministic generated micro-instances after oracle qualification | At least 10,000 |
| Search count or candidate count silently overflowing | 0 |

The authored independent oracle enumerates all 16 binary allocation masks for a two-request/two-offer problem at two shared-capacity ceilings. A second authored oracle enumerates all 81 edge-quantity vectors (each edge quantity 0–2) across shared-pool ceilings 1–4, comparing objective frontiers and feasible/infeasible leaf counts. Neither oracle is yet qualified passing on exact head. The corpus must still expand to lot increments, tighter budgets, multiple shared pools, capability profiles, deadline boundaries and invalid manifests before the 10,000-instance campaign.

## Baselines and regret analysis

Compare on identical frozen manifests:
- independent request-by-request optimization (diagnostic only; double pledge is infeasible);
- cheapest-first greedy;
- explicit-priority greedy;
- joint exact allocation.

For every baseline, run the same independent validator, count constraint violations, total common-currency cost, per-request shortage vector, maximum priority-weighted lateness only if its policy is explicit, and missed-request count. Never report a scalar “regret” across unlike shortage units. Report price premium for equal service vectors separately from service-vector dominance.

## Benchmark reporting

Only after all correctness gates pass, record:
- search nodes, feasible/infeasible leaves, frontier size and objective-dimensionality;
- p50/p95/max runtime and peak resident memory on pinned hardware/toolchain;
- median and worst-case speed ratios against greedy baselines;
- how performance degrades with requests, offers, alternative capability mappings and quantity domains;
- percentage of instances completed within the declared search budget and exact error counts for exhausted searches.

Do not invent a universal millisecond target before the manifest distribution and pinned reference hardware are declared.

## Research / non-novelty boundary

Hybrid/backup procurement, disruption-aware two-stage robust optimization, and distributionally robust supplier allocation are existing research areas. Examples include the 2026 two-stage robust supplier-selection and order-allocation paper (DOI 10.1016/j.ijpe.2026.110060) and the 2026 Wasserstein-ambiguity supplier selection/allocation study (DOI 10.1016/j.ijpe.2026.110160). The contribution sought here is a checkable bounded exact reference, explicit request-level shortage axes, currency-safe comparison, independent qualification and a clear completeness boundary—not a claim to have invented resilient sourcing.

- https://doi.org/10.1016/j.ijpe.2026.110060
- https://doi.org/10.1016/j.ijpe.2026.110160
