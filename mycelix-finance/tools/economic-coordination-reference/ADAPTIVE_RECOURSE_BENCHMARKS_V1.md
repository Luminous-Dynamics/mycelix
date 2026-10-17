# Adaptive Procurement Recourse v1 — benchmark and claim contract

**Status:** proposed qualification contract for the research/reference module `src/recourse.rs`. It is not evidence of production readiness or purchasing performance.  
**Tracking issue:** [PROC-RECOURSE-001 #5100](https://github.com/Luminous-Dynamics/mycelix/issues/5100).  
**Prerequisite work:** capability-first sourcing #5072 and exact static portfolio allocator #5099.

## Scope of this first implementation

This module evaluates a *fixed baseline allocation and fixed set of already-committed contingency options*. It calculates scenario-specific recourse over flexible and backup offers. It does not yet optimize which options should be contracted before a disruption; first-stage option selection is a follow-on optimization task.

It is effects-disabled. A scenario result is a conditional calculation over declared inputs, not a forecast probability, real inventory mutation, changed contract, purchase order, dispatch instruction or settlement.

## Decision semantics

The versioned scenario decision rule is lexicographic:

1. Maximize quantity recovered by the required deadline.
2. Minimize incremental recourse cost among equally recovered plans.
3. Minimize the latest arrival time among equal-quantity/equal-cost plans.
4. Apply a stable offer-ID tie-break for exact ties.

The cost model includes base unit costs and fixed order costs, plus all pre-committed option fees even when unused. A selected flexible/backup option adds unit cost and a one-time activation fee. A scenario cannot spend beyond the request's declared total budget.

The rule prioritizes on-time recovery before cost. It is a declared policy profile, not a universal objective; another jurisdiction or category may legitimately require a different reviewed policy.

## Evidence and resource contract

- Baseline allocations must exactly conserve request quantity, satisfy lot increments, be in the declared capability/unit/currency profile, pass qualification/quote/capacity/dependency evidence gates, and meet the deadline.
- Contingency options are pre-committed inputs with separate reservation fees, activation fees, activation lead and transit time.
- Qualification, quote, capacity and dependency evidence are independently freshness-checked at baseline evaluation and again at the scenario's observation time.
- Supplier and capacity-pool identity are always eligible failure-domain tags; supplied parent/site/sub-tier/utility/route/region tags are honored as provided.
- Pool snapshots must agree. The baseline quantity plus all reserved contingency activation quantities must not exceed the pool's declared capacity. This is conservative and may reject alternatives where two options are contractually exclusive; that requires a future explicitly encoded exclusivity policy, not implicit assumptions.
- Caller-supplied status, origin, timestamps and digest references are not cryptographically authenticated by this module. A value labelled Verified is not a substitute for verifier evidence.
- Empty failure-domain lists may be used only for an explicit no-disruption control, not as proof of supplier independence. Disruption cases require non-empty qualified failure-domain maps.
- Synthetic scenarios have no implied probability distribution; report per-scenario outcomes and worst-case shortage only.

## Required independent checks

Use an oracle separate from production recursion and ranking. On tiny instances, enumerate every allowable activation quantity and independently compute feasibility, recovered units, cost, arrival, and the declared lexicographic result.

Mandatory properties:
- base allocation totals equal demand exactly;
- recourse quantity does not exceed baseline lost quantity;
- no capacity pool is overcommitted after including pre-reserved backup/flexible capacity;
- no stale, future-dated, unverified or wrong-origin option becomes eligible;
- backup activation plus transport time is not treated as instantaneous and never qualifies after the deadline;
- source/site/parent/critical-sub-tier/route failures remove every dependent option for that scenario;
- reservation fee is paid once per pre-committed option, activation fee once per used option, and no fee or quantity overflows;
- total scenario cost never exceeds the declared budget;
- same input manifests yield identical reports independent of option and scenario input ordering;
- exceeded bounds return an error, never a partial report represented as exhaustive;
- no code path triggers a live supplier, stock, order, dispatch, or payment action.

## Frozen corpus and baselines

Start with at least 40 hand-authored scenarios plus deterministic generated micro-instances. Each fixture must carry a stable ID, source/revision, input digest, expected eligible options, expected recovered/unfilled units, expected incremental cost, reason codes, and the governing invariant.

Baseline strategies on identical fixture/scenario manifests:
- baseline only, no contingency;
- cheapest pre-committed backup;
- flexible capacity only;
- backup capacity only;
- both flexible and backup options under the versioned recourse policy;
- separate brute-force optimum for every tractable micro-instance.

Required negative cases:
1. Backup exists but activation or transit misses the deadline.
2. A contingency option expires between evaluation and disruption.
3. Two offer records double-count the same capacity pool.
4. A supplier name differs but the site/region/sub-tier failure domain is shared.
5. The last-unit capacity pool is also pledged to backup capacity.
6. Candidate ordering alters a tie-break.
7. Activation fee makes a recovery allocation exceed budget.
8. Very large quantity/options exhaust the search bound and return an explicit error.
9. The base allocation does not satisfy demand conservation.
10. The system reports a scenario set as probabilities without calibration evidence.
11. The same scenario is duplicated under a different input order.
12. A signed/digest reference is mislabeled as authenticated truth without verification evidence.

## Proposed acceptance targets (not measured)

| Property | Target |
|---|---:|
| Exact tiny-instance oracle | 100% agreement over every enumerated case |
| Demand and capacity invariants | Zero violations |
| Invalid/expired recourse options | Zero accepted |
| Late backup represented as on-time | Zero |
| Budget/arithmetic invariant | Zero over-budget recourse plans, overflow or wrapping |
| Deterministic replay | Byte-identical canonical serialized report for a frozen input manifest |
| Scenario coverage | 100% report of base lost, recovered, and unfilled units per supplied scenario |
| Generated micro-instances | At least 10,000 after the independent oracle is established |
| Runtime/memory | Measure p50/p95/max on pinned host/toolchain after correctness gates; do not set optimization pressure before baseline measurements |

These acceptance targets qualify reference-model correctness only. They do not prove real supplier availability, calibrated disruption probabilities, savings, real-world resilience, legal compliance or production readiness.

## Research basis

- Dai, Zou & Zhang (2026), two-stage robust sourcing with normal/flexible/backup modes: https://doi.org/10.1016/j.ijpe.2026.110060
- Li, Doan & Branke (online 2026/preprint), distributionally robust order allocation where disruption-impact probabilities are poorly known: https://doi.org/10.1016/j.ijpe.2026.110160
- Hosseini et al. (2019), disruption dependencies, backup sourcing and reactive recovery: https://doi.org/10.1016/j.ijpe.2019.03.018

The papers motivate strategies and controlled comparisons; their reported results must not be copied as expected performance for this implementation.
