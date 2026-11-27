# NRVCA Shared-Budget Arbitration v0.1

**Status:** synthetic deterministic-arbitration contract; not an operational controller  
**Machine-readable fixture:** [shared-budget arbitration known answers](../../../fixtures/nrvca-shared-budget-arbitration-synthetic-v0.1.json)  
**Rust reference test:** `mycelix-workspace/tools/nrvca-validator/tests/shared_budget_arbitration.rs`  
**Tracking issue:** [NRVCA-010 / #5427](https://github.com/Luminous-Dynamics/mycelix/issues/5427)  
**Upstream contract:** [NRVCA-009 / PR #5426](https://github.com/Luminous-Dynamics/mycelix/pull/5426)

## Problem and scope

NRVCA-009 defines observable, versioned triggers and places a per-action cap on synthetic cost. A per-action cap is not a shared-budget guarantee: simultaneous triggers can individually pass the cap while their combined action cost exceeds the available contingency reserve. Actions may also compete for a finite crew, vehicle, system, or other shared resource.

This increment makes the arbitration seam explicit. It consumes a set of eligible recommendations, then emits a deterministic receipt. It does not re-run the trigger evaluator, estimate real costs, determine local social value, or issue a command.

## Contract

The v0.1 policy demonstrates the following steps:

1. Validate candidate IDs, action IDs, authoritative catalogue costs, units, preconditions, ranks, resource claims, and the non-negative shared budget.
2. Treat the protected service-floor set as an atomic commitment. If a protected action lacks a required precondition, the protected bundle is over budget, or its resource claims are jointly infeasible, select nothing and request manual review. Do not quietly substitute lower-tier actions.
3. After protected actions are feasible and selected, consider discretionary actions in explicit policy order, with stable action-ID and candidate-ID tie-breaks.
4. For each candidate, check preconditions, then currently available resource capacity, then shared budget. Preserve exactly one terminal disposition and record cost plus budget-before/after.
5. Preserve recommendation-only authority. Automatic dispatch remains explicitly disabled.

These ranks, action costs, resource capacities, and scenarios are synthetic test values. A policy rank is not a measured estimate of human welfare, and the greedy discretionary selector is not claimed to be globally optimal. The fixture also does not encode all possible equity or service constraints; those remain explicit, non-compensable NRVCA constraints rather than hidden weights.

## Paradigm shift

Move from **independent threshold fires** to **portfolio arbitration with a receipt for every candidate**. The key output is not just a selected action list: it explains which eligible actions were selected, deferred on budget, blocked by resource contention, or held for manual review—and why.

The approach composes with established adaptive pathways and robust infrastructure planning rather than claiming to invent them. Dynamic Adaptive Policy Pathways links monitoring signposts to contingent actions under deep uncertainty ([Haasnoot et al., 2013](https://doi.org/10.1016/j.gloenvcha.2012.12.006)). Research on multi-actor water-infrastructure pathways warns that robustness compromises can transfer vulnerabilities between participants ([Zeff et al., 2020, *Deeply uncertain pathways*](https://www.sciencedirect.com/science/article/abs/pii/S0309170819306475)). Recent interdependent-infrastructure work also studies compound disruption, attacker/defender/operator interactions, budget sensitivity, and runtime scaling ([Oster et al., 2024](https://doi.org/10.1016/j.ijcip.2024.100723)). The contribution here is narrower: frozen arbitration semantics, fail-closed preconditions, a shared resource ledger, and reproducible per-candidate receipts.

## Frozen synthetic cases

- Three simultaneous recommendations with a synthetic budget of four units: backup power and one crew action are selected; a second action sharing the crew pool is deferred for resource conflict; reserve release is deferred for insufficient budget.
- Protected service-floor bundle costs five units against a four-unit budget: no action is selected, and manual review is required. Lower-priority actions cannot mask the infeasibility.
- Missing preconditions on a protected action block discretionary substitution.
- Resource-contention case verifies that a rejected crew-consuming action does not consume budget, allowing an independent reserve action to remain feasible.
- Empty candidate set, candidate-order permutations, budget sweep, and malformed-input mutations are explicit controls.

The four-candidate compound case has (4! = 24) possible input orders. The Rust test asserts all permutations produce the same canonical receipt. A budget sweep at 3, 4, 5, and 6 synthetic units checks budget reconciliation and the frozen selection set for those points. The non-decreasing selected set observed here is fixture-specific; fixed-priority allocation does not guarantee monotonic selection for every possible action-cost portfolio.

## Benchmark programme: acceptance targets, not achieved results

| Benchmark | Baseline / comparison | Target before empirical claims |
|---|---|---|
| Shared-budget integrity | Independent-trigger/per-action cap baseline | Zero overspend in all valid generated cases |
| Resource capacity | No shared-resource ledger | Zero capacity violations |
| Audit completeness | Selected list only | Exactly one terminal, reasoned receipt per candidate |
| Determinism | Input-order first-wins baseline | Identical selections and receipts for all permutations of small candidate sets |
| Protected constraints | Scalar weighted-score / relaxed-constraint alternatives | Zero silent protected-floor substitution; infeasibility is explicit |
| Allocation quality | Explicit priority policy vs greedy cheapest-first, trigger-order-first and an exact small-instance oracle | Report service-floor violations, unmet eligible actions, spend, unused budget, and runtime; do not infer that greedy is optimal |
| Scaling | Same generated workload across candidate and resource counts | Measure p50/p95 runtime and memory at 10, 100, 1,000, and 10,000 candidates; publish hardware and seed |
| Adversarial robustness | Mutation-based negative controls | Every malformed ID, cost, budget, unit, resource claim, or authority escalation is rejected or held for review |

The first hard acceptance target is **safety and determinism**, not an attractive utilization ratio. Budget utilization alone is not welfare: spending the full reserve can be worse than preserving slack, and a weighted score can hide service or distributional-floor failures. Quality comparisons should therefore keep hard constraints explicit and report the trade-off surface, not only an average score.

## Comparison with current research and practice

Robust water and infrastructure planning already uses scenario analysis, multi-objective optimization, dynamic pathways, and sometimes exact or robust optimization for resource allocation. This contract is not yet competitive evidence against those planners or solvers. It makes a small component independently inspectable and testable, especially at the boundary between signal detection and a constrained response portfolio.

The next meaningful comparison is not to claim that a transparent greedy selector beats state-of-the-art optimization. It is to benchmark three layers separately:

1. **Safety/reference layer:** deterministic rule-based arbitration with fixed priority, hard resource constraints and a receipt for every candidate.
2. **Quality oracle for small instances:** enumerate feasible subsets or use a constrained optimizer under exactly the same hard constraints; compare where the rule-based policy leaves avoidable service gaps or budget unused.
3. **Stress/generalization layer:** generated candidate sets with correlated triggers, stale preconditions, resource contention, changes in budget, and infeasible protected bundles. Report worst-case violations and regret to the oracle, not just mean performance.

A future optimized policy may outperform the rule-based baseline on allocation quality, but it must return to the same protected constraints, authority boundary, and audit contract. Until real costs, action effects, capacity evidence, and governance-approved priority classes are acquired, no performance claim transfers to Johannesburg or South African operations.

## Evidence boundary and next gate

All action effects, costs, class ranks, capacities, and cases in v0.1 are synthetic and uncalibrated. The test fixture is not empirical evidence, and fixture counts are not probabilities. No infrastructure action is executed or authorized.

Required next gates: execute the Rust tests on an exact, recorded commit; independently compare the outputs against an exhaustive small-instance oracle; source action/resource costs and capacities with vintage and provenance; obtain operator/community governance for protected classes; then validate on held-out incident traces before considering any live recommendation workflow.