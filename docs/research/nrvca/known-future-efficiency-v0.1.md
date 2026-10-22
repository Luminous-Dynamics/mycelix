# NRVCA Known-Future Cost-Efficiency Diagnostic v0.1

**Status:** synthetic, catalog-limited oracle diagnostic  
**Stacked implementation:** follow-up to [NRVCA-008 / PR #5422](https://github.com/Luminous-Dynamics/mycelix/pull/5422)  
**Input:** [NRVCA-005 synthetic stress fixture](../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json)

## Why add this comparator?

Robust pass fraction asks what happens across a declared set of futures. Cost-efficiency asks a different question: if one of those futures were known at decision time, which *available* policy would satisfy every hard constraint at the lowest cost? The gap between the ex-post catalog oracle and an implementable policy is a useful diagnostic, but it must not be mistaken for an unconstrained global optimum.

A March 2026 water-resilience benchmarking paper makes the value of this comparison concrete: it uses historical climate to assess cost-efficiency after implementation and a known-future experiment to estimate theoretically optimal planning decisions. In its particular flood-resilience case, a refinement increased costs by 16.5% while reducing resilience deficits by 74.7%. Those results belong to that study's model and case; they are not transferable performance claims for NRVCA. Source: [Liu, Pianosi & Mijic, *A benchmarking framework to enhance cost-efficiency in water systems resilience planning under deep uncertainty*, Journal of Cleaner Production 548 (2026), 147850](https://doi.org/10.1016/j.jclepro.2026.147850).

## Contract

For each scenario independently:

1. Recompute all declared hard constraints from the raw outcome fields: primary budget, non-negative contingency draw, reserve cap, total-cost cap, minimum total service, minimum underserved service, minimum reliability, and maximum emissions.
2. Build the oracle only from strategies that satisfy **every** constraint in that scenario.
3. Select the minimum total cost among those feasible catalog entries. Preserve every exact tie.
4. If no catalog entry is feasible, report `no_feasible_catalog_strategy`; do not relax constraints, pick the least-bad policy as a pass, or invent a finite cost-regret value.
5. For any policy that is infeasible in a scenario, cost regret is undefined for that pair. Report infeasibility separately rather than writing zero regret, infinity, or a weighted penalty that hides the violated service floor.
6. Assign no probabilities. The five scenarios remain an unweighted hand-authored stress set.

The oracle is **catalog-limited**: it chooses among three already specified strategy/outcome rows. It is not a new water-system simulator, an optimizer over all possible interventions, an empirical optimality certificate, or a forecast of South African service outcomes.

## Frozen synthetic known answers

| Scenario | Feasible strategies in current catalog | Minimum feasible cost | Interpretation |
|---|---|---:|---|
| Nominal operations | `equity_static`, `adaptive_staged` | 9 | Tie; keep both names |
| Seasonal drought | `adaptive_staged` | 12 | Only listed feasible policy |
| Grid disruption | `adaptive_staged` | 12 | Only listed feasible policy |
| Compound drought/grid/crew | `adaptive_staged` | 13 | Only listed feasible policy |
| Prolonged/repeated disruption | None | Undefined | The catalog has no feasible policy |

The expected policy summaries are:

| Strategy | Feasible scenarios | Defined cost-regret cases | Total catalog cost regret |
|---|---:|---:|---:|
| `output_only_static` | 0/5 | 0 | Undefined in all scenarios |
| `equity_static` | 1/5 | 1 | 0 on its sole feasible scenario |
| `adaptive_staged` | 4/5 | 4 | 0 across those feasible scenarios |

This reveals that the synthetic fixture's binding limitation is not excess cost among feasible policies. It is **coverage of feasible policies**: the prolonged/repeated case has no available action that meets all declared floors. The correct next modeling task is to add and evidence new recovery options (or change the scenario only when source-backed evidence justifies it), not tune cost weights to conceal infeasibility.

## Required next benchmark stage

Before any comparative effectiveness claim:

- Keep the catalog oracle as a lower-bound reference only.
- Add a reproducible search/optimization method that can propose decisions not present in the current three-policy catalog.
- Evaluate it on identical scenarios and budgets; compare hard feasibility before cost.
- Separate known-future cost gap, infeasible-scenario count, service shortfall, equity shortfall, reliability deficit, emissions excess and computation cost.
- Use the known-future result strictly as a benchmark ceiling; a policy that has access to future information is not an operational competitor.
- Replace or augment hand-authored outcomes with a provenance-bound hydraulic/energy/resource simulation and then time-stamped historical replay. The current fixture is not empirical calibration data.

The Rust test `known_future_efficiency.rs` independently recomputes the constraint checks, verifies the complete strategy/scenario matrix, tests row-order invariance and confirms that a mutation below a service floor makes the catalog oracle report infeasibility. Its passing status is unverified until the focused Rust workflow completes on the exact commit.
