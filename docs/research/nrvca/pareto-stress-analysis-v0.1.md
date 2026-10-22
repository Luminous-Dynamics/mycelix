# NRVCA Pareto Stress Analysis v0.1

**Status:** synthetic-only decision-analysis prototype  
**Implementation:** \`mycelix-workspace/tools/nrvca-validator/tests/pareto_stress.rs\`  
**Expected answers:** \`fixtures/nrvca-pareto-stress-expected-v0.1.json\`  
**Inputs:** [NRVCA-005 stress fixture](../../../fixtures/nrvca-correlated-stress-synthetic-v0.1.json)  
**Tracking issue:** [NRVCA-007 / #5419](https://github.com/Luminous-Dynamics/mycelix/issues/5419)  
**Stacked implementation:** [PR #5420](https://github.com/Luminous-Dynamics/mycelix/pull/5420)

## Research position: compose with established work

NRVCA is not claiming to invent Pareto optimization, Many-Objective Robust Decision Making (MORDM), Robust Decision Making (RDM), Decision Scaling, or scenario discovery. These are established methods in water-resource planning and decision support.

- Herman et al. (2015), *How Should Robustness Be Defined for Water Systems Planning under Change?* ([DOI](https://doi.org/10.1061/(ASCE)WR.1943-5452.0000509)) compares robustness approaches and shows that modeling and robustness definitions can change the recommendation. It calls for explicit, carefully elicited performance criteria.
- Matrosov et al. (2013), *Robust Decision Making and Info-Gap Decision Theory for water resource system planning* ([DOI](https://doi.org/10.1016/j.jhydrol.2013.03.006)) applies scenario analysis to water-system planning and explains how scenario discovery can reveal stress combinations associated with vulnerability.
- Webber et al. (2022), a review of DMDU applications for green infrastructure ([DOI](https://doi.org/10.1029/2021EF002322)), discusses Pareto fronts and regret-based evaluations.
- Kasprzyk et al. (2015), *Battling Arrow's Paradox to Discover Robust Water Management Alternatives* ([DOI](https://doi.org/10.1061/(ASCE)WR.1943-5452.0000572)), discusses the risk that aggregation of competing concerns can cause a subset of objectives to dominate the design.

The contribution here is a small, reproducible implementation slice within NRVCA: explicitly typed objectives, hard-constraint violation vectors, scenario-specific Pareto fronts, frozen expected outputs and negative controls. It is a test oracle, not an optimization engine.

## Paradigm shift: expose trade-offs without a hidden welfare score

NRVCA-005 reports robust pass fractions and worst-case shortfalls. Those are useful, but they do not show whether one policy is dominated by another, which dimensions cause a trade-off, or whether any policy is feasible in a particular scenario.

This prototype reports three different objects:

1. **Unconstrained Pareto front:** every candidate policy that is not strictly dominated on all five declared objectives.
2. **Feasible Pareto front:** the non-dominated policies among only those meeting every declared hard constraint.
3. **Constraint violation vector:** each floor shortfall and cap/budget excess independently, in its own unit.

The objective directions are explicit:

| Indicator | Direction | Unit |
|---|---|---|
| Lower-bound total service | Maximize | Synthetic service units |
| Lower-bound underserved-group service | Maximize | Synthetic service units |
| Lower-bound reliability | Maximize | Fraction in [0, 1] |
| Upper-bound emissions | Minimize | Synthetic tonnes CO2e |
| Total cost | Minimize | Synthetic cost units |

No objective weights are used. Violations of unlike dimensions are never added together. A service surplus cannot compensate for an equity-floor failure, nor can a lower cost compensate for an emissions-cap or reliability failure.

## Frozen synthetic results

The Rust test computes a front separately for each scenario. Current known answers:

| Scenario | All-policy Pareto front | Feasible Pareto front | Feasible choices |
|---|---|---|---:|
| Nominal operations | Output-only, equity static, adaptive staged | Equity static, adaptive staged | 2 |
| Seasonal drought | All three strategies | Adaptive staged | 1 |
| Grid disruption | Equity static, adaptive staged | Adaptive staged | 1 |
| Compound drought + grid + crew delay | Equity static, adaptive staged | Adaptive staged | 1 |
| Prolonged/repeated disruption | Equity static, adaptive staged | Empty — no strategy feasible | 0 |

The last row is a decision-critical finding *within this synthetic fixture*: none of the three hand-authored plans satisfies all current hard floors/caps after the prolonged repeated disruption. The correct output is explicit infeasibility, not silent threshold relaxation and not a ranking that hides the constraint violation.

Under the fixture's existing thresholds, NRVCA-005 robust pass fractions remain 0/5 for the output-only static strategy, 1/5 for equity static, and 4/5 for adaptive staged. These are unweighted fractions of five constructed scenarios, not probabilities of success, real utility outcomes or evidence of policy impact.

The current fixture is intentionally small. Its non-dominated front is only relative to these three candidates; it does not prove that any policy is globally optimal. Exact duplicate outcome vectors should remain jointly on the front rather than one being arbitrarily selected as superior.

## Constraint vector contract

For every strategy/scenario cell, calculate the positive part of each violation independently:

- primary-budget excess;
- contingency-reserve excess;
- total-cost-cap excess;
- service-floor shortfall;
- underserved-service-floor shortfall;
- reliability-floor shortfall;
- emissions-cap excess;
- scenario total cost below base cost.

A cell is feasible iff all eight quantities are zero within the declared numerical tolerance. These dimensions are reported as a vector; there is no summed penalty, weighted score, or compensation rule.

## Tests and mutation controls

The Rust integration test checks:

- unique scenario IDs, strategy IDs and strategy/scenario cells; exact cross-product coverage;
- direction-aware Pareto dominance, including transitivity and mixed direction trade-offs;
- known-answer fronts and every component of the frozen constraint-violation vectors;
- all-feasible, partly feasible, and empty feasible-front cases;
- label invariance and strategy-array-order invariance;
- non-compensability: very high output cannot cancel an underserved-service floor violation;
- absence of hidden objective weights or scalar score fields.

These tests verify deterministic logic against synthetic known answers. They do not establish the correctness of a hydraulic model or the quality of the input assumptions.

## Benchmarks and comparison to industry

### Synthetic qualification targets

- 100% of declared scenario × strategy cells are represented exactly once.
- 100% known-answer reproduction for fronts and component-wise violation vectors.
- Zero infeasible candidates included in the feasible Pareto front.
- Zero objective weighting or cross-unit aggregation unless a separate versioned policy explicitly declares such weights and sensitivity tests.
- Dominance is invariant to descriptive labels and candidate-array order.
- Original NRVCA-005 pass fractions and worst-case metrics are unchanged.

### Empirical qualification required to claim superiority

Compare NRVCA to an actual incumbent planning process and credible RDM/MORDM baselines using the same validated simulator, candidate portfolios, scenario sets, stakeholder constraints and model-evaluation budget. Preregister metrics before inspecting the held-out results:

- Pareto-front hypervolume or coverage with declared reference point and consistent units/normalization;
- front stability as scenario sets change, including held-out stress sets;
- robust-feasibility fraction with denominator and scenario-generation method published;
- worst-case service/equity/reliability shortfalls and number of infeasible futures;
- policy regret only against a specified, actually feasible per-scenario comparator and a declared objective; if no feasible comparator exists, label regret undefined/infeasible rather than fabricate zero;
- recovery time, cumulative service loss, reserve depletion and fully loaded lifecycle cost, when the simulator/data supports them;
- wall-clock/runtime and model-call budget under equal resource constraints;
- sensitivity to scenario-selection method, objective definitions, threshold profiles and uncertainty representations;
- reproducibility from source/input/model digests and independent execution.

Hypervolume is not unit-free by magic: objectives must be normalized or referenced with an explicitly documented method, and the reference point can influence comparisons. The initial test intentionally does not calculate hypervolume, scalar regret, probabilities, or a globally optimized alternative set. Adding any of those requires a separately reviewed metric contract.

## Next steps to become decision-useful

1. Generate a larger set of physically and temporally admissible futures only after a source-grounded model is available.
2. Add scenario discovery to explain which combinations of stressors lead to each infeasibility pattern; five hand-authored cases are insufficient for statistical inference or causal attribution.
3. Search a diverse candidate portfolio set, rather than only evaluating three preselected strategies, and preserve Pareto diversity.
4. Run held-out scenario-set sensitivity analyses, then historical replay and operator shadow evaluation with authorized data.
5. Compare against the incumbent and established RDM/MORDM approaches; make any superiority claim only after independent reproduction.

Until that work exists, treat this PR as better decision explanation and testable methodology—not as empirical evidence that the current adaptive strategy or NRVCA is superior in the real world.
