# PRAXIS-NRES-016: Finite-horizon adaptive recourse and dynamic resilience control

**Status:** protocol proposal and benchmark contract; no solver, learned policy or confirmatory result is claimed.  
**Issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5564.  
**Prerequisites:** NRES-007 cascade/recovery trajectories; NRES-009 joint service forecasting; NRES-013 finite-support joint-state DRO; NRES-014 conformal W1 calibration; NRES-015 metric-transfer, bulk-tail and dependence-cluster research.

## 1. Why a new decision model is needed

NRES-013–015 improve uncertainty representation and one-step decision robustness. They still do not fully express a recovery process in which the action taken now changes the next physical state, resource inventory, observation quality, available actions and distribution of future failures.

A service system is not a static failure mask. It is a controlled process. Resources deplete and replenish; repairs take time; procurement and transport have uncertain lead times; protection can change hazard consequences; observations can be missing or delayed; and a decision can be revised only after new information becomes available.

NRES-016 defines the next unit of analysis as a **finite-horizon adaptive policy over a controlled transition system**, not an independently optimized action for each forecasted snapshot.

The contribution is not “apply DRO to a longer horizon”. The benchmark must enforce a common information structure and exact small-instance oracle so that methods cannot win by using future information, resetting resource budgets, or averaging away critical service-floor failures.

## 2. Paradigm shifts

### 2.1 From one-step action selection to policy synthesis

An action is selected from the information available at the time. A policy maps observable history to action. Future scenario labels, unobserved shocks and latent trajectories must not influence a current action.

### 2.2 From fixed ambiguity sets to intervention-conditioned transition ambiguity

Uncertainty is attached to transition kernels, not only to an initial joint failure distribution. Protection, maintenance, repair, procurement and verification can change the conditional distribution of subsequent outcomes. Such action-dependence must be explicitly encoded; it is not inferred from correlation and it is not a causal claim unless separately identified.

### 2.3 From static tail penalties to time-consistent dynamic risk

A one-time CVaR of total path cost and a nested conditional risk recursion are different objectives. Candidate policies must state which is used. The primary dynamic baseline should use nested conditional risk (for example, recursively applied conditional CVaR) or an explicitly declared rectangular ambiguity recursion. Do not call a static terminal CVaR time-consistent.

### 2.4 From resource snapshots to conserved resource ledgers

Workforce, fuel, spares, funds, transport capacity and inventory must persist through the horizon. An action consumes resources; replenishment arrives only under a declared transition and lead time. Resource balances must be checked at every stage and under every scenario path.

### 2.5 From “more information is always better” to constrained dual control

Inspection, measurement, verification and information-sharing may be selectable actions. They can improve later state estimates without directly restoring service capacity. Their value is the improvement in downstream decisions net of time, cost, operational risk and privacy burden. Information acquisition is admissible only while service-floor, resource and risk constraints remain satisfied.

### 2.6 From compensating scores to non-compensatory service contracts

Critical service floors, legal/role boundaries, privacy restrictions and resource feasibility are constraints, not weighted preferences. Report each service and constraint separately. A high average score must never compensate for a critical service being below its floor.

## 3. Formal model

At stages (t=0,ldots,H-1), define:
- (x_t): physical state (service capacities, deficits, asset condition, repair status, transport and demand);
- (b_t): resource ledger (inventory, workforce hours, funds, fuel, spares and transport capacity);
- (y_t): observation received at stage t;
- (h_t=(y_0,a_0,y_1,ldots,y_t)): observable history / filtration;
- (a_tin A(h_t)): selected action;
- (w_{t+1}): uncertain disturbance or transition outcome.

State and resource transitions are:

[
x_{t+1}=f_t(x_t,b_t,a_t,w_{t+1}),qquad
b_{t+1}=b_t-u_t(x_t,a_t,w_{t+1})+r_t(w_{t+1}),
]

with an observation model:

[
y_{t+1}sim O_t(,cdotmid x_{t+1},a_t,w_{t+1}).
]

A valid policy is non-anticipative:

[
a_t=pi_t(h_t).
]

For any two scenario paths with identical observable histories through t, their selected actions at t must be identical. Scenario identifiers, future shocks and hidden outcomes are never admissible policy inputs.

The uncertainty set is conditional on the current state, observed history and action:

[
Q_{t+1}inmathcal U_{t+1}(x_t,h_t,a_t).
]

This permits decision-dependent uncertainty, but each dependency and uncertainty set must be predeclared and testable. For confirmatory work, transition-model parameters, action effects, ambiguity parameters and risk mappings are frozen before hidden outcomes are evaluated.

A candidate risk recursion is:

[
V_t(h_t)=inf_{a_tin A(h_t)}
sup_{Q_{t+1}inmathcal U_{t+1}(h_t,a_t)}
ho_t^{Q_{t+1}}left[
c_t(x_t,b_t,a_t,w_{t+1})+V_{t+1}(h_{t+1})
ight],
]

with terminal value (V_H). The risk map (ho_t), ambiguity set and order of robustification/risk aggregation must be explicit. Different orders can define different problems.

## 4. Information structure and non-anticipativity

The scenario tree is indexed by observable histories, not by the latent full scenario. Two paths that have identical observations, prior actions and public evidence up to a decision point belong to the same information state and share one action variable.

Required negative controls:
- change only future disturbances and verify that pre-observation actions do not change;
- permute hidden scenario labels without changing observable histories and verify policy invariance;
- duplicate a scenario path with the same observable history and verify that it does not create an extra decision variable;
- remove a future observation and ensure the policy cannot use it;
- provide an action unavailable at the information state and require fail-closed rejection.

A solver that assigns different actions to indistinguishable histories is invalid even if its objective value is lower.

## 5. Physical, resource and governance constraints

At every stage and scenario:
- resource balances are conserved and cannot become negative;
- actions are feasible for the observed state and role;
- repair/procurement/transport lead times are respected;
- stage transitions obey declared dependency and capacity rules;
- hard legal, privacy, safety and role boundaries are not represented as soft penalties;
- critical service floors are recorded per service, per stage and per path.

A test instance designated feasible must admit at least one policy satisfying every hard constraint. In such instances, the reference solver must have zero hard-constraint violations. In infeasible instances, return an explicit infeasibility witness or status; do not hide infeasibility by relaxing constraints silently.

A service-floor breach caused by exogenous shock may be unavoidable in some paths. Such breaches must be measured and reported separately, not described as policy compliance.

## 6. Candidate methods and baselines

Use common scenario paths, costs, observations, state transitions, feasibility constraints and compute budgets:

1. **Open-loop deterministic plan:** actions fixed before uncertainty is observed.
2. **Rolling-horizon deterministic MPC:** re-optimize after each new observation but use deterministic forecasts.
3. **Two-stage robust allocation:** first-stage preparation with one recourse stage.
4. **Multistage stochastic programming:** expected-cost recourse under a frozen scenario distribution.
5. **Multistage adaptive robust optimization:** worst-case recourse over a predeclared uncertainty set.
6. **Nested conditional CVaR:** time-consistent dynamic risk baseline.
7. **Distributionally robust adaptive recourse:** stagewise transition ambiguity, with explicit rectangularity or coupling assumptions.
8. **NRES-013/014/015 one-step policies:** ablation baselines to quantify the value of a dynamic policy.
9. **Perfect-information policy:** infeasible clairvoyant lower bound only; never a deployable baseline.

For distributed or multi-role variants, report where nonanticipativity, shared inventory and common ambiguity require centralized coordination. Decentralized local optimizers are not comparable if they violate shared constraints.

## 7. Benchmark ladder

### 7.1 Exact micro-instance

Start with a tiny finite state/action tree, short horizon and few disturbance outcomes. Solve by exhaustive dynamic programming or full policy enumeration. This is the oracle for transition semantics, non-anticipativity, risk recursion and objective values. Require agreement within 1e-8 in declared units.

### 7.2 Controlled medium instance

Increase services, time steps, resource types, repair lead times, partial observations and common-mode shocks. Compare rolling-horizon, two-stage, multistage stochastic, adaptive robust and nested-risk methods on identical paths. Record optimality gaps, anytime incumbents, memory and runtime.

### 7.3 Stress and scale instance

Increase horizon, state dimensionality, scenario branching, shared resources and actor count. Evaluate decomposition and approximation methods under fixed hardware/compute budgets. Report quality/runtime/communication trade-offs, not just solver objective.

### 7.4 Shift and dependence ladder

Include independent shocks, strong within-cluster dependence, common-mode disruptions, transition-kernel shift, seasonal changes, supply-chain disruption, staff unavailability, evidence degradation, gradual drift and abrupt change. Use independent source-family trajectories and preserve cluster boundaries across all derived paths.

## 8. Required reporting — no composite score

Per method × source family × shift × horizon × service report:
- integrated deficit area / service-shortfall area;
- time below each service floor and maximum shortfall magnitude;
- pathwise realized cost and statewise regret;
- worst-case pathwise regret and dynamic risk bound;
- CVaR95 and CVaR99, with static versus nested definitions clearly separated;
- resource depletion, replenishment delay and unused reserve;
- repair completion time, recovery time and recourse delay;
- action-switch / reversal counts and human-review deferrals;
- forecast scores, calibration coverage, ambiguity radius and vacuity;
- non-anticipativity violations (must be zero);
- infeasibility and hard constraint violations;
- runtime, peak memory, optimality gap, anytime incumbent and communication cost.

Confidence intervals are computed at the independent source-family / trajectory level. Time steps, service dimensions, scenario copies and random seeds do not multiply the independent sample count.

## 9. Proposed targets (targets, not current results)

- 100% transition, resource-ledger and information-set conformance on all exact micro-instances.
- Zero non-anticipativity violations in every solver and test.
- Exact objective agreement within 1e-8 on finite micro-instances.
- Zero hard-constraint or service-floor violations in fixtures proven feasible by the exact oracle.
- At least 5% lower paired realized statewise regret than the strongest preregistered static/rolling baseline on independent shift trajectories, with paired 95% confidence interval excluding zero.
- No more than 2% nominal in-distribution cost degradation.
- All infeasibility, timeout, risk-vacuity, abstention and non-improvement results disclosed.

These targets are not SOTA claims. Statistical power and number of independent trajectories must be established before confirmatory custody is opened.

## 10. Research frontier and honest comparison

Relevant external directions include:
- Li, Yu and Zhang's 2026 two-stage robust emergency-facility location/allocation model uses decision-dependent cascading uncertainty and jointly considers pre-disaster reinforcement and post-disaster recovery. Their Istanbul case study reports a reduction in the number of facilities to be opened while maintaining a target service level; that result is case-specific, not a general performance guarantee. DOI: https://doi.org/10.1016/j.seps.2025.102395.
- A 2026 multi-stage robust emergency-resource allocation paper integrates decision, execution and demand roles and compares adaptive multi-stage scheduling with deterministic, static-robust and rolling-horizon alternatives. https://pmc.ncbi.nlm.nih.gov/articles/PMC13328576/.
- Distributed optimization surveys identify multi-stage recourse, rich ambiguity sets, non-anticipativity and communication-efficient coordination as open scalability challenges. https://doi.org/10.3390/math14193489.
- Shaer et al. (ICML 2026) develop conditional conformal test martingales against a fixed reference set to avoid test-time contamination in sequential shift detection. https://proceedings.mlr.press/v306/shaer26a.html.
- Menadjiev et al. (ICML 2026) propose DistMatch, adaptive residual binning and online conformal updates for sequential prediction under time dependence and distribution shift. https://proceedings.mlr.press/v306/menadjiev26a.html.

These methods are candidate comparators and components, not claims to reproduce or outperform the published methods. NRES-016 must compare on identical information sets, source-family splits, transition models, scenario paths, constraints and compute budgets.

## 10.1 Industry and standards crosswalk

NRES-016 should be an evidence/decision analytics layer that composes with existing operational management systems—not a replacement for them.

| Industry framework | What it already governs | NRES contribution | Boundary |
|---|---|---|---|
| FEMA HSEEP | Exercise program management, design/development, conduct, evaluation and improvement planning; capability findings and AAR/IP corrective actions | Repeatable scenario trajectories, objective-level measures, service-floor evidence, pathwise action/regret analysis and machine-readable corrective-action evidence | Does not replace exercise leadership, evaluator judgement, AAR/IP ownership or whole-community participation |
| WHO PHEOC Framework, second edition (2026) | Governance/legal authority; plans/procedures; information systems/data standards; infrastructure/technology; workforce/surge/training/exercises | Quantitative support for scenario analysis, information/evidence lineage, resource ledgers, workforce capacity, recovery trajectories and monitoring | Does not grant legal authority, define national command structure or establish PHEOC capacity |
| ISO 22301:2019 + Amendment 1:2024 | Business continuity management-system requirements, including climate-action amendment | Traceable scenario evidence, continuity-impact measures, resource constraints, recovery-time and improvement evidence | Not a management system, certification assessment or conformity claim; organization retains BCMS responsibility |

As checked for this protocol revision, ISO lists ISO 22301:2019 with Amendment 1:2024 as published, and shows ISO/CD 22301 as a committee draft under development to replace the 2019 edition. The new draft is not a published requirement and must not be treated as one. The NRES crosswalk must be versioned if the standard is subsequently revised.

The appropriate industry success test is **complementary operational value**: can an authorized organization use NRES evidence to make an existing exercise/continuity process more measurable, reproducible and decision-aware without weakening authority, human oversight, inclusion or corrective-action governance? That is materially different from claiming that NRES "replaces HSEEP", "certifies ISO 22301" or "runs a PHEOC".

## 11. Implementation sequence

1. Freeze the micro-instance schema and prove exact dynamic-programming/policy-enumeration agreement.
2. Add explicit information-state identity and non-anticipativity negative controls.
3. Add resource-ledger conservation and pathwise service-floor checks.
4. Add static and rolling-horizon baselines.
5. Add multistage stochastic and two-stage robust baselines.
6. Add nested conditional risk and stagewise transition ambiguity.
7. Add decision-dependent action effects and observation/verification actions.
8. Scale with decomposition only after exact micro-instance conformance is stable.
9. Integrate shift alarms (fixed-reference conformal test martingales / adaptive conformal methods) as monitoring inputs, not as a substitute for valid control.

## 11.1 Exact public micro-oracle: NRES-016-EX0

A stacked child implementation adds EX0, an exact expected-cost dynamic-programming oracle over a finite DAG of unique observable information states. EX0 supports action-conditioned finite transition probabilities, one conserved resource ledger and pathwise service floors. On its tiny three-stage fixture the exact hand solution is expected cost 2.3; the lower-cost DEFER action is infeasible because one positive-probability successor violates the water floor.

EX0 proves only the expected-cost recursion on that finite micro-instance. It does not implement DRO, nested CVaR, partial-observation belief updates, vector resources or scale methods. See PRAXIS-NRES-016-EX0-EXACT-ADAPTIVE-RECOURSE.md. The Rust tests are authored and remain unexecuted until a real toolchain/CI run.

## 12. Safety and status

All NRES-016 examples remain synthetic, public-development-only and inactive. This protocol does not authorize real-world resource dispatch, public-health response, legal determinations, emergency command or autonomous infrastructure control. Human override, role authority, evidence provenance and hard constraints remain first-class. No personal-suitability or composite-resilience score is permitted.
