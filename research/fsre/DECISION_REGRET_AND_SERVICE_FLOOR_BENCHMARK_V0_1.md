# FSRE realised decision-regret and service-floor benchmark v0.1

**Date:** 2026-11-14  
**Status:** Benchmark protocol and input contract. No comparative advantage or field result is claimed.

## 1. Purpose

FSRE already has two related but different capabilities:

- `fsre-decision-value` computes finite-state expected value of sample information (EVSI) using exact integer probability weights and loss units. It enforces measurement safety/authorization, keeps the action admissibility set fixed, and selects at most one next measurement from a frozen evidence snapshot. Its own README correctly describes it as a greedy one-step reference, not a globally optimal multi-step acquisition policy.
- `fsre-cohort-outcome-envelope` composes independent service trajectories across declared cohorts, preserving quality failures, unknown observation time, worst-cohort outcomes and unweighted distributions.

Neither by itself answers the post-decision question:

**Did the action selected at the declared decision cutoff actually preserve more safe service, within its authorised constraints and resource budget, than the other eligible plans could have achieved in the realised scenario?**

This benchmark adds the missing evaluation contract. It does not add another optimiser. It evaluates the consequences of decisions made by existing operator procedures, physical optimisers, EVSI policies, AI agents and FSRE.

## 2. The unit of evaluation: a decision episode

A decision episode consists of:

- one frozen evidence snapshot and declared information cutoff;
- one service boundary, cohort manifest and population denominator version;
- one disturbance/hazard scenario or scenario family;
- one finite set of candidate recovery actions;
- one decision authority, permission set and hard safety constraints;
- one resource and lifecycle budget;
- one declared planning horizon and response window;
- one independent outcome reconstruction protocol.

Use a stable episode ID and bind every action, input, outcome and replay to that ID. The evaluation unit is the decision episode, not an interval row, action row, observation row or generated scenario variant. Correlated variants from the same event, utility, hazard generator or catchment must not be counted as independent episodes.

The information available at the decision cutoff is immutable for replay. Later corrections and observations are preserved separately and may be used by the outcome evaluator, but cannot be leaked into the original decision state.

## 3. Do not collapse distinct forms of regret

Three quantities are distinct and must not be reported as one scalar.

### 3.1 Expected decision value before the action

The existing EVSI reference estimates the expected reduction in decision loss available from acquiring a candidate measurement, after acquisition and delay costs, under a frozen finite hypothesis model. This is an ex-ante information-value calculation. It is not realised recovery performance and it is not evidence that a measurement was actually acquired or a plan executed.

Compare greedy one-step EVSI with:
- current operator information-gathering practice;
- random and no-new-measurement baselines;
- finite-horizon dynamic programming or adaptive acquisition where the model permits it;
- robust or distributionally robust acquisition policies under deep uncertainty.

Do not add individual EVSI values calculated from the same prior: candidate measurements can be redundant and their benefits non-additive.

### 3.2 Ex-post decision regret when counterfactual outcomes are identified

For a frozen episode with a declared, independently adjudicated candidate-action set and a known realised scenario, let:

- `a_chosen` be the action selected at the original decision cutoff;
- `A_eligible` be the actions that were authorised and admissible under the *same decision-time authority and safety policy*;
- `L(a, s)` be the pre-registered loss of action `a` under realised scenario `s`;
- `A_safe(s)` be the actions in `A_eligible` independently established to be physically safe and feasible in `s`, with all protected service floors met.

The service-floor-safe oracle is:

    a_star(s) = argmin { L(a, s) : a in A_eligible and a in A_safe(s) }

The regret is:

    regret(a_chosen, s) = L(a_chosen, s) - L(a_star(s), s)

Calculate this scalar only when:
1. the chosen action was authorised and admissible at the decision cutoff;
2. the chosen action's outcome is known;
3. at least one action in the eligible set is independently established as safe and floor-compliant;
4. the potential outcomes needed to compute the oracle are identified by the benchmark design; and
5. the loss function and all of its weights, units and exclusions were pre-registered.

If the chosen action violates a non-negotiable service floor, do **not** hide that violation inside an arbitrary finite penalty. Report a separate hard-floor failure. A scalar regret can be `NOT_COMPUTED_DUE_TO_FLOOR_VIOLATION` while the floor failure remains a decisive negative outcome.

If no eligible action meets the floor, report `NO_FLOOR_COMPLIANT_ACTION_IDENTIFIED`; do not manufacture a safe oracle. Report the best observed service deficit separately.

### 3.3 Model/eligibility gap

A planner may omit a physically feasible action because its model, data or feasibility gate says it is inadmissible. Conversely, it may admit an action that independent adjudication later finds unsafe or unavailable.

Report these separately:

- **False-safe admission:** selected or recommended action was not safe/feasible under independently adjudicated conditions.
- **False-infeasible exclusion:** an action excluded by the decision-time feasibility model was independently established safe, feasible and authorised under the same policy.
- **Authority violation:** the selected action did not have the authority required at the decision cutoff.
- **Constraint omission:** a known common-cause, workforce, access, inventory, quality or cohort-floor constraint was omitted.
- **Abstention/evidence debt:** the system withheld qualification because a required claim was unknown or inadmissible.

Do not retroactively give an algorithm permission it did not have. The oracle may compare only actions within the same legitimate authority boundary. Physical feasibility discovered with hindsight is not itself authority.

## 4. The counterfactual-identification boundary

The benchmark must declare which of three evidence regimes it is using.

### E0 — synthetic exhaustive potential outcomes

All candidate-action outcomes are declared in a frozen synthetic or mathematical test world. This supports exact regret calculation and deterministic reference agreement. It demonstrates implementation conformance, not real-world causal validity.

### E1 — validated physical-model potential outcomes

All action outcomes are generated by a sector-specific simulator or independently validated digital twin. The report must name the model, version, calibration data, validation envelope, uncertainty, scenario distribution and known model-form limitations. This estimates regret **within that model**, not automatically in the real system.

Use at least one independent physical model or held-out field evidence where possible. A planner must not use the same model as both its sole decision authority and its sole outcome judge without disclosing that common dependency.

### E2 — observed field outcomes

In ordinary operational data, only the chosen action's outcome is observed. The outcomes of unchosen plans are counterfactual and are not identifiable merely because an optimiser can simulate them.

For E2:
- report the realised service trajectory, costs, constraint events and cohort effects for the action actually taken;
- do not label the difference from a modelled alternative as measured regret;
- estimate comparative causal effects only under an explicit identification strategy such as randomised/stepped-wedge deployment, valid natural experiment, carefully justified instrumental variable, or a pre-registered off-policy estimator with overlap and confounding diagnostics;
- publish positivity/overlap, treatment-assignment assumptions, censoring, interference and missingness;
- if the comparison is not identified, report `COUNTERFACTUAL_REGRET=NOT_ESTIMABLE`.

A forecast of an unchosen action is not an observed potential outcome. A digital twin's prediction is not field ground truth. Retrospective knowledge of what happened is not permission to rewrite the decision-time evidence frontier.

## 5. Safety-first outcome vector before scalar loss

Do not make social priorities disappear inside a single weighted sum. Each decision episode must first report a vector of hard constraints and service outcomes.

### Hard constraints

- authority and legal permission at the decision cutoff;
- option, site and crew safety;
- common-cause failure exposure;
- workforce skill, hours, shift and fatigue limits;
- physical access and route/time-window limits;
- inventory, replenishment and resource conservation;
- water-quality / treatment-quality requirements;
- service floors for every protected cohort;
- prohibited transfers of risk between cohorts.

### Service-outcome vector

- safe service-hours by cohort;
- cumulative and peak service deficit by cohort;
- longest consecutive interval below floor;
- time to first and sustained safe restoration;
- quality-noncompliant and unsafe-delivery duration;
- relapse after apparent recovery;
- affected population/service denominator and missingness;
- cohort-set completeness relative to its declared manifest;
- distributional p50/p90, worst-cohort and gap measures;
- spatial access and remote/isolated cohort impacts.

### Cost and resource vector

- mobilisation and lifecycle cost;
- consumption, replenishment, stock loss and waste;
- crew-hours, specialist skills and overtime;
- energy/water/chemical use;
- environmental and downstream treatment cost;
- decision delay, measurement acquisition and verification cost;
- operational disruption imposed on other service areas.

Only after hard safety and floor constraints are disclosed may a study calculate a scalar loss. The scalar loss must be a governance-owned and pre-registered function, with explicit weights, units, discounting, time horizon and uncertainty. Where no legitimate scalarisation exists, use Pareto dominance and report the nondominated set rather than inventing social weights.

A lower inequality index is not an improvement if safe service falls for every cohort. A higher aggregate safe-service total is not an improvement if it is achieved by violating a protected cohort's minimum floor.

## 6. Comparator programme

Use matched episodes, the same information cutoff, same source inputs, same permissions, same candidate-action universe, same service floors, same resources and same outcome adjudication.

| Comparator | What it tests |
|---|---|
| B0 — current utility/operator runbook | real procedural baseline, escalation, coordination and human expertise |
| B1 — nominal redundancy and N-1 | what is gained beyond counting alternatives and summing nameplate capacity |
| B2 — common-cause/dependency analysis | value of shared-failure and common-resource qualification |
| B3 — deterministic RCPSP/MILP restoration | schedule and resource feasibility under known inputs |
| B4 — sector-specific physical restoration model, including REWET for water-network cases | hydraulic/physical plan quality; do not replace this model with FSRE |
| B5 — robust and distributionally robust multi-period optimisation | performance under deep uncertainty and future climate/operating conditions |
| B6 — current EAM/APM, GIS, inventory, workforce and operations workflows | value added over incumbent enterprise capability |
| B7 — qualified operator/engineer panel | expert judgement under identical evidence and time budgets |
| B8 — strong AI planner and strong AI+tools planner | value of evidence/authority gates beyond agent planning |
| B9 — existing FSRE decision-value EVSI reference | greedy one-step measurement acquisition |
| B10 — finite-horizon/adaptive measurement policy | value of planning information acquisition over the horizon |
| B11 — FSRE full composition and ablations | attributable benefit of as-of evidence, portfolio feasibility, cohort manifest, source-use gate and outcome reconstruction |

Task fit must be documented. Do not declare a method inferior because it does not implement a task outside its stated purpose.

## 7. Preregistered endpoints and comparative targets

These are proposed targets, not achieved performance.

### Primary endpoint

Choose exactly one before holdout access:
- counterfactual decision regret (E0/E1 or causally identified E2 only);
- worst protected-cohort cumulative safe-service deficit;
- lifecycle cost per independently verified safe-service hour above the comparator;
- time to sustained safe-service floors, with no floor violation in any protected cohort.

### Secondary endpoints

- false-safe and false-infeasible rates;
- authority violations and omitted constraints;
- common-cause exposure and option survival;
- measurement net EVSI and realised decision change;
- time-to-decision and time-to-service;
- lifecycle cost, crew contention and stockouts;
- worst-cohort and p90/p50 recovery disparity;
- abstention rate, evidence debt, human-review workload;
- uncertainty calibration, decision coverage and external validity.

### Research targets

| Metric | Proposed target |
|---|---:|
| Frozen deterministic decision-reference agreement | 100% |
| False-safe negative controls | zero observed; publish denominator and exact/cluster-aware interval |
| Unauthorized consequential action | 0 |
| Known service-floor failure hidden by aggregate score | 0 |
| Unidentified counterfactual reported as measured regret | 0 |
| Decision/evidence lineage for qualified episodes | 100% |
| Decision regret vs strongest matched baseline | ≥10% improvement where regret is identified and powered |
| Lifecycle cost per verified safe-service-hour | ≥10% improvement without worsening any cohort's floor |
| EVSI measurement acquisition | positive net value after actual acquisition, latency, maintenance and delay cost |
| Uncertainty calibration | pre-registered interval coverage and proper scoring rules, stratified by scenario family |

The 10% improvements are hypotheses. Do not claim success from a synthetic oracle corpus, unpaired cases, different permissions or unequal budgets.

For rare false-safe outcomes, zero failures across 50 independent negative controls still provides weak evidence. Under independent Bernoulli assumptions, a one-sided 95% upper bound below 1% requires at least 299 zero-failure episodes. Correlated variants from one generator, utility or hazard require cluster-aware analysis and more independent episodes.

## 8. Experimental protocol

1. Freeze the decision snapshot, action universe, authority, service floors, cohort manifest, resource budget and source-use status.
2. Freeze the primary endpoint, loss function, outcome evaluator and counterfactual-identification regime.
3. Give each comparator identical time, data, permissions, compute and human oversight.
4. Record each proposed plan, abstention, missing input, excluded action, constraint and chosen action before revealing the scenario outcome.
5. Reconstruct outcomes with an evaluator independent of the planning method. Preserve full per-cohort trajectories and quality metrics.
6. For E0, use exhaustive frozen potential outcomes and a separately authored answer key.
7. For E1, disclose model validity, calibration, solver gap, parameter uncertainty and independent-model checks.
8. For E2, report observed outcomes and only identified causal/off-policy effects. Otherwise regret remains NOT_ESTIMABLE.
9. Compare methods in paired decision episodes. Hold out complete utilities, geographies, hazard families, operators or time periods; do not randomly split correlated rows.
10. Report confidence intervals, exact denominators, clustering, missingness, counterfactual coverage, abstention and human effort. No scalar "winner" without its uncertainty and scope.

## 9. How the benchmark composes existing FSRE components

This benchmark should reuse rather than replace:
- `fsre-decision-value`: exact finite-state EVSI, safe/authorised measurement gating and one-step next-measurement selection;
- `fsre-time-expanded-portfolio-composer`: interval alignment, portfolio/stock-flow joins and decision-time evidence linkage;
- `fsre-cohort-outcome-envelope`: per-cohort quality, deficit, recovery and distributional summaries;
- `fsre-cohort-manifest-qualification`: declared cohort membership, denominator and identity-source reconciliation;
- the source-use preflight: explicit block for unresolved or expired source-use assertions.

The next code increment, if the benchmark is accepted by independent review, should be a small read-only reference that consumes a frozen episode manifest, action/eligibility receipts and outcome table. It should calculate regret only for exhaustive/identified counterfactual sets and fail closed otherwise. It should not schedule crews, optimise real dispatch, grant authority or choose a socially normative utility function.

## 10. Falsification conditions

The hypothesis that FSRE improves decision quality is weakened or falsified if:
- incumbent workflows achieve equivalent service and cost outcomes under matched evidence, permissions and budgets;
- FSRE reduces false-safe decisions only by abstaining on most consequential episodes;
- the evidence or audit layer delays a decision enough to worsen safe service;
- the regret reduction disappears on held-out utilities or hazard families;
- improvements arise solely from a different loss function or privileged human intervention;
- population/cohort coverage is materially incomplete;
- the outcome oracle shares the same model errors as the planner;
- unchosen-action counterfactuals cannot be identified outside synthetic/model-only tests.

The intended contribution is not to own every simulator or optimizer. It is to make the evidence, constraints, service distributions and comparative decision value behind their recommendations reproducible and honest.

## References

- Existing FSRE decision-value implementation: `tools/fsre-decision-value/README.md` (finite-state exact-rational EVSI; one-step greedy reference; not a finite-horizon optimum).
- Liu, L., Pianosi, F. & Mijic, A. (2026-03-05). *A benchmarking framework to enhance cost-efficiency in water systems resilience planning under deep uncertainty*. Journal of Cleaner Production 548, 147850. https://doi.org/10.1016/j.jclepro.2026.147850
- Arnold, W., Giuliani, M. & Castelletti, A. (2026-06-24). *Balancing equity and efficiency in transboundary water systems with Atkinson’s welfare function*. Nature Water. https://www.nature.com/articles/s44221-026-00671-4
- Naeimi, S. & Davidson, R. A. (2025). *Improving Restoration of Water Supply Service Following Disruptions*. Journal of Water Resources Planning and Management 151(7). https://doi.org/10.1061/JWRMD5.WRENG-6452
- Shamsi, N., Helmrich, A. & Deegan, M. (2026-11-07). *Toward system-of-systems resilience: a comparative framework for infrastructure interdependency assessment tools*. Environment Systems and Decisions. https://doi.org/10.1007/s10669-026-10162-2
