# FSRE time-expanded recovery portfolio experiment card v0.1

**Date:** 2026-11-14  
**Status:** Preregistration template and benchmark protocol. No operational comparison or FSRE advantage is claimed.

## 1. Research question

Under identical evidence, permissions, hazard scenarios, service floors, resource budgets and evaluation horizons, does an evidence-qualified portfolio workflow help an operator select a recovery plan that preserves more safe service for every protected cohort, with fewer unsupported feasibility claims and lower decision regret, than the operator's current procedure and the strongest applicable restoration-planning methods?

The unit of analysis is a **decision episode**: one frozen information frontier, one affected service boundary, one portfolio of candidate recovery options, one declared disturbance family and one evaluation horizon. Do not treat each interval in the same episode as an independent experiment.

## 2. Primary hypothesis and outcome

Select exactly one primary operational endpoint before accessing the holdout:

- **Decision regret:** realised service-weighted loss of the selected plan minus the best feasible plan identifiable under the same realised scenario and authorised action set; or
- **Worst-cohort service deficit:** maximum cumulative shortfall against the pre-registered service floor across protected cohorts; or
- **Lifecycle cost per independently verified safe-service unit:** full mobilisation, operating, replenishment, labour, environmental and recovery cost divided by independently measured safe-service delivery above the declared comparator.

The primary endpoint must have a directional hypothesis, minimum practically important effect, unit, denominator, time horizon, missing-data treatment and pre-specified analysis. The provisional 10% improvement hypothesis is a research target only. Do not select the endpoint or threshold after viewing holdout results. A primary endpoint cannot improve by sacrificing a protected cohort's minimum floor.

Secondary outcomes include safe service-hours, time to first and stable floor restoration, cumulative and longest consecutive below-floor period, unsafe-quality exposure, stockouts, crew overcommitment, option survival under each declared shock, common-cause concentration, spatial access, authority continuity, lifecycle cost, runtime, optimality gap, analyst time, and abstention/evidence-debt rates.

## 3. Safety gates and what sample size can support

The hard safety outcomes are:

- false-safe: the method declares a portfolio feasible when an independently adjudicated service, safety, authority, resource or common-cause constraint makes it infeasible;
- false-independent: the method claims independent recovery options despite an owner-qualified shared critical failure domain;
- false-floor-met: an affected cohort's floor is reported as met when the independently adjudicated quantity/quality record shows it was not;
- temporal leakage: a decision uses evidence not admissible at its declared decision cutoff;
- over-capacity: a plan is accepted despite resource, inventory, crew, time-window or unit constraints being exceeded;
- unauthorised action: the tool itself issues or commits an action outside the explicitly granted test authority.

The 50-case public recovery-option suite is a useful regression set, but a zero-failure result over 50 independent negative controls still has a one-sided exact 95% binomial upper confidence bound of approximately 5.82% for the underlying failure probability. It is not evidence of a near-zero operational error rate.

For zero observed failures in n independent Bernoulli trials, the one-sided exact 95% upper bound is:

    p_upper = 1 - 0.05^(1/n)

Under that simplifying independence assumption:
- 299 negative controls with zero false-safe outcomes bound the rate below approximately 1%;
- 2,995 negative controls with zero false-safe outcomes bound the rate below approximately 0.1%.

These are minimum arithmetic illustrations, not sufficient experimental designs by themselves. Scenario variants sharing one causal template, utility, hazard or data source are correlated. Count independent decision episodes, stratify by hazard and failure family, and use cluster-aware confidence intervals or hierarchical models when observations share a common scenario generator, geography, operator or time period. Report exact numerators, denominators, confidence bounds and clustering assumptions. Never report "0%" without its sample size and uncertainty.

No consequential live action is permitted in this experiment. The zero-unauthorised-action gate is architectural and procedural: the test harness must have no operational write credentials, and its logs must show no dispatch/write endpoint calls.

## 4. Challenge-set design

Maintain three distinct sets:

1. **Public regression set:** developer-visible examples, including the existing recovery-option cases and the new three-interval composition fixture. This set detects regressions; it does not establish independence.
2. **Blinded qualification set:** authored by a reviewer who did not implement or tune either evaluator. Keep input manifests, expected outcomes and adjudication notes access-controlled until the implementation freeze.
3. **Prospective shadow set:** owner-authorised live/historical operating data replayed with read-only credentials and no actuation. It tests data gaps, operator workflow and real-world distribution shift; it is not interchangeable with synthetic qualification cases.

Each challenge family contains positive controls, known-infeasible controls, unknown-evidence controls, temporal controls and compound interactions. Families should include:

- shared feeder, upstream supplier, communications, road corridor and control-plane failure;
- crew skill, shift overlap, fatigue and simultaneous mobilisation contention;
- stock depletion, replenishment delay/non-arrival, expiry, loss and overdraw;
- access closures, route-time windows and spatially isolated cohorts;
- option, evidence and authority expiry/revocation during a multi-interval horizon;
- water-energy-telecom cascades and common upstream supplier concentration;
- quantity recovery with quality relapse and unsafe delivery;
- aggregate-service recovery with a protected cohort still below floor;
- cohort boundary and infrastructure identity changes across reporting periods;
- delayed observations, clock uncertainty, future-information leakage and forecast/observation confusion;
- contradictory source records, missing units, incomplete ownership and unknown capacity;
- repair crew, spare-parts and equipment contention across otherwise feasible plans.

Use scenario-family holdouts, not only random row splits. At least one entire hazard/failure family, geography or operator period should remain untouched during design and tuning. Publish how the test set was constructed, but do not publish the sealed answers before the evaluation freeze.

## 5. Comparators and matched conditions

Use a tiered comparison. Do not force a tool to perform outside its intended method or claim a missing benchmark as a loss or a win.

| Comparator | What it tests |
|---|---|
| B0: current operator runbook and decision meeting | Real baseline, including human coordination, escalation and safety procedures |
| B1: N-1 / nominal redundancy count | Whether portfolio constraints add value beyond option counts |
| B2: dependency and common-cause/cut-set analysis without service-floor composition | Incremental value of explicit service and cohort feasibility |
| B3: deterministic RCPSP / MILP restoration plan | Resource-constrained schedule quality under known inputs |
| B4: strongest applicable domain simulator or optimiser, including REWET for water-distribution restoration cases | Whether FSRE complements or improves decisions beyond sector-specific physical restoration methods |
| B5: robust or distributionally robust multi-period optimisation where assumptions and data support it | Performance under epistemic/distributional uncertainty |
| B6: incumbent EAM/APM, inventory, workforce, GIS and operational workflow | Incremental value over installed enterprise systems |
| B7: qualified operator/expert panel | Human judgement under the same evidence and action budget |
| B8: strong AI planning agent under identical information and read-only permissions | Whether structured evidence gates improve on unstructured agent planning |
| B9: FSRE full composition and ablations | Value attributable to each evidence and assurance layer |

Freeze scenario definitions, data snapshots, information cutoff, service/cohort boundaries, permissions, resource budgets, solver limits, compute/hardware, tuning effort, analyst time, staffing assumptions and outcome measurement. Give every comparator equivalent source information and equivalent time to prepare a plan. Record unsupported inputs and unavailable functions as such; do not silently fill them.

For optimisation methods, publish feasibility tolerance, objective weights, solver, optimality gap, random seeds, solve time and whether the result is optimal, feasible or heuristic. For operator panels, record experience, briefing time, decision time, disagreement and adjudication method. For AI agents, freeze model/version, prompts, tools, context, budget, retrieval sources, action permissions and human intervention.

## 6. Outcome reconstruction and distributional reporting

An independent outcome evaluator, separate from the planning method, reconstructs the realised time series. Use source-owned telemetry, laboratory quality results, customer/cohort registers, operator logs, field repair records, stock movements and authority records with declared provenance and uncertainty.

For each scenario, cohort and interval retain:
- required quantity and quality floor;
- observed delivered quantity and quality;
- denominator, service boundary and missingness;
- whether delivery is safe, unsafe, unknown or not observed;
- time to first floor restoration, stable restoration and relapse;
- cumulative service deficit and longest continuous deficit;
- unserved population/time and any displacement between cohorts;
- evidence references, source owners, time basis and retrospective corrections.

Keep full trajectories and event milestones. Report the full distribution, worst case, median, quantiles and pre-registered tail metrics, not only the mean or an expected curve. Where a quality threshold is a safety condition, unsafe quantity is not counted as safe delivered service. A missing measurement is not assumed to meet the floor.

Cohort identity changes, merges, splits, boundary adjustments and owner changes require explicit comparability review. A national percentage or asset count is not a longitudinal system-level outcome without a defensible identity crosswalk and denominator.

## 7. Statistical analysis

Use paired scenario-level comparisons wherever all methods are evaluated on the same decision episode. Pre-register:
- primary endpoint and minimum important effect;
- analysis unit and clustering level;
- exclusion and missingness rules;
- confidence level and multiplicity handling;
- method for paired effect estimates and confidence intervals;
- subgroup and tail analyses;
- rules for handling failed, infeasible, abstaining or timed-out plans.

Prefer paired bootstrap or suitable hierarchical/mixed-effects analysis clustered at the decision-episode or scenario-family level. For rare failures, publish exact binomial bounds where independence is defensible and cluster-aware uncertainty otherwise. Do not treat interval rows, option rows or generated variants from one scenario as independent trials.

Report both coverage and abstention. A method cannot improve apparent safety by abstaining on every hard case, and it cannot improve apparent coverage by promoting unknowns. Use a two-axis view: false-safe rate among adjudicated cases and coverage of cases with sufficient admissible evidence. Report the cost and delay of obtaining missing evidence.

## 8. Ablations and causal attribution

Remove one factor at a time, preserving all other inputs and permissions:

- no owner-qualified common-cause receipts;
- no shared-resource/stock-flow reconciliation;
- no temporal validity or expiry;
- no cohort-specific service floors;
- no geospatial access;
- no authority continuity checks;
- no decision-time evidence filter;
- no independent evaluator;
- no provenance/lineage output.

Measure which errors and decision outcomes change. A component that increases abstention may still reduce false-safe classifications; report both. Do not attribute a result to "FSRE" when the benefit comes from an incumbent simulator, a changed objective weight or an additional human reviewer.

## 9. Current research baseline and honest SOTA position

The 2026 comparative review by Shamsi, Helmrich and Deegan (version of record 7 November 2026) identifies 24 tools in scope across energy, water and transport, with seven explicitly modelling cross-sector interdependencies. It reports an uneven landscape and a trade-off between model complexity and practical usability. This supports the need for interoperable, complementary methods; it does not establish that FSRE is better than any of those tools.

Water restoration is already an active optimisation/simulation field. Naeimi and Davidson's 2025 REWET study compares 15 restoration plans across Anytown and Los Angeles networks and reports substantial reductions in customer-days without service under its tested earthquake scenarios. FSRE must compare against REWET on matched water-network tasks rather than claim that portfolio accounting replaces physical network restoration or crew routing.

Resource-constrained project scheduling is established prior art for infrastructure recovery. Robust and distributionally robust multi-period optimisation are also active research directions. FSRE's defensible differentiation hypothesis is the composition of owner-qualified evidence, common-cause and resource constraints, time-valid authority, per-cohort service floors, decision-time admissibility and independent outcome reproduction—not the invention of scheduling, interdependency modelling or resilience curves.

The 2021 review by Poulin and Kane covers 273 resilience-curve publications, recommends careful selection of productivity/quality measures and milestone metrics, and cautions against collapsing ensembles into one expected trajectory without examining assumptions. This benchmark therefore retains cohort trajectories and distributions.

NIST SP 800-160 Vol. 2 Rev. 1 already frames cyber-resiliency engineering as a systems-engineering discipline used alongside systems security and resilience engineering. FSRE should align with established resilience engineering and security practice.

## 10. Deployment gates

Progress only through these gates:

1. **Public synthetic conformance:** deterministic fixtures, explicit negative controls, frozen expected outputs and exact-head CI.
2. **Independent review:** disjoint challenge corpus, reviewer-authored answers, documented adjudication and no tuning on sealed cases.
3. **Historical replay:** lawful and owner-authorised system-level data, identity comparability, complete denominator and read-only reproduction.
4. **Prospective shadow mode:** live data, read-only permissions, operator review, no writes or control endpoints.
5. **Limited decision support:** only after an approved safety case, local governance, auditability, rollback and explicit human authority. No autonomous dispatch is implied by this benchmark.

Every gate records what was implemented, what was structurally checked, what CI executed, what an independent reviewer adjudicated, and what remains unobserved or physically unvalidated.

## References

- Shamsi, N., Helmrich, A. & Deegan, M. (published 7 November 2026). *Toward system-of-systems resilience: a comparative framework for infrastructure interdependency assessment tools*. Environment Systems and Decisions. https://doi.org/10.1007/s10669-026-10162-2
- Naeimi, S. & Davidson, R. A. (2025). *Improving Restoration of Water Supply Service Following Disruptions*. Journal of Water Resources Planning and Management, 151(7). https://doi.org/10.1061/JWRMD5.WRENG-6452
- Poulin, C. & Kane, M. B. (2021). *Infrastructure resilience curves: Performance measures and summary metrics*. Reliability Engineering & System Safety, 216, 107926. https://doi.org/10.1016/j.ress.2021.107926
- NIST SP 800-160 Vol. 2 Rev. 1 (2021). *Developing Cyber-Resilient Systems: A Systems Security Engineering Approach*. https://csrc.nist.gov/pubs/sp/800/160/v2/r1/final
- Johnson, K., Leandro, J. & Ahadzie, P. (2024). *Quantifying hazard resilience by modeling infrastructure recovery as a resource-constrained project scheduling problem*. Natural Hazards and Earth System Sciences, 24, 2285–2302. https://nhess.copernicus.org/articles/24/2285/2024/
