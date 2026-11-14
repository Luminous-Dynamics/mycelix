# FSRE industry comparison and benchmark protocol v0.2

## Positioning

FSRE is not intended to replace specialised probabilistic linkers, GIS/EAM platforms, physical simulators or mathematical optimisers. It is an evidence-qualified decision assurance layer that composes them while independently qualifying population boundaries, temporal identity, source dependence, assumptions, safety constraints and realised outcomes.

The competitive unit is the complete decision episode, not a single record-pair score or solver objective.

## Industry reference points

### Entity resolution and clerical adjudication

The UK Ministry of Justice's published Splink material describes Fellegi–Sunter probabilistic linkage at production scale. A government case study reports use on approximately 15 million records in under an hour; the MoJ's more recent algorithmic transparency record describes linkage across courts, prisons and probation and warns that sampled clerical labels are not ground truth. These are useful scale and evaluation references, not an apples-to-apples result for infrastructure assets.

OpenRefine's reconciliation workflow makes human review and iterative reconciliation explicit. FSRE should interoperate with candidate-generation and adjudication tools rather than pretend a probability score or connected component proves identity.

### Statistical coverage assessment

The ONS Census 2021 methodology combines census data with a Census Coverage Survey, record linkage, Dual System Estimation, duplicate analysis, small-area estimation and bias adjustment. Its published matching quality requirements include no more than 0.1% false-positive links and 0.25% false-negative links for the particular coverage exercise. These are demanding domain-specific reference values, not universal thresholds.

This is the right conceptual comparator for service-population coverage: independently designed enumeration, not a pair of administrative lists merely agreeing with one another.

### Data-quality standards

ISO 19157-1:2023 describes geographic-data quality components, evaluation and reporting. It explicitly does not set universal minimum acceptable quality levels. FSRE's contribution should be executable, decision-specific acceptance gates with auditable justifications, not a claim that the standard itself prescribes our thresholds.

### Adaptive water and wastewater planning

Two 2026 studies provide concrete resilience/cost comparators:
- Liu, Pianosi and Mijic (Journal of Cleaner Production, 5 March 2026) report a refined real-options configuration that reduced resilience deficit by 74.7% with a 16.5% cost increase in their evaluated case.
- Li et al. (Journal of Environmental Management, 15 June 2026) report 4,860 adaptive pathways across 810 condition pathways for one wastewater/reclaimed-water case; compared with traditional planning they report 7–13% lower lifecycle cost and 5–25% higher resilience, and regret on at least 31% of future pathways when initial plans were not adapted.

These results establish that serious adaptive-planning comparators already exist. They are case-specific findings, not universal targets or directly transferable effect sizes.

## FSRE's differentiator

FSRE should add the assurance obligations that are often distributed across tools:

1. Qualify the service population and register boundary before evaluating cohorts.
2. Preserve source-native identity and bitemporal lineage across renames, splits, merges, transfers and boundary changes.
3. Detect common-source and common-process dependencies before treating registers as independent.
4. Keep missingness, sampling error, model uncertainty, identity uncertainty and legal/source admissibility distinct.
5. Compute sharp bounds only under explicit admissible-world assumptions.
6. Permit recommendations only among actions that satisfy hard authority and safety gates in every admitted world.
7. Replay the decision-time evidence and compare forecasts with realised outcomes.

## Benchmark tiers

### E0 — Exact synthetic worlds

Known population, event history, source dependence, intervention outcomes and safety truth. Use independent exhaustive enumeration as the oracle for finite cases. Every implementation and solver backend must match exact extrema, regret, rank invariance and safety disposition.

### E1 — Validated physical models

Water, energy and transport models with disclosed calibration, parameter uncertainty, boundary conditions and known limitations. Evaluate whether FSRE preserves the model's limitations through decision composition.

### E2 — Authorised operational evidence

Rights-authorised source snapshots, independent adjudication, field observations and realised intervention outcomes. Split by time, site, operator and geography to prevent leakage. A historical register is not treated as complete merely because it is official.

## Matched baselines

- Human/operator runbooks and expert adjudication.
- Deterministic identifiers and static one-to-one crosswalks.
- Splink/Fellegi–Sunter candidate linkage plus clerical review.
- OpenRefine-style iterative reconciliation.
- Post-enumeration survey and Dual System Estimation when the sampling and independence design supports them.
- Multiple-systems/log-linear estimators with explicit interaction sensitivity.
- Manski-style conservative bounds.
- LP partial-identification methods, including causal-order and canonical-domain reduction approaches.
- N-1 redundancy, common-cause analysis, RCPSP/MILP, robust and distributionally robust optimisation.
- Real-options and adaptive-pathway planning.
- Greedy EVSI / finite-horizon measurement acquisition.
- Tool-using AI planners.
- FSRE full stack and pre-registered component ablations.

Every comparator must receive the same frozen evidence, eligible sources, decision cutoff, authority, action set, safety/service floors, resource budget, world set/assumptions and evaluation horizon. Report hardware, versions, configuration and exclusions.

## Target metrics

These are proposed research targets, not achieved results.

| Dimension | Target |
|---|---:|
| Exact E0 oracle agreement | 100% |
| Bound truth inclusion when generated truth is in the declared world set | 100% |
| Adversarial false-safe decisions | 0 |
| Unknown rights/authority/scope promoted to eligible | 0 |
| Incomplete world/action grids accepted | 0 |
| False-positive identity cluster bridges | 0 in the adversarial gold corpus |
| High-confidence identity precision | >=99.5% on independently adjudicated holdout |
| Identity recall | >=95%, with confidence intervals and abstention reported |
| Nominal 95% bound coverage | >=95% on held-out generated/physical-model regimes |
| Historical as-of replay | 100% exact frozen-oracle agreement |
| Late-arrival leakage across recorded-time cutoffs | 0 |
| Deterministic output under input permutation | 100% |
| Safe-service improvement | >=10% on a pre-registered matched metric without floor violations |
| Scaling | 10^3 to 10^7 records/events; 10^8 as a stretch test after matched quality |

Cluster-level false merges and subgroup performance are mandatory; pairwise precision alone can conceal a bridge that corrupts a large connected component. Human labels are reference annotations with measured inter-rater disagreement, not unquestionable ground truth.

## Statistical reporting

- Report precision, recall, abstention, calibration, confidence intervals and cluster-level bridge rates.
- Separate sampling uncertainty, model uncertainty, missingness uncertainty, identity uncertainty and source dependence.
- For probabilistic coverage estimates, report interval coverage, interval width, bias and false-certainty rate by subgroup and scenario family.
- Exact finite-world bounds are expected to include truth in every generated case where the enumerated world set is constructed to contain it. Nominal statistical intervals have their nominal coverage target, not a 100% target.
- A solver certificate verifies the encoded mathematical problem, not its assumptions or evidence.

## How to outperform responsibly

Do not claim to beat Splink at 15-million-record linkage unless the dataset, hardware, precision, recall, cluster policy and runtime are matched. Start by using it as a baseline/candidate generator and measure whether FSRE reduces false bridges and denominator errors without unacceptable recall or review burden.

Do not claim to beat adaptive pathway planning by reporting a larger synthetic objective score. Reproduce the comparator's case or use a common open model, then compare lifecycle cost, resilience deficit, unmet service, regret, adaptation cost, distributional outcomes and safety-floor violations under identical scenario pathways.

The defensible SOTA opportunity is an integrated assurance benchmark: demonstrate that optimisers and physical models can be composed while detecting population, identity, source-dependence and admissibility failures that their input contracts do not establish on their own.

## Primary references

- MoJ, Splink: probabilistic record linkage at scale: https://www.gov.uk/government/publications/joined-up-data-in-government-the-future-of-data-linking-methods/splink-mojs-open-source-library-for-probabilistic-record-linkage-at-scale
- MoJ, Splink algorithmic transparency record: https://www.gov.uk/algorithmic-transparency-records/moj-splink-master-record
- ONS, Census 2021 coverage estimation: https://www.ons.gov.uk/peoplepopulationandcommunity/populationandmigration/populationestimates/methodologies/coverageestimationforcensus2021inenglandandwales
- OpenRefine reconciliation: https://openrefine.org/docs/manual/reconciling
- ISO 19157-1:2023: https://www.iso.org/standard/78900.html
- Liu, Pianosi and Mijic (2026), water resilience planning benchmark: https://doi.org/10.1016/j.jclepro.2026.147850
- Li et al. (2026), adaptive pathways for urban wastewater and reclaimed water systems: https://doi.org/10.1016/j.jenvman.2026.130062
- Rossetto and Antonucci (2026), Partial Identification under Causal Orders by Linear Programming: https://proceedings.mlr.press/v343/rossetto26a.html
- Choe et al. (2026), Canonical Domain Reduction for Partial Counterfactual Identification: https://proceedings.mlr.press/v337/choe26a.html
- Raghavan (2026), Causal Identification from Counterfactual Data: Completeness and Bounding Results: https://proceedings.mlr.press/v306/raghavan26a.html
- Zhang et al. (2026), Bounding the Causal Impact of ML-assisted Decision-Making via Counterfactual Correctness: https://proceedings.mlr.press/v337/zhang26b.html
