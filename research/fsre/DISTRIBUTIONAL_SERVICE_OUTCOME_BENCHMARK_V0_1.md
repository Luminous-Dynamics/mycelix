# FSRE distributional service-outcome benchmark v0.1

**Date:** 2026-11-14  
**Status:** Proposed benchmark and research protocol. Synthetic implementation is a reference composition utility, not a field validation study.

## 1. Research question

Does a recovery decision preserve safe service more equitably across protected cohorts, and can an independent evaluator identify which cohorts remain below their declared service floors, for how long, with what quality failures and with what evidence coverage?

The outcome unit is a **service-cohort lane** over a frozen time horizon. A cohort may represent a municipality, neighbourhood, hospital, school, customer class, service zone or another legitimately defined service population. The cohort definition, denominator, geography, identity mapping and source owner must be recorded; cohort labels must not be used to infer sensitive characteristics or assign normative priorities.

## 2. Why distributional reporting is a research priority

Infrastructure resilience literature increasingly treats social equity as part of service performance rather than a secondary narrative. Coleman et al. (2024) reviewed 99 studies and identified distributional-demographic, distributional-spatial, procedural and capacity equity dimensions. Beck, Cha and Peacock (2025; first published 9 October 2025) describe recognitional, distributional, restorative, procedural and transgenerational dimensions, and demonstrate Theil-T-derived measures for service-performance inequality. Their implementation is a limited hypothetical electric distribution network, so the metric is prior art to compare against, not evidence that a single inequality measure solves equity for every service.

Swanson et al. (first published 15 August 2026) studied essential-service access and post-event recovery following Hurricane Irma. They report that power, cellular service and school outages were among the most important factors in their recovery model, followed by access to essential services. This reinforces the need to preserve cross-sector access and service outcomes; it does not imply that the same ranking applies to South African water or sanitation.

The 2026 system-of-systems review by Shamsi, Helmrich and Deegan identifies 24 tools in the energy, water and transport scope, seven of which explicitly support cross-sector interdependency assessment. It finds no single tool covers the full breadth of interdependencies, and describes a trade-off between modelling complexity and practical usability. FSRE should therefore compose and qualify domain results rather than claim universal simulation superiority.

Water restoration is already an active modelling and planning field. Naeimi and Davidson's REWET study compared 15 restoration plans on Anytown and Los Angeles water networks across earthquake scenarios, and reports substantial reductions in customer-days without service in the tested cases. REWET is an important matched domain comparator; FSRE should not replace hydraulic/network restoration modelling.

## 3. Metric layers

### 3.1 Per-cohort service trajectory

Reuse the existing \`fsre-service-trajectory\` evaluator. For each lane retain:
- minimum safe-service floor and quality policy;
- known and unknown observation duration;
- full service and safe degraded service;
- below-floor and quality-noncompliant duration;
- unsafe service duration and longest unsafe episode;
- service deficit integral in fraction-seconds;
- first known breach, stable safe recovery and full recovery.

Unknown time is not healthy, failed, interpolated or extrapolated. It remains unknown. Quality non-compliance is reported separately and is also unsafe when quality is required.

### 3.2 Distributional cohort envelope

The new \`fsre-cohort-outcome-envelope\` reference composes several lane reports for one service family and one declared cohort set. It reports:
- raw per-cohort outcomes;
- unweighted nearest-rank p50 and p90 of known-time service deficit;
- worst observed deficit cohort and max-min deficit gap;
- longest unsafe episode and its cohort;
- maximum quality-noncompliance duration and its cohort;
- maximum below-floor duration and its cohort;
- counts of cohorts with known breach, observed safe recovery and breach without recovery;
- p50/p90 time-to-safe-recovery among cohorts with an observed recovery;
- unknown cohort count and total cohort-seconds of unknown observation.

Any lane with unknown time makes the envelope \`NOT_ESTIMABLE\` for the declared lane set. The report may still retain known-only statistics, but they must not be presented as complete cohort outcomes.

The cohort set itself is not authenticated or proven exhaustive. \`ESTIMABLE_FOR_DECLARED_LANES\` means only that every submitted lane is covered by known observations over the declared horizon. It does not mean every affected person or service zone is represented.

### 3.3 Equity analysis beyond v0.1

Do not equate equality with justice or automatically minimise disparities without an explicit policy mandate. Future benchmark versions should compare, and separately report:

1. **Threshold-based distributional metrics:** proportion of declared cohorts meeting the minimum floor, duration and magnitude of floor breaches.
2. **Gap-based metrics:** worst-minus-best and p90-minus-p50 service deficit or restoration-time gaps.
3. **Equality-deviation metrics:** measures such as Theil's T or Gini where the underlying measure has a defensible ratio-scale interpretation and zero handling is preregistered.
4. **Restorative equity:** differences between owner- and community-defined groups of concern, with explicit policy justification and baseline inequity.
5. **Procedural equity:** who defined cohort boundaries and floors, who could challenge evidence, who participated in prioritisation and who had authority.
6. **Transgenerational and adaptive outcomes:** whether recovery investments reduce future exposure or simply restore the prior unequal baseline.

Population weighting should be added only when denominators have source lineage, effective dates, cohort identity comparability and independent review. A missing or incompatible denominator must not be guessed. Social weights, priority classes and service entitlements must be operator/community/governance inputs, not inferred by the evaluator from cohort names.

## 4. Benchmark baselines

Evaluate identical service trajectories and frozen cohort boundaries using:

- B0: one aggregate mean trajectory (deliberately weak baseline; measures what aggregation hides);
- B1: service-floor compliance count only;
- B2: worst-cohort service deficit and recovery time;
- B3: unweighted p50/p90 and maximum gap;
- B4: population-weighted distribution metrics, but only for independently validated denominators;
- B5: Theil-T or other inequality metrics with documented zero handling and group decomposition;
- B6: restorative-equity policy objective using independently governed priority classes;
- B7: FSRE per-lane trajectory plus the unweighted cohort envelope, with source/evidence lineage and unknown-state handling.

For water-network restoration, compare the physical action plans and realised service trajectories with REWET on matched cases. Use its hydraulic and restoration model as the domain baseline. FSRE's testable contribution is the reproducible evidence and multi-cohort outcome qualification layer, not hydraulic accuracy.

## 5. Proposed acceptance targets

These are research targets, not achieved results.

| Metric | Proposed target |
|---|---:|
| Per-lane evaluator agreement with frozen answers | 100% |
| Unknown observation treated as known-safe or known-failed | 0 |
| Quality failure silently counted as safe service | 0 |
| Declared cohort record omitted from the output | 0 |
| Duplicate cohort identity accepted | 0 |
| Unit/horizon/service-scope mismatch accepted | 0 |
| Worst cohort hidden by aggregate output | 0 |
| Qualified cohort lane with owner, denominator, identity and horizon lineage | 100% |
| Population-weighted metrics using unknown/unreconciled denominator | 0 |
| Unweighted p50/p90 calculation agreement | 100% on frozen examples |
| Distributional metric reproducibility | byte-identical output from frozen inputs |
| Equity benchmark holdout | at least one blinded geography, hazard family and cohort definition |

Do not treat all zero-error targets as statistically qualified by a small synthetic test set. Publish numerators, denominators, exact and cluster-aware uncertainty bounds, cohort coverage and abstention rates.

## 6. Experimental design

Keep planning and outcome evaluation separate. The decision method receives only information admissible at the declared decision cutoff. The outcome evaluator may use retrospective observations, but must preserve observation, availability, validity and correction times so hindsight cannot leak into the original decision.

Use paired scenario-level comparisons. Freeze:
- source snapshots, cohort boundaries and denominator versions;
- service and quality floors;
- time horizon, observation hold and recovery stability;
- hazard/scenario family and operational budget;
- missingness, censoring and late-correction treatment;
- baseline metrics, percentile convention and any equity weights.

Hold out complete geographies, operators, hazards or cohort definitions rather than randomly splitting rows from the same scenario. Have an independent reviewer adjudicate the cohort boundaries, service outcomes and expected metric values. Report service floors and cohort coverage beside distributional metrics. A lower inequality score is not an improvement if every cohort receives less safe service.

## 7. South African water application

For Green Drop, a candidate cohort could be a municipal wastewater service area, but identity and population boundaries must be reconciled before comparison. A plant-level audit score is not itself a population service trajectory. The outcome boundary needs receiving-environment, population and safe-treatment measures appropriate to the question.

For potable-water studies, delivered volume without laboratory-confirmed quality is not safe service. For wastewater, throughput without verified treatment/discharge quality is not proof of safe sanitation service. Unknown laboratory observations must remain unknown.

The 2022 and 2025 Green Drop audit populations must not be treated as a fixed cohort until identity and ownership transitions are source-backed. National percentages cannot substitute for per-system or per-cohort outcome data. Use owner-authorised machine-readable records and a separate data-governance review before production use.

## 8. Limitations of the v0.1 implementation

The v0.1 envelope:
- accepts one lane file per cohort;
- requires a common service, service family, horizon and core observation/quality policy;
- permits cohort-specific safe-service floors;
- preserves denominator values but does not weight by them;
- calculates unweighted nearest-rank p50/p90 and worst-cohort metrics;
- rejects incomplete observation envelopes for an estimable-for-declared-lanes disposition;
- does not authenticate sources, cohort identities or population coverage;
- does not calculate Theil-T, Gini, social priority weights or causal effects;
- does not make allocation recommendations or authorise operations.

The next step after synthetic conformance is a blinded, independently authored cohort corpus and a historical read-only replay with valid cohort denominators, source lineage and community/utility review.

## References

- Coleman, N., Li, X., Comes, T. & Mostafavi, A. (2024-09-02). *Weaving equity into infrastructure resilience research: a decadal review and future directions*. npj Natural Hazards 1, Article 25. https://www.nature.com/articles/s44304-024-00022-x
- Beck, A. L., Cha, E. J. & Peacock, W. G. (first published 2025-10-09). *Development of an Equity Metric for Infrastructure Service Performance Assessment*. Risk Analysis 45(11), 3899–3914. https://doi.org/10.1111/risa.70126
- Swanson, T., Guikema, S., Nelson-Schmitz, B. & Bricker, J. (first published 2026-08-15). *Access to Essential Services: A Critical Factor for Predicting Recovery and Quantifying Resilience*. Risk Analysis. https://doi.org/10.1111/risa.70323
- Shamsi, N., Helmrich, A. & Deegan, M. (published 2026-11-07). *Toward system-of-systems resilience: a comparative framework for infrastructure interdependency assessment tools*. Environment Systems and Decisions. https://doi.org/10.1007/s10669-026-10162-2
- Naeimi, S. & Davidson, R. A. (2025). *Improving Restoration of Water Supply Service Following Disruptions*. Journal of Water Resources Planning and Management 151(7). https://doi.org/10.1061/JWRMD5.WRENG-6452
