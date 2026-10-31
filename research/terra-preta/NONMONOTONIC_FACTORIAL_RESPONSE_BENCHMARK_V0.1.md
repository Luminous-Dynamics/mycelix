# Non-monotonic factorial response benchmark v0.1

**Status:** research specification. Proposed numerical gates are qualification targets, not achieved results or biological recommendations.

## 1. Paradigm shift: replace the universal dose curve with a conditional response surface

Amendment response may depend on dose, root regime, wet–dry history, antecedent moisture, soil source, material batch, forcing and elapsed time. A scalar dose curve suppresses precisely the interactions that may control transport and plant service.

The research object is a conditional surface:

\[
Y = f(dose, root, wet\text{-}dry\ history, antecedent\ state, soil, forcing, time)
\]

The same dose may reduce leakage in bare soil, have a different effect in rooted soil, and change direction after repeated wet–dry cycling. The endpoint's sign also depends on the application: reduced conductivity may be desirable in an engineered cover but harmful in a cropped root zone if infiltration or drainage is impaired.

The question is not "what is the optimal biochar rate?" It is "which response surface is identifiable for this material, soil, root regime, hydrologic history and declared service—and where must the model abstain?"

## 2. Evidence that motivates the benchmark

Wen et al. (2026) studied compacted unsaturated landfill-cover soil with 0%, 5% and 10% biochar, vetiver root-density conditions, and three wetting–drying cycles. The abstract reports a non-monotonic dose response: 5% produced larger reductions in reported water/gas permeability than 10%, with root × amendment effects. This is an engineered barrier context, not an agricultural performance target. Full methods, units, specimen design, assignment, and uncertainty must be audited before numerical reproduction. https://doi.org/10.1016/j.jrmge.2026.06.041

Jiang et al. (2025) synthesized field experiments spanning 1–10 years and found time-, feedstock-, dose- and texture-dependent hydraulic effects. Its observation count must not be mistaken for the count of independent randomized treatment units. https://doi.org/10.1016/j.geoderma.2025.117318

These papers are not interchangeable and must not be pooled without a source-level design and endpoint reconciliation.

## 3. Connection to existing work

This benchmark connects without replacing:
- #5528: hydraulic retention, hysteresis, conductivity and transient flux;
- #5535: treatment contrasts and causal assignment;
- #5536: known-truth preferential-flow and identifiability cases;
- #5539 / #5545: roots and dynamic pore morphology;
- #5580: root/pore connectivity mediation;
- #5586: function-conditioned hydraulic service envelopes.

It asks whether those mechanisms change the *shape* of response, and whether the experimental design supports that conclusion.

## 4. Experimental-unit contract

Every observation must carry a true treatment-assignment unit ID. Sensor readings, cores, image tiles, tracer samples, events and time points are nested observations unless the design randomized them independently.

The response-surface auditor:
1. requires study, site, soil, batch, treatment, dose, root regime, wet–dry cycle count, block, forcing, endpoint, time point and allocation metadata;
2. rejects duplicate observation IDs, non-finite values, negative doses, placeholder identifiers and assignment-unit drift;
3. aggregates repeated measurements within assignment unit and a fully declared response cell before computing dose summaries;
4. reports observation counts separately from independent assignment-unit counts;
5. classifies a reversal only with at least three dose levels and the requested minimum number of independent units at every level;
6. computes descriptive root-regime dose-slope contrasts at matched cycle counts, and cycle-history contrasts at matched root regimes, using shared supported doses;
7. reports common block support and never converts descriptive summaries into causal certificates.

The default software support floor is not a power calculation. Confirmatory sample sizes must follow simulation or precision analysis using endpoint variance, blocks, clustering, attrition and the smallest decision-relevant effect.

## 5. Factorial design capable of identifying the mechanism

Where feasible, include untreated/zero-dose control, amendment-only dose series, root-only comparator, amendment × root factorial, and wet–dry history as a randomized or explicitly assigned conditioning factor. Replicate soil-source blocks and include independent material batches.

Dose levels should bracket plausible curvature without borrowing rates from unrelated materials or soil domains. Three levels enable a basic direction-reversal check; more levels or a designed response-surface plan are needed to separate plateau, threshold, U-shape, inverted-U, declining response and multiple turning points.

For expensive experiments, use a preregistered fractional-factorial or optimal-design plan. Preserve estimability of the primary dose × root and dose × cycle interactions. Do not alias dose with batch, operator, season, site, soil source or instrument. Freeze randomization maps, assignment-unit rosters and blocked analysis before collection.

Observed root biomass or pore connectivity measured after treatment is a mediator, not an automatically randomized factor. Root-regime assignment can identify an assigned root-treatment contrast; post-treatment mediation still requires temporal ordering, assumptions and sensitivity analysis.

## 6. Comparator bake-off

All models must receive the same permitted variables, calibration data, forcing, measurement support and frozen external partitions.

| Model family | Role |
|---|---|
| Linear dose + prespecified interactions | Transparent baseline |
| Quadratic / spline / generalized additive model | Curvature baseline |
| Hierarchical dose-response model | Partial pooling across soil, site, study and batch |
| Monotone-constrained learner | Negative control where monotonicity is imposed |
| Flexible non-monotonic learner | Candidate response-surface learner |
| HYDRUS / Richards / DualPerm | Physical hydraulic and solute-transport comparator where conditions support it |
| MACRO | Dual-permeability comparator for suitable structured-soil cases |
| USDA RZWQM2 | Cropping-system, soil-water, nutrient and macropore comparator |
| Symtropy topology-informed hybrid | Candidate with explicit pore/root/history state |
| Symthaea adaptive measurement policy | Candidate for measurement value-of-information under equal budget |

A complex model wins only if it improves external predictions and identifiability without violating conservation or service constraints. Do not claim superiority because a candidate has more parameters or receives more information.

## 7. Proposed qualification targets (not achieved)

- Zero pseudo-replication: repeated readings never increase the treatment replicate count.
- At least 0.90 sensitivity on high-signal synthetic reversal cases, with zero false reversal flags on constant and truly monotone negative controls.
- 100% abstention below preregistered dose support or independent-unit requirements.
- Recover dose × root and dose × wet–dry interaction direction on identifiable known-truth factorial designs; flag aliased designs.
- Nominal 90% predictive intervals achieve 85–95% empirical coverage on independent study/site/batch groups, with width and proper score reported.
- Target at least 10% lower primary physical-response error than the strongest fair comparator on held-out external groups, with group-aware uncertainty excluding zero improvement and no material worst-domain regression.
- Target at least 20% lower P90/P95 breakthrough-tail error without material median or mass-recovery degradation, where tracer support permits.
- Report service-profile constraint violations and Pareto feasibility; never compress water, gas, nutrient, root and crop response into one universal score.

Synthetic gates qualify software behaviour only. They do not validate actual soils, biochar effects, field recommendations or causal mechanisms.

## 8. Industry comparison and the real frontier

HYDRUS DualPerm already models mobile matrix and macropore regions, dual-domain water flow and solute transport, including pressure-driven water exchange and advective/diffusive solute exchange. MACRO is a process-oriented one-dimensional dual-permeability model for water and reactive-solute movement in macroporous soils. USDA RZWQM2 covers soil–plant management, drainage, preferential/macropore flow, crops and nutrients.

The frontier is not to reimplement established equations and call that a paradigm shift. It is to test whether additional observed pore topology, root state, dynamic wet–dry history and uncertainty-aware measurement selection improve external predictions of measured hydraulic and tracer outcomes. Every added mechanism must earn its complexity through identifiability, conservation, calibrated uncertainty and held-out domain performance.

## 9. New hypotheses

1. **Topology-conditioned curvature:** turning points transfer better when conditioned on measured pore topology and root architecture than on porosity and texture alone.
2. **History-dependent reversals:** apparent optima shift with wet–dry cycle count and antecedent moisture, making static dose coefficients misspecified.
3. **Mechanism substitution:** similar service outcomes can emerge from different combinations of matrix retention, macropore connectivity and root occupancy; models must quantify non-identifiability rather than force one explanation.
4. **Batch-sensitive apparent optima:** some apparent optima collapse under held-out material-batch evaluation.
5. **Adaptive experiment advantage:** value-of-information sampling near uncertain turning points and interactions outperforms one-factor-at-a-time and random sampling at equal cost.

## 10. Execution sequence

1. Validate the auditor with synthetic reversal, monotone, flat, batch-confounded and pseudo-replicated controls.
2. Reconcile dose, root, cycle, block, batch and assignment structure in Wen et al. before quantitative reproduction.
3. Crosswalk study and independent-unit structure in Jiang et al.; do not equate pooled observation counts with independent experiments.
4. Generate known-truth scenarios with controlled process truth, measurement noise, missingness, batch drift and site shift.
5. Compare transparent linear, quadratic, spline and hierarchical models before the topology-informed Symtropy candidate.
6. Freeze study/site/batch dependency groups before fitting; report external metrics and calibrated abstention.
7. Connect prediction outputs to service-profile constraints only after independently approved thresholds and methods exist.

## 11. Governance and boundaries

Research and audit only. This specification does not authorize a material recipe, application rate, crop treatment, field trial, commercial product, safety approval, carbon-permanence claim or South African transfer claim. Archaeological Terra Preta is not treated as a formula. Indigenous knowledge, land access, benefit sharing and local governance remain independent requirements.
