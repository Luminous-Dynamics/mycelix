# Paired-Forcing Causal Benchmark v0.1

**Status:** Experimental-design and analysis specification only. This is not a biochar formulation, rate, application instruction, trial authorization, fertilizer recommendation or environmental approval.

## 1. Separate prediction from causation

The NEON preferential-flow benchmark asks whether a model can predict event responses from observed weather and soil-moisture signals. A high predictive score does **not** answer whether a soil amendment changed preferential flow, nutrient loss, water retention or crop performance.

A treatment-effect claim requires an independently assigned treatment, an appropriate comparator, a defensible experimental unit, measured outcomes and a causal estimand. The core question is:

> Under the same measured hydrologic forcing and comparable initial soil state, does the assigned treatment change event-scale water partitioning, depth response or nutrient mass export?

This benchmark makes hydrologic forcing a design variable, not just a nuisance covariate. It aims to identify when an amendment changes the soil's response function across event intensity, duration and antecedent wetness—not simply report one average across unrelated storms.

## 2. Two complementary experimental tiers

### Tier L — controlled soil-core / lysimeter mechanics

Use characterized, independently sampled soil cores or mesocosms, with treatment allocation randomized within soil-source/batch blocks. Apply the same predeclared rainfall/irrigation hyetograph to treatment and comparator units in a block. Include multiple event shapes only where the design has enough independent units to estimate the chosen interactions.

This tier can support stronger matching of forcing and boundary conditions and can measure:
- event infiltration/runoff/drainage and water storage;
- multi-depth water content and pressure response;
- tracer arrival/breakthrough and recovery;
- event-integrated dissolved and particulate nutrient export;
- profile-resolved pre/post-event stocks and soil structure;
- repeated-event changes after wetting/drying and aging.

A core or mesocosm is the experimental unit if treatment was randomized to that core. Multiple sensors, depths, aliquots or storm repetitions within one core are repeated measures—not additional independent treatment replicates.

### Tier F — randomized field blocks

Randomize treatment and comparator to independently assigned plots within spatial blocks. Paired plots are observed during the same natural storm event, but record plot/block rainfall, throughfall (where relevant), irrigation, antecedent soil water, slope and local exposure. Do not assume the same weather station means equal plot-level forcing when canopy, runon, slope or gauge distance can differ.

This tier estimates field-relevant response under natural conditions, with greater residual variation. It requires multiple independent blocks and, where possible, multiple sites/seasons for transportability. A single matched pair or one storm is not a confirmatory efficacy study.

Tier L and Tier F answer related but distinct questions. Do not pool core and field outcomes as if their experimental units, boundary conditions or target populations were interchangeable.

## 3. Primary estimands: effect under matched forcing

Predeclare one primary treatment contrast and one primary endpoint. Suitable examples:

1. **Event hydrologic response:** paired treatment-minus-control difference in a declared event metric, such as infiltration depth, runoff depth, drainage depth or moisture-response onset at a specified profile depth.
2. **Event nutrient export:** paired absolute difference in cumulative elemental mass exported per area and event, in kg element/ha/event, reported by species and pathway.
3. **Profile redistribution:** paired difference in pre/post-event depth-specific stock or tracer recovery, keeping root-zone accumulation distinct from deep-profile movement and boundary loss.
4. **Treatment × hydrologic forcing interaction:** change in the treatment effect across preregistered event descriptors (e.g. rainfall peak intensity, event total, duration, pulse gap, antecedent moisture).

For dissolved export, the primary outcome must be based on measured, synchronized event water volume and concentration integration. For particulate export, use collected dry sediment mass × measured elemental concentration. Concentration-only data remain a separate endpoint and must not be converted into event mass.

A lower concentration, lower shallow moisture response, or lower bottom-boundary export is not automatically beneficial. Interpret alongside drainage volume, depth redistribution, plant uptake, nutrient inputs, crop service and unmeasured pathways.

## 4. How to achieve forcing comparability

### Controlled tier

- Randomize treatment assignment within soil-source/batch and core-geometry blocks.
- Apply identical programmed event hyetographs to every treatment arm in a block, with calibrated delivery and measured application at the specimen surface.
- Randomize event-shape order or use separate cores where carry-over and soil-state history would confound order.
- Standardize and measure antecedent moisture state before each event; do not use nominal irrigation amount as a substitute for measured initial state.
- Record actual rainfall/irrigation intensity time series, not only event totals.
- Keep event recovery, lysimeter collection efficiency and control-volume boundary constant across arms.
- Include pre-treatment baseline characterization and a sham/untreated comparator only where justified by the scientific question.

### Field tier

- Define paired plots within blocks before randomization.
- Treat a storm event as a shared temporal block, not as an independent experimental replicate.
- Measure plot/block-specific rainfall and relevant water input (including throughfall only where canopy interception applies); preserve instrument location and calibration.
- Record antecedent moisture, water table, slope/flow accumulation, crop stage, surface cover, irrigation, runon/runoff and spatial gradients.
- Predefine the event inclusion rule without observing treatment outcomes. Retain missed, partial and low-coverage events with explicit status.
- Record cross-contamination, lateral flow between plots and buffer/edge effects.

Natural rainfall does not guarantee equal exposure. Use measured plot-level forcing and sensitivity analysis; do not repair exposure differences by matching on post-treatment variables.

## 5. Experimental unit, nesting and analysis

The experimental unit is the unit to which treatment was independently assigned: plot, core, pot or mesocosm. Measurement hierarchy is nested:

`site → block → experimental unit → event → depth/sensor → time point/sample`

Analysis must preserve this hierarchy:
- repeated events are repeated observations within the same experimental unit;
- sensor depths and time points are not independent treatment replicates;
- technical replicates and laboratory aliquots do not increase sample size;
- shared rainfall events create common temporal shocks, not extra independent plots;
- repeated use of a core after wetting/drying requires explicit carry-over and time/history terms.

For a paired randomized block design, estimate the prespecified within-block treatment contrast. Suitable primary estimators include a block-adjusted mean difference or a preregistered hierarchical model with treatment and block effects; repeated events may be nested within experimental unit and storm event. Randomization inference must permute treatment labels only at the unit actually randomized, respecting blocks. Bootstrap uncertainty by independent experimental units/sites, not sensor rows or event records.

Do not choose a model because it yields a smaller p-value. Publish the design-based contrast, adjusted estimate, uncertainty interval, model diagnostics and all deviations. Small-cluster uncertainty should be handled with suitable small-sample methods and sensitivity analyses; state when the number of sites/blocks is insufficient for a reliable external-transfer estimate.

## 6. Treatment-effect heterogeneity under storm structure

Predefine a small number of scientifically meaningful moderators:
- antecedent water content / matric potential;
- peak rainfall/irrigation intensity;
- total event depth;
- event duration and inter-pulse gap;
- soil texture/mineralogy, structure and baseline hydraulic condition;
- depth and root-zone position;
- material batch/aging state, if included in the approved design.

Use continuous forcing variables where possible; avoid post-hoc binning chosen to maximize apparent effects. A treatment × event-intensity interaction is interpretable only if treatment allocation and event exposure jointly support it.

A useful response-function representation is:

`Y_{u,e} = f(F_e, S_{u,e}, H_{u,e}) + T_u g(F_e, S_{u,e}, H_{u,e}) + b_{site} + b_{block} + b_u + ε_{u,e}`

where (Y) is an event outcome, (F) is measured forcing, (S) is antecedent soil state, (H) is hydraulic history, (T) is randomized treatment, and (b) terms represent the actual nesting. This is a conceptual estimand model; the final statistical form must respect design and sample size.

Do not call a fitted interaction causal when treatment was not randomized or the forcing moderator is not measured before the event.

## 7. Negative controls, falsification and missingness

Required controls:
- **Pre-treatment placebo:** future treatment assignment must not predict pre-treatment outcomes after respecting randomization/block structure.
- **Expected-null endpoint:** an endpoint with no plausible treatment pathway should not show systematic separation; justify endpoint selection.
- **Sham / handling control:** where handling, transport, sieving or irrigation can itself alter soil structure.
- **Event-order control:** test whether treatment contrast depends on event sequence or antecedent wetting history.
- **Collection recovery:** independent recovery checks for water, tracer, sediment and nutrient-flux collection.

Required adversarial cases:
- identical rainfall total but different intensity profile;
- identical intensity peak but different event duration/total;
- same hyetograph but different antecedent moisture;
- lower concentration paired with larger drainage and larger mass export;
- lower shallow leachate mass but deeper nutrient redistribution;
- incomplete collector recovery in one arm;
- treatment-related changes in runoff that alter the water-volume denominator;
- sensor/time drift that differs by arm;
- repeated events incorrectly treated as independent replicates;
- a missing event selectively omitted from one treatment arm.

Missing events, below-detection observations, no-response events and failed collectors must remain distinct states. Never replace them with zero or silently drop them. Predeclare how incomplete coverage affects the primary estimand; often the event-total nutrient flux should be unqualified rather than imputed.

## 8. Benchmark splits and model use

For any prediction model used within the study:
- split by site, soil source/batch, block and experimental unit according to the deployment claim;
- keep all sensor/time/event descendants from one experimental unit within one split;
- fit normalization, imputation, feature selection, parameter estimation, calibration and threshold selection on training groups only;
- keep randomized treatment assignment and causal analysis independent from prediction-model tuning;
- compare against a design-based estimator, a block-adjusted mean-difference baseline and a hierarchical repeated-measures baseline;
- report prediction accuracy separately from the treatment-effect estimate.

NEON site-held-out PF performance can motivate hypotheses and model architecture. It cannot substitute for randomized amendment-effect evidence.

## 9. Proposed benchmarks and decision gates

Targets below are proposed, not achieved:

| Gate | Proposed target |
|---|---|
| Assignment integrity | 100% of confirmatory units have auditable randomization/block manifest and known assignment unit |
| Forcing parity (controlled tier) | Measured input hyetograph discrepancy between paired arms stays within preregistered delivery uncertainty; deviations reported, not silently corrected |
| Unit integrity | Zero pseudo-replication: unit of analysis and all nested sensor/event observations explicitly linked |
| Flux qualification | 100% of confirmatory event-mass outcomes have synchronized volume/concentration or sediment-mass/concentration and reviewed collection coverage |
| Causal identifiability | 100% primary contrasts have matched nutrient/management programme or clearly stated package estimand |
| Negative controls | Pre-treatment placebo and at least one justified expected-null control analysed and reported |
| Uncertainty | Effect intervals cover the prespecified estimand under a design-appropriate cluster/randomization method; small-cluster limitations disclosed |
| Heterogeneity | Treatment × hydrologic forcing terms preregistered and supported by independent units across forcing range |
| External validity | At least one genuinely independent site/season or laboratory domain; held-out groups never used for tuning |
| Safety/governance | All material, environmental, institutional and regulatory gates separately reviewed; no model-generated trial or rate authorization |

Do not promise a conventional percentage improvement target before estimating the smallest scientifically meaningful difference and variance/cluster structure. Power is determined primarily by independent experimental units, not the number of sensor time points or storm events.

## 10. Integration with the Luminous stack

- **Mycelix:** immutable treatment randomization, soil/batch/block/unit/event lineage, data custody, allocation digest, review and deviation provenance.
- **Symtropy:** replay identical forcing, simulate hydraulic and nutrient mechanisms, test paired event counterfactuals and preserve water/solute mass balance.
- **Symthaea:** manage competing mechanistic hypotheses, identify which measurements most reduce treatment-effect uncertainty and abstain from claims unsupported by the experimental design.
- **Independent field/lab reviewers:** approve the causal estimand, randomization, measurement coverage, safety, permissions and interpretation.

The model may propose a measurement or quantify uncertainty. It cannot select a treatment rate, authorize application or replace independent agronomic and regulatory review.

## References

- Hernán, M. A. & Robins, J. M. (2020). *Causal Inference: What If*. Chapman & Hall/CRC. https://www.hsph.harvard.edu/miguel-hernan/causal-inference-book/
- Li, B. et al. (2026). “Dominant Controls on Preferential Flow and Their Implications for Future Soil Water Fluxes.” *Earth's Future*. https://doi.org/10.1029/2026EF008296
