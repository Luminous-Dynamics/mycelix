# Function-Conditioned Hydraulic Service Envelope Benchmark v0.1

**Status:** research specification. No material, dose, treatment, or design recommendation is authorized. The cited studies are separate application domains and are not a pooled efficacy dataset. Proposed targets below are not achieved results.

## 1. Research shift: stop asking whether permeability is simply better

A soil hydraulic change has no universal positive direction.

- In a cropped root zone, infiltration, root-accessible water, aeration, drainage, and nutrient retention must coexist. Excessively low conductivity can restrict infiltration or drainage; excessively high conductivity can increase deep water and nutrient loss.
- In an engineered landfill cover, low water and gas permeability may be a primary design objective.
- In a vegetated slope or embankment, crack suppression, infiltration, pore-pressure response, deformation and mechanical stability may dominate.
- In growing media, water storage, air-filled porosity, rewetting behaviour, repellency and drainage need to be balanced.

The scientific target is therefore not a universal scalar "soil quality" or a universal optimal amendment. It is a **function-conditioned hydraulic service envelope**: the range of material × soil × root × moisture-history × forcing states that meets a declared set of domain-specific constraints, with uncertainty and failure probability reported.

This is distinct from:
- #5528, which characterizes hydraulic retention paths, conductivity and transient flux;
- #5539 / #5545, which characterize dynamic morphology and roots;
- #5580, which examines causal mediation through root / pore connectivity;
- #5525, which defines whole-system multi-objective soil functions.

This benchmark is the operational bridge from those measured mechanisms to **context-specific, non-compensable engineering and agronomic performance requirements**. It must not rank unlike application domains on one aggregate score.

## 2. Recent evidence that motivates the distinction

### 2.1 Biochar × root effects in an engineered barrier context

Wen et al. (2026), *Effects of biochar amendment and root density on water and gas permeability of unsaturated soils under wetting and drying cycles*, DOI 10.1016/j.jrmge.2026.06.041, was published online 11 September 2026 in the *Journal of Rock Mechanics and Geotechnical Engineering*. It examines compacted unsaturated soils for landfill-cover performance, with 0%, 5% and 10% biochar, vetiver root densities of 0, 3 and 6 clumps, and three wetting–drying cycles.

The abstract reports that fine biochar (<0.5 mm) reduced gas permeability by 43.27–82% and hydraulic conductivity by 96.61–99.7%; roots reduced gas permeability by 26.81–65.7% and hydraulic conductivity by 1.22–46.74%; reported root–biochar combinations reduced both further. The response was non-monotonic with biochar percentage: 5% produced greater reductions than 10% for both reported permeability measures. Wetting–drying may increase permeability in bare soil via macropore formation, while root growth occupied macropores in root-permeated soil.

This is a useful evidence case for interaction and non-monotonicity. It is an engineered landfill-cover context; reduced water/gas permeability is not automatically a desirable agricultural outcome. The abstract's reported units, compaction, specimen basis, exact parameter uncertainty, and full statistical design must be audited before using the numerical effects as reproducible targets.

Article: https://doi.org/10.1016/j.jrmge.2026.06.041

### 2.2 Biochar and desiccation cracking in clayey-sand slopes

The 2026 *Soils and Foundations* paper, *Influence of biochar addition to clayey sand on desiccation cracks formation and water infiltration under wetting and drying*, DOI 10.1016/j.sandf.2026.101806, examines compacted clayey-sand columns with rice-husk and wood-chip biochar at 0%, 10% and 30% by bulk volume under successive wetting–drying cycles.

The abstract reports that wood-chip biochar reduced surface cracking relative to the control; however, wet–dry cycles increased effective saturated conductivity and the van Genuchten alpha parameter across mixtures as cracks and connected porosity developed. A 10% volume fraction generally delayed and reduced pore-water-pressure rise relative to 0% or 30% in this particular slope/barrier experiment.

This is a separate engineering application, not an agricultural rate recommendation. It reinforces that intact-core conductivity, crack-mediated effective conductivity, and pressure transmission can move in different directions.

Article: https://doi.org/10.1016/j.sandf.2026.101806

### 2.3 Agricultural field evidence has a different time profile

Jiang et al. (2025), *Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments*, DOI 10.1016/j.geoderma.2025.117318, synthesizes 61 field studies. It reports that effects on porosity, field capacity and available water capacity reached their maximum around the second year and then declined, with positive effects remaining after year three in the included data. Feedstock and soil texture modified the reported responses.

This supports a time- and domain-dependent evaluation, not a universal effect or a recommendation to reapply at a particular interval. The study's underlying data are reported as available on request.

Article: https://doi.org/10.1016/j.geoderma.2025.117318

### 2.4 Growing-media evidence is a separate domain

Shahmansouri et al. (2026), *Stability of physical and hydraulic characteristics of biochar- and rockwool-based growing media under wetting and drying processes*, DOI 10.1016/j.jhydrol.2026.136082, reports that wetting/drying changed hydraulic properties of growing media, increased micropores, saturated water content, available water and air content, and that increased tortuosity and micropores decreased hydraulic conductivity; the abstract also reports increased water repellency.

Growing media are not interchangeable with field mineral soils. This is a separate test domain for substrate design, not a training substitute for agricultural field effects.

Article: https://doi.org/10.1016/j.jhydrol.2026.136082

## 3. Performance ontology: define the use before scoring

Every benchmark case must declare one service_profile_id, its intended use, the primary service, the constrained services, the relevant adverse outcomes, the measurement support, and the independent reviewer who approved the thresholds.

Initial profiles:

| Profile | Primary service | Guardrails / co-services |
|---|---|---|
| cropped_root_zone | Reliable plant-available water and root-zone access | Infiltration, aeration, drainage, runoff, nutrient export, root function, crop response |
| dryland_water_resilience | Water capture and storage across drought / intense-rain sequences | Deep drainage, nutrient loss, waterlogging, erosion, plant stress, recovery |
| engineered_low_permeability_cover | Limit water percolation and gas transport | Crack formation, slope stability, root intrusion, settlement, climate-cycle durability |
| vegetated_slope | Limit hazardous crack / pore-pressure response | Surface runoff, erosion, infiltration, drainage, deformation, vegetation persistence |
| horticultural_substrate | Root-zone water/air balance and reliable rewetting | Water repellency, drainage, salt accumulation, nutrient availability, substrate collapse |

Do not transfer thresholds between profiles without a separately justified engineering or agronomic basis. No profile is enabled until its measurable service thresholds, unacceptable failure modes, boundary conditions and evidence owner are preregistered.

## 4. State-conditioned response surface

The benchmark should estimate a response vector as a function of:

Y = f(soil, material_batch, root_state, compaction, suction, wetting_or_drying_branch, reversal_history, cycle_number, event_forcing, temperature, salinity)

Keep the independent factors and their interactions explicit. A treatment may help only within a moisture or forcing interval; a mean treatment effect can hide this.

### Required physical responses

- Water retention on main drying, main wetting and supported scanning branches.
- Hydraulic conductivity Kw(h or theta) with method, support and uncertainty.
- Gas permeability Ka(h or air-filled porosity) with method and uncertainty.
- Infiltration, runoff, drainage, water storage and water-balance closure under declared boundary conditions.
- Air-filled porosity and, where measured, oxygen diffusion / redox state.
- Crack intensity, crack width/depth/connectivity and cycle-dependent crack healing/reopening.
- Pore morphology and connectivity with image resolution, segmentation and 2D/3D limitations.
- Matric suction / pressure-head response and its timing under repeated events.
- Solute / tracer breakthrough and recovered mass; nutrient export only with measured, synchronized flow or volume.
- Root-zone service where plants are present: root penetration/distribution, plant water uptake, stress and biomass/yield endpoints appropriate to the declared use.

A retention curve does not establish conductivity; an intact-core conductivity does not capture crack bypass unless cracks are included in the support; gas permeability is not a proxy for oxygen delivery without moisture/geometry context; concentration alone is not exported mass.

## 5. Non-monotonicity and factorial interaction challenge

Do not assume that greater biochar fraction, root density, porosity, pore connectivity, total carbon or water retention monotonically improves a service.

The minimum retrospective / synthetic challenge should include:
- monotonic beneficial response;
- monotonic harmful response;
- inverted-U and U-shaped dose response;
- threshold / saturation response;
- treatment × root interaction;
- treatment × moisture-state interaction;
- treatment × cycle-history interaction;
- root × cycle-history interaction;
- texture × treatment interaction;
- a treatment that improves one service but violates another service's constraint;
- identical average effect but different failure probability in the worst 10% of forcing sequences.

A fractional factorial or response-surface design may be used only where its estimands are identifiable and the independent unit count supports the interactions. Do not infer a high-order interaction from many repeated sensor readings within a few specimens.

For literature-extracted data, distinguish randomized factorial comparisons from observational subgroup patterns. The Wen et al. landfill-cover study motivates an interaction audit; its abstract does not by itself establish independent causal effects of every combination.

## 6. Comparator matrix

All eligible comparators receive the same soil / material characterization, initial moisture state, forcing, geometry, cycle history and held-out groups.

| Comparator | Appropriate comparison |
|---|---|
| Untreated soil / domain-native reference | Baseline service performance |
| Root-only | Root effect without amendment |
| Material-only | Amendment effect without roots |
| Root + material | Interaction against both individual controls |
| Conventional organic / mineral comparator | Whether effects are specific to the material or general amendment response |
| Measured response interpolation | Non-model reference within observed support |
| van Genuchten–Mualem / Brooks–Corey | Hydraulic constitutive baseline in its valid domain |
| Hysteretic / scanning-curve model | Path-dependent hydraulic baseline |
| Richards matrix-flow solver | Single-domain transient baseline |
| MACRO / HYDRUS DualPerm / RZWQM2 | Dual-domain / agricultural process baseline only where the configuration matches the endpoint |
| State-space / hybrid physical-ML model | Test whether history, roots and pore state improve external prediction |

A model is not declared superior because it reduces one error metric while violating a service constraint or degrading another required endpoint. Do not fit one combined score across landfill, slope, growing-media and cropped-soil contexts.

## 7. Validation design

### A. Reproduction

Reproduce each source study independently with its own materials, compaction, moisture path, root setup, endpoints and cycle protocol. Record all discrepancies from the paper and retain failed fits / excluded samples.

### B. Cross-context generalization

Hold out entire studies, sites, soil sources, material batches and laboratories where IDs support it. Keep engineered cover, slope, growing-media and cropped-root-zone tasks separate. Do not claim that one application-domain result validates another.

### C. Prospective controlled experiment

If separately authorized, randomize at the core / column / plot unit. A minimum mechanism design includes untreated, root-only, material-only and root-plus-material arms where the scientific question requires interaction identification. Use source-soil and material-batch blocks; measure baseline and follow-up structure; record actual compaction and water state; and use controlled, calibrated wetting/drying trajectories.

Repeated cycles, sensors, depths, pressure steps, image tiles and technical replicates are nested within the randomized unit. They do not increase treatment n. Natural storm events are temporal blocks, not independent plot replicates.

### D. Field transfer

Require independent sites and seasons, measured local forcing, relevant land-use profile, predeclared service thresholds, and monitoring of all guardrail outcomes. South African transfer requires locally measured soils and forcing; NEON and non-African case studies are not local validation.

## 8. Scoring: Pareto feasibility, not a universal leaderboard

For every service profile report:

1. Probability of satisfying each required service threshold under its declared distribution of moisture histories and event sequences.
2. Joint probability of satisfying all non-compensable constraints.
3. Pareto frontier across physical, biological, environmental and mechanical outcomes.
4. Worst-domain and lower-tail performance, including 5th / 10th percentile forcing-sequence outcomes where data support them.
5. Uncertainty calibration, interval width and proper predictive score.
6. Failure and abstention rate, missing measurement coverage and OOD status.
7. Lifecycle cost / emissions only with measured or documented system boundaries.

A profile qualifies only if every hard constraint is met with its preregistered confidence requirement. A favorable available-water result cannot compensate for unacceptable nutrient export, gas transport, cracking or crop failure. Conversely, reduced permeability is not called a benefit in an agricultural profile without evidence that infiltration, aeration, drainage and root access remain acceptable.

### Proposed benchmark gates (targets, not results)

- 100% of scored observations are bound to a declared service profile, measurement method, state, branch / cycle history, soil and material lineage.
- Zero silent conversion of measurement direction into "benefit"; all metrics have a profile-specific desired range or constraint.
- Nominal 90% predictive intervals achieve 85–95% coverage on independent held-out groups, with sharpness and proper score reported.
- At least 10% lower predeclared primary physical-response error than the strongest valid comparator on a frozen external group, with group-aware uncertainty excluding zero improvement and no hard-constraint regression.
- For threshold / failure prediction, report calibrated probability of violating each service limit; target Brier skill above a preregistered baseline without degrading worst-domain calibration.
- Detect non-monotonic response families and treatment × root / moisture / history interactions in known-truth tests; return an uncertainty set or abstain when the design cannot identify them.
- Show a Pareto-feasibility improvement over the domain-native comparator, not merely a higher scalar score.
- No causal or performance claim crosses application domains without a separate validation set and qualified review.

The 10% improvement and interval-coverage goals are research targets, not universal minimum important effects. Domain thresholds must be supplied by qualified agronomic / engineering reviewers before confirmatory scoring.

## 9. Adaptive experiment selection

Symthaea should choose the next observation that most reduces uncertainty about **whether a candidate lies inside the declared service envelope**.

Compare:
- fixed expert schedule;
- random sampling;
- one-factor-at-a-time design;
- factorial / response-surface design;
- active learning / expected value of sample information.

Score decision-relevant uncertainty reduction per unit cost and elapsed time. Reward discovery of constraint violations, null effects, interaction reversals and out-of-domain cases—not only identification of beneficial candidates. Keep an exploration budget for uncertain regions and avoid selecting only where the current model predicts success.

## 10. Execution plan and project ownership

1. Add the Wen et al. (2026) landfill-cover study and the clayey-sand slope paper to the source register as separate application-domain sources; audit their full methods, supporting data, experimental units, material basis and uncertainties before extracting numerical observations.
2. Add Jiang et al. (2025) field meta-analysis as a time-dependent agricultural context source; request / audit study-level data if legally available.
3. Create service-profile schema and test fixtures for opposing metric directions (e.g. lower Kw may satisfy a cover constraint but violate a crop-root-zone infiltration constraint).
4. Implement a response-surface / non-monotonicity audit that preserves factorial treatment assignment and refuses unsupported interactions.
5. Link wetting/drying branch, conductivity and transient flux to #5528; cycle-dependent morphology and roots to #5539/#5545; causal effects to #5535/#5580; long-term trajectories to #5527; multiobjective system outputs to #5525.
6. Publish profile-specific, externally validated Pareto-feasibility reports before any material / design recommendation.

- **Mycelix:** service-profile authority, evidence lineage, source and batch custody, preregistered thresholds, treatment allocation and frozen splits.
- **Symtropy:** conservative water / gas / solute and mechanical response modelling, hysteresis, crack-sensitive effective conductance and physical constraints.
- **Symthaea:** competing response hypotheses, non-monotonicity detection, uncertainty / abstention and value-of-information experimental selection.
- **Independent domain reviewers:** specify application limits, audit comparator parity, assess causal identification, and evaluate safety / legal / ecological requirements.

## 11. Boundaries

This work does not recommend biochar percentage, root density, application rates, material recipes, landfill design parameters, slope-stability parameters, crop interventions or any physical implementation. The studies cited use different soils, compaction, material properties, roots, cycle protocols and intended functions. Their numerical outcomes must not be pooled as if they were one comparable experiment. No result establishes Terra Preta efficacy, archaeological reconstruction, South African transfer, product safety or regulatory compliance.

## References

- Wen et al. (2026), *Effects of biochar amendment and root density on water and gas permeability of unsaturated soils under wetting and drying cycles*, DOI 10.1016/j.jrmge.2026.06.041.
- *Influence of biochar addition to clayey sand on desiccation cracks formation and water infiltration under wetting and drying* (2026), DOI 10.1016/j.sandf.2026.101806.
- Jiang, Z., Huang, S. and Meng, Z. (2025), *Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments*, DOI 10.1016/j.geoderma.2025.117318.
- Shahmansouri et al. (2026), *Stability of physical and hydraulic characteristics of biochar- and rockwool-based growing media under wetting and drying processes*, DOI 10.1016/j.jhydrol.2026.136082.
- Pires et al. (2020), *X-ray microtomography analysis of soil pore structure dynamics under wetting and drying cycles*, DOI 10.1016/j.geoderma.2019.114103.
