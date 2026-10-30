# Coupled Hydrology–Nutrient Transport Benchmark v0.1

**Status:** Research and measurement benchmark only. This document is not a formulation, feedstock selection, application-rate recommendation, field-trial authorization, product-safety decision, or claim of site-specific efficacy.

## 1. The missing objective: predict nutrient mass movement, not only concentration or equilibrium retention

Nutrient fate in soil is a coupled water–structure–chemistry–biology problem. Water inputs, hydraulic history, matrix and preferential pathways, solute speciation, sorption/desorption, mineralization/immobilization, plant uptake and export jointly determine where nutrients move.

Three common shortcuts can produce false conclusions:

1. **Concentration is not mass flux.** A lower leachate concentration does not prove less nutrient export when drainage volume changes. For dissolved species, exported mass requires concentration integrated against measured water volume and collection coverage.
2. **Low lower-boundary loss is not necessarily retention in the root zone.** Nutrient may move into deeper soil, lateral flow, runoff, sediment or an unmonitored compartment. Profile redistribution and boundary export are separate endpoints.
3. **Equilibrium retention is not transient transport.** A static water-retention fit does not by itself validate infiltration, event-scale drainage, preferential flow, hysteretic scanning paths or solute breakthrough.

The benchmark evaluates the water balance, nutrient balance, profile redistribution, and plant/ecosystem service together. A system does not pass by improving one endpoint while a relevant loss or displacement is unmeasured.

## 2. Current evidence motivating this benchmark

- Nimmo et al. (2025) reviewed 77 studies that identify preferential flow from soil-moisture time series. Methods include non-sequential responses between sensors at different depths and rapid wetting-front velocity criteria. The review describes false-positive/false-negative risks, subjective choices and the value of corroborating tracer or geophysical evidence. It notes that event-responsive measurements generally need more than one observation per hour to resolve transient behaviour. These are methodological findings, not a universal sensor threshold for every soil.
- Yan et al. (2026) review flow-rate-dependent, dynamic nonequilibrium in soil-water retention: suction and water-content changes may not remain in instantaneous equilibrium. The review highlights unresolved parameter scaling, path-dependent hysteresis and the absence of a unified theory coupling dynamic capillarity with soil deformation. Dynamic-capillarity models should therefore be treated as testable higher-fidelity candidates, not as an automatically superior default.
- Zhang et al. (2026) report a two-year alternate-wetting-and-drying (AWD) paddy field experiment in a Northeast China alluvial plain. AWD reduced P leaching but also promoted available-P migration into 100–200 cm layers; the study's AWD-plus-biochar treatment reduced P leaching and the reported deep migration. This is an important example of why profile redistribution and leaching must be evaluated together. The study's site, crop, hydrology and treatment are not a South African recommendation or a transferable application rate.
- Rahman et al. (2026) report saturated-column experiments using poultry-litter solution and specific pristine, iron-modified and magnesium-modified pinewood biochars. In that experimental system, the magnesium-modified material retained both nitrate-N and phosphate more effectively, whereas iron-modified material primarily improved nitrate-N retention; the authors note that longer-term leaching and plant-availability work is still needed. These material-specific column results demonstrate that nutrient pathways can differ by material and chemical species; they do not establish a field recipe, safe process or sustained crop benefit.
- Šimůnek et al. (2003) compare non-equilibrium and preferential-flow models ranging from single-domain approximations to dual-porosity and dual-permeability formulations. They emphasize that complex models require sufficiently comprehensive field data to identify parameters and distinguish among competing conceptual models.

## 3. Coupled state and flux model

The conceptual state graph is:

`forcing → hydraulic state and history → matrix/preferential flow partition
→ water and solute transport → reactions and biological transfers
→ plant uptake, profile stocks and boundary exports`

Relevant moderators include soil texture/mineralogy, aggregation, bulk density, pore architecture, organic matter, amendment batch and aging, root distribution, crop stage, temperature, redox state, rainfall intensity, irrigation regime and antecedent moisture.

The model must expose these state variables where measured or independently identifiable. Missing state is not silently inferred as fact.

### 3.1 Water balance

For a declared soil–plant control volume over event or period (t_0 ldots t_1):

`R_w = P + I + R_{in} + L_{in} + C_{up}
       - ET - Q_{runoff} - D_{lower} - L_{out} - ΔS_w`

where precipitation (P), irrigation (I), runon (R_{in}), lateral inflow (L_{in}), capillary rise (C_{up}), evapotranspiration (ET), runoff, lower-boundary drainage (D_{lower}), lateral outflow (L_{out}) and soil-water storage change (ΔS_w) must be defined consistently with the boundary.

Do not count infiltration as an external input if precipitation/irrigation is already the surface-boundary input. Do not count root uptake as a loss from a soil–plant control volume when the plant pool is inside the boundary. Do not use potential ET as measured actual ET without an explicit, validated estimate label.

Report event, seasonal and cumulative totals separately. The residual is a conservation diagnostic; it is not automatically a model score or evidence of correct partitioning.

### 3.2 Nutrient mass balance

For each element and chemically meaningful species, use the companion boundary-budget equation:

`R_n = N_{external,in} - N_{external,out} - ΔS_n`

Keep the following separate:
- dissolved and particulate transport;
- nitrate, ammonium, dissolved organic N and gaseous N species;
- total, dissolved reactive, dissolved organic and particulate P, plus extraction-defined pools;
- solution, exchangeable, non-exchangeable/mineral and total K;
- soil, standing plant biomass and retained residue stocks;
- internal transformations and transfers versus external boundary flows.

For dissolved species, compute event mass as `M = ∫ C(t) Q(t) dt` (or an explicitly justified flow-weighted equivalent), where concentration (C) and discharge (Q) are aligned in time and units. A grab concentration multiplied by a seasonal-average or assumed water volume is not a measured flux.

For particulate nutrient export, use measured sediment mass and its elemental concentration; water concentration alone is insufficient. For gas exports, integrate a measured species-specific flux over valid temporal coverage and convert to elemental basis with a recorded transformation. Preserve unmeasured pathways and below-detection observations as distinct states.

## 4. Event-resolved measurement contract

A confirmatory event record should link the following evidence to one event, control volume and experimental unit:

**Forcing and water input**
- rainfall depth and sub-event intensity/time series;
- irrigation volume, duration and delivery uniformity;
- antecedent moisture and time since previous wetting;
- temperature and relevant management/water-regime context.

**Hydraulic response**
- volumetric water content and/or matric pressure at multiple depths;
- sampling frequency sufficient to resolve the event (preferably more than one observation per hour where rapid preferential flow is the target);
- spatial replication across plausible structural heterogeneity, rather than one vertical probe string alone;
- infiltration, surface ponding, runoff, drainage, water-table response and water-storage change where measurable;
- branch/history context: wetting, drying, scanning path, reversal point and antecedent state;
- sensor calibration, installation, response time, drift, method uncertainty and missing intervals.

**Pathway corroboration**
- conservative water tracer, dye, stable isotope or solute tracer where scientifically and locally appropriate;
- suction samplers, lysimeters, drainage collectors or other depth-specific flux evidence;
- independent tracer/geophysical evidence to test sensor-derived preferential-flow classifications;
- explicit detection limits, recovery fraction, collection efficiency and sampling disturbance.

**Nutrient response**
- event-aligned discharge volume and flow-weighted or time-resolved nutrient concentrations;
- species/analyte identity, dissolved/particulate status, filtration and preservation method;
- event mass export and cumulative mass export, not concentration alone;
- profile samples at predeclared depths and dates to distinguish retention from deeper redistribution;
- plant uptake/standing biomass/harvest export, retained residue and relevant gaseous fluxes;
- extraction method and availability indicator separately from total-stock accounting.

No single preferential-flow signature is sufficient as universal ground truth. Classify detections against independent tracer or other validated reference evidence; quantify false positives, false negatives, missed events and uncertainty. A non-sequential sensor response may be a useful indicator, not a unique proof of a specific pore mechanism.

## 5. Model hierarchy: complexity must earn its parameters

Run all models on matched inputs, boundaries, calibration data and frozen held-out groups.

| Tier | Model class | What it can test | Required evidence / limitation |
|---|---|---|---|
| B0 | Measured event accounting and empirical interpolation within observed support | Conservation, event response and data integrity | No extrapolation; missing volumes or collection windows block flux qualification |
| B1 | Single-domain Richards flow + advection–dispersion solute transport (ADE), with a declared conventional retention/conductivity parameterization | Matrix-flow baseline and transport under stated assumptions | Independently measured or justified hydraulic parameters; measured initial/boundary conditions; species-specific transport parameters |
| B2 | Non-equilibrium / mobile–immobile or dual-porosity model | Delayed exchange, mobile bypass and tailing | Independent evidence for mobile/immobile domains or exchange times; do not fit unidentifiable domains only to improve aggregate error |
| B3 | Dual-permeability / matrix–macropore exchange model | Separate matrix and fast-path transport with inter-domain exchange | Multi-depth transient response plus tracer/breakthrough data adequate to identify partition and exchange parameters |
| B4 | Hysteretic scanning and dynamic nonequilibrium capillarity coupled to transport | Hydraulic-history and flow-rate effects during transient wetting/drainage | Main wetting/drying and scanning observations, time-resolved pressure/water content, rate variation and independent transient validation |

A higher tier does not win merely because it has more parameters. Require parameter identifiability, plausible bounds, sensitivity, convergence, predictive calibration and out-of-domain abstention. If available observations cannot distinguish a dual-domain model from a single-domain model, report the ambiguity and keep the simpler model as the operational baseline.

Compare matched model implementations to a reference solver (for example, HYDRUS-1D) only when governing equations, constitutive functions, geometry, units, initial conditions, boundaries, numerical tolerances and parameter values are equivalent. Solver agreement is not independent physical validation.

## 6. Required benchmark tasks

### T1 — Water-volume and flux integrity
Recompute event water volumes and cumulative water balance from measured records. Detect duplicate events, time misalignment, impossible flow values, unit errors, missing collections and inconsistent integration windows. Preserve unresolved water-balance residuals.

### T2 — Preferential-flow detection
Compare sensor-derived detections with independent tracer/reference evidence. Report sensitivity, specificity, precision, false discovery rate, missed-event rate, event timing error and dependence on threshold choice. Stratify by soil, antecedent moisture, rainfall/irrigation intensity and sensor geometry.

### T3 — Solute breakthrough
Evaluate species-specific breakthrough curves for arrival time, peak concentration, peak timing, tailing, cumulative exported mass and mass recovery. Report concentration error separately from flux and mass error. A model that fits concentration while missing discharge is not a valid export predictor.

### T4 — Depth redistribution
Measure or estimate profile-resolved stocks at predeclared depths and dates. Report the depth distribution of dissolved/available and total pools separately. Deep accumulation is neither automatically loss nor safe retention; it must be interpreted against root depth, aquifer/vadose boundaries and the declared objective.

### T5 — Water–nutrient interaction
Estimate treatment-by-water-regime interactions only where the experiment identifies them. Include event intensity, antecedent moisture and wetting/drying history. Do not transfer a result from saturated columns, flooded paddy systems or a single texture to an unrelated soil/climate domain.

### T6 — Aging and repeated-event robustness
Test first-flush and subsequent events, wetting/drying cycles, compaction/structure changes and later-season responses. Distinguish material aging from changes in rainfall, root development, water management and background nutrient pools.

### T7 — Conservation and uncertainty
For water and each nutrient, publish event and cumulative residuals alongside measurement uncertainty. Propagate concentration and volume uncertainty with covariance where known. If covariance or coverage is missing, mark the closure result unqualified or explicitly approximate; do not label an invented tolerance as validation.

## 7. Experimental contrast and causal design

The study question must state whether it estimates:
- water-regime effect under otherwise matched management;
- amendment/material effect under a matched water regime and nutrient programme;
- nutrient-programme effect with all other components held constant;
- amendment × water-regime interaction;
- an inseparable management package, retained as a package contrast only.

Where relevant, use a factorial or staged design that separates water management, amendment presence and nutrient programme. Randomize at the actual independently assigned plot/pot/mesocosm level; block on site gradients where appropriate. The comparator and co-interventions must be explicit. No amendment rate or formulation is supplied by this benchmark.

Do not infer fertilizer substitution from a package contrast. Do not interpret reduced leaching as improved nutrient efficiency when nutrient input or crop uptake also fell. Preserve null, adverse and contradictory outcomes.

## 8. Evaluation splits, metrics and proposed targets

Freeze connected groups before confirmatory fitting: publication/study, experiment, site, treatment batch, soil/core, season and repeated event series. Keep all observations from one event and its derived concentration/flux summaries in the same split. Random row splits are diagnostic only.

Report:
- water-balance residual and cumulative water-flux error;
- concentration MAE/RMSE/bias by species and event;
- breakthrough arrival/peak/tailing metrics and cumulative mass-export MAE/RMSE/bias;
- depth-profile error and deep-redistribution detection;
- preferential-flow event sensitivity/specificity against independent reference evidence;
- interval coverage, width and proper interval score;
- calibration/abstention, parameter identifiability and fit-failure rates;
- subgroup/domain metrics by texture/mineralogy, structure, water regime, soil depth, climate and measurement laboratory.

**Proposed targets only; not achieved results:**

| Gate | Proposed acceptance target |
|---|---|
| Flux identity | 100% of confirmatory dissolved-flux records have synchronized concentration, measured water volume, area, interval and collection-coverage metadata |
| Preferential-flow truth | No confirmatory PF classification based only on one sensor signature; independent reference evidence or an explicit uncertain/indeterminate status |
| Mass accounting | All event and cumulative residuals reported; no unmeasured or below-detection value silently converted to zero |
| Group integrity | Zero connected study/site/core/batch/event leakage across confirmatory train/validation/test partitions |
| Uncertainty | Nominal 90% predictive intervals target 85–95% empirical coverage on independent held-out groups, with sharpness and interval score reported |
| Model value | At least 10% lower error than the strongest valid baseline on a predeclared flux/trajectory endpoint, with group-aware uncertainty and no material domain regression |
| Complexity | No higher-tier model promoted without identifiable parameters and improvement on an independent transient test |
| Transportability | South African/African site or laboratory context held out as an external domain, not mixed randomly into training |
| Safety and authority | Zero model-generated formulation, rate, trial, physical-action or regulatory approvals |

A scalar leaderboard is not sufficient. Publish a Pareto profile for event prediction, mass conservation, calibration, domain robustness, model complexity and missing-data abstention.

## 9. Falsifiable hypotheses and adversarial cases

- **H1 — concentration/flux decoupling:** concentration-only metrics can rank treatments differently from cumulative mass export when drainage volume differs.
- **H2 — hidden redistribution:** lower shallow or lower-boundary flux can coincide with greater nutrient accumulation at deeper sampled layers.
- **H3 — preferential-path dominance:** a small number of fast events can contribute a disproportionate share of seasonal nutrient export.
- **H4 — hydraulic-history dependence:** retention/conductivity parameters estimated on a single drying branch may underpredict or mis-time transport under wetting/scanning conditions.
- **H5 — model parsimony:** dual-domain and dynamic-capillarity models improve held-out transient predictions only in domains where independent observations identify their extra parameters.
- **H6 — interaction:** water-regime effects and material effects are not generally additive; any interaction claim must be estimated from a design that identifies it.

Required negative/adversarial cases include:
- same concentration but different drainage volumes;
- lower concentration with higher cumulative exported mass;
- reduced leachate mass but increased deep-profile accumulation;
- preferential flow detected by one sensor with no tracer corroboration;
- tracer arrival with no resolvable moisture response;
- missed first-flush event and irregular sampling;
- concentration × volume timestamps misaligned;
- missing or low-recovery lysimeter collection;
- water-balance closure achieved only by silently imputing ET or drainage;
- calibration fit improves while held-out event timing or cumulative mass worsens;
- dual-domain parameters are non-identifiable or collapse to single-domain limits;
- below-detection, censored and measured-zero concentrations are mixed;
- N/P species are silently aggregated or converted on inconsistent mass bases.

## 10. Roles in the Luminous stack

- **Mycelix:** immutable identities and provenance for event, site, plot, sample, sensor, tracer, nutrient analyte, treatment, batch, boundary and review.
- **Symtropy:** measured-parameter hydraulic and transport kernels; conservation checks; explicit single/dual-domain model tiers; reference-solver comparisons with matched equations and boundaries.
- **Symthaea:** competing hypothesis management, missing-measurement detection, recommendation-only selection of approved measurements and model abstention when parameters or boundaries are not identifiable.
- **Independent reviewers/laboratories:** verify calibration, collection coverage, analytical methods, experimental-unit hierarchy, causal estimands, uncertainty and local permissions.

Hashes and model receipts prove artifact identity and computation provenance. They do not prove field sampling representativeness, chemical speciation, physical flux completeness, agronomic benefit or regulatory approval.

## 11. Event-record schema

The companion machine-readable event contract is
[`tools/coupled-event-audit/event-record.schema.json`](./tools/coupled-event-audit/event-record.schema.json),
with a synthetic fixture and validation instructions in
[`tools/coupled-event-audit/README.md`](./tools/coupled-event-audit/README.md).
It distinguishes measured-qualified integrated dissolved export from
estimated or concentration-only observations, separates particulate export,
preserves sensor-only versus corroborated preferential-flow classifications,
and discloses unmeasured pathways. Structural schema validation does not prove
time synchronization, complete collection or scientifically valid tracer
evidence.

## 12. Work packages and dependencies

1. **Event record:** define immutable event identity, forcing, initial state, sensor time series, water volume, flux sample, analyte and coverage semantics.
2. **Hydrology audit:** verify time alignment, water units, event partitions, water-balance residual and missing-collection disclosure.
3. **Preferential-flow audit:** implement multi-depth signatures with uncertain/indeterminate states and reference-tracer comparison.
4. **Nutrient flux audit:** integrate time-aligned concentration × discharge and particulate export separately; preserve censoring and detection limits.
5. **Transport benchmark:** build synthetic known-truth breakthrough, preferential bypass, exchange and tailing cases.
6. **Model bake-off:** compare B1–B4 only where parameter identifiability is demonstrated; retain B0 measured accounting.
7. **Independent data campaign:** acquire exact publisher/licensed data, reconcile source counts, and add laboratory/field data only with method and custody metadata.
8. **South African context:** establish a held-out local measurement domain with qualified institutional and regulatory review before any field application.

Dependencies: the hysteresis and transient hydraulic contract is tracked in Mycelix #5528 and Symtropy #2861; nutrient stock/flow accounting is tracked in Mycelix #5529; this document defines the coupled benchmark between them.

## References

- Nimmo, J. R. et al. (2025). “Identifying preferential flow from soil moisture time series: Review of methodologies.” *Vadose Zone Journal* 24(2), e70017. https://doi.org/10.1002/vzj2.70017
- Yan, G. et al. (2026). “Advances in dynamic soil water retention behaviour and implications for vadose zone hydrology.” *Capillarity* 19(1), 26–38. https://doi.org/10.46690/capi.2026.04.03
- Zhang, W. et al. (2026). “Biochar-mediated soil phosphorus leaching and distribution behaviors in AWD paddy fields of a Northeast China alluvial plain.” *Agricultural Water Management* 324, 110155. https://doi.org/10.1016/j.agwat.2026.110155
- Rahman, A. et al. (2026). “Pinewood engineered biochar as a solution to nutrient leaching management: Mitigating phosphorus and nitrate loss from poultry litter under saturated porous conditions.” *Journal of Environmental Management* 416, 130641. https://doi.org/10.1016/j.jenvman.2026.130641
- Šimůnek, J., Jarvis, N. J., van Genuchten, M. T. & Gärdenäs, A. (2003). “Review and comparison of models for describing non-equilibrium and preferential flow and transport in the vadose zone.” *Journal of Hydrology* 272, 14–35. https://doi.org/10.1016/S0022-1694(02)00252-4
