# Wastewater Service-Chain Completeness and Unobserved-Flow Benchmark v0.1

**Status:** Research protocol proposal; public-summary replay is bounded, system-level extraction and physical validation are pending.  
**Version:** 0.1.0  
**Date:** 2026-11-13  
**Claim ceiling:** This document defines a service-boundary and benchmark method. It does not establish municipal condition, pollution causation, intervention efficacy or operational readiness.

## 1. Research thesis

A wastewater treatment plant is not the wastewater service.

A complete sanitation service includes a connected chain:

`generation → collection → conveyance → treatment → discharge/reuse → receiving environment`

The chain is socio-technical. It also depends on electricity, chemicals, telemetry, laboratories, technicians, maintenance, roads, finance, procurement, land access, environmental permissions and emergency response.

An assessment that measures only treatment works can miss failures in sewer reticulation, pumping, bypasses, unrecorded discharges, household connections and the gap between wastewater generated and wastewater that reaches a treatment works. Plant-level condition remains important, but it is not a complete service-outcome measure.

Graham, Pattinson and Still (2025) identify ineffective sewage reticulation management and wastewater not reaching treatment works as a major issue not adequately emphasised in Green Drop reporting. Their Pietermaritzburg/Darvill case estimates a 19.66 ML/day gap between predicted and measured daily treated volume in 2022 and reports correlation with high river E. coli concentrations. This is an observational result from that case, not proof that the estimated gap caused the observed E. coli concentrations or a nationally generalisable loss rate.

Reference: Graham, P. M., Pattinson, N. B. & Still, D. (2025), "The state of wastewater management in South Africa: data gaps, missing wastewater, and Green Drop reporting", Water SA 51(2), 78-89. DOI: https://doi.org/10.17159/wsa/2025.v51.i2.4152

## 2. First public-source replay

The DWS release published 1 April 2026 reports that the 2025 Green Drop assessment covered 848 municipal wastewater systems for the 2023/24 financial year. It reports 396 critical systems (47%) and 66 good/excellent systems (8%), compared with 334 (39%) critical and 118 (14%) good/excellent systems in the 2022 report. SAnews' same-day account states that the 2022 municipal comparison covered 850 systems for 2020/21 and that the two-system difference reflects decommissioning or merging. Green Drop certification counts were 22 in 2022 and 14 in 2025.

The WRO catalog describes the broader 2022 full audit as 995 wastewater treatment works across municipal, Department of Public Works and private ownership. This 995 all-sector count is not the denominator for the 334/850 municipal critical-system comparison. FSRE records both scopes explicitly and does not merge them. The WRO catalog describes the 2025 report as an 848-municipal-system audit in 144 Water Services Authorities.

Primary source: https://www.gov.za/news/media-statements/water-and-sanitation-crowns-steve-tshwete-local-municipality-2025-green-drop
Secondary source for the 2022 denominator and system-boundary change: https://www.sanews.gov.za/south-africa/majodina-calls-accountability-and-collective-action-protect-water-resources

The executable fixture is `tools/fsre-green-drop-replay/fixtures/ZA_GREEN_DROP_PUBLIC_SUMMARY_2025.psv`. Its Rust validator checks both report vintages' rounded municipal rates: 334/850 and 118/850 in 2022; 396/848 and 66/848 in 2025. It separately records the 995 all-sector 2022 audit scope from the WRO catalog without using it as a municipal denominator. The three sources and report-boundary change are retained. System identity linkage and denominator-consistent longitudinal reassessment remain pending.

The WRO catalog describes the Green Drop collection as published PDF reports and states that system-level tables were not extracted into machine-readable form. As viewed on 13 November 2026, both the dataset page and individual 2025 PDF resource page state "No License Provided"; the resource metadata was updated 11 November 2026. Do not infer permission to download, extract or redistribute the report. Resolve rights with the data custodian before system-level extraction.

Catalogue:
- Dataset: https://wrcwro01.arc.agric.za/en/dataset/green-drop-reports-2009-2025-south-african-municipal-wastewater-management-performance
- 2025 report resource: https://wrcwro01.arc.agric.za/en/dataset/green-drop-reports-2009-2025-south-african-municipal-wastewater-management-performance/resource/6a2ea6c7-9d34-4560-8224-753ffd17bab5

## 3. Paradigm shifts

| Conventional assessment | Service-chain engineering shift |
|---|---|
| Treatment-works condition as a proxy for sanitation | End-to-end sanitation service and environmental boundary |
| Plant count as the denominator | Explicit population, catchment, generated-flow and connected-system denominators |
| "No data" silently treated as zero or omitted | Unknown, unreported, unmeasured, and not-applicable remain distinct |
| Plant inlet volume treated as total wastewater generated | Reconcile estimated generation, collection, transfers, treatment, bypass, reuse and discharge |
| A single national percentage | Distribution across catchments, households, facilities and vulnerable cohorts |
| Asset failure list | Time-dependent service-chain dependency and common-cause graph |
| Static remediation priority | Recovery options qualified against physical access, power, spares, staffing, safety, dependencies and authority |
| Model fit as success | Out-of-sample service, environmental and equity outcomes with independent verification |

The new measurable object is the **boundary-qualified service-flow balance**: an interval-valued account of what entered, moved through, left and remains unobserved across a declared service boundary, each term tied to source, period, unit, geography, population and uncertainty.

This is not a simplistic conservation assertion. Water entering a catchment is not all wastewater generated; infiltration/inflow, stormwater ingress, industrial inputs, evaporation, storage, reuse, bypasses and timing mismatch matter. A residual is "unaccounted within this declared boundary and uncertainty model", not automatically leakage, theft, illegal discharge or pollution.

## 4. Minimum service-chain evidence contract

Every observed or estimated flow term must bind to:

1. Stable facility, network segment, catchment and service-cohort identifiers.
2. A measurement/estimation method and unit (preferably volume/time with explicit standard conditions).
3. Observation interval, clock basis, time alignment, and sampling frequency.
4. Source owner, source record, revision/digest where available, collection method and admissibility.
5. Instrument identity, calibration/quality flags, missingness and uncertainty bounds.
6. Whether the value is directly metered, sampled, model-estimated, imputed, reported or unknown.
7. Whether the record covers normal operation, wet weather, emergency bypass, planned shutdown or unobserved time.
8. Population/service denominator and its revision history.
9. Environmental boundary and relevant effluent/receiving-water measurement methods.
10. Independent review and the exact claim that the record is allowed to support.

Do not equate:
- no reported volume with zero volume;
- no sampled exceedance with compliant effluent;
- treatment-works capacity with actual throughput;
- connected population with served population;
- a national average with each catchment's condition;
- source-reported status with independently verified physical state.

## 5. Flow accounting and uncertainty

For a declared catchment and interval, construct an accounting identity only after all relevant terms are defined:

`G + I = T + B + R + ΔS + U`

where:
- `G`: estimated wastewater generated within the declared scope;
- `I`: other known inflows (for example infiltration/inflow or permitted imported flows);
- `T`: flow delivered to treatment;
- `B`: known bypasses or overflows leaving the collection system;
- `R`: measured reuse/export or other declared outflow;
- `ΔS`: change in known storage/retention within the interval;
- `U`: residual unaccounted flow under this model.

Use intervals or distributions for uncertain terms, not point values disguised as certainty. A residual may be negative or too wide to identify, which is a model/data adequacy result rather than an excuse to clip it to zero. Carry correlated measurement uncertainty where instruments or estimation methods share common sources. Keep separate physical-volume balance from microbiological/chemical compliance: concentration measurements require flow-weighted interpretation and their own sampling/quality evidence.

The model must disclose boundary exclusions, wet-weather conditions, meter coverage, known unmetered contributors, population estimation method, time alignment, and whether the accounting period is sufficiently closed.

## 6. Benchmark design

### Tier A — published-summary conformance

- Reproduce published counts and rounded rates.
- Preserve the exact reporting vintage, observation period, geography, unit and denominator status.
- Detect rounding inconsistencies and denominator substitution.
- Never infer 2022 system denominators from rounded national percentages.

### Tier B — system-level historical replay

When licensed and obtained from DWS, extract each published system record with immutable source references and human-reviewed transcription. Reconcile totals against the official national/provincial tables. Keep source PDF page/table references and original row text. Record system additions/removals and Water Services Authority boundary changes; do not compare scores without a reviewed crosswalk.

### Tier C — catchment/reticulation flow reconstruction

Use historical meter readings, plant inflow/outflow, pump telemetry, sewer incidents, rainfall, population/catchment estimates, bypass reports, effluent sampling and receiving-water observations. Separate directly measured, estimated and unknown terms. Publish the boundary and uncertainty model before assessing the residual.

### Tier D — blind fault injection and adversarial missingness

Create sealed cases for:
- unmetered sewer branches;
- pump outage and partial bypass;
- rainfall-driven infiltration/inflow;
- faulty meter drift and clock skew;
- duplicated or delayed telemetry;
- missing laboratory samples;
- undocumented industrial flows;
- changed catchment/population boundaries;
- common-cause electricity and communications failure;
- incomplete source submissions and denominator drift.

Ground truth must be generated/held independently from the inference team. Include both benign and adverse cases to measure false attribution.

### Tier E — prospective, operator-authorised shadow study

Start read-only. Require the water services authority and operators to approve data access, purpose, retention, safety review and publication. Do not write to SCADA, control systems, maintenance workflows or public notifications. Any later intervention requires separate human authority, operational safety procedures and independently defined outcome measurement.

## 7. Metrics and proposed research targets

Targets below are research goals, not current results.

| Metric | Proposed target | Failure interpretation |
|---|---:|---|
| Source/denominator lineage completeness for every published aggregate | 100% explicit; unknown is allowed but must be visible | Any silent denominator substitution fails |
| Arithmetic reproduction of all published rounded ratios where numerators and denominators exist | 100% | Any unexplained mismatch fails |
| Correct unknown/zero/absent-row distinction | 100% on sealed semantics cases | One false zero is a material failure |
| Flow interval coverage | Nominal 90% intervals should cover 85–95% of held-out observed values, after pre-registration | Poor coverage signals miscalibration |
| False causal attribution to a specific pipe, operator or community | 0 on the sealed negative-control suite | Any false attribution blocks release |
| Unobserved-flow residual localisation | Report precision/recall and calibrated abstention by spatial scale; no target until ground truth exists | Do not manufacture a percentage target without a labelled baseline |
| Catchment service-chain boundary coverage | 100% of included terms have owner, unit, interval, geography and method; omitted terms disclosed | Hidden boundary exclusions fail |
| Cohort service reliability | Publish median, lower tail and worst supported cohort with uncertainty | Averages cannot hide service deprivation |
| Independent reproduction | Independent team reproduces all published summary arithmetic and a sealed sample of extracted rows | Shared transcription is not independence |
| Operational safety | 0 unauthorised actuation; 0 unreviewed source promotion | Hard constraints, never trade-offs |

### Primary comparison

Compare, on identical frozen inputs and equal compute/hardware budgets:

1. Plant-only baseline using Green Drop/report-card indicators.
2. Plant + finance + asset-maintenance baseline.
3. Coupled infrastructure interdependency tools (IN-CORE, InfraRisk, REFIT, PRIIA or others where task fit/licensing permit).
4. Service-chain flow accounting with explicit unknown terms.
5. Full FSRE composition: owner-native service, spatial, physical, dependency, reasoning, authority and evidence receipts.

Measure not only accuracy but also data burden, time to usable result, calibration, abstention, false attribution, operator effort, reproducibility, and cost per independently verified unit of service improvement.

IN-CORE is identified in the 2026 comparative study as having broad coverage of physical, functional, geographic, economic and social interdependencies. FSRE should use it as a strong comparator, not claim superiority without matched runs. The comparative study also identifies a trade-off between breadth, data requirements, compute, expertise and accessibility; the benchmark should quantify that trade-off for the concrete wastewater task.

### Sewer-specific sensor-placement comparators

Do not treat sensor placement as an unexplored FSRE contribution. A 2026 *Water Research* paper proposes topology-derived flow inference (TDFI) to estimate pipe-specific minimum detectable inflow/infiltration from upstream sensors, then uses deterministic Sequential Backward Selection (SBS) to construct sensor layouts. It evaluates one synthetic and two real-life sewer-network case studies and reports SBS results close to a differential-evolution optimiser. A 2025 *Water Research* study also proposes entropy-based I&I sensor placement and reports 7–15% higher detection reliability against its compared methods. These are direct sewer-network comparators, not proof of universal superiority across utilities.

References:
- “A topology-derived flow inference approach to optimize sensor placement for effective inflow and infiltration detection in sewer networks”, *Water Research* 302 (2026), 126140, DOI: https://doi.org/10.1016/j.watres.2026.126140
- “Enhancing inflow and infiltration detection in urban sewer networks with a new deterministic sensor placement method”, *Water Research* 268 (2025), 122591, DOI: https://doi.org/10.1016/j.watres.2024.122591

A fair wastewater experiment should compare, on identical network topology, sensor budget, observation period, noise, missingness and event catalogue:
1. existing operator placements and routine monitoring;
2. uniform/random placement;
3. entropy-based placement;
4. TDFI + SBS;
5. full physical-model or optimisation baseline where calibrated and available;
6. FSRE active investigation using expected decision value, owner authority and service-cohort outcomes.

Measure minimum detectable I&I, false alarms per sensor-day and per network length/time, event detection delay, pipe/catchment localisation error, false attribution, sensor lifecycle and maintenance cost, operator effort, SCADA/GIS integration burden, and verified service/environment decision value. Do not compare a drinking-water distribution leak benchmark directly with wastewater sewer-network I&I without a matched task definition.

A 2026 review of 86 drinking-water leak-detection studies identifies operational false-alarm rates, computational feasibility and SCADA/GIS integration as important evaluation requirements, and highlights the gap between simulated and operational settings. This supports explicit operational metrics; it does not establish wastewater performance.

## 8. Falsification conditions

The hypothesis that service-chain completeness improves decisions is weakened or falsified if:
- plant-only or current operator workflows identify the same actionable faults with equal or better calibrated performance and lower cost;
- broader accounting increases false alarms or false accusations without improving independent localisation;
- residuals remain unidentifiable at the resolution needed for safe intervention;
- the additional data burden makes the method inaccessible to resource-constrained utilities;
- the system cannot preserve privacy, lawful data rights, operator sovereignty or equitable treatment;
- prospective studies do not improve independently verified service or environmental outcomes relative to a matched control.

## 9. Integration with FSRE owners

- **Sol-Atlas:** versioned catchments, network topology, spatial exposure, asset and service-area identity.
- **Symtropy:** hydraulic/physical accounting, flow and storage dynamics, meter uncertainty, scenario replay and model adequacy.
- **Symthaea:** competing hypotheses, value-of-information for inspections/sensors/sampling, causal investigation, and abstention when non-identifiable.
- **Mycelix:** source/evidence provenance, owner authority, cross-organisation permissions, intervention commitments and completed-effect lineage.

No owner should inherit authority merely because it can read another owner's evidence. The service-chain profile adds no new data ledger and no actuation path.

## 10. Immediate execution queue

1. Keep the current official-summary replay as a bounded national baseline.
2. Resolve DWS source access and the resource-level licence before extracting the 2025 PDF tables.
3. Obtain a legitimate system-level table export or prepare a documented human transcription with dual review and page/row citations.
4. Create municipality/catchment identity crosswalks before longitudinal joins.
5. Secure a water services authority/operator partner for read-only data validation.
6. Obtain flow, pump, bypass, effluent and receiving-water observations before claiming a measured service-chain residual.
7. Preregister sealed missingness, meter-drift, common-cause and hidden-flow benchmarks before evaluating localisation.

**Current status:** Tier A national-summary replay is implemented as a parser/arithmetic check using the DWS release and a separately cited SAnews denominator/boundary statement. `tools/fsre-flow-balance` provides a synthetic interval-accounting reference with fail-closed unknown handling and explicit residual non-attribution; it is not real catchment flow reconstruction or a hydraulic model. Tiers B–E are proposed; no system-level extraction, observed catchment-flow balance, operator partnership, field result or causal effect has been completed.
