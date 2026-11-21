# Nutrient Harmonization and Causal-Contrast Contract v0.1

**Status:** Research/data-harmonization specification only. It is not a field protocol, agronomic recommendation, amendment specification, fertilizer substitution claim, or regulatory approval.

## 1. Purpose: make unlike measurements comparable without erasing what they measured

A harmonized table is not automatically a scientifically comparable dataset. Nutrient observations differ in elemental basis, chemical species, extraction method, sampled depth, soil mass, time window, hydrological volume, control-volume boundary, and treatment contrast. Converting all values to a common numeric unit can create false comparability when these dimensions are not aligned.

The rule is **preserve first, derive second**:

1. Retain the source value, original unit, analytical method, sampling geometry, time interval, detection limit, and source-page/table/row evidence.
2. Record every conversion as a versioned transformation with inputs, formula, assumptions, software version and output digest.
3. Keep observations incomparable when essential conversion metadata are missing. Do not repair them by guessing bulk density, soil depth, moisture basis, drainage volume, or elemental fraction.
4. Preserve method-defined pools as distinct variables. Do not pool chemical species or extraction methods merely because their labels both contain “available” or “total”.

## 2. Canonical dimensions and conversion gates

| Measurement family | Required dimensions | Permitted harmonization | Fail-closed cases |
|---|---|---|---|
| Soil stock | Element/species, kg element/ha, sampled depth, bulk density or measured soil mass, coarse-fragment correction, equivalent-soil-mass method, date | Convert concentration × measured dry-soil mass to kg element/ha; keep layer-resolved stocks before aggregation | Missing depth, bulk density/soil mass, coarse fragments, moisture basis, or elemental basis |
| Leachate/runoff flux | Element/species, concentration, event/period water volume, area, collection coverage, sampling interval | Integrate concentration × measured volume and scale by area; event totals remain available | Concentration without water volume; incomplete collection without a labelled estimate; unknown area |
| Plant uptake/export | Element/species, sampled organ, dry-matter mass, concentration, harvest fraction, date | Organ concentration × dry biomass; distinguish standing biomass, removed harvest and retained residue | Concentration without biomass; unspecified edible/removed fraction; crop residue counted in two pools |
| Gas flux | Element/species or chemical species, chamber/micrometeorological method, area, time coverage, calibration | Integrate time-resolved flux over the measured interval with uncertainty | One concentration reading treated as season-long flux; unknown temporal coverage or conversion to elemental mass |
| Extractable availability | Exact extraction method/version, soil basis, depth, sampling design, unit, date | Method-stratified comparisons or validated calibration bridges | Cross-method pooling without paired calibration; treating extraction as a disjoint stock |
| Microbial/biological pools | Compartment, assay, marker/functional endpoint, unit, sampling time | Compare within assay/compartment, or use validated cross-assay calibration | Bulk soil, root, rhizosphere, earthworm gut or compost treated as interchangeable |

The preferred common mass basis is kg of **element** per hectare per explicitly stated period or stock date. Convert only when all required quantities exist. For example, concentration multiplied by collected water volume can become a mass only when integrated volume and collection fraction are known; concentration per dry-soil mass can become a stock only with measured dry soil mass per area for the same layer and declared coarse-fragment/equivalent-mass treatment. Area conversion does not create temporal coverage. Oxide or product mass requires an explicit conversion step; the v0.1 auditor accepts only elemental mass.

Concentration is not a flux. Concentration multiplied by an assumed seasonal water volume is a model estimate, not a measured export. Every modelled term must preserve model identity, inputs, uncertainty and validation status.

## 3. Nutrient identity and pool ontology

Keep analytical and chemical identity intact:

- **Nitrogen:** nitrate, ammonium, dissolved organic N, total dissolved N, particulate N, gaseous N species and total N are separate analytes or pools. Do not call all of them “mineral N”.
- **Phosphorus:** total P, dissolved reactive P, dissolved organic P, particulate P, and each extraction-defined pool (for example, Bray-1, Olsen, Mehlich) remain method-specific.
- **Potassium:** solution K, exchangeable K, non-exchangeable/mineral K and total K remain distinct pools unless an explicitly reviewed analytical model connects them.
- **Carbon:** total organic carbon, inorganic carbon, pyrogenic carbon and amendment-derived carbon are not interchangeable. Carbon authority/permanence claims remain separately governed.

An extractable fraction is often a subset of total stock; it cannot be added to total stock. When stocks are reported by nested size/fraction or chemical extraction, the crosswalk must say whether values overlap and select a mutually exclusive partition before summation.

## 4. Treatment and comparator identity

Classify each literature contrast before effect estimation:

1. Biochar-only versus matched no-biochar comparator with otherwise equivalent management.
2. Biochar + fertilizer versus the same fertilizer programme without biochar.
3. Biochar + fertilizer versus no fertilizer or a different fertilizer rate.
4. Biochar + organic amendment or compost versus its matched comparator.
5. Multi-factor design with separately identifiable main effects/interactions.
6. Uncontrolled or bundled package with no isolatable biochar contrast.
7. Unclear contrast: retained for descriptive evidence only.

Do not label category 2 as fertilizer replacement. Do not assign a contrast to biochar alone when the comparator differs in fertilizer, manure, water, crop, tillage, or another co-intervention. For factorial designs, use design-supported main-effect and interaction terms; do not infer them from one pairwise outcome if the design cannot identify them.

Record the randomized/assigned treatment unit, comparator arm, allocation method, baseline, co-interventions, treatment batch and application timing. Keep plot/pot/mesocosm as the independent experimental unit. Technical aliquots and soil subsamples do not increase treatment sample size.

## 5. Time and hydrology alignment

- Keep stocks as time-point measurements and fluxes as integrated intervals; do not subtract a stock at one season from a non-matching period without an explicit interval definition.
- Preserve event-level rainfall, irrigation, drainage and leachate collection where available; event aggregation must not conceal episodic high-loss events.
- Record whether flux windows include establishment, growing season, fallow, harvest, wetting/drying cycles and post-application first flush.
- A season is not automatically a year; cumulative annual and multi-year totals are distinct estimands.
- For water-mediated flux, report water volume/drainage alongside nutrient concentration and mass export. Drying can reduce leaching because less water moves, which is not necessarily improved nutrient retention or plant service.
- Retain depth-stratified stocks and flux boundary depths. A 0–10 cm result cannot silently represent a 0–30 cm profile.

## 6. Uncertainty, censoring and dependence

- Retain analytical, sampling, spatial, flow-volume, temporal-integration and conversion uncertainties separately where possible.
- Propagate concentration × water-volume uncertainty with covariance when known; otherwise disclose the simplifying assumption and do not call the output a complete confidence interval.
- Preserve detection limits, left/right censoring, blank samples, below-detection values, missing values and structural zeros as different states. “Not reported”, “not measured”, “below detection” and measured zero are not interchangeable.
- For multi-arm studies with shared controls, preserve shared-control covariance; do not count every derived pairwise contrast as independent.
- Bootstrap or model uncertainty at the randomized experimental-unit and study/site dependency level, not at the row level.
- Split confirmatory data by connected source-study, experiment, site, batch and repeated-curve dependencies before training. Random-row splits are diagnostic only.
- Retain null and adverse results, not just significant or positive outcomes.

## 7. Causal estimands and outcomes

Predeclare outcomes separately for N, P and K, and by pathway. Suitable estimands include:

- Absolute difference in season/year elemental export (kg element/ha/period).
- Absolute difference in stock change (kg element/ha/period) with matching depth and equivalent-soil-mass basis.
- Plant uptake or harvest export (kg element/ha/period) separated from standing stock and retained residue.
- Ratio/log response ratio only when the denominator is positive, comparable and scientifically meaningful; retain the absolute difference alongside it.
- Time-to-threshold, cumulative loss, and treatment-effect trajectory under repeated observations.
- Pathway-specific leaching/runoff/erosion/gaseous losses, not merely summed export.

A smaller nutrient loss cannot be assumed beneficial if nutrient inputs or crop uptake also fall. Higher extractable availability cannot be assumed beneficial if export, contaminant mobility or ecological harm rises. A lower residual is accounting quality, not agronomic efficacy.

## 8. Evaluation ladder and proposed benchmarks

All target values below are proposed acceptance targets, not measured achievements.

| Gate | Proposed criterion |
|---|---|
| H0 — immutable provenance | 100% of confirmatory observations retain source digest, exact source location, raw value/unit, method and transformation chain |
| H1 — unit/identity integrity | 100% of confirmatory records have explicit element/species, soil/water basis, depth, area, time interval and conversion metadata |
| H2 — treatment estimability | 100% of confirmatory contrasts have explicit matched co-interventions, experimental unit and comparator; unidentifiable bundles remain descriptive |
| H3 — censoring/missingness | No missing, below-detection or unmeasured pathway silently converted to zero |
| H4 — dependency integrity | Zero connected study/experiment/site/batch leakage across confirmatory train/validation/test partitions |
| H5 — uncertainty | Nominal 90% intervals target 85–95% empirical coverage on independent held-out studies, with proper interval score and interval width reported |
| H6 — useful prediction | Candidate must improve at least 10% over the strongest valid baseline on a predeclared endpoint, with group-aware uncertainty and no material domain regression |
| H7 — local transportability | Independent South African/African site or lab context is a held-out domain, not random rows mixed into training |
| H8 — falsification | At least one negative-control and one expected-null fixture; model must abstain when treatment identity, method or boundary is not identifiable |

Benchmark baselines include no-effect/zero-change, historical mean by endpoint, mixed-effects study-level models, and physically interpretable water/nutrient balances where required parameters are measured. Compare candidates on the same frozen study/site/batch partitions. A model with better pooled random-row accuracy but worse held-out-study calibration does not pass.

## 9. Cross-system authority boundary

Mycelix preserves source, subject, treatment, batch, sample, measurement and reviewer lineage. Symtropy may simulate transport/sorption after parameters and boundary conditions have independent support. Symthaea may prioritize the next approved measurement or propose a preregistered contrast; it may not promote an unidentified causal effect, authorize a formulation, or convert model confidence into physical authority.

## References

Use the sources and source-register entries attached to the Terra Preta benchmark. The harmonization layer records measurement semantics; it does not create data that publishers have not made available.
