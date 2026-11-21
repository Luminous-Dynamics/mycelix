# Field-Realized Greenhouse-Gas and Nutrient-Risk Benchmark v0.1

**Status:** Research contract and synthetic-first measurement benchmark. Not a product specification, carbon-credit methodology, fertilizer recommendation, trial authorization or field-performance claim.  
**Version date:** 2026-11-21  
**Issue:** [TERRA-GHG-001](https://github.com/Luminous-Dynamics/mycelix/issues/6405)  
**Programme:** Terra Preta / regenerative soil systems  
**Related:** #5525, #5527, #5529, #5514, #5390

## 1. Purpose and claim boundary

Qualify whether a soil-management intervention produces a **field-realized net climate benefit** while preserving or improving soil function and avoiding unacceptable nutrient, ecological, health and livelihood trade-offs.

This benchmark is deliberately narrower than the complete functional-soil-system benchmark. It adds a measurement and accounting layer for greenhouse-gas flux, lab-to-field transfer and the joint interpretation of carbon, nitrogen and other nutrient flows.

The core distinction is:

- **Material carbon storage** is not the same as a whole-project net greenhouse-gas benefit.
- **Laboratory mitigation potential** is not a field mitigation result.
- **A soil CO₂ flux** is not, by itself, a measured change in soil organic-carbon stock.
- **A change in nutrient availability** is not a complete nutrient budget.
- **A biochar-plus-fertilizer package effect** is not a biochar-only causal effect.
- **A registry entry, model estimate or ledger receipt** is not independent verification of a physical climate outcome.

No single net score can compensate for failed safety, pollution, water, biodiversity, land-rights or community-consent requirements.

## 2. Evidence motivating the benchmark

### 2.1 Field response can diverge sharply from laboratory response

Wang et al., *From laboratory evidence to field-realized N₂O mitigation by biochar: a global meta-analysis and mechanistic synthesis*, *Journal of Cleaner Production*, published 15 November 2026, report 874 paired N₂O comparisons: 450 laboratory incubations and 424 field comparisons. The abstract reports an 11% field reduction with a reported interval of −27% to +10%, versus a 31% laboratory reduction with an interval of −41% to −19%. The field interval crosses zero. Reproduction must preserve the original response scale and sign convention; these abstract values must not be silently transformed into a universal mitigation factor.

DOI: [10.1016/j.jclepro.2026.149851](https://doi.org/10.1016/j.jclepro.2026.149851)

**Implication:** field N₂O response must be observed and estimated under the target soil, crop, water, temperature and nutrient regime. Laboratory results are a separate evidence class and may inform hypotheses, not substitute for field evidence.

### 2.2 Nutrient retention is conditional and package-specific

Dissanayake et al., *Applying Biochar With Fertilisers Decreases Nitrogen and Phosphorus Leaching and Improves Phosphorus Availability: A Meta-Analysis*, *GCB Bioenergy*, first published 6 August 2026, synthesize 1,798 observations from 48 articles. The reported effects concern combined biochar/fertilizer applications and vary with application method, time, soil organic carbon, feedstock and pyrolysis temperature.

DOI: [10.1111/gcbb.70165](https://doi.org/10.1111/gcbb.70165)

**Implication:** preserve treatment components and comparators. A package contrast cannot be relabelled as the isolated effect of biochar, compost or fertilizer. Measure nutrient concentration and water volume together when quantifying leaching.

### 2.3 Soil-function effects are time-dependent

Jiang, Huang & Meng (2025), *Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments*, synthesize 61 field studies. Their analysis reports time-varying hydraulic responses, with some porosity and water-retention-related effects peaking around year two and declining thereafter; feedstock and soil texture influence response.

DOI: [10.1016/j.geoderma.2025.117318](https://doi.org/10.1016/j.geoderma.2025.117318)

**Implication:** do not infer multi-year function or permanence from a short incubation, one harvest, one gas-flux campaign or one material-carbon assay.

### 2.4 Methodology and regulatory comparators

- The [Puro.earth Biochar Certification Framework, Edition 2025 v2](https://puro.earth/cdr-infrastructure/methodologies/) defines a cradle-to-grave project boundary and includes operational and embodied emissions, biomass sourcing, production and end-use. It is a carbon-removal methodology comparator, not a general soil-health or South African fertilizer approval.
- [Commission Delegated Regulation (EU) 2026/285](https://eur-lex.europa.eu/eli/reg_del/2026/285/oj) establishes EU carbon-removal certification methodologies and includes biochar soil-use requirements for contaminants, pollution prevention and ecosystem/soil-health safeguards. EU rules do not automatically apply in South Africa.
- South Africa's [Agriculture Inputs Control Directorate](https://www.nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control) regulates agricultural inputs under Act 36 of 1947. Government guidance for Group 2 fertilizers requires registration to manufacture or sell and an accredited laboratory certificate of analysis. The actual category of a proposed formulation and claim must be confirmed with the Registrar; do not infer it from the word "biochar" or "compost". [Group 2 guidance](https://www.gov.za/services/fertilizers-farm-feeds-agricultural-remedies/register-group-2-fertilizer).

## 3. Benchmark unit, experimental hierarchy and treatment contrasts

### 3.1 Preserve the true assignment unit

The unit of treatment replication is the smallest unit independently assigned to a treatment (for example, a plot, pot or mesocosm). The following are generally nested observations rather than independent treatment replicates:

- chamber deployments and repeated chamber closures;
- cores, subsamples and depths within a plot;
- sensor readings and timepoints;
- image tiles, soil aggregates and laboratory aliquots;
- repeated measurements across a season.

Every observation must preserve the study, site, block, independent experimental unit, plot/pot, treatment assignment, material batch, soil source, crop, management regime, method, instrument, time and sample hierarchy where applicable. The analysis must not inflate treatment sample size by counting nested observations as independent.

### 3.2 Define the estimand before looking at outcomes

For each study, state the intended causal or descriptive quantity in plain language and machine-readable form. Examples include:

- difference in cumulative seasonal N₂O-N emissions between matched treatments;
- difference in cumulative CH₄ emissions over a declared water regime;
- difference in elemental nitrate-N export per hectare over a declared drainage period;
- difference in equivalent-soil-mass native SOC stock over a declared depth and time horizon;
- difference in net project GHG balance per tonne of dry biochar actually used;
- difference in crop output or water productivity under the same nutrient and water programme.

Do not mix these estimands. For example, a treatment that receives more nitrogen cannot support a fertilizer-substitution claim unless the nutrient programme and the relevant counterfactual are designed to identify that effect.

### 3.3 Minimum contrast families

Select the smallest defensible set needed to identify the predeclared question. Candidate contrasts include:

1. local baseline practice / untreated reference;
2. characterized biochar-only versus matched reference;
3. compost or other organic-input-only versus matched reference;
4. biochar plus compost versus the corresponding component contrasts;
5. biochar plus fertilizer versus nutrient-matched fertilizer-only;
6. conventional local agronomic practice versus the candidate complete system.

These are design classes, not a recipe or authorization to use a material. Match nutrient supply, water regime, tillage, crop, residue management and other relevant co-interventions whenever required by the estimand. Randomize at the actual independent treatment-assignment unit and block on important spatial or soil gradients.

## 4. Field gas-flux observation contract

The machine-readable contract is defined in [field-ghg-observation.schema.json](./schemas/field-ghg-observation.schema.json), with synthetic-only examples in [synthetic-field-ghg-observations.json](./fixtures/synthetic-field-ghg-observations.json).

### 4.1 Required semantics

Each observation records:

- gas species: N₂O, CH₄ or CO₂;
- exact mass basis: N₂O versus N₂O-N must never be ambiguous;
- unit and sign convention;
- measurement interval and event context;
- evidence class: laboratory incubation, pot/mesocosm or field;
- study/site/block/experimental-unit and treatment-assignment keys;
- chamber geometry and measurement method;
- instrument identity and calibration reference;
- raw source artifact SHA-256 and processing/evaluator version;
- uncertainty, detection/quantification limit and quality-control state;
- missingness and exclusion reasons when applicable.

The canonical positive direction is flux **to the atmosphere**; negative values represent net uptake from the atmosphere. A source that uses another convention must be explicitly transformed with a recorded operation, not silently relabelled.

### 4.2 Do not collapse measurement bases

Examples of distinct bases:

- mg N₂O m⁻² h⁻¹;
- mg N₂O-N m⁻² h⁻¹;
- mg CH₄ m⁻² h⁻¹;
- mg CO₂ m⁻² h⁻¹.

The N₂O-to-N₂O-N conversion is a stoichiometric transformation (multiply N₂O mass by 28/44 to obtain N₂O-N mass), not a change in measurement identity. Preserve the original value and basis alongside every derived value.

### 4.3 Temporal coverage and event dependence

Fluxes can be event-sensitive. The sampling schedule and integration method must be specified before outcomes are inspected. The plan should explicitly cover relevant windows around rainfall/irrigation, fertilizer or manure application, wetting/drying transitions, tillage and other declared management events.

- Report the planned and observed coverage of each event window.
- Preserve timestamp, chamber closure duration, concentration-time observations where permitted, flux calculation method and fit/QC diagnostics.
- Integrate only over observed support using a declared numerical method.
- Do not interpolate across a missing event peak or extrapolate a short campaign into an annual result without a qualified model and uncertainty.
- If a material event window is missing, mark the cumulative estimate incomplete or unqualified rather than assigning zero emissions.

CO₂ chamber flux supports process interpretation, but cannot by itself separate plant/root respiration, microbial respiration, decomposition of native organic matter and biochar oxidation. Use stock measurements and appropriate source-partition evidence to support carbon-stock claims.

## 5. Net climate accounting boundary

### 5.1 Keep gross carbon, project emissions and soil response separate

For a declared functional unit and monitoring period, represent:

[
B_{mathrm{net}} =
C_{mathrm{biochar,,persistent}}
+ Delta C_{mathrm{native,SOC}}
- E_{mathrm{upstream}}
- E_{mathrm{conversion}}
- E_{mathrm{transport}}
- E_{mathrm{application}}
- Delta E_{mathrm{soil,,nonCO_2}}
- E_{mathrm{leakage}}
]

This is a conceptual bookkeeping identity, not a crediting methodology. Each term must have a declared boundary, unit, source, method, uncertainty and time horizon. Positive (B_{mathrm{net}}) denotes net benefit under the declared sign convention.

- (C_{mathrm{biochar,,persistent}}): independently supported persistent biochar carbon, with the methodology and permanence fraction stated.
- (Delta C_{mathrm{native,SOC}}): treatment-minus-counterfactual change in native soil organic carbon on an equivalent-soil-mass basis where supported. Do not count added biochar carbon a second time as native SOC.
- (E_{mathrm{upstream}}): feedstock collection, alternative/competing feedstock use, preprocessing and relevant land-use effects.
- (E_{mathrm{conversion}}): process energy, fuel, electricity, fugitive gases, start-up/shutdown and co-product allocation.
- (E_{mathrm{transport}}) and (E_{mathrm{application}}): measured or source-qualified logistics and incorporation.
- (Delta E_{mathrm{soil,,nonCO_2}}): treatment-minus-counterfactual N₂O and CH₄ emissions, converted using a named GWP assessment/version and time horizon.
- (E_{mathrm{leakage}}): relevant displaced production, foregone residue use and other material counterfactual effects.

Measured soil CO₂ flux must not simply be subtracted as though it were all net soil-carbon loss when stock-change terms already account for carbon storage. Prevent double counting between biochar carbon, native SOC change, soil gas exchange and lifecycle inventories.

### 5.2 Report multiple functional units

At minimum, report separately:

1. per tonne of dry biochar actually used;
2. per hectare over the declared crop season/monitoring period;
3. per unit of agricultural service where meaningful (for example, per tonne of marketable crop or per unit of nutrient export avoided).

Comparisons must match system boundary, time period, feedstock counterfactual, nutrient inputs, crop/service output and the denominator. A per-tonne-biochar result cannot be substituted for a per-hectare result.

### 5.3 Missing terms and uncertainty

Every material inventory term is one of: measured, derived from named measurements, modeled, literature-imputed, bounded or unmeasured. Missing terms are not zero. Any material unmeasured/unbounded term blocks a qualified net-climate result.

Propagate measurement, sampling, model, allocation and counterfactual uncertainty. Preserve correlations and shared sources of uncertainty; do not add independent error bars where measurements share a batch, instrument, site or model. Report a partial inventory where full net qualification is unavailable.

## 6. Coupled nutrient accounting

Integrate with #5529 rather than creating a competing nutrient-budget system.

- Use elemental N, P and K mass bases with compatible area, depth and time.
- Leachate export requires both measured concentration and compatible water volume/flux.
- Distinguish plant uptake/removal, external inputs, external exports, stock changes and internal transfers.
- Internal transformations (mineralization, immobilization, sorption/desorption, root uptake and microbial cycling) do not cross the system boundary unless a declared external transfer occurs.
- Availability indicators are not total nutrient stocks.
- Preserve N₂O/N₂O-N basis and fertilizer product basis separately from elemental N.
- Report nutrient retention, nutrient availability and nutrient losses as separate endpoints.

A reduction in nitrate concentration is not automatically a reduction in nitrate mass export unless drainage volume is also known.

## 7. Laboratory-to-field transportability

Keep laboratory incubation, greenhouse/pot/mesocosm and field trials in distinct evidence classes. Build matched comparisons where possible using the same characterized material batch and soil source, with declared differences in temperature, moisture, plant presence, nutrient supply and forcing.

Required evaluations:

- lab-to-pot and pot-to-field transfer error;
- field-only held-out site and season performance;
- calibration and interval coverage by evidence class;
- sensitivity to soil texture, pH, initial SOC, water regime, crop, nutrient treatment, feedstock and material age;
- model disagreement and out-of-domain abstention;
- the direction and magnitude of adverse as well as beneficial responses.

A model that predicts laboratory responses accurately but fails on held-out field observations has not passed field qualification. Do not pool evidence classes to conceal that failure.

## 8. Comparator hierarchy

Use capability-matched comparators and state exact versions, configurations, units and boundaries.

| Comparator | Role | What it does not establish |
|---|---|---|
| Direct chamber/field observations with transparent integration | Primary field reference for gas flux | Annual result when important periods are missing |
| Laboratory incubation and pot/mesocosm models | Controlled mechanistic and transfer references | Field effect without external validation |
| Hierarchical/mixed-effects and event-aware statistical models | Transparent field heterogeneity and uncertainty baselines | Physical mechanism by model form alone |
| IPCC inventory methods and factors where applicable | Inventory fallback and reporting context | Site-specific measurement or treatment effect |
| Puro Biochar methodology 2025 v2 | Biochar CDR process/LCA and permanence comparator | Whole-soil health certification or South African permission |
| EU Delegated Regulation 2026/285 | EU CRCF biochar removal and sustainability comparator | South African regulatory approval or automatic equivalence |

No single generic "industry baseline" is adequate. Compare on the same source set, target population, functional unit and boundary.

## 9. Synthetic-first qualification suite

Before any empirical qualification, implement hand-checkable cases and negative controls for:

1. exact N₂O ↔ N₂O-N conversion and unit conversion;
2. signed flux integration over known time intervals;
3. chamber area, volume and closure-duration metadata consistency;
4. positive and negative fluxes without sign inversion;
5. missing rainfall/fertilizer event windows that must block a complete seasonal total;
6. detection-limit censoring and non-detect handling without replacing values by zero;
7. repeated chamber readings nested within one plot;
8. duplicated treatment assignment or cross-split site/batch leakage;
9. correlated instrument/calibration errors;
10. lab-to-field shift where laboratory mitigation does not reproduce in field;
11. increased field N₂O or CH₄ despite higher persistent biochar carbon;
12. nutrient concentration reduction with increased drainage volume and unchanged/increased mass export;
13. native SOC loss/priming despite measured biochar carbon;
14. feedstock counterfactual and competing-use changes;
15. double counting between biochar C, native SOC, soil CO₂ flux and lifecycle emissions;
16. missing uncertainty, missing inventory terms, placeholder digests and tampered raw artifacts.

The evaluator may hold synthetic truth unavailable to candidate policies; the acquisition/model layer must not receive evaluator-only truth.

## 10. Proposed qualification gates and targets

All numeric values below are **targets**, not achieved performance.

| Gate | Proposed target | Failure interpretation |
|---|---|---|
| Contract integrity | 100% confirmatory observations bind gas, basis, unit, sign, time, method, experimental unit and raw digest | Unqualified record |
| Replication | Zero pseudo-replication in treatment-effect uncertainty | Invalid inference |
| Unit integrity | Zero silent N₂O/N₂O-N, elemental/product basis or measured/modelled conversions | Reject report |
| Event completeness | Every planned high-risk window has observed coverage or an explicit missingness disposition | Incomplete seasonal estimate |
| Synthetic accounting | Exact hand cases close within a preregistered numerical tolerance | Accounting implementation failure |
| Predictive calibration | Nominal 90% intervals target 85–95% empirical coverage on independent held-out groups where sample size permits | Recalibrate, stratify or abstain |
| Transportability | Report field-held-out results separately; no lab-to-field substitution | No field claim |
| Net result | All material terms measured or conservatively bounded with propagated uncertainty | Partial / unqualified |
| Safety / rights | All applicable approvals, contamination and ecological gates pass | Stop / quarantine / no application |

Do not impose a universal N₂O reduction or yield target before a local estimand and minimum important effect are defined by qualified reviewers and trial design.

## 11. Implementation sequence

### E0 — contracts and synthetic fixtures
- Version the observation schema and fixture.
- Add an independent Rust validator/auditor.
- Add exact unit, chronology, event-coverage and digest negative controls.
- Add no field-collection or external transaction capability.

### E1 — reference calculations
- Implement flux integration and N₂O/N₂O-N conversions.
- Implement per-gas seasonal accounting with explicit gaps and censoring.
- Keep carbon stock, gas flux and LCA terms separate.
- Validate against independent hand calculations.

### E2 — matched evidence bridge
- Audit the source and replicate hierarchy of available literature datasets.
- Reproduce the recent field/laboratory N₂O synthesis where licensed data support it.
- Harmonize method, soil, material, crop, nutrient regime and forcing fields.
- Freeze study/source/site/batch splits before modeling.

### E3 — independent field qualification design
- Co-design with South African agronomy, soil-science and laboratory partners.
- Confirm the legal category and trial permissions for the actual formulation and proposed claims.
- Preregister a power/precision-based design and event-aware measurement schedule.
- Proceed to physical activity only after written approvals, competent sampling plans, landholder/community permissions and stop-work protocols.

## 12. South African safety and regulatory boundary

South Africa's Agriculture Inputs Control Directorate administers agricultural-input oversight under Act 36 of 1947. Government guidance states that Group 2 fertilizers must be registered to manufacture or sell and require an accredited-laboratory certificate of analysis. The June 2019 [Group 3 fertilizer guideline](https://www.dlrrd.gov.za/images/Branches/AgricProducHealthFoodSafety/agriculture-inputs-control/guidelines/guidelines-for-registration-of-group-3-fertilizers.pdf) describes evidence of efficacy and no harmful/detrimental soil or plant effects, requires suitably controlled and replicated scientific investigations for applicable claims, and specifies at least three trials in total to indicate an efficacy trend when a whole crop group is claimed. The guideline's applicability depends on the actual category and claim; it is not a universal prescription for every research study. Classification depends on the actual material, ingredients, intended use and claims.

Before any physical manufacturing, trial, distribution or application:
- obtain written direction from the Registrar on applicable classification, registration and trial requirements;
- obtain relevant environmental, occupational, institutional and site approvals;
- use a qualified agronomist and competent/accredited laboratory to approve material characterization and trial sampling;
- ensure landholder and community consent, safe handling, incident reporting and lawful disposal;
- quarantine material where required characterization is missing, inconsistent or fails review.

No model, schema, digest, synthetic pass, open-source repository or carbon methodology replaces these controls.

## 13. Sources

- Wang et al. (2026-11-15), *From laboratory evidence to field-realized N₂O mitigation by biochar: a global meta-analysis and mechanistic synthesis*. [DOI: 10.1016/j.jclepro.2026.149851](https://doi.org/10.1016/j.jclepro.2026.149851)
- Dissanayake et al. (2026-08-06), *Applying Biochar With Fertilisers Decreases Nitrogen and Phosphorus Leaching and Improves Phosphorus Availability: A Meta-Analysis*. [DOI: 10.1111/gcbb.70165](https://doi.org/10.1111/gcbb.70165)
- Jiang, Huang & Meng (2025), *Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments*. [DOI: 10.1016/j.geoderma.2025.117318](https://doi.org/10.1016/j.geoderma.2025.117318)
- Puro.earth, [Biochar Certification Framework, Edition 2025 v2](https://puro.earth/cdr-infrastructure/methodologies/).
- European Commission, [Commission Delegated Regulation (EU) 2026/285](https://eur-lex.europa.eu/eli/reg_del/2026/285/oj).
- South African Government, [Register a Group 2 fertilizer](https://www.gov.za/services/fertilizers-farm-feeds-agricultural-remedies/register-group-2-fertilizer).
- South African Department of Agriculture, [Agriculture Inputs Control](https://www.nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control).

---

**Release rule:** A passing schema or synthetic test qualifies only the declared software behavior. It does not qualify a measurement instrument, material batch, soil response, field outcome, net carbon removal, product, regulatory status or carbon credit.
