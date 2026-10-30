# Terra Preta Trial Design Template v0.1

**Status:** Research planning template only. This is not a formulation, application instruction, fertilizer recommendation, or authorization to run a physical trial.  
**Use with:** [Protocol v0.1](./PROTOCOL_V0.1.md), [Benchmark v0.1](./BENCHMARK_V0.1.md), and [contrast-record.schema.json](./contrast-record.schema.json).

## 1. Study identity

- Study ID:
- Protocol version and commit:
- Responsible agronomist / investigator:
- Institution and site permissions:
- Regulatory classification and authorization references:
- Laboratory plan and sample custody lead:
- Proposed crop, cultivar, season, and site:
- Target soil class and baseline characterization:
- Research question:
- Study type: laboratory / pot / greenhouse / field / other:
- Study status: planned / approvals pending / approved / in progress / paused / complete / withdrawn:

Do not mark a study approved until all requirements applicable to the actual site, material, trial, and distribution plan are documented. A protocol document or model output is not an authorization.

## 2. State the estimand before choosing treatments

Choose one primary question. Do not combine these claims into one outcome.

### A. Soil-water response

Example estimand: mean difference in a preregistered water-retention or plant-available-water endpoint between a characterized material and a comparator under the same soil, management, and nutrient program.

- Primary endpoint and unit:
- Measurement method and reference:
- Soil depth / sampling domain:
- Matric-potential interval or other scientifically justified measurement domain:
- Treatment and comparator material identities:
- Follow-up times:
- Minimum meaningful effect, justified before data collection:
- Primary effect estimate and uncertainty method:

### B. Incremental biochar effect

Example estimand: effect of adding a characterized biochar while keeping the nutrient program and other management equivalent between arms.

- Biochar contrast:
- Nutrient program held constant:
- Other amendments held constant:
- Primary endpoint:
- Source of the assumed comparability:
- Known remaining confounders:

### C. Fertilizer substitution / nutrient reduction

Example estimand: difference in a specified crop/soil outcome between a predeclared nutrient-reduction strategy and the locally appropriate conventional nutrient program, with or without a characterized amendment.

This is a separate and more demanding claim than improved soil moisture. The design must compare nutrient programs explicitly, account for nutrient contributions from all amendment ingredients, and use a comparator appropriate to the substitution claim.

- Locally appropriate standard program and source:
- Alternative nutrient program(s) and agronomic rationale:
- Nutrient contributions from amendment(s), measured or unknown:
- Non-inferiority / superiority margin and justification:
- Outcomes required to judge performance and downside risk:
- Conditions under which substitution is not identifiable:

### D. Mixture / interaction effect

If biochar, compost, and nutrient programs are all varied, use a factorial structure when feasible so their main effects and interactions can be distinguished. A mixture beating an untreated control does not by itself show that its components interact beneficially.

- Factors and levels:
- Main effects to estimate:
- Predeclared interactions:
- Contrast matrix:
- Interaction effects that this design cannot identify:

## 3. Treatment structure

List treatment arms by opaque IDs before randomization. Complete this table only after material characterization, regulatory advice, and qualified agronomic review.

| Arm ID | Biochar factor | Compost/manure factor | Nutrient program | Other management | Primary role |
|---|---|---|---|---|---|
| T01 | Defined level | Defined level | Defined level | Held constant / specified | Primary comparator |
| T02 | Defined level | Defined level | Defined level | Held constant / specified | Primary treatment |
| T03 | Defined level, if required | Defined level, if required | Defined level, if required | Held constant / specified | Mechanism / interaction |
| ... | ... | ... | ... | ... | ... |

Factors and levels are research-design variables, not application instructions. Actual rates and material identities require separate approval and justification. Avoid testing a large factorial matrix without a clear estimand and sufficient independent experimental units.

**Comparator discipline**
- For an incremental biochar question, match nutrient inputs and other management wherever scientifically possible.
- For a substitution claim, include the locally appropriate conventional nutrient program and a clearly defined alternative program.
- If nutrient contributions from the amendment are unknown, the substitution estimand may not be identifiable.
- Do not use a zero-input or untreated arm merely by convention; justify its scientific relevance, feasibility, and appropriateness for the crop/site.
- Record all treatment components and control-arm details with the contrast schema.

## 4. Experimental unit, blocks, and randomization

- Experimental unit (plot, pot, mesocosm, or other):
- Why this is the correct unit for treatment assignment:
- Block definition and blocking variables:
- Site layout / spatial gradient:
- Buffer and cross-contamination controls:
- Randomization algorithm/version:
- Random seed:
- Allocation manifest hash:
- Person who generated allocation:
- Person who implemented allocation:
- Masking/blinding of sample IDs and outcome assessors:
- Deviations and re-randomization rules:

Use a randomized block design when material spatial gradients (such as soil texture, slope, drainage, or irrigation) justify it. All planned treatments must be represented as required within each block, subject to the power and design analysis. Pot/soil-core subsamples are not independent experimental replicates of a plot; record the nesting explicitly.

Never treat sensor timepoints, plant subsamples, or repeated measurements from a single experimental unit as independent replicates. Preserve randomization assignments before outcomes are observed.

## 5. Sample-size and power justification

Do not inherit a conventional replicate count from a published study without checking whether it can answer this study's primary question.

- Primary endpoint:
- Target estimand:
- Minimum scientifically meaningful effect:
- Variance / intraclass correlation source:
- Pilot or literature source and limitations:
- Significance level / interval target and multiplicity plan:
- Desired statistical power:
- Number of independent experimental units per arm:
- Block/site/season structure:
- Attrition and missing-data assumptions:
- Analysis method:
- Sensitivity analysis:
- Approval by qualified statistician/agronomist:

If variance, effect-size, or clustering information is insufficient, state that the power calculation is provisional and collect pilot information under appropriate approvals before a confirmatory trial. Do not claim significance or adequate power merely because a study has a familiar number of replicates.

## 5.1 Matched-forcing event design

For event-scale hydraulic or nutrient effects, use the companion
[Paired-Forcing Causal Benchmark](./PAIRED_FORCING_CAUSAL_BENCHMARK_V0.1.md).
Predefine randomized unit, block, matched hyetograph or plot-level rainfall
measurements, antecedent water state, repeated-event structure and event-mass
estimand. Storm events, depths, sensors and time points are not independent
treatment replicates.

## 6. Measurement plan and data quality

For every endpoint, define units, method, sampling schedule, instrument/lab, calibration, limits of detection/quantification when relevant, quality controls, and the rule for failed or missing measurements.

Candidate endpoint families, selected according to the actual question:
- soil-water retention at defined potentials, infiltration, drainage, soil moisture by depth, and irrigation volume;
- crop emergence, plant-health and phytotoxicity indicators, biomass, nutrient status, and marketable yield;
- soil pH, electrical conductivity, relevant nutrient forms, bulk density, texture, organic carbon, and other predeclared soil properties;
- nutrient leaching/runoff indicators where suitable methods and environmental permissions exist;
- economics, labor, transport, processing energy, and input cost;
- longer-term observations required for an explicit persistence claim.

Record pre-treatment baseline, every planned follow-up, actual timestamp, weather/irrigation/management, protocol deviations, and missingness. Do not reinterpret an endpoint after viewing treatment results without labeling the analysis exploratory.

## 7. Predeclared analysis

- Primary endpoint and one primary contrast:
- Secondary endpoints:
- Main effects and interaction terms:
- Model formula / hierarchy:
- Experimental unit and blocking structure:
- Missing-data and exclusion policy:
- Multiple-comparison control:
- Effect sizes and uncertainty intervals:
- Negative/null/adverse-result publication plan:
- Sensitivity analysis and outlier rules:
- Independent reviewer:
- Code commit and environment lock:
- Data dictionary and dataset digest:
- Planned analysis freeze date:

Report effect estimates and uncertainty, not only p-values or percent improvements. Report negative outcomes and all preregistered endpoints. Keep confirmatory and exploratory analyses visibly distinct.

## 8. Longitudinal and transfer plan

- Time zero definition:
- Follow-up horizons:
- Material/batch age:
- Seasonal and crop-stage markers:
- Expected changes from aging or soil movement:
- Site/soil transfer test:
- Forecast horizon and extrapolation boundary:
- Rule for reporting insufficient evidence:

An initial-season response cannot establish multi-year persistence. Do not pool lab, pot, and field results as if they were exchangeable. The analysis should explicitly represent study type, soil, batch, and time.

## 9. Stop rules and safety

Set thresholds and responsible authorities before the trial. The exact criteria depend on material composition and applicable requirements.

- Required laboratory characterization complete:
- Required contaminant/biological safety checks:
- Regulatory classification/authorization:
- Phytotoxicity / adverse plant-health trigger:
- Human, animal, environmental, or worker incident trigger:
- Sample contamination or chain-of-custody breach:
- Unexpected material/batch deviation:
- Stop-work authority:
- Notification and incident record:
- Quarantine and disposal plan:

A simulation, randomized design, provenance hash, disclaimer, or trial registration does not establish material safety. A failed required gate blocks trial progression; do not trade a safety failure against yield or cost outcomes.

## 10. Release decision

Choose one state supported by documentary evidence:

- **Protocol-only:** publication of study plan; no physical-use authorization implied.
- **Characterization in progress:** no field-use or efficacy claim.
- **Approved limited study:** only the specific study/activity allowed by relevant approvals.
- **Study completed — evidence provisional:** outcomes reported for this site/batch/design only.
- **Independent reproduction complete:** defined findings reproduced under a separate, documented protocol.
- **Regulatory status verified:** only for the precise product, purpose, and activity documented by the relevant authority.
- **Stopped / withdrawn / superseded:** show the reason and preserve all prior data and claims.

## References and design rationale

- Mostafa et al. (2025), “Mesoporous biochar reshapes soil water dynamics under shallow groundwater: interactions with nitrogen management,” *Frontiers in Soil Science*, [DOI: 10.3389/fsoil.2025.1718929](https://doi.org/10.3389/fsoil.2025.1718929). Illustrates a randomized block design with biochar and N-rate factors; its results include changed irrigation demand without statistically significant crop-yield/quality differences, reinforcing the need for separate endpoints. It does not supply a universally transferable rate or a design size for South Africa.
- Jiang, Huang & Meng (2025), “Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments,” *Geoderma*, [DOI: 10.1016/j.geoderma.2025.117318](https://doi.org/10.1016/j.geoderma.2025.117318). Supports explicit time, feedstock, and soil-texture stratification rather than a universal persistence assumption.
- [South African Agriculture Inputs Control](https://nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control). Confirm requirements for the exact material, claims, distribution, and study activity before any physical trial.
