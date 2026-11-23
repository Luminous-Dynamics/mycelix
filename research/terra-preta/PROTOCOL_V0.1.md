# Mycelix Terra Preta Research Protocol v0.1

**Status:** Open research protocol draft; not a product specification, fertilizer recommendation, or authorization to manufacture, distribute, or apply a material.  
**Version date:** 2026-10-29  
**Proposed public research-protocol release:** 2026-11-09  
**Stewardship:** Luminous Dynamics / Mycelix, with independent agronomy, laboratory, and regulatory review required before physical trials.

## 1. Purpose

Build a reproducible evidence pipeline for determining when a locally sourced biochar/compost soil amendment is safe, useful, affordable, and appropriate for a specified South African soil–crop–climate context.

The initiative is not trying to prove that “terra preta works” in general. It is testing narrower, falsifiable claims about a precisely identified material, feedstock, manufacturing process, soil, crop, dose, and use condition.

The historical term *terra preta* describes complex anthropogenic soils; it is not a standardized commercial formulation. Project records must therefore use a formulation identifier and measured properties, never the label alone as evidence of equivalence.

## 2. Safety, scope, and release boundary

This version publishes research methods and record formats only. It does not authorize public recipes, field application, product claims, free samples, or sales.

Before any physical material is manufactured for trial, distributed, or applied:
1. Obtain written advice from South Africa's Agriculture Inputs Control directorate on the applicable category, registration status, trial requirements, labels, and permitted claims for the exact formulation and proposed activities.
2. Obtain appropriate environmental, occupational, biosafety, and institutional approvals for the actual process and site.
3. Have a qualified agronomist and accredited/appropriately competent laboratory approve a pre-trial sampling and characterization plan.
4. Establish a documented stop-work and incident-reporting process.
5. Confirm that all trial locations, landholder permissions, crop uses, and disposal arrangements are authorized.

South African government guidance says Group 2 fertilizers require registration to manufacture or sell and an accredited-laboratory certificate of analysis. The current Agriculture Inputs Control directorate also publishes Group 3 registration guidance. The correct category for a biochar/compost product cannot be inferred from its name alone; classification must be requested for the actual formulation and claims.

**No AI model, simulation, Mycelix record, open-source license, or disclaimer can establish product safety or replace required legal approvals.**

## 3. Research questions and preregistered hypotheses

Each study must publish a timestamped protocol before outcomes are inspected.

- **H1 — soil-specific physical response:** A characterized amendment can alter water-retention and/or infiltration measurements in at least some target soils; the sign and magnitude may vary by soil and material.
- **H2 — heterogeneous agronomic response:** Treatment effects vary with measurable soil properties, feedstock/material properties, crop, climate, water regime, and time. A universal effect size is not assumed.
- **H3 — mixture interaction:** A biochar-plus-compost treatment may differ from the sum of single-component effects. This interaction must be estimated, not presumed positive.
- **H4 — predictive generalization:** A model evaluated on held-out batches and sites can outperform transparent baseline models on preregistered metrics, with calibrated uncertainty.
- **H5 — decision value:** Active selection of the next approved experiment reduces uncertainty or decision regret per unit cost versus a fixed experimental schedule, without bypassing safety gates.

Null, adverse, and inconclusive results are first-class results and must be published alongside favorable ones.

## 4. Initial material scope

To keep the first study interpretable, begin with **one traceable, untreated woody-residue feedstock stream**, selected with a local agronomy/laboratory partner based on lawful and sustainable availability. This is a research-design default, not a claim that woody feedstock is universally optimal. No live-tree harvesting, treated/painted timber, mixed unknown waste, sewage sludge, or uncharacterized feedstock is in scope for the initial study.

The exact feedstock identity, source, handling, conversion process, process parameters, and resulting material properties must be recorded before the material is treated as a distinct candidate. Any change in feedstock source or process that could alter properties creates a new batch and may require a new formulation version.

Do not publish a universal production recipe or application rate in v0.1. Those decisions depend on material characterization, the target soil/crop, regulatory advice, and qualified review.

## 5. Study sequence and gates

### Gate 0 — protocol and permissions

- Select one target production region, soil class, crop, and decision question with a local partner.
- Preregister hypotheses, outcomes, exclusion rules, analysis, sample handling, and power calculation.
- Obtain written regulatory classification and required approvals before producing or applying trial material.
- Define a conventional agronomic reference appropriate to the crop and site.

**Stop:** No physical testing/application until the required approvals and safety plan are in place.

### Gate 1 — material characterization

For every independent batch, obtain a representative sample and use qualified laboratories and validated methods appropriate to the material. The plan should consider, as applicable:

- feedstock provenance and processing history;
- moisture, ash, bulk density, pH, electrical conductivity/salts, and carbon;
- total and plant-available nutrients relevant to intended claims;
- potentially harmful elements;
- PAHs and other process-related contaminants where relevant to feedstock/process;
- maturity, phytotoxicity, and pathogen indicators where compost/manure or biological materials are involved;
- uncertainty, method, detection/quantification limits, laboratory identity, and certificate identifier.

The test panel and thresholds must be set with the applicable South African requirements and qualified specialists. Missing values are recorded as missing; they must never be silently treated as safe or as zero.

**Stop:** Any failed required threshold, unknown critical contaminant result, sample-integrity problem, or unexplained batch deviation quarantines the batch pending independent review.

### Gate 2 — controlled laboratory/pot evaluation

Use randomized allocation, blocking where needed, blinded sample IDs where feasible, a predeclared primary endpoint, and an analysis plan fixed before unblinding.

The minimum comparison structure is:
- untreated/reference soil;
- locally appropriate conventional agronomic practice, where needed to contextualize a crop claim;
- compost-only;
- characterized biochar-only;
- the corresponding biochar-plus-compost treatment.

This is a treatment template, not an instruction to apply uncharacterized material. Application levels are set only after Gate 1 and qualified review. If nutrient replacement is claimed, include a suitable nutrient-matched comparator and account for nutrients supplied by each treatment. If water retention is the primary claim, predefine the water-retention range and measurement methods.

Use a power analysis based on a defensible minimum detectable effect and expected variance to set sample size; do not choose replicate counts after viewing results.

**Stop:** Predefined phytotoxicity, plant-health, contamination, or other safety triggers pause the trial and activate the incident plan.

### Time dependence and aging

Biochar treatment effects may change with material properties, soil texture, and time since application. A 2025 synthesis of field studies reported that several soil hydraulic-property effects peaked around the second year and subsequently declined, with variation by feedstock and soil texture. This is a reason to measure duration explicitly, not a basis for assuming that a new formulation will have the same response. [Jiang, Huang & Meng (2025)](https://doi.org/10.1016/j.geoderma.2025.117318).

For any trial intended to support a persistence or long-term claim:
- collect and document baseline measurements before treatment;
- predeclare follow-up times appropriate to the target outcome and crop season;
- retain time-since-application, batch age, and observation date as separate fields;
- report each planned follow-up and all missing follow-ups, including null/adverse observations;
- do not infer multiyear persistence from an initial season or from simulation alone.

The follow-up schedule must be designed with the local agronomy partner and available resources. This protocol does not prescribe an application rate or imply that a multi-year trial is necessary for every narrower laboratory question.

### Gate 3 — field trials

Only after the previous gates pass and all required approvals are obtained. Use randomized blocks or another justified design; record weather, irrigation, management, soil baseline, crop variety, planting density, and deviations. Start with a narrowly scoped pilot and independent oversight before considering multi-site trials.

No public crop-yield, fertilizer-substitution, carbon-removal, water-saving, or economic claim is permitted from simulation alone or from a single unreplicated trial.

### Gate 4 — independent reproduction and regulatory review

At least one independent reviewer must receive the full protocol, analysis code, data dictionary, and permitted raw/de-identified data needed to reproduce the result. Batch and site generalization must be evaluated explicitly. Commercialization is a separate decision requiring all applicable legal approvals, product-quality controls, labeling, and evidence requirements.

## 6. Model architecture: physics before optimization

### Symtropy — transparent physical reference model

Start with a modest, testable water-balance and state-transition kernel, not a claimed complete soil simulator. State variables and units must be explicit. Each time-step must account for water inputs, outputs, and storage change. Add soil-water retention/hydraulic constitutive models only when the necessary parameters and independent validation data exist.

Then extend incrementally to:
1. heat and water transport;
2. dissolved nutrient transport and sorption/desorption;
3. carbon pools and decomposition;
4. plant uptake and water/nutrient stress;
5. biological processes only where measurable parameters and validation support them.

Every subsystem must identify which terms are conservation laws, established constitutive models, fitted parameters, and unvalidated hypotheses. Use published/recognized equations and compare against established reference software where assumptions and boundary conditions align. Do not claim equivalence to HYDRUS, SWAP, or another reference until a matched benchmark supports it.

### Symthaea — uncertainty-aware experiment planner

Symthaea may rank *approved* experiments by expected information gain, expected reduction in decision regret, cost, and relevance to a defined decision. It must preserve parameter uncertainty and distinguish in-domain predictions from extrapolation.

A model must be allowed to answer **“insufficient evidence”**. No single score or cognitive-agent output can override a physical constraint, an independent safety failure, or a regulatory gate.

### Mycelix — evidence and provenance fabric

Connect each claim to a versioned chain of evidence:
- unique material/formulation and batch IDs;
- feedstock source and process record;
- sample custody and lab certificate;
- pre-registered protocol and analysis commit;
- input dataset hashes and access/licensing conditions;
- simulation version/configuration/seeds;
- deviations, negative outcomes, adverse events, and reviewer decisions;
- claim status and expiry/review date.

Hashes support integrity; they do not prove that a sample was representative, a laboratory competent, a measurement correct, or a result independently reproduced. Store the actual accessible evidence where law and consent permit, and preserve clear links to the source of truth.

## 7. Evidence record and claim states

Every outward-facing claim must be attached to one of these explicit states:

- **Proposed:** hypothesis only; not evidence.
- **Simulated:** output from a named model/configuration; not a safety or efficacy result.
- **Lab-characterized:** named batch has specific measured properties and associated methods/certificates.
- **Controlled-trial observed:** result for the stated protocol, batch, soil, crop, site, and period.
- **Independently reproduced:** independent party reproduced a predefined result.
- **Regulator-reviewed/registered:** the precise legal status is linked to the relevant official record. Never use this state without documentary support.
- **Superseded/withdrawn:** prior claim no longer supported, or an error/incident requires correction.

Each claim record should state its population and conditions, effect estimate, uncertainty interval, sample size, missingness, adverse results, baseline comparator, model/data versions, reviewer, and limitations.

## 8. Operational responsibility

Assign a named accountable lead for each physical trial, laboratory relationship, regulatory interaction, dataset, and public claim. Mycelix is an audit and coordination layer; decentralization does not dissolve operator duties. Keep commercial/marketing decisions separate from scientific review. Publish corrections and adverse findings with the same visibility as positive findings.

## 9. v0.1 acceptance criteria

The protocol is ready for public research discussion when it provides:
- a preregistration template and explicit safety gates;
- a repeatable batch/sample/measurement record;
- a plan for grouped out-of-sample evaluation;
- clear separation of simulation, lab evidence, trial evidence, and regulatory status;
- no universal recipe, application rate, safety claim, or crop-performance guarantee;
- source links and a changelog.

**Not an acceptance criterion:** a positive crop response. A null result can be scientifically successful if the protocol is followed and results are reproducible.

## 10. Selected references

- South African Government, [Register a Group 2 fertilizer](https://www.gov.za/services/fertilizers-farm-feeds-agricultural-remedies/register-group-2-fertilizer).
- South African Department of Agriculture, [Agriculture Inputs Control](https://nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control).
- South African Department of Agriculture, [Guidelines for registration of Group 3 fertilizers](https://www.dlrrd.gov.za/images/Branches/AgricProducHealthFoodSafety/agriculture-inputs-control/guidelines/guidelines-for-registration-of-group-3-fertilizers.pdf).
- European Biochar Certificate, [Guidelines and analytical methods](https://www.european-biochar.org/en/certificate).
- International Biochar Initiative, [Biochar standards](https://biochar-international.org/biochar-standards/).
- Bekchanova et al., “Biochar improves the nutrient cycle in sandy-textured soils and increases crop yield: a systematic review,” *Environmental Evidence* 13, 3 (2024), [open article](https://doi.org/10.1186/s13750-024-00326-5).
- “Maximizing crop yield and water productivity through biochar application: A global synthesis of field experiments,” *Agricultural Water Management* (2024), [DOI](https://doi.org/10.1016/j.agwat.2024.109134).
- “Long-term effects of biochar on the hydraulic properties of soil: A meta-analysis based on 1–10 years field experiments,” *Geoderma* (2025), [DOI](https://doi.org/10.1016/j.geoderma.2025.117318).

## Changelog

- **v0.1 (2026-10-29):** Initial evidence-first protocol. No physical formulation or efficacy claim is approved by this document.
