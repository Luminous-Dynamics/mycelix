# Terra Preta Benchmark v0.1

**Status:** Proposed benchmark contract; target values below are acceptance goals, not achieved results.  
**Pairs with:** [Protocol v0.1](./PROTOCOL_V0.1.md)

**Specialized benchmark tracks:** [Preferential-flow transfer](./PREFERENTIAL_FLOW_TRANSFER_BENCHMARK_V0.1.md) · [Synthetic known-truth and identifiability](./SYNTHETIC_KNOWN_TRUTH_BENCHMARK_V0.1.md) · [Industry physics comparators](./INDUSTRY_PHYSICS_COMPARATOR_BENCHMARK_V0.1.md) · [Paired-forcing causal inference](./PAIRED_FORCING_CAUSAL_BENCHMARK_V0.1.md) · [Multi-method PF triangulation](./PREFERENTIAL_FLOW_MULTIMETHOD_TRIANGULATION_BENCHMARK_V0.1.md) · [Profile-level NSR audit](./tools/profile-flow-audit/README.md) · [PF threshold sensitivity audit](./tools/pf-threshold-audit/README.md) · [NEON preprocessing contract](./NEON_PREFERENTIAL_FLOW_PREPROCESSING_CONTRACT_V0.1.md) · [Trajectory-signature audit](./tools/trajectory-signature-audit/README.md) · [Evidence triangulation audit](./tools/pf-triangulation-audit/README.md) · [Profile NSR audit](./tools/profile-flow-audit/README.md) · [Dynamic structure/root benchmark](./DYNAMIC_STRUCTURE_ROOT_ARCHITECTURE_BENCHMARK_V0.1.md) · [HydroShare PF derived database](./ACQUISITION_V0.1.md#published-neon-preferential-flow-event-database-hydroshare-v1-1)

## 1. Benchmark objective

Evaluate whether a transparent soil-amendment model and evidence pipeline can make more accurate, better-calibrated, more useful decisions than simple and established baselines on *held-out batches, soils, sites, and seasons*.

The benchmark does **not** rank formulations by predicted crop yield alone. It has separate gates for software correctness, physical accounting, predictive accuracy, uncertainty calibration, treatment-effect estimation, economics, and safety. Safety and regulatory failures are vetoes, never tradeable against higher yield or lower cost.

## 2. Evaluation tracks

### Track A — deterministic accounting (synthetic tests first)

Build a reference water-storage model with explicit units and boundary fluxes before adding coupled processes.

For a control volume over a time step:

\[
\Delta S = P + I - ET - R - D
\]

where storage change \(\Delta S\), precipitation \(P\), irrigation \(I\), evapotranspiration \(ET\), runoff \(R\), and drainage \(D\) are all expressed as equivalent water depth over the same area and time interval. The equation is an accounting identity, not a claim that each flux is accurately predicted.

**Required tests**
- zero-input/zero-output: storage remains unchanged;
- rainfall/irrigation only: storage increases by exactly the accounted input until constrained by the chosen model;
- drainage/evaporation only: storage decreases by accounted outflow without becoming negative;
- unit conversion and area/time scaling invariance;
- timestep refinement does not create or destroy water;
- invalid units, negative physical quantities (unless explicitly meaningful as signed fluxes), non-finite values, and impossible state values fail closed;
- replaying the same version, configuration, seed, and inputs gives deterministic output within a documented numerical tolerance.

**Provisional software gate:** synthetic mass-balance residual within \(10^{-9}\) relative scale for well-conditioned double-precision cases, or a stricter/looser explicitly justified tolerance for the solver used. This is a numerical test, not a permissible field-measurement error.

### Track B — laboratory-property prediction

Predict only properties that have reliable paired inputs and measured labels. Report MAE and RMSE in the property's physical units, bias, performance by property range, and uncertainty calibration. For positive, non-near-zero quantities, also report relative error. Do not hide poor performance behind a combined score.

### Track C — soil-water behavior

For causal amendment effects on event response, use the
[Paired-Forcing Causal Benchmark](./PAIRED_FORCING_CAUSAL_BENCHMARK_V0.1.md).
Predictive preferential-flow performance is not treatment-effect evidence.



The comparator hierarchy and parity rules are specified in
[Industry and Physics-Based Comparator Benchmark](./INDUSTRY_PHYSICS_COMPARATOR_BENCHMARK_V0.1.md).
Industry and process-model comparisons must identify exact executable,
module, configuration, parameter source and boundary conditions; a generic
"HYDRUS" or "industry baseline" label is not adequate.



For event-scale preferential-flow reproduction and site transfer, use the
[preferential-flow transfer benchmark](./PREFERENTIAL_FLOW_TRANSFER_BENCHMARK_V0.1.md)
and its [NEON source plan](./neon-preferential-flow-source-plan.v0.1.json).
This is a distinct evidence track from amendment-response estimation: NEON
hydrology data do not supply biochar treatment labels or nutrient-export labels.



For event-scale water and nutrient transport, use the companion
[coupled hydrology–nutrient benchmark](./COUPLED_HYDROLOGY_NUTRIENT_TRANSPORT_BENCHMARK_V0.1.md)
and its [event-record schema](./tools/coupled-event-audit/README.md). A
retention-curve fit alone does not validate transient water or nutrient export.



Use measured water-retention curves, infiltration, drainage, and soil moisture where available. Stratify by soil class and experimental setup. Before scoring any model, audit the underlying desorption curves with the [water-retention metrology contract](./WATER_RETENTION_METROLOGY_V0.1.md) and [Rust curve auditor](./tools/water-retention-audit/README.md). Require explicit water-content basis, density-backed gravimetric conversion, declared FC/PWP potentials, raw input digests, observed brackets for every target, and zero silent smoothing or extrapolation. Evaluate time-series error and event-level error, not just a pooled correlation.

Initial aspirational target for a *predefined, well-supported domain*: median absolute relative error no greater than 10% for selected water-retention measurements and 90% prediction-interval coverage between 85% and 95%. These are development targets, not claims of current performance. The team must publish the sample size, properties/units, domain, reference methods, baseline scores, and confidence intervals; if the target is not attainable on the available data, report the failure and revise only prospectively.

For values near zero or across mixed units, use absolute/normalized errors and do not rely on percentage error.

### Track D — crop response and treatment effects

Primary outcomes must be predeclared with agronomists for each crop/site. Candidates include emergence/germination, phytotoxicity indicators, biomass, marketable yield, plant nutrient status, water productivity, and post-trial soil properties.

Report treatment effects against a matched comparator with uncertainty intervals and all relevant null/adverse results. Split by entire study/batch/site/season groups, not random rows from the same experiment. Use cluster-aware uncertainty and avoid treating subsamples as independent experimental units.

A model passes only if it improves on a transparent baseline on held-out data and uncertainty is sufficiently calibrated for the decision at hand. No global yield-accuracy target is set until a dataset audit establishes the number and diversity of independent observations.

**Provisional superiority target (not achieved):** for a preregistered primary endpoint, aim for at least a 10% relative reduction in MAE versus the strongest valid baseline on fully held-out groups, with a 95% cluster-bootstrap interval for improvement that excludes zero. The analysis plan must require an adequate number of independent study/site/batch clusters. If the dataset cannot support that inference, report the comparison as inconclusive rather than lowering the bar after seeing results.

### Track E — decisions and economics

Report, where data support it:
- cost per hectare and cost per unit of marketable yield;
- gross-margin or net-benefit distributions under uncertain prices and yields;
- water productivity and nutrient-use indicators with clear denominators;
- sensitivity to transport distance, feedstock opportunity cost, processing energy, application, and labor;
- downside risk, not only mean outcome;
- comparison against farmer practice and a suitable conventional nutrient program.

A formulation must not be described as “replacing fertilizer” unless nutrient supply and crop response have been tested against an appropriate nutrient-matched reference, for the stated crop and conditions.

### Track F — active-learning value

Compare Symthaea's constrained experiment ranking against:
1. random selection from the same approved candidate set;
2. a fixed, expert-designed sequence;
3. a simple uncertainty-based greedy baseline.

Metrics: reduction in uncertainty on preregistered target parameters, decision regret, time/cost to reach a predefined evidence threshold, calibration, and the number of unsafe/disallowed candidates proposed. The target for unsafe/disallowed candidates is exactly zero after the independent safety gate. Include repeated seeds and confidence intervals.

**Provisional efficiency target (not achieved):** at least 20% lower total cost to reach a preregistered uncertainty or decision threshold versus the better of the fixed expert-designed and random schedules, evaluated over repeated seeds with confidence intervals. Lower runtime alone does not count if laboratory, field, review, or safety costs increase. Any proposed candidate bypassing the safety gate is an automatic failure.

### Track G — longitudinal aging and transfer

Biochar effects are time-dependent. A 2025 meta-analysis of 61 field studies (observations across 1–10 years) reported that available water capacity effects varied with time, with some hydraulic effects peaking around year two and declining thereafter; effect direction and persistence also varied with feedstock and soil texture. This finding motivates a longitudinal benchmark, but it does **not** justify transferring a published application rate or effect size to a new material or South African site. [Jiang, Huang & Meng (2025)](https://doi.org/10.1016/j.geoderma.2025.117318).

Requirements:
- Record time since application / material age, observation date, soil texture, feedstock class, process conditions, and trial duration when available.
- Separate immediate response, within-season response, and later-season/annual outcomes; do not pool these into a single undifferentiated target.
- Compare a static model against a time-aware baseline on grouped held-out studies and, where possible, leave-later-time-period-out evaluation.
- Test whether uncertainty widens appropriately when forecasting beyond observed follow-up.
- If longitudinal evidence is absent, the system must not make claims about persistence, durability, or long-term performance.

**Provisional target (not yet measured):** a time-aware model should improve a preregistered longitudinal endpoint against a static baseline on held-out study/time groups, with calibration reported by follow-up horizon. No numeric global persistence target is set until the available longitudinal data and endpoint definitions are audited.

## 3. Estimand discipline: predict outcomes separately from treatment effects

The primary scientific target should be a *predeclared contrast*, not just a raw crop-yield prediction. A model can predict yield accurately while still estimating the effect of an amendment incorrectly.

For a positive outcome with compatible treatment and comparator measurements, a candidate study-level effect is the log response ratio:

`lnRR = ln(mean_treatment / mean_comparator)`

When outcomes may be zero/negative or the ratio is scientifically inappropriate, predeclare an absolute or standardized difference instead. Use reported sample sizes and variance estimates when available; do not fabricate precision when variance data are missing.

Every contrast must identify the comparator and all treatment components. For example, a biochar-plus-NPK result versus untreated soil does **not** isolate the incremental value of biochar and does not establish that biochar can replace NPK. Such a claim requires a suitable comparator with a matched nutrient program and otherwise comparable conditions. If the required contrast is absent, record the claim as non-identifiable from that dataset.

Because literature-extracted outcomes from the same study can be correlated, use a study-aware hierarchical/meta-regression baseline or a defensible cluster-robust alternative. Report effect direction and magnitude by feedstock, soil properties, time since application, crop, and treatment context where supported. Use study/site grouped validation and leave-study-out evaluation; do not assume thousands of rows imply thousands of independent experiments.

Treat this public dataset as observational literature synthesis, with possible selective reporting, variable methods, and incomplete covariates. Do not upgrade an association to a causal claim just because a model predicts it. Publish evidence provenance, missingness, sensitivity analyses, and the conditions under which the estimand cannot be identified.

## 4. Required baselines

At minimum, compare to:
- **No-change / treatment-effect-zero baseline:** predicts no amendment effect.
- **Simple empirical baseline:** predeclared linear or mixed-effects model using only available covariates.
- **Persistence baseline:** for time-series values, predicts the last observed value where that is a valid comparator.
- **Established domain model:** such as HYDRUS-1D or SWAP for a specifically matched soil-water problem, only when assumptions, inputs, units, and boundary conditions are aligned.
- **Current agronomic practice:** selected with local agronomist/farm partner for crop and location.

Do not claim superiority over a reference implementation if no matched, reproducible comparison was performed. Every baseline must receive the same data split and permitted input information.

## 5. Dataset split and anti-leakage rules

Use grouped splits, with frozen manifests committed before model fitting. Record each split with [split-manifest.schema.json](./split-manifest.schema.json), and independently run [verify-split-manifest.sh](./tools/verify-split-manifest.sh) before treating a split as confirmatory:
- **in-domain holdout:** unseen samples within known soil/site group, if the use case supports it;
- **batch holdout:** whole biochar/compost batches held out;
- **soil holdout:** an entire soil class or source location held out;
- **site/season holdout:** entire sites or growing seasons held out;
- **prospective evaluation:** protocol and model frozen before new data arrive.

The group-list verifier recomputes train/validation/test set disjointness and refuses a `passed` manifest if required review assertions fail. The row-level auditor in [tools/split-audit](./tools/split-audit/README.md) checks whether one reviewed grouping key crosses split labels. For a confirmatory split with multiple dependence dimensions, additionally use the [multi-key dependency auditor](./tools/dependency-audit/README.md): it unions rows connected by any selected publication/study/experiment/site/batch/curve key and rejects connected components that span split labels, including transitive chains across different keys. Both tools are structural checks: neither can prove that identifiers correctly capture related studies, shared sites, repeated observations, or unrecorded source overlap; that still requires a reviewed source crosswalk.

A 2026 review reported potential leakage in 64/64 highly cited biochar ML studies it examined, with 98% using random point-level splits and inflation up to 86% in five reanalyses; those are study findings, not a universal correction factor. Therefore, random-row metrics may be retained only as a diagnostic comparator. Confirmatory rankings must prioritize grouped holdouts with all learned preprocessing, calibration, feature selection, hyperparameter selection, and threshold selection confined to training folds. See [Transportability Benchmark §10](./TRANSPORTABILITY_BENCHMARK_V0.1.md).

No shared sample, repeat measurement, derived feature, or augmentation from a held-out group may enter training. Report both interpolation and extrapolation separately. Results on a random row split alone are insufficient.

## 6. Safety and evidence gate matrix

| Gate | Required evidence | Failure behavior |
| --- | --- | --- |
| Material identity | Traceable feedstock, process, batch, representative sample | Quarantine; no model recommendation for use |
| Required laboratory panel | Current result, method, units, detection limits, acceptance criteria | Block; unknown is not pass |
| Model validity | In-domain flag, uncertainty, baseline comparison | Return **insufficient_evidence** outside validated domain |
| Numerical integrity | Unit tests, balance residuals, deterministic replay | Fail build / reject result |
| Trial authorization | Required written classification/approvals and responsible lead | No physical trial |
| Efficacy | Predeclared, adequately powered and appropriately controlled trial | No efficacy claim |
| Independent review | Reproducible code/data path and reviewer disposition | Claim remains provisional |
| Commercial release | Applicable registration and all relevant quality/legal obligations | Not eligible for sale/distribution |

Exact chemical/biological acceptance thresholds must be specified for the actual formulation by qualified specialists and applicable rules; this benchmark does not invent universal thresholds. Industry certification schemes such as EBC/WBC provide useful quality-management, characterization, and traceability comparators, but their certification must not be presented as proof of site-specific crop efficacy or as a substitute for South African regulatory classification/registration.

## 7. Reporting contract

Every run must report:
- protocol/benchmark version, run ID, timestamp, code commit, config digest, dependency/tool versions, RNG seed;
- dataset version, source citations, license/access restrictions, hashes, group split manifest;
- batch, soil, crop, site, season, and experimental-unit identifiers (pseudonymous where needed);
- metric definitions and units, per-group scores, aggregate scores, confidence intervals, missingness, and exclusions;
- for interval forecasts, nominal coverage level, empirical coverage, mean width, and a preregistered proper interval score (for this v0.1 prediction-audit tool: interval score for nominal 90% intervals, width plus a penalty of 20 times the distance for an out-of-interval miss);
- baseline comparisons and failure cases;
- model applicability domain and uncertainty/calibration results;
- laboratory/approval gate references and status, without treating attestations as proof of the underlying measurement;
- complete deviation log and whether the run is exploratory or confirmatory.

Do not publish personal or commercially confidential farm data without permission. Publish de-identified data or reproducible aggregation where appropriate; record any restrictions.

## 7A. Preregistered experimental design

Before any confirmatory pot or field study, complete [TRIAL_DESIGN_TEMPLATE_V0.1.md](./TRIAL_DESIGN_TEMPLATE_V0.1.md) and freeze the primary estimand, endpoint, treatment/comparator arms, experimental unit, randomization, block structure, sample-size/power justification, analysis, missing-data rules, and stopping rules. Use a factorial design only when it identifies a predeclared scientific question; do not add factors merely to maximize the number of combinations.

For water-retention research, keep nutrient management and other relevant inputs comparable when estimating the incremental biochar effect. For nutrient-substitution claims, explicitly compare nutrient programs and account for nutrients contributed by the material. If the treatment contrast cannot identify the claimed effect, classify it as non-identifiable rather than repairing the design after the outcome is observed.

Randomize the true experimental unit, preserve the allocation/seed, and model blocks, sites, and repeated observations at their proper levels. A sample-size target must be justified from an effect of scientific interest and credible variance/clustering assumptions; a familiar replicate count is not itself evidence of adequate power. Report water behavior, crop yield/quality, nutrient status, and economic results separately. A null yield response does not erase a real water-management response, nor does a water-management response establish yield improvement or fertilizer substitution.

## 8. Release gates

- **B0 — executable spec:** metric definitions, JSON Schema, split rules, and synthetic test cases accepted.
- **B1 — model sanity:** all synthetic conservation/property tests pass; deterministic replay and invalid-input tests pass.
- **B2 — baseline benchmark:** dataset audit and baseline results published, including failed targets.
- **B3 — independent evaluation:** frozen model tested on a batch/site/season holdout by a reviewer not responsible for model fitting.
- **B4 — decision support:** documented domain of applicability, calibrated uncertainty, safety workflow, and human agronomic review. The output is a *research recommendation for testing*, not permission to use/sell a product.

## 9. Benchmark interpretation

The goal is not to “beat SOTA” on a single aggregate number. It is to beat appropriate baselines on held-out data while maintaining physical consistency, calibrated uncertainty, repeatable provenance, and zero bypass of safety gates. If a simpler model performs just as well, prefer the simpler model until complexity adds demonstrated value.

A positive crop response is not required to pass the scientific benchmark; trustworthy identification of no benefit, harm, or a narrow domain of benefit is valuable.

## 10. Contrast graph and comparator semantics

Every literature observation must be represented as a directed contrast from a precisely described control arm to a precisely described treatment arm. A high-level label such as “biochar alone” or “biochar + fertilizer” is insufficient unless the comparator's own components are known.

For each contrast, create an arm record with the known components and uncertainty:
- biochar present/absent, feedstock/type, dose, and process properties when reported;
- mineral nutrient program (N/P/K and dose) and manure/compost components when reported;
- crop, soil/site, management, application timing, and follow-up;
- measured endpoint, units, sample sizes, variance source, and source citation;
- explicit unknown/ambiguous fields.

Then assign a *contrast class* only when supported by the original paper:
1. **Incremental biochar under matched fertilizer:** treatment and comparator use the same nutrient plan, with biochar as the intended contrast.
2. **Biochar-plus-input package vs no-input/other baseline:** bundled intervention; cannot isolate a biochar effect or establish fertilizer substitution.
3. **Biochar vs untreated control:** estimates that experiment's total contrast, not performance versus best agronomic practice.
4. **Nutrient-matched substitution comparison:** suitable for a specific substitution question only if nutrient input, management, and other relevant conditions are made comparable.
5. **Unclassifiable:** the source does not specify sufficient comparator detail.

The BiocharDS paper describes the CSV as a simplified version containing target information and says the full workbook includes a `Reference` sheet and an `Explanation and unit` sheet. The paper also warns that treatment classification depends on the control treatment. Therefore, if the CSV lacks a reliable source-study key or arm details, the benchmark **must obtain the full workbook and construct a reviewed crosswalk** before training any model that claims study-level generalization or treatment-effect identification. A header heuristic from the audit tool is a discovery aid, not an approved grouping key.

This is a causal-identifiability gate: if the estimand cannot be recovered from the treatment/comparator pair, report `not_identifiable_from_source` rather than filling gaps with a model assumption. Each reviewed contrast should be represented using [contrast-record.schema.json](./contrast-record.schema.json); schema validation is necessary but does not itself establish that the record is scientifically correct.

## 11. Initial dataset strategy and acquisition gates

Start with available external evidence rather than fabricating synthetic labels or waiting for a field trial:

1. **Global biochar treatment-response data:** Li et al.'s BiocharDS_V1.0 dataset is publicly available on [Figshare (CC0 1.0)](https://springernature.figshare.com/articles/dataset/BiocharDS_V1_0_csv/24781764), DOI [10.6084/m9.figshare.24781764.v1](https://doi.org/10.6084/m9.figshare.24781764.v1). The published CSV is reported as 2,439 rows by 209 columns and links observations to 367 source studies; the paper describes 891 independent experiments across crop yield, soil properties, and greenhouse-gas outcomes. The row count is not an independent-sample count. First step: download the dataset and data dictionary, audit study/treatment identifiers, units, missingness, sampling depth, and duplicated/related observations. This is a *literature-extracted effect dataset*, not a substitute for raw, local calibration time series. Split by source study (and by site where identifiable), never random rows.
**Independent synthesis / longitudinal source to reconcile:** the 2025 PNAS meta-analysis [“Sustained benefits of long-term biochar application for food security and climate change mitigation”](https://doi.org/10.1073/PNAS.2509237122) reports 3,229 paired observations from 438 publications and 29 long-term field experiments spanning 4–12 years. Add its study list and time-series evidence to the source audit. Do **not** call it an independent holdout until source-study overlap with BiocharDS_V1.0 is reconciled; overlapping papers, sites, or reused experiments must remain in the same validation group. Use the longer studies to predefine time horizons and quantify how thin the evidence is at each horizon, not to assume a universal persistence curve.

2. **African soil context:** [Soils4Africa / Soil Data of Africa](https://africasis.isric.org/download.html) offers sampled soil properties including texture, bulk density, organic carbon, and chemical properties as CSV/GeoPackage, with regional soil maps. Its own guidance says the recommended application scale is national to continental and discourages field-level use. Use it to stratify hypotheses and help select candidate regions, not as a replacement for soil sampling from a trial plot.
3. **Standardized profiles and gaps:** [WoSIS](https://isric.org/explore/wosis) is a global, standardized soil-profile resource. Coverage is property-dependent and access is subject to source data-provider licenses. Record exactly which properties are measured versus predicted/harmonized; do not treat a map estimate as a laboratory observation.
4. **Crop-water reference model:** FAO [AquaCrop v7.3](https://www.fao.org/aquacrop) is an established crop water-productivity model with source code and standalone versions. Use it as a version-pinned *external comparator* for matched crop/water questions when inputs, calibration, and boundary assumptions align. It is not a biochar-specific effect model and must not become an unexamined production dependency in the Rust kernel.

**Executable audit before modeling:** run the standalone Rust tool at [tools/biochar-audit](./tools/biochar-audit/README.md). It computes a digest of the exact input file and reports shape, missingness, ragged records, exact duplicate rows, ambiguous headers, and candidate study/treatment keys. Validate its JSON output against [audit-report.schema.json](./tools/biochar-audit/audit-report.schema.json). These are structural checks only; they do not approve grouping keys or treatment semantics.

The published paper describes the CSV as a simplified data file and identifies `Reference` and `Explanation and unit` sheets in the full workbook. If the CSV does not provide a reviewed source-study key or does not provide enough treatment/comparator detail, stop the model benchmark, acquire the full workbook, and create a reviewed crosswalk. Do not proceed by treating the row index as an experiment identifier.

**Executable audit before modeling:** run the standalone Rust tool at [tools/biochar-audit](./tools/biochar-audit/README.md). It fingerprints the exact input and reports shape, missingness, ragged records, exact duplicate rows, ambiguous headers, and candidate study/treatment keys. Validate the report with [audit-report.schema.json](./tools/biochar-audit/audit-report.schema.json). These are structural checks only; they do not approve grouping keys or treatment semantics.

The paper describes the CSV as a simplified dataset and references `Reference` and `Explanation and unit` sheets in the full workbook. If the CSV lacks a reviewed study identifier or enough treatment/comparator information, stop the model benchmark, obtain the full workbook, and construct a reviewed crosswalk. Do not use row indices as experiment identifiers.

**Dataset go/no-go checklist:** capture the source URI and license; retain the original files unchanged; hash files; audit variables and units; map each row to study, site, soil, crop, material/batch (when supplied), treatment, comparator, and outcome; log missingness and unit conversions; identify repeated measures and correlated comparisons; create group-level split manifests before fitting. Publish the dataset audit before reporting performance.

Maintain four separate data classes: (a) synthetic fixtures for software invariants, (b) literature-extracted published observations, (c) new lab measurements, and (d) controlled/field trial observations. They must never be merged without explicit source labels, compatible measurement definitions, and a documented modeling plan. External datasets alone cannot establish South African field efficacy or local product safety.

## References

- Li, X. et al. “A global dataset of biochar application effects on crop yield, soil properties, and greenhouse gas emissions,” *Scientific Data* 11, 57 (2024), [paper](https://doi.org/10.1038/s41597-023-02867-9) and [dataset](https://doi.org/10.6084/m9.figshare.24781764.v1).
- [Soils4Africa observations and maps](https://africasis.isric.org/download.html).
- [WoSIS standardized soil profile data](https://isric.org/explore/wosis).
- [FAO AquaCrop v7.3](https://www.fao.org/aquacrop).
- [Global synthesis of biochar field experiments on yield and water productivity (2024)](https://doi.org/10.1016/j.agwat.2024.109134).
- [Systematic review: biochar and nutrient cycling in sandy soils (2024)](https://doi.org/10.1186/s13750-024-00326-5).
- [Long-term soil hydraulic effects meta-analysis (2025)](https://doi.org/10.1016/j.geoderma.2025.117318).
- [European Biochar Certificate (EBC) guidelines and updates](https://www.european-biochar.org/en/certificate) (the public update history lists v10.5E dated 14 August 2025).
- [World Biochar Certificate / current industry certification context](https://biochar-international.org/biochar-testing-laboratories-for-certification/) (IBI reports that its former separate standard/certification program merged into the World Biochar Certificate administered by Carbon Standards International; verify the current standard and lab scope before a procurement decision).
- [South African Agriculture Inputs Control](https://nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control) and [Act 36 of 1947](https://www.gov.za/documents/fertilizers-farm-feeds-seeds-and-remedies-act-28-may-2015-1101): request classification for the actual product, claims, and use before treating any registration path as known.
