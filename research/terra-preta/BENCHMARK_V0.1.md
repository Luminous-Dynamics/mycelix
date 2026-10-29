# Terra Preta Benchmark v0.1

**Status:** Proposed benchmark contract; target values below are acceptance goals, not achieved results.  
**Pairs with:** [Protocol v0.1](./PROTOCOL_V0.1.md)

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

Use measured water-retention curves, infiltration, drainage, and soil moisture where available. Stratify by soil class and experimental setup. Evaluate time-series error and event-level error, not just a pooled correlation.

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

## 3. Required baselines

At minimum, compare to:
- **No-change / treatment-effect-zero baseline:** predicts no amendment effect.
- **Simple empirical baseline:** predeclared linear or mixed-effects model using only available covariates.
- **Persistence baseline:** for time-series values, predicts the last observed value where that is a valid comparator.
- **Established domain model:** such as HYDRUS-1D or SWAP for a specifically matched soil-water problem, only when assumptions, inputs, units, and boundary conditions are aligned.
- **Current agronomic practice:** selected with local agronomist/farm partner for crop and location.

Do not claim superiority over a reference implementation if no matched, reproducible comparison was performed. Every baseline must receive the same data split and permitted input information.

## 4. Dataset split and anti-leakage rules

Use grouped splits, with split manifests committed before model fitting:
- **in-domain holdout:** unseen samples within known soil/site group, if the use case supports it;
- **batch holdout:** whole biochar/compost batches held out;
- **soil holdout:** an entire soil class or source location held out;
- **site/season holdout:** entire sites or growing seasons held out;
- **prospective evaluation:** protocol and model frozen before new data arrive.

No shared sample, repeat measurement, derived feature, or augmentation from a held-out group may enter training. Any learned preprocessing parameters must be fitted on training groups only. Report both interpolation and extrapolation separately. Results on a random row split alone are insufficient.

## 5. Safety and evidence gate matrix

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

Exact chemical/biological acceptance thresholds must be specified for the actual formulation by qualified specialists and applicable rules; this benchmark does not invent universal thresholds.

## 6. Reporting contract

Every run must report:
- protocol/benchmark version, run ID, timestamp, code commit, config digest, dependency/tool versions, RNG seed;
- dataset version, source citations, license/access restrictions, hashes, group split manifest;
- batch, soil, crop, site, season, and experimental-unit identifiers (pseudonymous where needed);
- metric definitions and units, per-group scores, aggregate scores, confidence intervals, missingness, and exclusions;
- baseline comparisons and failure cases;
- model applicability domain and uncertainty/calibration results;
- laboratory/approval gate references and status, without treating attestations as proof of the underlying measurement;
- complete deviation log and whether the run is exploratory or confirmatory.

Do not publish personal or commercially confidential farm data without permission. Publish de-identified data or reproducible aggregation where appropriate; record any restrictions.

## 7. Release gates

- **B0 — executable spec:** metric definitions, JSON Schema, split rules, and synthetic test cases accepted.
- **B1 — model sanity:** all synthetic conservation/property tests pass; deterministic replay and invalid-input tests pass.
- **B2 — baseline benchmark:** dataset audit and baseline results published, including failed targets.
- **B3 — independent evaluation:** frozen model tested on a batch/site/season holdout by a reviewer not responsible for model fitting.
- **B4 — decision support:** documented domain of applicability, calibrated uncertainty, safety workflow, and human agronomic review. The output is a *research recommendation for testing*, not permission to use/sell a product.

## 8. Benchmark interpretation

The goal is not to “beat SOTA” on a single aggregate number. It is to beat appropriate baselines on held-out data while maintaining physical consistency, calibrated uncertainty, repeatable provenance, and zero bypass of safety gates. If a simpler model performs just as well, prefer the simpler model until complexity adds demonstrated value.

A positive crop response is not required to pass the scientific benchmark; trustworthy identification of no benefit, harm, or a narrow domain of benefit is valuable.

## 8. Initial dataset strategy and acquisition gates

Start with available external evidence rather than fabricating synthetic labels or waiting for a field trial:

1. **Global biochar treatment-response data:** Li et al.'s BiocharDS_V1.0 dataset is publicly available on [Figshare (CC0 1.0)](https://springernature.figshare.com/articles/dataset/BiocharDS_V1_0_csv/24781764), DOI [10.6084/m9.figshare.24781764.v1](https://doi.org/10.6084/m9.figshare.24781764.v1). Its paper describes data extracted from 367 peer-reviewed studies, including 891 experiments and 2,438 paired items across crop yield, soil properties, and greenhouse-gas outcomes. First step: download the dataset and data dictionary, audit study/treatment identifiers, units, missingness, sampling depth, and duplicated/related observations. This is a *literature-extracted effect dataset*, not a substitute for raw, local calibration time series. Split by source study (and by site where identifiable), never random rows.
2. **African soil context:** [Soils4Africa / Soil Data of Africa](https://africasis.isric.org/download.html) offers sampled soil properties including texture, bulk density, organic carbon, and chemical properties as CSV/GeoPackage, with regional soil maps. Its own guidance says the recommended application scale is national to continental and discourages field-level use. Use it to stratify hypotheses and help select candidate regions, not as a replacement for soil sampling from a trial plot.
3. **Standardized profiles and gaps:** [WoSIS](https://isric.org/explore/wosis) is a global, standardized soil-profile resource. Coverage is property-dependent and access is subject to source data-provider licenses. Record exactly which properties are measured versus predicted/harmonized; do not treat a map estimate as a laboratory observation.
4. **Crop-water reference model:** FAO [AquaCrop v7.3](https://www.fao.org/aquacrop) is an established crop water-productivity model with source code and standalone versions. Use it as a version-pinned *external comparator* for matched crop/water questions when inputs, calibration, and boundary assumptions align. It is not a biochar-specific effect model and must not become an unexamined production dependency in the Rust kernel.

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
- [European Biochar Certificate guidelines](https://www.european-biochar.org/en/certificate).
- [International Biochar Initiative standards](https://biochar-international.org/biochar-standards/).
