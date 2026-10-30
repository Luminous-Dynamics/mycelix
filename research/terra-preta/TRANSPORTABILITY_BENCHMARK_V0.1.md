# Terra Preta: South African Transportability Benchmark v0.1

**Status:** Proposed validation contract; no local efficacy or transportability results are claimed.  
**Date:** 2026-10-29  
**Companion:** [Benchmark v0.1](./BENCHMARK_V0.1.md) and [Protocol v0.1](./PROTOCOL_V0.1.md).

## 1. Research question

Can evidence learned from heterogeneous global biochar studies support a calibrated, decision-relevant prediction for a specified South African soil–material–crop–management context? When it cannot, can the system detect the gap and identify the most valuable safe measurement or experiment?

This is a transportability benchmark, not a claim that global mean effects apply locally. A soil-property map, a literature-extracted biochar response, a laboratory measurement, and a field-trial outcome are different evidence classes and must remain distinguishable.

## 2. The proposed paradigm shift

Treat every prediction as a conditional claim with a documented domain of applicability. The complete loop is:

1. **Mycelix — evidence and provenance:** preserve source documents, raw-file hashes, licenses, study/experiment/arm crosswalk, measurement method, units, and reviewer disposition.
2. **Symtropy — constrained physical models:** enforce water accounting and evaluate parameterized soil-water relationships only within their stated assumptions. A constitutive curve is not a calibrated site model or field trial.
3. **Symthaea — uncertainty and experiment selection:** rank *authorized measurement candidates* by expected reduction in decision-relevant uncertainty, with safety, cost, logistics, and study design as hard constraints.
4. **Independent verifier — admission and reporting:** reject leakage, unidentified contrasts, unsupported extrapolation, failed safety gates, and unprovenanced data. When evidence is insufficient, return `insufficient_evidence`, not a confident recommendation.

The aim is not full automation of agronomic judgment. It is to make the system good at identifying what it does not know and at designing the next informative, ethical, affordable experiment.

## 3. Data classes and their limits

Keep the following stores physically or logically separate, with stable provenance identifiers:

| Evidence class | Appropriate use | Not sufficient for |
| --- | --- | --- |
| Synthetic fixtures | Software invariants, known-shift and failure-injection tests | Efficacy, calibration to nature, or field safety |
| Global literature-extracted effect records (e.g., BiocharDS) | Study-level meta-analysis, moderator exploration, study-held-out tests after crosswalk review | Site-specific South African predictions without transport checks |
| Long-term synthesis records | Time/horizon stratification and follow-up hypotheses after source-overlap audit | An independent holdout if the publications/experiments overlap training sources |
| African soil-property data (e.g., Soils4Africa) | Describe regional coverage, define candidate strata, flag covariate extrapolation | Biochar-effect labels, parcel-level truth, or direct efficacy evidence |
| Laboratory-measured local soil/material properties | Calibrate specified model parameters and screen characterized batches | Crop response or season-long benefit by themselves |
| Prospective controlled pot/field observations | Confirmatory, locally scoped treatment effects under preregistered design | Universal effects beyond the studied domain |

### Current external data notes

A second, distinct Soils4Africa release is available as the [field-data dataset, DOI 10.5281/zenodo.17699297](https://doi.org/10.5281/zenodo.17699297), published 26 June 2026 as version 1. The [publisher's 55-field tableschema](https://zenodo.org/records/17699297/files/s4a_field_data-tableschema.csv?download=1) describes a 7.4 MB field CSV with coordinates (X/Y, longitude/latitude), `plot_code`, `plot_id`, `tsu_id`, country identifiers, survey date and altitude, soil-observation/erosion indicators, landform and slope, crop type and land use, crop-residue/green-manure/animal-manure/inorganic inputs, water supply, and irrigation fields. The publisher reports observations from more than 15,000 locations across 33 African countries and explicitly says quality control is ongoing and new versions are expected. The retrieved landing page leaves its Rights/License field blank, so confirm the applicable terms with the data steward before use or redistribution.

**Source-specific audit requirements:** preserve the published version and both source files unchanged; record retrieval timestamp, own SHA-256 hashes, publisher-provided MD5s, and the tableschema revision; inspect coordinate bounds, duplicate `plot_id`/`plot_code`/TSU relationships, country-code consistency, impossible dates/coordinates, missingness, and categorical encodings; report coverage by country and any South African strata only after inspecting the actual CSV. These are survey/context fields, not biochar-response labels. A plot or TSU identifier may encode nested sampling; it must not be treated as an independent experimental unit without checking the published sampling design.

Do not merge this field-observation release with the separate MIR-derived soil-property estimates without a reviewed sample/site/depth/version crosswalk. Maintain separate source hashes and measurement/derivation labels. Use these resources for regional context coverage, stratification, and missing-measurement planning—not as a substitute for plot-level lab measurements or treatment-effect data.


- **BiocharDS_V1.0:** the Nature HTML describes 2,439 rows and 209 columns from 367 studies, but its PDF abstract reports 2,438 items. The HTML also describes 891 independent experiments while listing 700 under B and 314 under BF (sum 1,014). These published figures are internally inconsistent as stated and must be reconciled from exact source files and definitions; do not choose one count or infer the meaning from text alone. The CSV has a separate [Springer Nature Figshare item](https://springernature.figshare.com/articles/dataset/BiocharDS_V1_0_csv/24781764); the full [BiocharDS_V1.0.xlsx workbook is a distinct Figshare item](https://springernature.figshare.com/articles/dataset/BiocharDS_V1_0_xlsx/24781737), with Reference and Explanation and unit material. Both remain unacquired and unhashed in this run. Resolve study, experiment, treatment arm, and repeated-measure identities against the workbook/source papers before confirmatory modeling. See the machine-readable [source acquisition register](./source-register.v0.1.json).
- **Long-term biochar synthesis:** Yang et al. report 438 studies, 3,229 paired observations, and 29 long-term field experiments (4–12 years). The paper reports that variance was estimated for some records with missing SD, so preserve the authors' observed-versus-imputed variance distinction and reproduce their dependency assumptions before reusing estimates. Crosswalk sources against BiocharDS before calling either dataset an independent holdout. Check the applicable license/terms of the paper and supplements before redistribution.
- **Soils4Africa / Africa Soil Information System:** ISRIC announced on 5 October 2026 an initial release of estimates for 19 soil properties derived from MIR spectra for more than 11,000 samples collected at more than 6,100 locations across 21 African countries, alongside a pesticide-residue dataset from 272 topsoil locations. These are regional context and coverage resources. Store property-specific measurement/derivation method, geographic coverage, uncertainty, provenance, and access/license terms; do not silently relabel a spectrally derived estimate as a direct lab measurement. Confirm South African site coverage and access rights from the actual dataset before planning a local analysis.
- **AquaCrop 7.3:** FAO announced availability on 30 July 2026, including Python source. It is a crop-water productivity comparator when model scope, input data, calibration, and boundary assumptions genuinely match. Keep it as a version-pinned external reference, not an automatic baseline for biochar causal effects or an unreviewed production dependency.

### Acquisition ledger

The machine-readable [source register v0.1](./source-register.v0.1.json) is the authoritative acquisition ledger for this benchmark. Its contract is [source-register.schema.json](./source-register.schema.json). It distinguishes published metadata from files actually acquired, records the known publisher MD5 values for the Soils4Africa README/tableschema/CSV, and leaves our SHA-256 fields null until the unchanged original bytes are obtained and hashed locally. Known source counts and file sizes reported on publisher pages must not be silently converted into exact byte counts.

As of the register's review date (2026-10-29), BiocharDS_V1.0 and its full workbook are not downloaded; the 2025 PNAS long-term paper/supplement is not downloaded; and the Soils4Africa field CSV download attempt failed. Metadata and the tableschema were reviewed, but no raw-source structural audit or South African coverage calculation has been completed. Treat these statuses as open acquisition gates, not as evidence that the files are unavailable from the publisher.

Before any model fit, update the register with exact retrieval time, original URL, version/terms review, publisher checksums, independently computed SHA-256, and artifact-specific audit report. Keep the original bytes immutable; do not replace an existing source version in place when the publisher updates a dataset.

## 4. Evaluation ladder — never skip a level

### T0: Data and provenance integrity

Before fitting any model, publish a versioned data audit with original-source hashes, licenses/access constraints, exact row/column counts, units, missingness, duplicates, study/experiment/site/batch identifiers, repeated measures, treatment and comparator definitions, and a reviewed source crosswalk.

**Fail closed:** if no defensible study grouping can be established, study-held-out metrics are unavailable. A row number or guessed header is not an experimental-unit identifier.

### T1: Software and physical invariants

Run deterministic water-accounting, units, invalid-input, extreme-value and conservation tests. Compare the van Genuchten–Mualem implementation against independently calculated reference values over a preregistered grid including saturated, dry, intermediate, and extreme finite pressure heads. Record numerical tolerances and the source of each reference value.

Passing T1 certifies implementation properties only. It does not validate fitted parameters or forecast soil dynamics.

### T2: Global study-held-out benchmark

Freeze splits by source study and, where recoverable, experiment/site. Compare every candidate with:
- zero treatment-effect and overall-mean baselines as appropriate to the estimand;
- a transparent linear or multilevel meta-regression using available covariates;
- a defensible study-clustered baseline;
- a time-aware baseline for longitudinal targets.

Report outcome prediction and treatment-effect estimation separately. Use cluster-aware intervals; do not count multiple endpoints, arms, depths, or repeated times from one study as independent clusters.

### T3: Geographic and covariate shift stress

Predefine strata using observed covariates with justified agronomic meaning: texture, pH, SOC, climate/aridity, crop, material/feedstock/process descriptors, application regime, follow-up time, and management. Conduct leave-region, leave-climate-zone, leave-soil-stratum, and leave-material-family-out tests only where a source crosswalk and sample counts support them.

Use Soils4Africa data to quantify *context coverage* and identify missing local soil measurements. It does not provide biochar response labels, so do not score an efficacy predictor against it. Report the fraction of intended deployment contexts inside, near, and outside the training covariate support; show subgroup results, not only a global mean.

If a subgroup has too few independent source studies, report it as unvalidated rather than hiding it inside an average.

### T4: Local lab calibration

Before using a model for a site-specific hypothesis, measure the actual soil and material with suitable methods and chain-of-custody records. Fit retention parameters only to the declared calibration data; evaluate on held-out measurements from the same experimental unit/study where feasible. Report parameter identifiability, residual structure, uncertainty, and sensitivity to measurement error.

Do not use the same curves for both fitting and claiming independent accuracy. Do not infer a biochar-induced parameter change from untreated-soil measurements.

### T5: Prospective local trial

Preregister crop/site/season, treatment arms, matched comparator, primary estimand, experimental unit, block/randomization, power or precision justification, adverse-event and stopping rules, follow-up period, analysis, and independent review. Lock the model and protocol before outcome data are inspected. Evaluate against local practice and a scientifically matched comparator; a nutrient substitution claim requires an explicitly comparable nutrient program.

Only this level can support a properly bounded local efficacy claim. Regulatory classification, batch safety characterization, and required permissions are independent gates, not model metrics.

## 5. Metrics and acceptance rules

Publish a metric vector; do not collapse all dimensions into a single leaderboard score.

**Predictive quality**
- MAE, RMSE, bias, and normalized/relative error where mathematically appropriate;
- performance by study, site, material, soil stratum, and follow-up horizon;
- cluster-bootstrap confidence intervals and independent-cluster counts.

**Uncertainty**
- nominal 90% prediction-interval coverage and mean interval width;
- calibration slope/intercept where suitable;
- weighted interval score or another preregistered proper interval score;
- calibration by subgroup and forecast horizon.

Existing project targets remain provisional: for an adequately supported primary endpoint, aim for at least 10% lower MAE than the strongest valid baseline on a fully held-out grouped test, with a 95% cluster-bootstrap interval for improvement excluding zero. For supported water-retention properties, target median absolute relative error ≤10% and nominal 90% interval coverage of 85–95%. These are targets, not results; the dataset audit must first show that the inference is adequately powered.

**Applicability and abstention**
- risk–coverage curve: error among predictions the system elects to make versus the share it covers;
- OOD/shift detection sensitivity and false-abstention rate, reported separately for each predefined shift family;
- fraction of intended use cases flagged as unsupported;
- worst-supported subgroup and performance under deliberate missingness/noisy covariates.

Create synthetic shift families before model fitting (e.g., unseen texture class, shifted pH/SOC range, unseen feedstock, long-horizon extrapolation, unit error, missing study identifier). All malformed records and hard safety violations must be rejected. On scientific covariate shift, report measured sensitivity/false-positive tradeoffs; do not claim that every real-world shift is detectable. A model must abstain when a mandatory input or safety gate is missing, even when its point prediction looks favorable.

**Decision and experiment value**
- regret or net benefit against a preregistered decision rule;
- time and total cost to reach a predefined evidence threshold;
- calibration/decision quality at equal measurement budget;
- performance versus random selection, fixed expert-designed sequence, and uncertainty-greedy selection.

Symthaea is useful only if it selects the next experiment better than these baselines under the same candidate set, data, cost model, seeds, and safety constraints. Runtime alone is not improvement. Keep the candidate proposal separate from physical authorization.

## 6. Reference-model fairness

- **Symtropy water kernel:** compare accounting invariants and curve values only on matching definitions/units. The current kernel is not a transient Richards solver.
- **HYDRUS-1D or another Richards-equation solver:** only for matched transient-flow problems with compatible geometry, boundary conditions, retention/conductivity parameters, and time steps. Do not compare a static retention curve to a dynamic flow prediction.
- **FAO AquaCrop 7.3:** only for matching crop-water/yield questions with required inputs and calibration. It does not identify a biochar treatment effect automatically.
- **Meta-regression / multilevel meta-analysis:** mandatory strong baselines for literature treatment-effect synthesis, including study dependence and effect-size uncertainty.
- **Local agronomic practice:** required decision comparator selected with a competent local partner; not a generic global benchmark.

Record version, source hash, configuration, inputs, calibration data, output, and known limitations for every external comparator. If a fair execution is not possible, label it “not compared”; never imply superiority.

## 7. A measurable Symthaea–Symtropy–Mycelix experiment loop

For each candidate question:
1. Mycelix stores a reviewed provenance record and declares the estimand plus hard safety/regulatory gates.
2. Symtropy runs only within calibrated parameter domains and returns outputs plus parameter/input uncertainty and a versioned configuration digest.
3. Symthaea proposes the next *measurement* or approved experimental contrast with expected information gain, uncertainty reduction, cost, and reason for selection.
4. A separate gatekeeper rejects candidates that are unauthorized, unsafe, non-identifiable, or outside the responsible reviewer's approved scope.
5. The run is preregistered; results are recorded with measurement methods and adverse/null results.
6. An independent evaluator updates the benchmark, tests calibration/transportability, and decides whether the validated domain expands. Domain expansion never happens silently after observing a favorable result.

A synthetic simulation can test the loop's software and decision logic but must be labeled synthetic. It cannot turn simulated soil behavior into an observed treatment effect.

## 8. Industry quality and compliance comparison

Scientific model scores and commercial material qualification are separate workstreams.

- **EBC:** the public guideline update history lists version 10.5E dated 14 August 2025. Treat it as a version-pinned quality/production comparator, and verify the actual class, feedstock eligibility, analytical methods, and current revision before using it as a specification.
- **World Biochar Certificate (WBC):** the International Biochar Initiative reports that its former separate standards/certification program merged into the WBC administered by Carbon Standards International. The IBI laboratory list was updated in March 2026 and describes laboratory capability for WBC-related analyses. Verify current accreditation, method scope, sampling plan, and batch-specific certificate; do not rely on a brand name alone.
- **South Africa:** the Department of Agriculture's Agriculture Inputs Control Directorate states that it regulates manufacturing, distribution, importation, sale, use, and advertisement of covered agricultural inputs under Act 36 of 1947, and evaluates efficacy and safety for relevant products. Official registration guidance exists for fertilizer groups, but the exact classification and obligations for a particular biochar/compost formulation depend on the product and proposed claims. Obtain written regulator guidance rather than assuming either that registration applies or that it is exempt.

The research benchmark must therefore report at least two independent outcomes: (1) material identity/quality and compliance status for the exact batch; and (2) measured agronomic performance for the specified site, crop, and management context. A product-quality certificate does not establish a local treatment effect. Strong predictive performance does not establish material safety. Neither can compensate for a failed mandatory gate.

**Current external references:** [EBC guideline history](https://www.european-biochar.org/en/ct/7-Updates), [IBI/WBC laboratory and certification update](https://biochar-international.org/biochar-testing-laboratories-for-certification/), and [South African Agriculture Inputs Control](https://nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control).

## 9. Reproducibility and go/no-go artefacts

Each confirmatory run should contain:
- exact data-source versions, hashes, license/terms review, and source crosswalk revision;
- code commit, dependency/tool versions, model/config digest, random seeds, split manifest, and baseline version;
- declared estimand and reason it is identifiable from the source;
- independent cluster counts, exclusions, missingness, outcome definitions/units, metrics and uncertainty intervals;
- OOD tests and coverage-risk analysis, including failures and unsupported subgroups;
- safety/regulatory gate references with responsible reviewer and current status;
- deviations from preregistration and whether the run was exploratory or confirmatory.

**Go:** evidence and provenance are sufficient for the explicitly stated claim; grouped evaluation is valid; baseline and calibration reports are available; independent review signs off.

**No-go / insufficient evidence:** unknown or mismatched comparators; unresolved dataset overlap; unreviewed study keys; failed schema/integrity checks; uncalibrated extrapolation; inadequate independent clusters; missing mandatory safety or regulatory gates.

No favourable mean score can override a failed gate.

## 10. Leakage-risk stress test informed by recent biochar ML audits

A 2026 review of 64 highly cited biochar machine-learning studies reports potential leakage in all 64 reviewed studies, random point-level splits in 98%, and accuracy inflation up to 86% in five reanalysed datasets. These are the review's findings and should be reproduced against source data where possible—not treated as a universal inflation factor. A separate 2026 comparative study of six approaches to water-retention prediction reports lower performance on a held-out source than on an inner test drawn from the training-source pool. These findings make source-held-out evaluation a primary acceptance test, not optional extra analysis.

The recent six-model soil-water-retention comparison gives a concrete, but not directly transferable, external reference: four tree-based models reported (R^2) around 0.85–0.88 on an inner-test split from the 35-source development pool, while on a held-out source the reported (R^2) values were RF 0.666, XGBoost 0.643, CatBoost 0.610, and LightGBM 0.611. The held-out-source result is much weaker, which is precisely why source-level holdout is material. It is still only one source-held-out test from that paper, and its dataset, endpoint, splits, and inputs differ from our planned benchmark; do not compare the numbers as if they were a head-to-head result.

**Protocol:**

1. Before model fitting, identify the independent unit for the intended claim. Create a reviewed crosswalk from observation to publication/study, experiment, site, material/feedstock or batch, treatment/comparator arm, time, and repeated-measure identity.
2. Freeze the primary grouping key and the evaluation split. Run `tools/split-audit/` on the row-level manifest for the chosen key. When required, create separate frozen manifests for study-, site-, batch-, or temporal-holdout evaluation; passing one dimension does not imply passing another.
3. Keep random-row results only as a **diagnostic comparator** that quantifies how much a naive split can overstate generalization. Never use it as confirmatory evidence or as the main model-ranking criterion.
4. Perform all preprocessing, feature selection, imputation, transformation, calibration, hyperparameter selection, and threshold choice inside the training portion of each grouped/nested evaluation. Do not inspect the final grouped test to tune the model.
5. Report candidate and baseline results for random-row diagnostic and grouped holdout separately, including MAE/RMSE/bias, (R^2) where appropriate, prediction-interval coverage and width, per-source scores, and the number of independent groups. Avoid a single "leakage inflation percent" unless the data, split, and estimator define that quantity precisely.
6. Run hard negative controls: deliberately place repeated rows from one group in multiple splits and confirm the audit fails; duplicate row IDs; supply placeholder identifiers; and put empty required splits in the manifest. A failed negative control blocks confirmatory benchmark status.
7. Cross-check source IDs across BiocharDS and any long-term synthesis. Different paper titles do not prove independence, and a shared publication, experiment, site, or reused observations must not be treated as an independent validation source.

**Acceptance:** the grouped holdout, source crosswalk, frozen split manifest, model configuration, and prediction report are reviewable together. If a credible grouping key cannot be reconstructed from the source papers/workbook, report the generalization result as not evaluable rather than replacing the missing key with row IDs.

References: Jang et al., “Illusions of Accuracy: Widespread Data Leakage Distorts Machine Learning Results in Biochar Research,” first published 25 August 2026, [doi:10.1111/gcbb.70173](https://doi.org/10.1111/gcbb.70173); “Predicting the Water Retention Behavior of Biochar-Amended Soils: A Comparative Study of Six Machine Learning Approaches,” [Applied Sciences (2026)](https://www.mdpi.com/2076-3417/16/20/10219). The benchmark results reported in those papers are context, not claims about Luminous Dynamics' implementation.

## References

- Li et al., “A global dataset of biochar application effects on crop yield, soil properties, and greenhouse gas emissions,” *Scientific Data* (2024), [paper](https://doi.org/10.1038/s41597-023-02867-9), [dataset DOI](https://doi.org/10.6084/m9.figshare.24781764.v1).
- Yang et al., “Sustained benefits of long-term biochar application for food security and climate change mitigation,” *PNAS* (2025), [doi:10.1073/PNAS.2509237122](https://doi.org/10.1073/PNAS.2509237122).
- ISRIC, “Soils4Africa makes 11,000 soil data points from 21 African countries available,” 5 October 2026, [release](https://isric.org/news/soils4africa-makes-11-000-soil-data-points-from-21-african-countries-available/).
- FAO, AquaCrop model page and version 7.3 resources, [official site](https://www.fao.org/aquacrop).
- South African Department of Agriculture, Agriculture Inputs Control, [official guidance](https://nda.gov.za/index.php/core-business/agricultural-production/inspection-services/agriculture-inputs-control).
