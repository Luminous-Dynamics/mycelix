# NeuroTwin MDD cross-trial benchmark protocol v0.2

**Status:** preregistration candidate / research design. Not an executed benchmark, registered protocol, clinical validation, or treatment recommendation.  
**Tracking:** [NEUROTWIN-001 / #6276](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

## 1. Research question

Can longitudinal multimodal and brain-network representations improve the transportability, calibration and decision value of MDD treatment-outcome models beyond carefully specified clinical-only baselines—and can any apparent benefit survive an independent randomized/prospective evaluation?

The programme separates three estimands:

1. **Prognosis:** probability or expected magnitude of outcome under the treatment actually received.
2. **Treatment-effect heterogeneity:** expected difference between outcomes under alternative treatments for a defined population, conditional on baseline features.
3. **Treatment-policy value:** outcome under a fully specified decision policy compared with a prespecified comparator policy in an appropriate randomized or prospectively evaluated design.

These are different scientific claims. A response classifier is not a personalized treatment selector. A model's confidence score is not a causal effect estimate.

## 2. Published starting points

### 2.1 Primary cross-trial benchmark: EMBARC and CAN-BIND-1

Zhukovsky et al. (2025), *JAMA Network Open*, DOI [10.1001/jamanetworkopen.2025.1310](https://doi.org/10.1001/jamanetworkopen.2025.1310), reported cross-trial prediction using clinical variables and dorsal anterior cingulate cortex (dACC) functional connectivity.

The analysed cross-trial sample comprised 363 participants: 225 from EMBARC and 138 from CAN-BIND-1. Reported AUCs included:

| Training → test | Reported AUC | Interpretation |
|---|---:|---|
| CAN-BIND-1 → EMBARC stage 1 | 0.62 | Cross-trial prognostic baseline |
| CAN-BIND-1 → EMBARC stage 2 | 0.67 | Cross-trial prognostic baseline |
| EMBARC stage 1 → CAN-BIND-1 | 0.66 | Reverse-direction cross-trial baseline |

The article also reported balanced-accuracy and symptom-change associations; these must be extracted alongside the AUCs during an exact reproduction rather than reconstructed from memory. Do not pool these AUCs with results using different endpoints, participant filters, modality availability, or validation methods.

The sample size is the analytic sample in this paper, not the total size of either dataset release. CAN-BIND-1 has larger released cohort counts in other analyses; other CAN-BIND publications report different analytic subsets. Every result must name its own inclusion rule and denominator.

### 2.2 Prospective translation comparator: SMART-D

Zhukovsky et al. (2026), *Nature Mental Health*, DOI [10.1038/s44220-026-00671-z](https://doi.org/10.1038/s44220-026-00671-z), derived sertraline- and bupropion-related biomarker models in EMBARC and evaluated biomarker-guided assignment prospectively in SMART-D (NCT05537584).

The training/leave-one-out results reported AUCs from 0.66 to 0.86. In the prospective sample, 47 participants had at least one week of treatment and 41 completed eight weeks. The preregistered comparison of biomarker-consistent versus inconsistent assignment did not show a significant difference in MADRS trajectories (time × assignment: F(1, 38.18) = 0.03, P = 0.87); response rates also did not significantly differ between inconsistent and consistent groups (66.7% vs 61.5%, P = 0.72).

The authors reported differences between marker-status groups (both-positive, one-positive, both-negative), but that is not the same estimand as proving that assignment to the biomarker-matched drug improves outcomes. The trial was small and cannot establish that no treatment-matching effect exists. It does establish that retrospective discrimination—even AUCs approaching 0.86—does not by itself qualify a treatment policy.

**Required NeuroTwin lesson:** optimize and evaluate the scientific claim actually intended. If the goal is treatment selection, primary success must be measured at the randomized treatment-policy level, not inferred from prognostic AUC.

### 2.3 Broader evidence ceiling

The 2026 umbrella review by Esmaeilian et al., DOI [10.1111/pcn.70053](https://doi.org/10.1111/pcn.70053), included 14 reviews/meta-analyses covering 17,855 people with MDD. It reported high AUCs in some multimodal studies, but rated the evidence quality low to very low overall, with substantial methodological heterogeneity and limited external/prospective validation.

Therefore, an isolated AUC above 0.85 is not a credible field-leading claim without matched cohorts, leakage controls, independent validation, calibration, uncertainty and prospective utility.

### 2.4 Digital-twin architecture comparator (not the same task)

The 2026 Digital Twin Brain work (DOI [10.34133/bmef.0231](https://doi.org/10.34133/bmef.0231)) demonstrates a connectome-conditioned, multitask behavioural modelling direction. It is an architecture comparator, not a directly comparable antidepressant-response or randomized treatment-policy benchmark. Report task, sample, split and metric before comparing numbers.

## 3. Cohorts and data authority

### EMBARC
A multisite randomized study involving sertraline and placebo in stage 1, with a subsequent stage-2 treatment pathway. Access is controlled through the applicable NIMH Data Archive process and its current data-use requirements. The cross-trial paper's 225-participant analytic subset is not a blanket access entitlement.

### CAN-BIND-1
A multisite MDD cohort with multimodal measures including clinical assessments, imaging and electrophysiology. Controlled access is administered through Brain-CODE. The Brain-CODE access process requires a reviewed data-access request, applicable research ethics board approval or exemption, a signed Data Use Agreement, and analysis within an approved Brain-CODE workspace. Dataset release counts and paper-specific analytic samples must not be conflated.

### Data admission gate
No dataset is admitted until all of the following have been checked by the responsible institution/data custodian:

- exact dataset/release identity and version;
- approved research question and analysis scope;
- applicable ethics/REB approval or exemption;
- signed DUA and user access approvals;
- source licence/terms and restrictions on linkage, onward transfer, publication and retention;
- approved storage/compute location and deletion/return requirements;
- permitted modalities, derived features, model training, federated updates and publication outputs.

This protocol does not request, download or authorize restricted data. Public documentation and synthetic fixtures are the only data used in this repository change.

## 4. Prespecified task and endpoints

### Primary retrospective task
Reproduce the published cross-trial prognostic task before adding new model families. Use each paper's stated outcome definition and inclusion criteria exactly. Freeze the endpoint, timepoint, cohort filter, covariates, preprocessing and trial direction before model fitting.

### Clinical outcomes
The candidate primary clinical outcome is the study-defined MADRS response at the specified follow-up; where the original paper defines response as at least a 50% reduction from baseline, reproduce that exact definition. Also preserve continuous MADRS change as a separate endpoint. Do not silently substitute remission, response, symptom change, or estimated endpoint scores.

### Baseline-only and longitudinal prediction
Keep separate:
- baseline-only predictors;
- baseline plus week-2 clinical change;
- later longitudinal prediction using only observations available at the declared prediction time.

A week-2 model must not be described as a baseline prediction. No future value, post-outcome value, post-randomization information unavailable at the prediction time, or data-derived transformation fitted on the full cohort may enter a training fold.

## 5. Validation design

### Tier A — exact reproduction
Reconstruct the published participants, labels, trial assignment, feature definitions, preprocessing and split logic. Report deviations as a separate analysis; do not label a deviating analysis an exact reproduction.

### Tier B — cross-trial generalization
Use trial-held-out directions, preserving the three published directions above. Fit imputation, harmonization, feature selection, scaling and hyperparameter selection within the training data only. Where model selection is needed, use nested validation inside the training trial. Freeze the test trial until the entire pipeline and analysis plan are locked.

### Tier C — independent trial
Evaluate on an additional independent trial that was not used for feature selection, architecture selection, threshold selection or model updates. A third trial is necessary before claiming transportability beyond the two-study replication.

### Tier D — silent prospective evaluation
Lock the model and preprocessing before prospective enrolment. Generate predictions without displaying them to clinicians or participants and without changing care. Measure calibration, missingness, abstention, drift and workflow feasibility.

### Tier E — randomized policy-impact study
Only after prior gates and independent ethics/regulatory review, compare a fully specified policy with an appropriate comparator in a randomized design. Predefine the treatment-policy estimand, randomization, primary endpoint, missing-data plan, subgroup analysis, stopping rules and clinical oversight.

## 6. Mandatory model ladder

Every comparison must use identical participant filters, outcome, prediction time, splits, available data and tuning budget.

1. Clinical-only regularized baseline (e.g. logistic/elastic-net or a prespecified equivalent).
2. Strong clinical longitudinal baseline where week-2 information is permitted.
3. Reproduced published dACC + clinical baseline.
4. Imaging-only and connectome/graph baselines.
5. EEG-only baseline where a valid modality-matched cohort exists.
6. Multimodal late-fusion baseline with explicit missing-modality handling.
7. HDC-only and CfC/LTC-only candidates.
8. Connectome + HDC and connectome + CfC/LTC candidates.
9. A prespecified treatment-effect model for randomized arms, compared with a clinical-only treatment-effect baseline.

Symthaea methods are experimental candidates. No model family receives privileged status, and no hand-authored biological profiles or synthetic fallback data may be reported as empirical results. Synthetic data are for software-contract testing only.

## 7. Metrics and reporting

### Prognostic performance
Report AUROC, AUPRC, balanced accuracy, sensitivity, specificity, Brier score, log loss and calibration intercept/slope with confidence intervals. State the event prevalence and operating threshold. Use paired participant-level bootstrap intervals where permissible under the data-use agreement; do not bootstrap across correlated observations as if they were independent.

### Treatment-effect and policy performance
A treatment-selection claim requires an explicitly defined estimand for treatment-effect heterogeneity and a prespecified policy. For randomized datasets, evaluate policy value using the known assignment mechanism and an appropriate randomized or doubly robust estimator. Report uncertainty, overlap/positivity diagnostics, assignment probabilities, missingness and sensitivity analyses. Do not use a high outcome-prediction AUC as a substitute for treatment-effect validation.

### Calibration, uncertainty and abstention
Report calibration overall and by predeclared clinically relevant strata where permitted. Include risk–coverage or selective-prediction analysis. A model must abstain when the population, site, modality, preprocessing, data quality or uncertainty falls outside its declared support.

### Clinical usefulness
Report decision-curve/net-benefit analyses only for prespecified, clinically defensible thresholds and intended actions. A threshold chosen after looking at test outcomes is exploratory. No single metric establishes clinical utility.

### Subgroups and fairness
Predefine which demographic/site strata can be analysed lawfully and with adequate sample size. Report denominator, uncertainty and suppression for small cells. Do not overstate fairness from a handful of underpowered subgroups or disclose sparse categories.

## 8. Benchmark targets: ambition without leaderboard gaming

| Gate | Target | Claim allowed |
|---|---|---|
| Exact reproduction | Match published baseline within a predeclared tolerance after cohort/endpoint verification; explain all deviations | Reproduction only |
| Incremental value | Demonstrate a paired, uncertainty-bounded improvement over the strongest clinical-only and published imaging baselines on the same held-out people | Retrospective incremental prognostic value |
| Cross-trial stretch | AUC ≥ 0.70 in both prespecified directions is an **aspirational research target**, not a clinical threshold; also require calibration, uncertainty and decision-utility reporting | Stronger two-trial transport result, not clinical utility |
| Third-trial validation | Preserve useful discrimination and calibration without retuning on the external test cohort | Independent transportability evidence |
| Treatment-policy value | Positive, prespecified policy-value difference versus the best clinical-only comparator, with uncertainty excluding no benefit in an adequately powered randomized evaluation | Evidence supporting further clinical evaluation |
| Prospective impact | Independently reviewed randomized impact study demonstrates better patient-centred outcomes without unacceptable harms or subgroup degradation | Only then consider clinical translation |

The project should not claim to “beat SOTA” by exceeding an AUC reported in another task, population or validation design. The stronger target is to outperform the best matched comparator across trials, remain calibrated under shift, abstain safely outside support, and demonstrate treatment-policy value prospectively.

## 9. Paradigm-shift hypotheses

1. **Static response label → longitudinal state trajectory.** Model symptom, sleep, behaviour and network changes as time-indexed evidence, while strictly separating prediction times.
2. **One dense brain scan → missingness-aware multimodal evidence.** Combine clinical, fMRI, dMRI and EEG only where provenance and participant overlap are verified; retain clinical-only operation as a baseline.
3. **Response prediction → counterfactual treatment benefit.** Estimate whether the expected outcome differs by treatment, rather than whether a person is generally likely to improve.
4. **Black-box score → evidence-carrying prediction.** Bind every output to source release, feature manifest, model hash, validation regime, calibration, uncertainty and limitations.
5. **Forced output → calibrated abstention.** Return “not supported” under distribution shift, missing inputs, unvalidated demographic/site strata or stale policy.
6. **Single random split → lifecycle validation.** Use trial/site/time held-out testing and silent prospective evaluation.
7. **Central data pooling → governed computation near the data.** Explore federated computation only where institutions approve it and where update leakage, secure aggregation and differential-privacy composition are measured.
8. **Model benchmark → decision-system benchmark.** Measure outcome benefit, harms, calibration, access burden, time-to-result, costs and subgroup impact—not only discrimination.

## 10. Failure and leakage controls

The frozen challenge suite must include:
- duplicate participants or related visits across splits;
- site/scanner/batch confounding;
- outcome leakage and post-baseline information leakage;
- preprocessing/harmonization fitted on test data;
- feature selection before split;
- trial-arm imbalance and treatment confounding;
- outcome definition mismatch;
- missing modality coded as a meaningful zero;
- subject-derived embeddings/gradients treated as anonymous;
- small subgroup disclosure;
- out-of-distribution site, device and population;
- stale, revoked or scope-mismatched research authorization.

A failed data-authority or privacy gate blocks analysis regardless of model quality.

## 11. Reproducibility receipt

Every run should emit a versioned aggregate-only receipt conforming to [evaluation-receipt-v0.1.schema.json](schemas/evaluation-receipt-v0.1.schema.json). The receipt binds:
- approved study and analysis scope;
- dataset release and governed access context;
- protocol, source code and preprocessing digests;
- exact train/test direction and endpoint;
- model and environment identity;
- aggregate metrics, uncertainty and limitations.

It must not include participant identifiers, row-level features, predictions, raw MRI/EEG, exact dates of individual visits, embeddings, gradients or small-cell statistics. Dataset hashes and model hashes can themselves reveal linkage in some threat models; disclosure review remains mandatory before publication.

## 12. References

- Zhukovsky P, et al. Cross-trial prediction of antidepressant response. *JAMA Network Open* (2025). [10.1001/jamanetworkopen.2025.1310](https://doi.org/10.1001/jamanetworkopen.2025.1310).
- Zhukovsky P, et al. A precision medicine trial of bupropion and sertraline using biomarker-guided sequential multiple-assignment design. *Nature Mental Health* (2026). [10.1038/s44220-026-00671-z](https://doi.org/10.1038/s44220-026-00671-z).
- Esmaeilian M, et al. Predictive neuroimaging biomarkers of MDD treatment response: an umbrella review (2026). [10.1111/pcn.70053](https://doi.org/10.1111/pcn.70053).
- Takahashi et al. Digital Twin Brain (2026). [10.34133/bmef.0231](https://doi.org/10.34133/bmef.0231).
- Ruhrberg Estévez et al. Causal inference and digital twins: a roadmap for the future of clinical trials. *npj Digital Medicine* (2026).
- Brain-CODE Data Access Request process: https://indocconsortium.atlassian.net/wiki/spaces/JSDNXT/pages/2001895483/Data+Access+Request+Form+FAQ
- NIMH Data Archive: https://nda.nih.gov/
- HL7 FHIR R5 Consent: https://hl7.org/fhir/R5/consent.html
