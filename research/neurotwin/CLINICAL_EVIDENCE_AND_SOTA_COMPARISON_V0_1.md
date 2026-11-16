# NeuroTwin clinical evidence landscape and SOTA comparison v0.1

**Status:** literature synthesis and proposed qualification criteria; no NeuroTwin clinical result is claimed.  
**Canonical home:** `Luminous-Dynamics/mycelix`.  
**Tracking:** [NEUROTWIN-001 / #6276](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

## Executive finding

The field's central gap is no longer simply finding a high-performing depression-response classifier. It is demonstrating that a model transports across trials and sites, remains calibrated, improves decisions beyond strong clinical-only information, and benefits patients under prospective evaluation.

The current evidence makes this distinction concrete:

- The 2025 EMBARC/CAN-BIND cross-trial study reported AUC 0.62–0.67 for its best baseline clinical + dACC models across the main cross-trial directions in 363 unique participants. It also reported that baseline clinical-only models were around AUC 0.58–0.61, while early-treatment clinical models using week-2 change reached AUC 0.66–0.73 in SSRI out-of-trial comparisons.
- The 2026 SMART-D prospective study reported retrospectively trained marker AUCs of 0.66–0.86, but its preregistered biomarker-consistent versus inconsistent assignment comparison did not show a significant treatment-outcome difference (MADRS trajectory interaction P=0.87; response 61.5% vs 66.7%, P=0.72). A separate marker-count analysis found a difference in MADRS trajectories (time × group P=0.029), but the response-rate comparison across marker-count groups was not statistically significant (Fisher P=0.45). These are different estimands.
- A 2026 umbrella review included 14 reviews/meta-analyses and 17,855 people with MDD. It described some multimodal studies with AUC above 0.85, while judging the overall evidence quality low to very low and noting that most models lacked external validation.

The defensible NeuroTwin strategy is therefore **not to chase the highest reported AUC**. It is to build the strongest reproducible evidence chain from source data to prognostic estimate to treatment-policy value.

## 1. What the strongest published comparators actually show

| Evidence | Population / design | Reported result | What it supports | What it does not establish |
|---|---|---|---|---|
| Zhukovsky et al., JAMA Network Open (2025) | 363 unique participants from EMBARC (225) and CAN-BIND-1 (138); retrospective cross-trial prognostic evaluation | Baseline clinical-only out-of-trial AUC around 0.58–0.61; adding dACC yielded roughly 0.61–0.68, with highlighted directions 0.62, 0.67 and 0.66. Week-2 clinical models reached 0.66–0.73 in SSRI out-of-trial comparisons. | A real, explicit cross-trial baseline and evidence that early clinical trajectory can matter at least as much as baseline imaging additions. | A prospective treatment-selection effect, a universal biomarker, or clinical utility. |
| Zhukovsky et al., Nature Mental Health (2026), SMART-D | Biomarker development from EMBARC, then small prospective randomized sertraline/bupropion study; 47 participants had ≥1 week, 41 completed 8 weeks | Development/LOOCV marker AUCs 0.66–0.86. Consistent vs inconsistent assignment: MADRS trajectory interaction F(1, 38.18)=0.03, P=0.87; response 61.5% vs 66.7%, P=0.72. Marker-count groups differed in MADRS trajectory (P=0.029), but their response-rate difference was not significant (Fisher P=0.45). | A necessary prospective translation comparator; it shows why marker status and actual treatment assignment must be analysed as different questions. | Definitive evidence that biomarker guidance works or does not work: the prospective sample was small and underpowered for many effects. |
| Esmaeilian et al., umbrella review (2026) | 14 systematic reviews/meta-analyses; 17,855 people with MDD; multiple imaging modalities and treatments | Some multimodal models reported AUC >0.85, but methodological quality of the review-level evidence was low to very low; most studies lacked external validation. | A broad map of candidate neuroimaging features and a warning about the quality of the literature. | A directly comparable benchmark result or a validated treatment-selection rule. |
| Takahashi et al., Digital Twin Brain (2026) | Connectome-conditioned model / task-specific behavioural predictions | Strong task-specific BOLD/behaviour prediction in its own setting. | An architecture and mechanistic-modelling comparator. | Direct comparison to MDD antidepressant-response AUC or randomized treatment-policy value. |

**Interpretation rule:** an AUC from one study is not ranked against another unless population, endpoint, prediction time, modality availability, inclusion, split, tuning, and evaluation design are aligned. A high internal/LOOCV result cannot be used as external-validation evidence.

### JAMA cohort and endpoint details that must be reproduced

The published analytic populations need to be represented as a participant-flow graph, not simply added as independent groups:

- CAN-BIND-1 escitalopram: 138 participants.
- EMBARC stage-1 sertraline: 110 participants with complete data.
- EMBARC stage-1 placebo: 115 participants.
- EMBARC stage-2 sertraline: 60 participants, who are a **subset of the stage-1 placebo participants** who did not respond to placebo and then received sertraline.

The headline 363 is 225 EMBARC participants plus 138 CAN-BIND-1 participants. It is not 110 + 115 + 60 + 138 independent participants. The stage-2 subgroup is reused to study a later treatment stage and cannot be counted as a new independent cohort.

The primary response definition is a 50% or greater reduction in depression severity. EMBARC uses HDRS and CAN-BIND uses a validated MADRS-to-HDRS mapping. The CAN-BIND week-8 outcome uses the closest week-6 assessment when week-8 data were unavailable. This endpoint harmonization and its assumptions must be reproduced explicitly; it must not be silently replaced by a single common scale or timepoint.

The paper compared five baseline predictor sets (clinical, clinical + global functional connectivity, clinical + dACC connectivity, clinical + rACC connectivity, and clinical + cortical thickness) and analogous early-treatment sets using week-2 change instead of baseline depression severity. It evaluated 10 machine-learning models in two training scenarios. Its cross-trial bootstrapping used a smaller predictor set of 52 clinical and dACC variables selected after regularization in CAN-BIND-1. The paper reports that early-treatment models performed best overall. A ComBat sensitivity analysis reduced out-of-trial performance slightly without changing the overall pattern.

These details make the reproduction protocol stricter:
1. Preserve subject-level overlap and stage progression; no participant may appear as independent training and testing evidence.
2. Keep baseline-only and week-2 prediction times as distinct tasks.
3. Reproduce both the clinical-only baseline and dACC increment; do not compare Symthaea only with the imaging-enhanced version.
4. Reproduce scale mapping, endpoint timing, missing week-8 substitutions and trial-arm definitions exactly.
5. Bind feature selection, regularization and harmonization to training-only data in each evaluation direction.

## 2. Industry comparison: compare capabilities, not slogans

Clinical prediction and precision-psychiatry programmes generally combine clinical features with candidate biomarkers and evaluate them in development or trial cohorts. The public literature above shows that high retrospective discrimination is achievable in selected samples, while cross-trial performance and prospective treatment assignment remain harder.

NeuroTwin should benchmark against the strongest **matched** comparator, not make unsupported claims about proprietary vendor performance:

| Capability | Published research / industry baseline to respect | NeuroTwin should exceed it by |
|---|---|---|
| Clinical prognosis | Regularized clinical models; early-treatment clinical trajectories; published clinical + dACC baseline | Add repeatable value on the same held-out participants, endpoint and timepoint—not merely on a different cohort. |
| Multimodal modelling | MRI, EEG, behavioural and clinical fusion; selected studies report high internal AUC | Show modality-specific ablations, missing-modality robustness, nested preprocessing, calibration and external trial transport. |
| Brain-network modelling | Functional connectivity and connectome-informed prediction | Demonstrate that topology/network representations add value beyond simpler clinical and imaging baselines; reject hand-authored or synthetic connectomes as biological validation. |
| Digital twins | Mechanistic and connectome-conditioned simulation models | Bind every counterfactual to validated state variables, intervention semantics, uncertainty and prospective observation; changing a graph edge is not automatically a real treatment. |
| Federated learning | Cross-site computation with secure aggregation and optional differential privacy | Quantify leakage risk, privacy-budget composition, update robustness and predictive utility versus approved centralized baselines. Federated is a topology, not a privacy proof. |
| Clinical utility | Decision curves and prospective trial endpoints | Evaluate a locked policy against clinical-only care in randomized data, then prospectively; do not substitute AUROC for patient benefit. |
| Evidence governance | TRIPOD+AI reporting and PROBAST+AI quality/risk-of-bias appraisal | Produce versioned, aggregate-only evidence receipts with dataset authority, exact code and preprocessing digests, splits, model, calibration, decision metrics, exclusions and limitations. |

We should not claim a product is worse or better than a named commercial system until we have a public, independently reproducible, task-matched comparison. Where proprietary methods/data prevent that, state the gap rather than invent a leaderboard.

## 3. Reporting and appraisal standards

### TRIPOD+AI

The 2024 TRIPOD+AI statement provides 27 reporting items for clinical prediction-model studies using regression or machine learning. It is a **reporting guideline**, not a guarantee of low bias or model quality.

NeuroTwin publication receipts and papers should include, at minimum, the intended use, participants and data sources, predictor timing, outcome definition, missing data, model development, internal/external evaluation, performance with uncertainty, calibration, subgroup analysis, and limitations. For clustered/multisite data, consider the additional TRIPOD-Cluster guidance.

Reference: Collins et al., BMJ 2024, DOI [10.1136/bmj-2023-078378](https://doi.org/10.1136/bmj-2023-078378).

### PROBAST+AI

The 2025 PROBAST+AI framework distinguishes model-development quality/applicability from model-evaluation risk of bias/applicability. It covers participants/data sources, predictors, outcomes and analysis, and explicitly addresses fairness and algorithmic bias.

NeuroTwin should use a documented PROBAST+AI appraisal at each major gate:
- before accepting a published result as a comparator;
- after freezing a new model-development pipeline;
- for every external-trial evaluation;
- before treating any prospective evaluation as clinical-translation evidence.

A checklist score must not be used to turn an unvalidated model into a validated one; it helps expose risk and applicability limits.

Reference: Moons et al., BMJ 2025, DOI [10.1136/bmj-2024-082505](https://doi.org/10.1136/bmj-2024-082505).

### External validation

External validation requires a relevant dataset not used in development. Evaluate discrimination, calibration and clinical usefulness; calibration should be examined over the prediction range, and decision-curve/net-benefit analysis is relevant when predictions are intended to direct decisions.

Reference: BMJ external validation guidance, DOI [10.1136/bmj-2023-074820](https://doi.org/10.1136/bmj-2023-074820).

## 4. Proposed NeuroTwin benchmark ladder

These are internal research targets and gates, not clinical cutoffs or regulatory standards.

### Gate 0 — benchmark integrity
- Extract the original paper's exact cohort, endpoint, participant filters, preprocessing, feature set, train/test direction and statistical method.
- Reproduce all three JAMA cross-trial directions before claiming improvement.
- Keep the published result and any modified pipeline in separate tables.
- Add negative controls for label leakage, subject overlap, site/scanner confounding, test-fold preprocessing and post-baseline leakage.

**Acceptance:** reproduction is within a preregistered tolerance after exact cohort/endpoint recovery; every deviation is itemized.

### Gate 1 — matched incremental value
Compare all model families on the exact same participants, outcomes, prediction times, splits and tuning budgets:
1. regularized clinical-only;
2. clinical-only with week-2 features when that is the prediction time;
3. published clinical + dACC baseline;
4. imaging/connectome-only;
5. EEG-only where overlap is sufficient;
6. missingness-aware multimodal fusion;
7. HDC-only, CfC/LTC-only, and connectome + HDC/CfC/LTC.

**Proposed stretch:** paired AUC improvement of at least 0.03 over the strongest matched baseline in both prespecified cross-trial directions, with a paired 95% interval that excludes zero. This is a research target, not a universal minimal clinically important difference. Retain all negative results.

### Gate 2 — calibration and shift
- Report AUROC, AUPRC, balanced accuracy, sensitivity, specificity, Brier score, log loss, calibration intercept/slope and uncertainty.
- Test leave-one-trial-out, leave-one-site-out and time-ordered holdouts where the study permits them.
- Report distribution-shift detection, risk–coverage curves and selective abstention.
- Do not recalibrate on the test cohort and then report that same cohort as untouched external validation.

**Proposed stretch:** maintain useful discrimination with calibration slope between 0.8 and 1.2 after an explicitly disclosed calibration process; demonstrate that abstention reduces error at a predeclared coverage. These ranges are engineering targets to test, not standards of clinical safety.

### Gate 3 — treatment-effect and policy value
- Freeze candidate and clinical-only policies before evaluation.
- In randomized data, use the known randomization probabilities and an estimator appropriate to the design; report positivity/overlap, effective sample size, policy coverage and uncertainty.
- Compare policy value with the clinical-only policy using paired estimates.
- Predefine the clinical outcome, missing-data strategy, intercurrent events and clinically meaningful threshold before analysis.

**Acceptance:** policy-value difference is positive with uncertainty excluding no benefit in a sufficiently powered, independently governed evaluation, and the result survives protocol-consistent sensitivity analyses. A retrospective response model cannot pass this gate.

### Gate 4 — independent prospective evaluation
- Lock preprocessing, features, model, calibration and policy before prospective enrollment.
- Begin with silent predictions that cannot change care.
- Continue to a randomized impact study only after scientific, ethics, governance and clinical review.
- Evaluate patient-centred outcomes, harms, access, time-to-result, workload and subgroup impact—not only model metrics.

## 5. Research paradigm shifts

1. **Response label → longitudinal trajectory.** Model a person's observed course over time with explicit prediction cutoffs. Baseline-only and week-2 models are different tasks.
2. **Prognosis → treatment-effect heterogeneity.** Ask which intervention changes outcomes, not simply who is likely to improve.
3. **One population → transportable model.** Treat trial, site, scanner, geography, treatment protocol and time as explicit sources of distribution shift.
4. **Forced prediction → calibrated abstention.** A system should say that the person, site, modality or input is outside validated support rather than output a confident number.
5. **Opaque score → evidence-carrying counterfactual.** Bind each prediction to input provenance, source-release identity, preprocessing, model version, validation regime, calibration, uncertainty and allowed interpretation.
6. **Static brain graph → intervention-qualified dynamical model.** Require measurable state variables, units, equations, parameter identifiability, intervention correspondence and independent biological validation before interpreting simulated interventions.
7. **Centralized or federated as a binary → governed computation.** Data stays with the authorized provider; privacy threat models, update leakage, secure aggregation and DP composition are explicit.
8. **AUC leaderboard → patient-benefit benchmark.** The final objective is better outcomes with acceptable harms and equitable access, not the largest retrospective discrimination statistic.

## 6. What would count as doing better than SOTA?

The programme can credibly claim leadership only by delivering a combination that current literature rarely demonstrates together:

- reproducible cross-trial performance on a frozen published protocol;
- incremental value over the strongest clinical-only model, including week-2 clinical information where applicable;
- calibration and uncertainty under site/trial shift;
- safe abstention for unsupported cases and missing modalities;
- independent treatment-policy value in randomized data;
- prospective patient-centred benefit in an adequately powered study;
- privacy-preserving, institutionally authorized analysis with independent evidence receipts.

No single AUC threshold can stand in for these gates. A model with AUC 0.90 on a leaky or narrow internal split is not ahead of a calibrated model with modest AUC that transports and improves randomized treatment policy value.

## 7. Reusable source links

- Cross-trial baseline: [JAMA Network Open 2025](https://doi.org/10.1001/jamanetworkopen.2025.1310).
- Prospective biomarker-guided trial: [Nature Mental Health 2026](https://doi.org/10.1038/s44220-026-00671-z).
- Umbrella review: [Psychiatry and Clinical Neurosciences 2026](https://doi.org/10.1111/pcn.70053).
- Digital Twin Brain comparator: [Bioengineering & Translational Medicine / BME Frontiers 2026](https://doi.org/10.34133/bmef.0231).
- TRIPOD+AI: [BMJ 2024](https://doi.org/10.1136/bmj-2023-078378).
- PROBAST+AI: [BMJ 2025](https://doi.org/10.1136/bmj-2024-082505).
- External validation: [BMJ 2024](https://doi.org/10.1136/bmj-2023-074820).

This document is a synthesis and proposed qualification plan. Literature results are not NeuroTwin results, and published metrics are not directly comparable unless their full task and validation context match.
