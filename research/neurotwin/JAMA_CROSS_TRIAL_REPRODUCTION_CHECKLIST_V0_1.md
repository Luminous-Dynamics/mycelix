# JAMA EMBARC / CAN-BIND cross-trial reproduction checklist v0.1

**Source:** Zhukovsky et al., *JAMA Network Open* 2025;8(3):e251310. DOI [10.1001/jamanetworkopen.2025.1310](https://doi.org/10.1001/jamanetworkopen.2025.1310).  
**Status:** extraction from the publication; no participant data or author code were run.  
**Tracking:** [NEUROTWIN-001 / #6276](https://github.com/Luminous-Dynamics/mycelix/issues/6276).

This sheet is a source-to-implementation map for the first NeuroTwin benchmark. It deliberately records the publication's cohort relationships, outcome definitions and split logic before any controlled-data access.

## 1. Cohort identity and overlap

### Trial populations

| Source group | Analytic n | Treatment/stage | Relationship |
|---|---:|---|---|
| CAN-BIND-1 | 138 | Escitalopram, stage 1 | CAN-BIND analytic sample included participants with at least 6 weeks of escitalopram and complete baseline prediction data. |
| EMBARC stage 1 sertraline | 110 | Sertraline | Participants with complete data in the stage-1 sertraline arm. |
| EMBARC stage 1 placebo | 115 | Placebo | Stage-1 placebo arm. |
| EMBARC stage 2 sertraline | 60 | Sertraline after placebo nonresponse | **Subset of the 115 stage-1 placebo participants**, not a separate set of unique baseline participants. |

The article's headline sample is **363 unique participants = 225 from EMBARC + 138 from CAN-BIND-1**. Do not add 110 + 115 + 60 + 138 and call the result the participant sample. The 60 stage-2 participants are a treatment-stage subset of the stage-1 placebo arm.

### Trial design context

- EMBARC: two-stage study; 4 academic sites in the United States; adults aged 18–65 with MDD. Stage 1 randomized sertraline or placebo. At 8 weeks, sertraline nonresponders switched to bupropion; placebo nonresponders switched to sertraline; responders continued.
- CAN-BIND-1: two-stage study; 6 outpatient centres in Canada. Stage 1 escitalopram for 8 weeks; nonresponders could receive escitalopram augmentation with aripiprazole; responders continued.
- The cross-trial task is not simply "US vs Canada": treatment, stage, inclusion, endpoint timing and cohort composition also differ.

## 2. Outcomes and predictor timing

### Primary binary response

Response = at least 50% reduction in depression severity.

- EMBARC severity uses HDRS-17.
- CAN-BIND uses MADRS converted to HDRS using a validated mapping described in the paper supplement.
- EMBARC stage-1 outcome: week 8.
- EMBARC stage-2 outcome: week 16.
- CAN-BIND outcome: week 8; closest available week-6 assessment was used when week-8 data were unavailable.

Any reimplementation must preserve this mapping and timing or label a changed definition as a separate analysis.

### Secondary continuous outcome

The paper used partial least squares regression (PLS-R) to predict change in depression severity. Out-of-trial predicted-versus-observed correlations for SSRI-to-SSRI directions were reported around 0.31–0.39. Do not compare these correlations directly with AUC.

### Predictor sets

Baseline clinical model included:
- age;
- sex;
- employment status;
- body-mass index;
- baseline depression severity (HDRS-17 in EMBARC; mapped MADRS in CAN-BIND);
- anhedonia (SHAPS).

Imaging additions included:
- seed-based functional connectivity features from dACC, rACC and global functional connectivity;
- cortical thickness / structural MRI features.

Early-treatment models replaced baseline depression severity with change in depression severity from baseline to week 2. This is a separate prediction-time task; do not report it as baseline prediction.

## 3. Published model-performance table

The following values are copied from the article's Table 2 (AUC / balanced accuracy). They are literature baselines, not NeuroTwin results.

| Training → test group | Clinical baseline AUC / BA | Clinical + dACC AUC / BA | Early clinical AUC / BA | Early clinical + dACC AUC / BA |
|---|---:|---:|---:|---:|
| CAN-BIND-1 → EMBARC stage-1 sertraline | 0.58 / 0.61 | 0.62 / 0.63 | 0.68 / 0.69 | 0.68 / 0.64 |
| CAN-BIND-1 → EMBARC stage-2 sertraline | 0.58 / 0.60 | 0.67 / 0.65 | 0.73 / 0.66 | 0.79 / 0.73 |
| CAN-BIND-1 → EMBARC stage-1 placebo | 0.63 / 0.59 | 0.57 / 0.55 | 0.71 / 0.66 | 0.70 / 0.69 |
| EMBARC stage-1 sertraline → CAN-BIND-1 | 0.59 / 0.59 | 0.66 / 0.64 | 0.66 / 0.69 | 0.59 / 0.58 |
| EMBARC stage-1 sertraline → EMBARC stage-2 sertraline | 0.61 / 0.60 | 0.70 / 0.71 | 0.68 / 0.66 | 0.71 / 0.70 |
| EMBARC stage-1 sertraline → EMBARC stage-1 placebo | 0.52 / 0.55 | 0.62 / 0.61 | 0.63 / 0.66 | 0.64 / 0.63 |

The first, second and fourth rows are the most relevant treatment-group cross-trial directions (CAN-BIND↔EMBARC sertraline). The third and sixth rows are placebo-transfer checks. The fifth row is a different EMBARC treatment stage in the same parent trial and must not be misdescribed as an independent external trial.

Important interpretation:
- The article's text highlights AUC 0.62 for CAN-BIND→EMBARC stage 1, 0.67 for CAN-BIND→EMBARC stage 2, and 0.66 for EMBARC stage 1→CAN-BIND.
- In several directions, early clinical information matches or exceeds baseline clinical + dACC models. Week-2 predictors are not available at baseline.
- dACC additions do not improve every transfer direction. Global functional connectivity did not improve the SSRI cross-trial pattern consistently.
- The article reports moderate overall performance, two trials only, and non-preregistered analytic methods; it is a baseline to reproduce, not a definitive clinical standard.

## 4. Preprocessing and modelling details to recover

- Structural and resting-state fMRI processed with fMRIPrep 22.1.1 for EMBARC and 23.0.2 for CAN-BIND-1.
- Fixed confound regression for denoising and 6-mm smoothing.
- Five baseline predictor sets: clinical, clinical + global FC, clinical + dACC FC, clinical + rACC FC, and clinical + cortical thickness.
- Five analogous early-treatment sets that use week-2 symptom change instead of baseline severity.
- Ten machine-learning models were evaluated across two training scenarios.
- Within-trial elastic-net analyses used 100 random train/test splits; model training used 10-fold CV on the training partition.
- Cross-trial analyses used bootstrapping; the cross-trial model subset included 52 clinical and dACC variables retained after CAN-BIND regularization. The article also reports a full 366-predictor bootstrap sensitivity analysis.
- Secondary PLS-R analyses used 360 dACC connectivity features plus clinical/demographic predictors, with 5,000 permutations and 5,000 bootstraps for significance/feature stability analyses.
- ComBat batch-harmonization sensitivity analysis slightly reduced out-of-trial performance but did not change the overall pattern.

Before claiming exact reproduction, obtain and inspect the article's eAppendix 2, eAppendix 3, participant-flow eFigure 1, and supplementary data table. The summary above is a faithful extraction of the public main article, not a substitute for its full analysis appendix.

## 5. Reproduction execution checklist

### Before data access

- [ ] Confirm source dataset release/version and source-specific access approval.
- [ ] Confirm exact inclusion counts against the participant-flow figure.
- [ ] Confirm legal/ethics permission for each modality and derived feature.
- [ ] Confirm whether the analysis workspace permits code, package, model, or aggregate-result export.
- [ ] Record prohibited linkage, retention, deletion and publication rules.

### In the approved controlled environment

- [ ] Create a participant-flow manifest with unique individuals and repeated treatment-stage observations clearly distinguished.
- [ ] Verify all stage membership relationships; do not treat EMBARC stage 2 as new independent participants.
- [ ] Recreate outcome mapping, endpoint time, response threshold and missing-week-8 substitution.
- [ ] Recreate baseline and week-2 tasks separately.
- [ ] Reproduce all source predictor sets and all three primary cross-trial treatment-group directions.
- [ ] Keep imputation, scaling, harmonization, regularization and feature selection inside the training data for each direction.
- [ ] Report both AUC and balanced accuracy; also sensitivity, specificity, calibration and uncertainty where supported by original or new analysis.
- [ ] Keep placebo-transfer and same-trial stage-transfer results as distinct secondary analyses.
- [ ] Preserve every failed reproduction and pipeline deviation in the run receipt.

### After reproduction

- [ ] Compare NeuroTwin model families against clinical-only, week-2 clinical, and clinical + dACC baselines on identical people and splits.
- [ ] Report paired uncertainty for incremental performance, not a difference of unrelated point estimates.
- [ ] Use an untouched third trial for external validation.
- [ ] Keep treatment-effect and policy-value evaluation separate from response prediction.

## 6. Common reproduction errors this checklist is designed to prevent

1. Counting EMBARC stage 2 as 60 additional unique people when reporting cohort size.
2. Treating stage-1 placebo and stage-2 sertraline as independent when they share participants.
3. Calling a week-2 model a pretreatment model.
4. Substituting MADRS response for the article's mapped HDRS outcome without documenting a changed endpoint.
5. Ignoring the week-6 substitution rule in CAN-BIND.
6. Training feature selection or harmonization on the held-out trial.
7. Ranking AUC, balanced accuracy and symptom-change correlation as the same metric.
8. Comparing baseline NeuroTwin predictions to the published early-treatment clinical model without matching prediction time.
9. Claiming a treatment-selection effect from a prognostic response model.
10. Treating a successful structural audit or BIDS validator run as proof of clinical outcome validity.

## 7. Source

Zhukovsky P, Trivedi MH, Weissman M, et al. Generalizability of Treatment Outcome Prediction Across Antidepressant Treatment Trials in Depression. *JAMA Network Open*. 2025;8(3):e251310. [https://doi.org/10.1001/jamanetworkopen.2025.1310](https://doi.org/10.1001/jamanetworkopen.2025.1310).

Cohort counts, trial-stage relationships, predictor timing, outcome definitions, model comparisons, performance values and sensitivity analyses in this checklist are extracted from the article's main text and Table 1/Table 2. This repository has not run the original analysis, downloaded controlled data, or verified the supplementary code.
