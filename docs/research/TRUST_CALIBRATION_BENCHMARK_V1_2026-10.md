# Trust Calibration Benchmark V1 — Score Arithmetic Before Model Claims

**Status:** research-only score reference and evaluation protocol; no TCDM calibration result is claimed  
**Reference profile:** `BinaryProbabilityScoreReferenceV1`  
**Source branch:** `research/trust-calibration-benchmark-v1`  
**Input corpus:** `crates/mycelix-core-types/tests/data/trust-calibration-reference-v1.json`  
**Rust oracle:** `crates/mycelix-core-types/tests/trust_calibration_reference_v1.rs`  
**Follow-up implementation:** [TCDM transition-contract PR #5223](https://github.com/Luminous-Dynamics/mycelix/pull/5223)

## Paradigm shift: distinguish trust state from a probability forecast

A trust value, interaction score, evidence grade, posterior parameter, calibrated probability and governance decision are not interchangeable objects. A model can emit a scalar in `[0,1]` without that scalar being a probability of a named future event.

This reference accepts only an explicitly declared probability forecast (p_i\in(0,1)) for a consistently defined binary outcome (y_i\in\{0,1\}). It does **not** consume production `TCDMState`, reinterpret `trust_level` as probability, or assert that the current confidence update is calibrated.

The immediate objective is narrower: verify score arithmetic and freeze adversarial examples before running a real calibration study. The study itself needs a prospective label definition, time/source/entity holdout, leakage checks and sufficient independent observations.

## Scores and diagnostics

For (n) forecast/outcome pairs:

[
\operatorname{BS}=\frac{1}{n}\sum_{i=1}^{n}(p_i-y_i)^2
]

[
\operatorname{LogLoss}=-\frac{1}{n}\sum_{i=1}^{n}
\left[y_i\ln p_i+(1-y_i)\ln(1-p_i)\right]
]

Both are proper scores for binary probability forecasts; lower is better. Brier score combines calibration/reliability, resolution and outcome uncertainty, so it is **not** a pure calibration measure by itself. Log loss makes confidently wrong forecasts costly and requires probabilities strictly inside ((0,1)). See the [scikit-learn probability calibration guide](https://scikit-learn.org/stable/modules/calibration.html) and [Nixon et al., *Measuring Calibration in Deep Learning* (CVPR Workshops 2019)](https://openaccess.thecvf.com/content_CVPRW_2019/html/Uncertainty_and_Robustness_in_Deep_Visual_Learning/Nixon_Measuring_Calibration_in_Deep_Learning_CVPRW_2019_paper.html).

A real evaluation must report a **profile**, not one magic number:

- Proper scores: Brier and log loss, with bootstrap confidence intervals and paired comparisons against a base-rate forecast and the existing baseline.
- Reliability: observed event frequency versus predicted probability, calibration intercept/slope, sample counts and uncertainty intervals.
- Resolution/discrimination: the ability to rank genuinely different outcome rates, reported separately from calibration.
- Selective prediction: coverage-risk curves and outcomes among abstained/referred decisions. A system that refuses hard cases must not improve its apparent scores merely by silently dropping them.
- Stability/fairness: the same metrics by preregistered task, source quality, time period and relevant operating slice, with minimum support counts and interval estimates.
- Dependence: group by independent issuer, source lineage, identity and time. Near-duplicate/correlated reports must not be counted as independent confirmations.
- Operational harm: false acceptance, false rejection, delayed escalation and downstream decision cost according to an explicitly reviewed policy.

Expected calibration error (ECE) may be included only as a **secondary, fully specified diagnostic**. Its result depends on binning, and a single ECE number can conceal local miscalibration. Report the bin rule, per-bin support, reliability diagram and uncertainty; do not set ECE as the sole release gate.

## Frozen reference corpus

The v1 corpus contains **18 cases: 9 valid and 9 invalid**. Valid cases cover good discrimination, overconfident errors, constant forecasts, under/overprediction under a base-rate shift, and near-boundary interior probabilities. Invalid cases cover (p=0), (p=1), negative and above-one probability, non-finite values, empty samples, mismatched lengths and non-binary outcomes.

The independent Rust oracle checks each metric against frozen expected values with a relative/absolute initial tolerance of (10^{-12}\cdot\max(1,|expected|)). It also asserts two decision principles:

1. Two forecasts may share the same mean probability and event rate yet have very different proper scores. Aggregate calibration alone does not establish useful discrimination.
2. A base-rate shift can make an old constant forecast poor even when its old prediction rate had been appropriate for a prior period.

Six mutation controls target wrong Brier arithmetic, incorrect log-loss terms, boundary probability acceptance, non-finite/range errors, malformed labels/batches, and the false claim that aggregate calibration or raw TCDM confidence proves predictive validity.

## Study protocol before calibrating Mycelix

### 1. Name the target

Define the forecasted event before collecting labels, including the entity, action/outcome, horizon, censoring, label authority and treatment of missing/contested outcomes. A vague target like “is trustworthy” is not a statistical estimand.

Keep three stages separate:

- **Evidence admission:** whether an observation is admissible for a task under provenance/time/rights rules.
- **Probability forecast:** probability of a named future or withheld binary outcome.
- **Decision policy:** action threshold, abstention or referral based on costs and authorization.

Better probability calibration does not itself authorize a transaction, grant identity or prevent collusion.

### 2. Split without leakage

- Partition by time and independent source/issuer or entity lineage before fitting calibrators.
- Keep duplicates, copied claims and linked records in one partition.
- Fit any calibration transform only on the training/calibration split; select thresholds without inspecting the locked test set.
- Keep a true held-out final period. Publish sample sizes and class prevalence.
- Compare the legacy method, an uninformative empirical base-rate forecast and any new posterior/transform on identical held-out examples.

### 3. Preserve adversaries and missingness

Test outcome base-rate changes, cold-start entities, missing and delayed evidence, disputed/withdrawn evidence, duplicated reports, colluding sources, shared upstream issuers, and distribution shift. Evaluate each slice with support counts and intervals. A source-count increase must not automatically become an independent-evidence increase.

### 4. Measure decision consequences

Do not choose a production threshold from Brier score alone. Report a threshold sweep with false-accept, false-reject, abstention/referral, coverage and declared decision costs. If calibration error differs sharply across source or task slices, do not average it away.

## Initial acceptance gates

These are **proposed targets for the reference/study protocol**, not measured model achievements.

| Gate | Initial target | Interpretation |
|---|---:|---|
| Score arithmetic reference | 9/9 valid cases match frozen scores | Pure numeric contract only |
| Domain rejection | 9/9 invalid cases reject | No (p=0/1), invalid range, non-finite input or malformed batch accepted |
| Proper-score integrity | Brier and log-loss terms match the equations on every valid case | ECE is never the sole score |
| Determinism | 100/100 repeated fixture evaluations serialize identically | Same fixed inputs, outputs and environment |
| Forecast completeness | 100% of held-out eligible examples counted or explicitly assigned an abstention/censoring reason | No silent dropping of hard cases |
| Held-out evaluation | No calibration fit on locked test labels | Leakage is a study failure |
| Baseline comparison | Paired bootstrap intervals for Brier/log loss against both empirical base-rate and existing method | Report effect and uncertainty, not just a point winner |
| Calibration diagnostics | Reliability diagram plus intercept/slope and declared-bin ECE, each with support/uncertainty | No single metric can hide a failing slice |
| Dependence tests | Explicit source/lineage-clustered split; no copied source counted as independent | Correlated-source resistance is not inferred from score calibration |
| Decision safety | False-accept / false-reject / coverage-risk curves for the reviewed decision policy | Probabilities do not replace authority rules |

A future research hypothesis could be **at least 5% relative Brier-score improvement over both the base-rate and incumbent forecasts on a preregistered, sufficiently powered held-out corpus, without a critical-slice regression**. This is not a release threshold until the outcome, sample size, subgroup support and confidence interval are agreed before scoring.

## Relationship to other Mycelix work

- [PR #5221 — posterior calculation reference](https://github.com/Luminous-Dynamics/mycelix/pull/5221) tests bounded posterior calculations. It is not an empirical calibration study.
- [PR #5223 — TCDM transition-contract candidate](https://github.com/Luminous-Dynamics/mycelix/pull/5223) tests candidate state-transition semantics, including invalid inputs and time sentinel behavior. Its proposed confidence increment is not validated as a probability.
- [Issue #5222 — TCDM update hardening](https://github.com/Luminous-Dynamics/mycelix/issues/5222) remains the production hardening tracker. Source ownership/mirror questions must be resolved before a production transition change.

This PR adds neither a production confidence-to-probability mapping nor a governance threshold. The next stage is a held-out empirical study backed by independently defined outcomes and authoritative evidence, not a rename of existing scores.

## Sources

- [Nixon et al. (2019), Measuring Calibration in Deep Learning](https://openaccess.thecvf.com/content_CVPRW_2019/html/Uncertainty_and_Robustness_in_Deep_Visual_Learning/Nixon_Measuring_Calibration_in_Deep_Learning_CVPRW_2019_paper.html)
- [scikit-learn: Probability calibration](https://scikit-learn.org/stable/modules/calibration.html)
- [Mycelix posterior reference, PR #5221](https://github.com/Luminous-Dynamics/mycelix/pull/5221)
- [Mycelix TCDM transition candidate, PR #5223](https://github.com/Luminous-Dynamics/mycelix/pull/5223)
