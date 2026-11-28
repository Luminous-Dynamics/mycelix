# Praxis benchmark next frontier: decision quality, not just prediction

Status: proposed qualification agenda; no learning-efficacy results claimed.

## Why this exists

The current capability-plan PR establishes deterministic planning contracts and synthetic software-cost workloads. It does not establish educational effectiveness, model superiority, fair outcomes, or credential validity. This document proposes the next evaluation increment and keeps these claims separate.

Recent research signals worth operationalizing:
- Badran and Preisach, *Enhancing Knowledge Tracing through Leakage-Free and Recency-Aware Embeddings* (published 2026-10-01): expanded knowledge-component sequences can leak the same question's label across components; recency-aware inputs may improve prediction. Use a time-valid information contract and preserve question grouping during evaluation. https://link.springer.com/article/10.1007/s42979-026-05344-3
- Jiang et al., NeurIPS 2026, *An Information-Theoretic Evaluation Framework for Benchmark and Model Diagnosis in Knowledge Tracing*: global scores can hide local predictability and benchmark saturation; diagnose by interaction context rather than relying only on aggregate AUC. https://github.com/KernelInterrupt/kt-benchmark-diagnosis
- Kim and Lu, *BeShift: Behavior-informed data augmentation for robust and fair deep knowledge tracing under distributional shifts* (Knowledge-Based Systems, 2026-06-07): reports improved FairAUC under shifts on two datasets. Treat this as a robustness baseline to reproduce, not a guaranteed Praxis result. https://doi.org/10.1016/j.knosys.2026.115947
- A 2026 EdNet analysis reports strong aggregate calibration can hide subgroup miscalibration and incorrect mastery-promotion decisions. Add decision-level and subgroup metrics. https://ieeexplore.ieee.org/document/11596401/

## Paradigm shift

Do not optimize only for "predict the next response." Optimize for "choose an intervention that improves independently demonstrated capability while respecting learner agency and uncertainty."

This requires separating:
1. prediction: probability of a response to an item;
2. diagnosis: which capability is uncertain and what evidence supports that estimate;
3. decision: which eligible intervention is worth trying next;
4. outcome: whether the learner improves on a held-out task and retains that improvement;
5. authority: whether a credential or authorization policy is satisfied (not decided by a model score alone).

## Evaluation tracks

### A. Information validity and leakage
- Freeze the prediction-time information set for each example.
- Keep question-level grouping when one response maps to multiple concepts.
- Test shuffled labels, duplicated rows, future timestamps, same-question leakage, and target-derived feature injection.
- Report the gap between naive and leakage-safe metrics; never report only the inflated number.

### B. Predictive quality and diagnosis
- Baselines: simple item/KC frequency, logistic regression, BKT, DKT, SAKT, AKT, SAINT and a reproducible current strong baseline appropriate to each dataset.
- Report AUC and AUCPRC alongside Brier score, log loss, calibration error, calibration slope/intercept and decision curves.
- Stratify by prior-performance band, data sparsity, item/KC multiplicity, recency, and relevant access conditions when ethically collected and adequately powered.
- Add per-context/local-uncertainty diagnostics where an independent benchmark implementation is available. Avoid comparing scores from different splits/preprocessing.

### C. Intervention value (the product metric)
- Randomized or defensibly matched comparison of: fixed route; existing Praxis adaptation; Praxis + bounded Symthaea advice; and an external adaptive baseline.
- Primary endpoint: independently graded transfer on unseen tasks after a preregistered interval.
- Co-primary or key secondary endpoints: 30/90-day retention, time to demonstrated competence, calibration, false-mastery rate, learner drop-out, assistance-off performance and learner-reported agency.
- Report confidence intervals, effect sizes, missingness, attrition and subgroup effects; power the study before interpreting a null result.
- A stretch hypothesis of >=10% lower median time-to-competence is acceptable only if the retention non-inferiority margin is preregistered and met. Do not choose the margin after seeing results.

### D. Robustness and fairness under shift
- Hold out a later time period and at least one different course/cohort where data agreements permit.
- Evaluate fatigue/recency/forgetting shifts, sparse-history learners, class imbalance, changes in item difficulty, and curriculum changes.
- Report worst-group and distribution-shift performance, subgroup calibration, false-mastery and false-non-mastery rates, not only global averages.
- Synthetic augmentation must be ablated against no augmentation, random augmentation and a robust-optimization baseline; synthetic data must never be presented as observed learner data.

### E. Software and contract qualification
- Keep the current graph-shape runtime benchmark separate from educational metrics.
- Add cycle, unknown-reference, infeasible budget, maximum-size and arithmetic-boundary negative controls.
- Add a property-based generated DAG strategy with shrinking only after checking existing dependency policy; retain every minimized failure as a regression case.
- Record exact commit, data versions/digests, split generator, preprocessing, seed, environment, raw predictions and evaluator version.
- An absent runner or missing result is "unverified", never PASS.

## SOTA comparison rules

A result is comparable only when dataset version, split, filtering, label grouping, prediction-time information, metrics and evaluation code match. Reproduce the strongest accessible baseline first. Publish per-dataset results and confidence intervals, not only a macro-average. If the result does not replicate, report that.

Do not claim "state of the art" from a single public leaderboard, an unpaired split, an aggregate AUC improvement, or faster planner timing. A credible improvement should either:
- improve a preregistered decision/outcome metric while maintaining retention and fairness constraints; or
- provide equal outcome quality with materially lower runtime, memory, compute or data requirements.

## First implementation increment

1. Add this protocol and an issue checklist.
2. Audit the current benchmark/evidence contract for data provenance, timestamps, task/rubric versions and prediction-time availability.
3. Build a small, public synthetic leakage fixture with a known answer and tests that fail if the same question's label is visible through another KC.
4. Create an evaluator interface that accepts frozen prediction records and computes calibration, AUCPRC, subgroup metrics and decision-level false-mastery without accessing training code.
5. Keep all personal learner records local/private by default; publish only synthetic fixtures and aggregate results that pass privacy review.
