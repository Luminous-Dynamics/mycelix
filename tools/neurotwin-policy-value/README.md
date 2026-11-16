# NeuroTwin randomized policy-value reference

**Status:** research prototype. Synthetic tests only; not a clinical tool or validated estimator for an actual study.  
**Issue:** [NEUROTWIN-002 / #6278](https://github.com/Luminous-Dynamics/mycelix/issues/6278).

This Rust crate evaluates two **already-frozen** treatment policies in a two-arm randomized dataset using an augmented inverse-probability weighted (AIPW) estimator. It compares a candidate policy with a comparator (normally a strong clinical-only policy) using paired per-observation scores.

It does not train a model, learn a treatment rule, estimate individualized treatment effects, verify the randomization protocol, check cross-fitting provenance, or authorize a treatment decision.

## Why this exists

A prognostic model estimates outcomes under observed care. A treatment policy claims that choosing one action rather than another improves outcomes. Those are different estimands.

For a policy \(\pi(X)\), the reference computes:

\[
\widehat V_{DR}(\pi)=\frac{1}{n}\sum_{i=1}^{n}\left[
\widehat\mu_{\pi(X_i)}(X_i)
+\frac{\mathbb{1}\{A_i=\pi(X_i)\}}{e_{A_i}(X_i)}
\left(Y_i-\widehat\mu_{A_i}(X_i)\right)
\right].
\]

- \(A_i\) is the randomized assigned arm.
- \(e_{A_i}(X_i)\) is the **known, row-specific assignment probability from the trial protocol**.
- \(Y_i\) is the prespecified outcome, oriented so larger values are better.
- \(\widehat\mu_a(X_i)\) is the predicted outcome under arm \(a\), generated out-of-fold or on a genuinely independent training sample.
- Policies and prediction models must be frozen before evaluation on the test participants.

The candidate-minus-comparator standard error uses paired per-observation influence values. AIPW policy-value estimation is a recognised approach in individualized treatment-rule research; see Zhao et al. (2021), DOI [10.1080/01621459.2020.1725522](https://doi.org/10.1080/01621459.2020.1725522), and the 2025 randomized-trial ITE methods review [10.1007/s10654-024-01185-7](https://doi.org/10.1007/s10654-024-01185-7).

## Input contract

The input is a strict tab-separated file. The header must match exactly:

```text
observation_key<TAB>assigned_arm<TAB>p_arm_a<TAB>p_arm_b<TAB>outcome<TAB>policy_candidate<TAB>policy_comparator<TAB>mu_arm_a<TAB>mu_arm_b
```

Columns are tab-delimited, not space-delimited. Values:

- `observation_key`: opaque local-only key, used for duplicate detection and deterministic row ordering. It is never emitted in the report.
- `assigned_arm`, `policy_candidate`, `policy_comparator`: exactly `A` or `B`.
- `p_arm_a`, `p_arm_b`: actual row-specific randomization probabilities; both must be strictly between 0 and 1 and sum to 1.
- `outcome`: one prespecified, fully observed outcome; larger must mean better.
- `mu_arm_a`, `mu_arm_b`: arm-specific predicted outcomes for that held-out observation.

The tool rejects extra/missing columns, blank rows, duplicate keys, unsupported arm labels, non-finite numbers, non-positive probabilities and probabilities that do not sum to one. It does not infer assignment probabilities from observed arm frequencies or clip extreme weights.

## Run

From this directory:

```bash
cargo fmt -- --check
cargo test --locked
cargo clippy --all-targets -- -D warnings
cargo run --locked -- fixtures/synthetic-randomized-trial.tsv /tmp/neurotwin-policy-value.json
```

The CLI refuses to write an aggregate report when fewer than 20 observations are present. The minimum may be increased as required by the provider or ethics protocol; it cannot be lowered below 20. A provider's stricter disclosure rule always wins. The 20-observation floor is a conservative tool default, not a universal disclosure standard.

The output contains aggregate policy estimates, confidence intervals, assignment-match coverage, effective sample size and maximum inverse-probability weight. It never serializes observation keys, raw rows or individual scores. Keep input/output inside the approved controlled environment; aggregate outputs can still disclose information in small or highly distinctive cohorts.

## Assumptions and limits

- Exactly two treatment arms with known randomized assignment probabilities and positivity.
- One independent row per randomized participant. Repeated observations, cluster randomization, adaptive/dynamic regimes and multi-arm trials are unsupported.
- Fully observed outcome. Missing outcomes, censoring, competing risks and intercurrent-event strategies require a separate prespecified estimator.
- Both arm-specific predictions and policies are out-of-fold or externally trained. The tool cannot establish this; bind their source, split and configuration digests in the upstream run receipt.
- The supplied propensity probabilities exactly match the trial randomization mechanism, including any stratification.
- The normal-approximation 95% interval is exploratory and may be unreliable in small samples. Final trial inference should use a prespecified design-appropriate method, potentially randomization inference, bootstrap or cluster-robust methods.
- Doubly robust estimation does not fix leakage, positivity violations, incorrect estimands, non-random assignment, model overfit, missing data or external-validity failures.

This is a reference calculation, not an ITE learner or treatment recommendation. A positive estimate on synthetic or retrospective data does not establish patient benefit. No clinical decision should use this output.


## Multi-arm extension

The `multi_arm` module and `neurotwin-multiarm-policy-value` binary generalize the same estimator to 2–8 randomized treatment arms. The input header declares the arm registry and supplies row-specific randomization probabilities and arm-specific predictions. Probability and prediction arm order must match exactly.

The evaluator reports candidate/comparator AIPW policy values, paired difference and uncertainty, plus assignment and policy-selection support per arm. It rejects unknown arms, duplicate keys, missing arm fields, non-finite values, positivity violations, probabilities that do not sum to one, and policies that choose an arm with no observed randomized match in the evaluation sample.

```bash
cargo run --locked --bin neurotwin-multiarm-policy-value -- \
  fixtures/synthetic-three-arm.tsv /tmp/multiarm-report.json
```

The three-arm iSPOT-D cohort is only a potential future validation source. Its published symptom-specific C-index analysis is not an antidepressant-response AUROC or treatment-policy result. No iSPOT-D data are included or accessed here.
