# NeuroTwin finite-sample randomized policy inference v0.1

**Status:** research reference; exact-head Rust qualification is required before treating the implementation as checked. Synthetic data only. No real cohort access or clinical use is represented.  
**Tracks:** [NEUROTWIN-006 / #6290](https://github.com/Luminous-Dynamics/mycelix/issues/6290), extending [the two-arm policy-value reference / #6279](https://github.com/Luminous-Dynamics/mycelix/pull/6279).

## What changes

The v0.1 AIPW evaluator reports a paired influence-function normal approximation. This extension adds a separate, conservative finite-sample confidence interval for one narrow design class:

- one independent row per randomized participant;
- independent participant-level Bernoulli treatment assignment;
- exact, known, row-specific assignment probabilities from the trial protocol;
- two randomized arms;
- one fully observed outcome per participant, bounded by caller-declared finite limits;
- no interference between participants;
- fixed policies and arm-specific outcome predictions frozen before evaluation, with predictions produced out-of-fold or on independent training data.

It deliberately rejects blocked or stratified permuted-block randomization, cluster/site randomization, repeated measures, adaptive regimes and unsupported designs. A design is not made valid by supplying row-level probabilities if assignments are dependent.

## Estimand and bound

For participant i, let pi_i and kappa_i be the fixed candidate and comparator actions, and let Y_i(a) be the participant's fixed potential outcome under arm a. The paired AIPW contrast contribution is:

```
D_i = mu_pi - mu_kappa
    + 1(A_i = pi_i) * (Y_i - mu_pi) / e_i(pi_i)
    - 1(A_i = kappa_i) * (Y_i - mu_kappa) / e_i(kappa_i)
```

Under the declared independent Bernoulli assignment mechanism, conditional on fixed potential outcomes and frozen policies/predictions, E[D_i] = Y_i(pi_i) - Y_i(kappa_i). The finite-sample target is the average of those individual policy contrasts over the evaluated participants.

For declared outcome bounds [L,U], the row contribution is bounded by the maximum and minimum of its candidate-matched and comparator-matched residual corrections. Let its range be R_i. Hoeffding's inequality gives the two-sided interval radius:

```
r = sqrt( log(2 / delta) * sum_i(R_i^2) / (2 n^2) )
```

The report is the paired AIPW estimate ± r. This is a finite-sample randomization-design guarantee, not an asymptotic standard-error interval. It is intentionally conservative; weak positivity and wide outcome bounds increase R_i and can make the interval uninformative. The existing normal interval is included only as an explicitly exploratory comparator.

## Run

The CLI requires an explicit design declaration and finite outcome bounds:

```bash
cargo run --locked --bin neurotwin-policy-value-finite-sample -- \
  fixtures/synthetic-randomized-trial.tsv /tmp/finite-sample-report.json \
  --randomization-unit participant \
  --assignment-mechanism independent_bernoulli \
  --outcome-bounds 0,20 \
  0.05
```

Optional final argument: MIN_REPORT_N, default 20; it cannot be lowered below 20. Outputs are aggregate-only and created with create-new semantics so an existing file is never overwritten.

## What is not claimed

- This does not cover stratified, blocked, cluster, site, repeated-measure or sequential/adaptive randomization.
- It does not correct missing outcomes, censoring, intercurrent events, non-adherence, interference or protocol deviations.
- It does not prove that randomization probabilities are correct, models are cross-fitted, policies were locked, or declared outcome bounds are appropriate.
- It is not a universal confidence interval for observational AIPW estimates.
- It is not a treatment recommendation, a clinical validation result or evidence of patient benefit.

Next methods should be separate, design-specific additions: stratified randomization inference with within-stratum assignment replay, cluster-level inference using the randomized cluster count, and longitudinal estimands with explicit repeated-measure and missingness strategies. Never dispatch a generic participant bootstrap for all designs.
