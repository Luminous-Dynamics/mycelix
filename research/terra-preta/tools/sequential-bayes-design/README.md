# Sequential Bayesian experiment-design benchmark v0.1

Synthetic-only Rust programme with a sequential categorical acquisition executable and a reusable bivariate-Gaussian model-posterior library. The continuous likelihood component is not yet wired into the acquisition loop.

## Run

```sh
cargo test --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-sequential-scenario.json \
  sequential-bayes-report.json
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-sequential-infamily-control.json \
  sequential-bayes-infamily-report.json
```

## Policies

- `static_eig`: selects using the initial prior's expected information gain (static ranking baseline).
- `conditional_eig`: recomputes expected information gain after every observed outcome.
- `decision_focused_evsi`: selects by expected value of sample information for a declared service decision.
- `discrepancy_guarded_evsi`: uses decision-focused EVSI plus an EIG tie-break and, after a low-predictive-probability event, prioritizes a predeclared diagnostic modality if one remains.
- `seeded_random`: deterministic feasible baseline.

For every selected action the evaluator samples a categorical observation from the hidden synthetic world. The selector updates a finite posterior over candidate models using the observed outcome and candidate likelihoods. The discrepancy flag is triggered when the observation's posterior-predictive probability falls below the preregistered threshold.

## Separation and metrics

Selector-facing types contain model priors, candidate-model likelihoods, declared decision utilities, eligible actions, costs, strata and modality metadata. They do not contain hidden true observation probabilities or true decision utilities. A negative-control test mutates all evaluator truth fields and verifies that the first selector choice is unchanged.

The report aggregates repeated deterministic seeds and records true synthetic decision utility/regret, predictive surprise events, diagnostic measurements, final posterior entropy, prequential negative log predictive score, prequential multiclass Brier score, and action-selection frequencies. It also scores every final posterior against the same evaluator-only held-out probes, using expected negative log score and expected multiclass Brier score overall and by target stratum. These common probes are not selected by the policies and do not consume acquisition budget; this removes the direct comparison confound caused by policies selecting different measurement modalities. A nuisance-signature control demonstrates that information gain can be substantial while decision EVSI is zero: calculated EIG is 0.3774776716 nats versus zero EVSI for the declared service decision.

## Calibration interpretation

Prequential scores describe prediction on each policy's selected measurement stream; the measurement operators can differ between policies, so those values are operational diagnostics rather than a fully matched calibration contest. Use `mean_heldout_validation_log_score`, `mean_heldout_validation_brier_score`, their standard errors / 95% normal-approximation intervals, and `validation_by_stratum` for the matched comparison: they score every policy's final posterior against the same fixed evaluator-only probe set and synthetic truth distribution.

Intervals are normal-approximation intervals over independent replicate means with the standard error estimated from the sample variance. They are not Bayesian credible intervals, do not account for scenario-family uncertainty, and should be supplemented with paired bootstrap intervals when comparing policies across large preregistered scenario suites. Common random numbers reduce action-level Monte Carlo noise but do not make different adaptive policies identical or remove the need for replicated worlds.

## Limits

This is an exact finite categorical model, not a continuous soil simulator. It assumes conditionally independent action outcomes and does not model correlated sensor noise, heteroscedastic likelihoods, continuous measurements, censored data, a learned discrepancy posterior, or physical water/solute equations. A surprise is evidence that the candidate model mixture assigned low predictive probability; it does not by itself identify the missing physical mechanism.

No field data or physical experiment is authorized. The synthetic true world is deliberately outside the candidate model family to test misspecification handling, not to represent a measured soil regime.


## Simulation-based calibration reference test

The test suite includes a discrete prior-predictive SBC check with 4,096
replicates. It checks randomized ranks for the true model index and for the
data-dependent joint log likelihood. A deliberate likelihood-misspecification
negative control must violate the 7–13% frequency band in at least one of the
two rank histograms. This is an exact finite categorical reference test, not
SBC certification for continuous physical, covariance or discrepancy models.


The discrepancy-state tests also vary its prior weight and residual-outcome
likelihood. The posterior discrepancy weight must be monotonic in both, and the
declared decision must switch from acting under physical candidates to
DIAGNOSE_FIRST at the preregistered synthetic threshold. This is sensitivity
analysis for an author-specified state, not empirical prior calibration.


## Conditional information gain

The `information_gain` module estimates model-identity information in a
candidate next measurement from conditional Gaussian mixture components:
`I(M; Y_next | Y_history)`. It uses deterministic seeded Gaussian sampling,
stable log-mixture-density evaluation and reports Monte Carlo standard error
and total sample count. Identical predictive components and a single-positive-
weight model are exact zero-information controls.

Its `estimate_action_information_gain` API connects the estimator directly to
the stable observation layout, retained history and a candidate action. An
adversarial CT–tracer control has identical univariate marginals under two
candidate models but opposite cross-modal correlations: CT alone and tracer
alone carry zero model information, while the joint pair has positive
information. After observing CT, the same tracer action gains positive
conditional information. This is an executable synthetic demonstration of
complementarity through a correlated observation history.

This estimate concerns model discrimination, not scientific decision value.
It does not include Monte Carlo finite-sample bias, covariance-estimation
uncertainty, model-family uncertainty or physical-model discrepancy. It is
not yet consumed by the categorical CLI selector and is not EVSI.
## Stable observation identities and append-only history

The library now includes `observation_history` with a canonical
`ObservationLayout` (stable coordinate ID, modality, physical support ID and
unit), `ObservationAction` (the coordinates produced by one acquisition),
and append-only `ObservationHistory`. It resolves stable IDs to covariance
indices in the correct value order, rejects unknown/duplicate coordinates,
rejects repeated acquisitions under the same coordinate identity, and rejects
dimension mismatch or non-finite measurements before returning a new history.
A repeated measurement must have a new coordinate identity. Support metadata
is retained as metadata; support overlap is not translated into a correlation
coefficient.

The `conditional_prediction_for_action` bridge takes candidate models,
the layout, an observed history and a candidate action, then returns the
posterior-weighted conditional Gaussian predictive components for that action.
This removes manual numeric-index mapping from downstream conditional EIG/EVSI
code. It is still a library operation, not yet wired into the CLI selector or
used to derive covariance from support relationships.

## Posterior-SBC reference control

The library test suite includes a separate **conditional posterior-SBC** test
in `src/posterior_sbc.rs`. It fixes an observed dataset, computes its exact
finite categorical posterior, draws a model from that conditional posterior,
generates a posterior-predictive observation, updates the posterior with the
augmented data, and computes randomized discrete PIT histograms for both model
identity and joint log likelihood. The exact update must satisfy the
preregistered 7–13% per-bin reference band across 4,096 replicates; an
intentionally overconfident likelihood-power-4 update must fail at least one
histogram.

This is an exact finite categorical reference for the conditional Posterior
SBC identity. It is not a continuous covariance/discrepancy posterior test
and does not establish physical-model adequacy. Prior SBC remains a separate
test.

## Correlated continuous-likelihood posterior component

The crate also exposes `multivariate_gaussian`, a dense-covariance
reference for vector-valued measurement histories. It computes multivariate
Gaussian log densities, scores arbitrary observed subsets, updates finite
candidate-model posterior weights, and derives conditional Gaussian
predictions for unobserved coordinates using the Schur complement. Its
`conditional_predictive_models` operation returns the posterior-weighted
Gaussian mixture components for a proposed next action:
`p(Y_target | Y_history) = Σ_m p(M_m | Y_history)
p(Y_target | Y_history, M_m)`. Empty history is supported and returns prior-
weighted marginal predictive components. The module fails closed on
asymmetric or non-positive-definite covariance, repeated or out-of-range
indices, dimension mismatches and non-finite values. It performs no silent
jitter or eigenvalue clipping.

This is the mathematical handoff needed by conditional EIG/EVSI. It is still
not wired into the acquisition policy or CT/tracer observation operators and
does not estimate covariance parameters; those are separate integration and
calibration tasks.

The crate now exposes a reusable library module, `correlated_likelihood`,
with an exact bivariate-Gaussian joint log density and a stable log-sum-exp
update over a finite set of candidate physical models. Each model declares
its prior mass, two-dimensional observation mean, marginal standard
deviations and correlation. The update returns normalized model posterior
weights and joint log evidence.

Tests cover independence equivalence at zero correlation, conditional
Gaussian factorization, model discrimination from correlation despite equal
marginals, preservation of prior odds under equal likelihoods, channel-swap
symmetry, zero-prior exclusion, evidence calculation, invalid covariance and
prior inputs, and numerical failure when every candidate likelihood is
unrepresentable.

**Integration boundary:** this is production library code, not just a test
helper, but the categorical CLI selector does not yet consume continuous
observations. It updates one jointly observed pair; it does not yet retain
cross-action covariance through a sequential history, estimate covariance
from paired data, support general multivariate covariance, or consume CT /
tracer support metadata. Correlation must come from an explicit simulator or
appropriate paired/repeatability evidence, never support overlap alone.


## Exact adaptive decision-tree reference

The test suite also enumerates the optimal finite adaptive measurement policy
under the declared model prior and service-decision utility. It branches over
candidate categorical outcomes, updates the posterior, preserves mandatory
stratum coverage and budget, and compares the optimal value with the
conditional-greedy decision-EVSI trajectory. This is a test-only exact small
case oracle; it does not access hidden truth and is not yet included as a
runtime report metric. Larger problems need memoization, node caps and
solver-gap/bound reporting.
