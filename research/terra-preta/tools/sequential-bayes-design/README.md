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


## Executable conditional CT–tracer information bake-off

The `conditional-information-bakeoff` binary exercises the stable measurement
layout, retained observation history, multivariate conditional Gaussian
mixture, and seeded conditional information-gain estimator together. Its
synthetic two-model fixture has equal CT and tracer marginals but opposite
cross-modal correlations. It evaluates CT-only, tracer-only and joint
acquisition from the prior, then evaluates tracer after the CT coordinate has
been observed.

The focused workflow requires:
- zero model information for either individual modality under the prior;
- positive model information and top rank for the joint CT–tracer action;
- positive conditional information for tracer after observing CT;
- Monte Carlo standard errors and budget/eligibility checks;
- input and report schema validation and report artifact upload.

This is an executable synthetic action-selection reference, not the full
categorical policy engine, decision EVSI or physical CT/tracer validation.

## Executable covariance-identifiability bake-off

The `covariance-identifiability-bakeoff` binary reads a strict synthetic JSON
fixture and emits a SHA-256-bound report. The three designs are: a single
confounded CT–tracer pair, a batch-only pair, and a fully crossed
support × batch × instrument × modality replication design. The report
includes structural rank, per-component identifiability, Gram condition
number, and exact noiseless coefficient recovery where identified.

Run:

```sh
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml \
  --bin covariance-identifiability-bakeoff -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-covariance-identifiability.json \
  covariance-identifiability-report.json
```

The focused workflow validates both schemas, executes the CLI, asserts
rank/attribution and recovered coefficients, validates the report, and uploads
the artifact. It is a structural covariance-design oracle, not a noisy
variance estimator.

## Transient macropore–matrix exchange

The `transient_exchange` module implements a time-resolved reference for
`q_exchange(t) = a_wetted(t) * j_macropore(t)`, following the dimensional
structure proposed by Chaif et al. (2025, Journal of Hydrology 661, 133791,
DOI 10.1016/j.jhydrol.2025.133791). It takes synchronized wetted-interface
area and macropore-flux histories as explicit inputs, linearly interpolates
each signal, computes their product, and analytically integrates that product
between samples. It also provides a paired macropore-loss / matrix-gain
balance check.

The synthetic tests include constant-input exact integration, exact
integration of the product of two linear signals, partial irregular intervals,
rainfall-onset exchange decaying to zero, reverse flux, invalid-input rejection,
and paired storage closure.

**Boundary:** this is not the full paper model. It does not derive interface
area from CT, solve the horizontal Richards equation, estimate the flux from
macropore dynamics, assimilate states, or establish field performance. The
source paper's infiltration and CT data are available upon request, not a
publicly bundled fixture.

## Covariance-component identifiability

The `covariance_identifiability` module builds exact upper-triangle covariance
basis matrices for shared physical support, preparation batch, instrument and
independent residual effects. It computes the weighted design rank, a Gram
spectral condition diagnostic, and whether each component is individually
identifiable. It also provides a noiseless exact-recovery oracle for synthetic
covariance matrices, which refuses to estimate when the decomposition is
structurally rank-deficient.

The controls demonstrate:
- one CT–tracer pair sharing support, batch and instrument has rank 2/4 and
  cannot attribute its shared covariance;
- a crossed support × batch × instrument × modality design reaches rank 4/4;
- a batch-only CT/tracer pair identifies batch dependence but cannot certify
  a physical shared-support variance component.

This is structural identifiability of the declared linear covariance basis,
not finite-sample precision, posterior calibration or empirical covariance
estimation. Support equality must be explicitly declared; overlap does not
automatically become a correlation coefficient.

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

The module also implements the paper's **conditional Rényi-information
identity**. For each outcome it calculates
`D_alpha[p(M | outcome) || p(M)]` using the ordinary nominal posterior, then
reconstructs Sibson alpha-MI as the risk-sensitive exponential average under
the nominal outcome marginal. This is intentionally independent of the direct
closed-form Sibson calculation and is checked to 1e-10 on multiple alpha
values. Do not confuse the nominal posterior in the Rényi divergence with the
alpha-tilted worst-case posterior used for robust belief updates.

## Exact discrete robust-information reference

The `robust_information` module computes exact finite-discrete Shannon
mutual information and Sibson α-mutual information for a candidate design,
and returns the corresponding α-tilted posterior
`p_alpha(M | x) ∝ p(M) p(x | M)^alpha`.

It also constructs the finite worst-case joint distribution from the
published geometric-mixture expression and reports its KL divergence to the
nominal joint. A bisection routine inverts this single-design KL cost to
recover α for a declared ambiguity radius, checking monotonicity on a grid
and failing when the requested radius exceeds the configured alpha range.
This is a finite deterministic reference; the paper's ambiguity set is
defined over a stochastic design policy with an average KL budget.

Twenty-seven authored tests cover independence/zero-information, convergence to
Shannon MI, monotonicity over an alpha grid, CT–tracer correlation-only
complementarity, posterior tempering, worst-case posterior identities,
worst-case KL cost, radius-to-alpha inversion, and invalid inputs.

The same module now includes a deterministic one-step candidate selector,
`select_discrete_design`, which compares nominal Shannon or Sibson-α
information over a finite action pool while enforcing eligibility and hard
cost budget before ranking. It uses a stable action-ID tie break and returns
`None` when no candidate is feasible. An adversarial ranking fixture shows
that the nominal Shannon-maximizing action can differ from the Sibson-α
choice at the same action pool; this is a one-step deterministic comparison,
not a multi-step experiment policy.

The library also supports a declared **per-design KL ambiguity radius**:
`robust_information_at_kl_radius` numerically inverts each fixed design's
worst-case joint KL cost to choose alpha, and
`select_discrete_design_at_kl_radius` ranks candidate actions at the same
requested per-design radius while enforcing eligibility and budget. It
reports requested/effective radius, alpha and whether the search is clamped.
An uninformative design remains zero-information even under a positive
requested radius. A **perfect deterministic** measurement is a distinct
zero-KL edge case: its q* distribution may equal nominal while Sibson
information remains positive. The implementation scores it at the minimum
alpha and marks the requested positive radius as unattainable/clamped rather
than incorrectly returning zero information.

A second executable path now jointly re-evaluates alpha and the Gibbs
distribution as the common beta varies. For each beta it forms
`alpha=beta/(1+beta)`, computes all finite Sibson rewards, obtains the exact
Gibbs maximizer of
`E_pi[reward] - KL(pi||pi0)/lambda`, and searches the resulting
`E_pi[I_alpha] - KL(pi||pi0)/lambda - beta*rho` objective over the declared
finite beta bracket. It reports the common beta, alpha, action probabilities,
policy KL, complexity penalty, adversarial penalty and robust regularized
value. The zero-radius limit uses Shannon information and beta→infinity.

This joint finite Gibbs/dual reference is stronger than evaluating a
fixed-alpha Gibbs policy at a separate beta, but it still uses exact finite
Sibson rewards. It is not the paper's nested-Monte-Carlo empirical reward
estimator or PAC-Bayes lower-bound certificate, and its numeric outer search
does not certify global optimality beyond the declared finite search settings.

This is a deterministic fixed-design ambiguity comparison. It is **not** the
paper's stochastic-policy problem with one shared *average* KL budget over
the policy distribution; that problem needs a common dual parameter and the
`-beta * rho` term and is still unimplemented.

This is **not** the full maximin policy optimizer, nested Monte Carlo estimator
or PAC-Bayes lower-bound optimizer. The exact finite component is not a
decision-EVSI calculation or a physical-data result.
## Shared-KL stochastic policy and PAC-Bayes Gibbs algebra

The `stochastic_policy` module implements the finite shared-average-KL dual
for a **fixed stochastic policy**:
`sup_beta { E_pi[I^S_{beta/(1+beta)}(action)] - beta * rho }`.
One common beta is used across all actions. It is not inverted separately for
each action. The zero-radius limit returns the policy-averaged Shannon MI.

It also implements the finite Gibbs policy
`pi*(action) ∝ pi0(action) exp(lambda * reward(action))`, which exactly
maximizes the reward-minus-KL variational objective for supplied rewards, and
the arithmetic of the paper's PAC-Bayes lower-bound formula. The latter
requires external justification of the paper's regularity/boundedness
assumptions and constants; the code checks sample-size and numeric conditions
but does not certify them. The nested Monte Carlo estimator is not yet
implemented.

The executable robust bake-off now constructs a Gibbs stochastic policy from
the fixed-alpha Sibson reward estimates, using a uniform prior over feasible
actions and zero prior mass on ineligible/over-budget actions. It then
evaluates that policy through the one-common-beta shared-average-KL dual.
The report preserves action probabilities, policy KL, expected reward,
variational objective, expected cost, common beta, robust policy value and
adversarial penalty. CI asserts normalization, hard feasibility and the dual
value identity. Rewards are exact synthetic Sibson scores, not nested-MC
estimates, so this does not claim a PAC-Bayes lower-bound certificate.

The shared-KL dual is the correct policy-level comparator for the
maximin-robust paper. The earlier fixed per-design KL selector remains a
separate finite comparator and must not be presented as the shared-policy
algorithm.

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


## Executable robust one-step bake-off

The `robust-information-bakeoff` binary reads
`fixtures/synthetic-robust-information-scenario.json` and writes a
machine-readable report. It compares nominal Shannon, fixed-alpha Sibson and
fixed per-design KL-radius selection under identical model/action inputs.
The report binds the exact input SHA-256 and includes each candidate's score,
feasibility, cost, alpha, effective ambiguity radius, search floor and bisection
budget.

Run:

```sh
cargo run --manifest-path research/terra-preta/tools/sequential-bayes-design/Cargo.toml \
  --bin robust-information-bakeoff -- \
  research/terra-preta/tools/sequential-bayes-design/fixtures/synthetic-robust-information-scenario.json \
  robust-information-report.json
```

The focused sequential workflow parses both robust schemas, runs this binary,
asserts the expected ranking reversal and hard-feasibility behavior, validates
the report schema, and uploads the report as an artifact. The fixture is
synthetic and has no hidden physical truth supplied to the selector.

