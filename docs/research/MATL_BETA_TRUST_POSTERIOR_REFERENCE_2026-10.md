# MATL Trust Posterior Reference v1 — Evidence, Uncertainty, and Conservative Decisions

**Status:** research-only arithmetic reference; not a production trust replacement  
**Source baseline:** `Luminous-Dynamics/mycelix@3a7833513144e90c230a65488c78e7c31bf685a3`  
**Canonical code inspected:** `crates/mycelix-core-types/src/trust.rs`, blob `cb2d5c18dd58be7d237cc2050d4df1b55fd4d62a`  
**Reference fixture:** `crates/mycelix-core-types/tests/data/beta-trust-posterior-reference-v1.json`  
**Oracle:** `crates/mycelix-core-types/tests/beta_trust_posterior_reference_v1.rs`

## Research question

Can Mycelix distinguish **a high score** from **strong evidence that a score is reliable** without smuggling unsupported assumptions into governance? The first pilot is deliberately narrow: binary interaction outcomes under a stationary, conditionally independent Bernoulli model. It creates a reproducible mathematical baseline against which MATL/TCDM and later robust, provenance-aware models can be compared.

This is not a claim that human trust, community worth, consciousness, identity, or governance legitimacy is Bernoulli-distributed. The binary outcome is only a bounded proxy for one explicitly defined kind of interaction.

## Current implementation observations

At the source baseline above:

- `TrustScore` computes a fixed weighted score: (T=0.4\,PoGQ+0.3\,TCDM+0.3\,Entropy). It represents the result primarily as a point score.
- `TCDMState::compute_tcdm_score` returns `trust_level * confidence`. This collapses estimated quality and confidence into one value, so downstream users cannot recover the underlying uncertainty from the product alone.
- `TCDMState::update_from_interaction` caps `weight` above at 0.3, but does not clamp it below zero or reject a non-finite weight. It also adds 0.05 confidence for every interaction regardless of weight. These semantics merit explicit tests and review; the note does not assert that every caller can supply arbitrary weights.
- `TCDMState::apply_decay` derives elapsed time from `current_time - last_update` without an explicit backward-time policy. For a decay rate in ((0,1)), a negative elapsed duration yields a factor above one; an old timestamp should not silently boost trust. Very large timestamp differences also deserve overflow review.
- `TrustScore::is_valid` checks its three components, but does not validate that the public cached `total` is finite or equals the recomputed weighted sum. Public fields permit stale or inconsistent cached values until recalculation.
- `PoGQMetrics` exposes floating-point inputs, including quality score, whose domain and finite-value behavior should be checked at all boundaries where untrusted values enter.

These are code-review findings to turn into focused regression tests and API contracts. They are not by themselves a full security assessment, and this reference deliberately does not patch those production methods.

## Reference mathematics

Let (	hetain[0,1]) be the success probability of the narrowly defined interaction. Choose finite prior parameters (\alpha_0>0) and (\beta_0>0). Given (s) distinct, conditionally independent successes and (f) failures, the Beta–Bernoulli update is:

[
\alpha=\alpha_0+s,\qquad \beta=\beta_0+f.
]

The posterior mean and variance are:

[
\mu=\frac{\alpha}{\alpha+\beta},\qquad
\sigma^2=\frac{\alpha\beta}{(\alpha+\beta)^2(\alpha+\beta+1)}.
]

A fixed point score is not enough to communicate uncertainty. As a deliberately conservative and dependency-free research bound, this profile applies Cantelli's one-sided inequality to the posterior random variable. For an input tail probability (0<\varepsilon<1), define:

[
L_\varepsilon=\max\!\left(0,\;\mu-
\sqrt{\sigma^2\frac{1-\varepsilon}{\varepsilon}}\right).
]

Cantelli states that (\Pr(X-\mathbb{E}X\le -d)\le\sigma^2/(\sigma^2+d^2)) for finite-variance (X) and (d>0). Substituting (d=\sqrt{\sigma^2(1-\varepsilon)/\varepsilon}) bounds the posterior mass below (L_\varepsilon) by (\varepsilon); when the result is clamped to zero the bound is trivially valid for a variable supported on ([0,1]). This can be quite conservative; its purpose is to create a simple, transparent reference rather than to claim an optimal interval. See [MathWorld's statement of the one-sided Chebyshev/Cantelli inequality](https://mathworld.wolfram.com/One-SidedChebyshevInequality.html).

The prior is explicit, not inferred from an individual's identity or perceived value. The corpus uses Beta(1,1) as the uniform default and Beta(1/2,1/2) as a prior-sensitivity case. A different prior changes the results and therefore must be justified before deployment.

## Assumptions and hard boundaries

The reference applies only when every observation really is a distinct, correctly labelled, conditionally independent binary trial over a time window where the success probability is sufficiently stationary.

It **does not** by itself solve:

- duplicate-event replay or event deduplication;
- Sybil identities, collusion, source dependence, or correlated evidence;
- source authenticity, appeals, corrections or evidence revocation;
- nonstationarity, drift, temporal weighting or decay;
- multi-valued outcomes, missing-not-at-random evidence, or selection bias;
- fairness, legitimacy, consent, moral status, or the normative choice of a governance threshold.

Those belong in separate contracts. Counting copies of the same event as independent observations makes the posterior overconfident. Provenance and dependency controls must be established before production evidence is fed to the posterior.

The lower bound is a probability bound under the assumed Bayesian posterior model, **not** a frequentist confidence interval, a guarantee that the model is calibrated on future data, or proof that a source tells the truth. A governance decision additionally requires domain-specific loss, an explicit abstention/defer path, accessibility and appeal requirements, and authorized policy—not merely comparing this number to a generic threshold.

## Frozen reference and checks

The v1 corpus contains **22 cases: 13 valid and 9 invalid**. It covers:

- uniform and Jeffreys priors, cold start and success/failure extremes;
- balanced and asymmetric evidence;
- outcome ordering invariance and reflected success/failure cases;
- zero/negative/non-finite prior parameters, invalid tail probabilities, posterior overflow, and non-boolean evidence;
- five named semantic mutation controls: ignored observations, swapped outcomes, missing variance denominator term, inverted Cantelli ratio, and replacing the uncertainty-adjusted bound with the mean.

The Rust integration oracle uses only the existing `serde_json` dev dependency. It is separate from the current MATL/TCDM implementation; passing it validates the reference arithmetic and fixture contract, not production correspondence.

Initial reference acceptance gates:

| Gate | Initial target |
|---|---:|
| Frozen valid corpus | 13/13 accepted and numerically matched |
| Frozen invalid corpus | 9/9 rejected |
| Deterministic parameter grid | 3,456 checks; all variance/range/bound monotonicity invariants hold |
| Named semantic mutants | 5/5 caught by tests |
| Reference numerical tolerance | (10^{-12}\max(1,|expected|)) |
| Production claims from this PR alone | 0 |

## Experiments that could justify a production adapter

Run the following in order, preserving the present MATL behavior as the baseline and making no decision-policy changes in this reference PR.

1. **Analytical qualification.** Run the integration oracle locally with the pinned toolchain, review the fixture diff and test output, and independently recompute the 13 valid examples.
2. **Same-data comparison.** Generate synthetic Bernoulli streams from known rates. Compare current MATL/TCDM, posterior mean, and conservative lower bound on identical data. Plot risk/coverage and confidence-versus-effective-sample size; report prior sensitivity.
3. **Calibration.** On a frozen held-out suite, record Brier score, log loss, calibration slope/intercept, reliability bins with preregistered boundaries, and coverage of posterior intervals against the generating process. Report sample counts and uncertainty, not only point estimates.
4. **Adversarial evidence.** Test duplicate IDs, repeated source reports, correlated source clusters, missing outcomes, source concentration, cold-start gaming, timestamp regression and distribution drift. Until the higher-layer provenance contract is qualified, mark the duplicate/correlation cases as known non-claims of this posterior primitive.
5. **Decision outcomes.** Compare false-accept/false-reject risk at matched decision coverage and with explicit abstention. A threshold is a policy decision and must be selected using the cost of error in its target domain.
6. **Only if justified:** propose a separate production adapter with source identity, event uniqueness, time semantics and failure behavior typed at the boundary. Never replace the existing MATL score based only on this mathematical reference.

Proposed research targets—not achieved results:

- at least 5% relative improvement in held-out Brier score versus the point-score baseline under the preregistered stationary Bernoulli suite;
- report log-loss and calibration metrics alongside that result; target expected calibration error no greater than 0.05 with fixed published binning;
- no regression larger than 2 percentage points in any adequately powered, predeclared subgroup;
- zero invalid numeric inputs accepted and zero duplicate event replays counted twice in the future integrated evidence pipeline;
- zero promotion to production if the assumptions are violated or calibration fails under the tested shift regimes.

If these targets fail, keep the research artifact, publish the result and revise the model rather than relaxing gates after seeing results.

## Cross-project implications

- **Symthaea:** posterior/uncertainty separation may inform belief reporting and decision abstention, but EIG, calibration and truth remain distinct.
- **Symtropy:** use an analogous separation between model state, numerical uncertainty and empirical validation; a mathematical invariant is not automatically a physical measurement.
- **Mycelix:** the immediate potential is to avoid treating a cached score as an evidence-strength guarantee. Provenance, dependency and governance semantics must be qualified independently before this posterior has any production influence.

## Source links

- [Mycelix core trust types at the inspected baseline](https://github.com/Luminous-Dynamics/mycelix/blob/3a7833513144e90c230a65488c78e7c31bf685a3/crates/mycelix-core-types/src/trust.rs)
- [Mycelix core-type crate manifest](https://github.com/Luminous-Dynamics/mycelix/blob/3a7833513144e90c230a65488c78e7c31bf685a3/crates/mycelix-core-types/Cargo.toml)
- [Cantelli's one-sided inequality](https://mathworld.wolfram.com/One-SidedChebyshevInequality.html)
