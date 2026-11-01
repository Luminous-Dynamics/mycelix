# Robust Posterior and Acquisition Qualification Benchmark v0.1

**Status:** mathematical specification and synthetic qualification plan. No robust acquisition algorithm, posterior-SBC engine, or physical-data result is claimed by this document.

**Tracker:** [TERRA-ROBUST-002 / #5648](https://github.com/Luminous-Dynamics/mycelix/issues/5648)

**Dependencies:** #5632 (covariance and Bayesian calibration), #5633 (multi-world evaluation), #5643 (CT–tracer observation operators), and the sequential implementation in #5601.

## 1. Why nominal EIG is insufficient

Expected information gain is optimal only relative to its specified prior, candidate models, likelihood and utility. If the observation model is misspecified, a high-information action can concentrate belief in the wrong mechanism, amplify a biased observation operator, or select an unrepresentative measurement regime.

The new benchmark must separate:

- **Inference correctness:** does the computational method approximate the posterior implied by the declared model?
- **Model adequacy:** can the declared model family represent the observed physical system?
- **Design robustness:** does the acquisition policy remain decision-useful when the nominal data-generating law is perturbed within a declared ambiguity set?
- **Domain representativeness:** do selected measurements cover the intended target soil/support/forcing domain rather than only the easiest-to-model region?

Passing one dimension does not imply passing the others.

## 2. Maximin / distributionally robust design comparator

Abdulsamad et al. (2026) formulate robust Bayesian design as a max–min problem between an experimenter and an adversarial data-generating process constrained to a Kullback–Leibler neighborhood around a nominal model. Their formulation produces Sibson α-mutual information as a robust information objective, and uses PAC-Bayesian stochastic design policies to obtain high-probability lower bounds on robust utility.

Our implementation must define the ambiguity set explicitly. For a candidate design ξ and nominal distribution q₀(y | θ, ξ), a representative ambiguity set is:

```text
Q_ε(ξ) = { q : D_KL(q || q₀(· | θ, ξ)) ≤ ε }
```

The exact orientation, conditioning, base measure, and whether the ambiguity set applies to joint or conditional observations must follow the selected published formulation. This line is a schema reminder, not a complete implementation definition.

The target is not to add an arbitrary worst-case penalty. It is to reproduce the paper's objective and assumptions faithfully, then test where its guarantees do and do not apply.

### Mandatory comparator ladder

1. Nominal conditional EIG.
2. Decision-focused EVSI.
3. Nominal robust-risk / worst-case expected decision regret.
4. Sibson α-mutual information at preregistered α values.
5. Stochastic policy optimized using a PAC-Bayes lower confidence bound.
6. Random and expert-fixed controls.
7. Exact enumeration for finite small-world reference cases.

Include α limits and identity/reference cases where analytical values are available. Report Monte Carlo bias/variance, sample count, confidence bounds and non-convergence; do not substitute point estimates for certified lower bounds.

The R-IDeA method must be a separate comparator. A local representativeness/error-amplification heuristic must not be labelled as a reproduction of R-IDeA.

## 3. Physical ambiguity sets for Terra Preta

Build declared perturbation families around the nominal CT–tracer and hydraulic models:

- CT resolution, reconstruction and segmentation variants;
- support misalignment and boundary truncation;
- specimen, block, batch and instrument random effects;
- cross-modality and temporal covariance misspecification;
- tracer censoring, interval-missingness, concentration bias and tail under-sampling;
- omitted hydraulic activation, matrix–macropore exchange, lateral flow or dynamic morphology;
- parameter/prior shifts and discrepancy-prior/likelihood sensitivity;
- rare target strata and unseen site/soil/forcing distributions.

Keep two separate channels:

**Aleatoric / measurement variation:** repeated outcomes under the same declared physical model.

**Epistemic / discrepancy variation:** uncertainty or omitted-process alternatives about the observation and physical model.

The adversary can perturb only the declared probabilistic data-generating mechanism inside its ambiguity set. It cannot change hard eligibility, budget, assignment unit, target estimand, safety limits, or evaluator/selector separation.

Report results across multiple ambiguity radii. A result that only works at one tuned radius is not a robustness result.

## 4. Prior SBC and posterior SBC are different qualification stages

### Prior SBC

Prior simulation-based calibration tests the inference implementation across prior-predictive worlds. For each replicate:

1. draw θ' from the prior;
2. simulate y' from the likelihood conditional on θ';
3. obtain posterior draws θ'' conditional on y';
4. calculate randomized/discrete PIT ranks for parameter and data-dependent test quantities;
5. compare rank/PIT distributions to their preregistered reference distribution.

Include joint log likelihood as a test quantity in addition to model identity, covariance parameters, discrepancy parameters and scientific decision functionals. Modrák et al. (2025) show that test-quantity choice changes SBC sensitivity; a posterior equal to the prior can evade parameter-only checks if the data are ignored.

### Posterior SBC

Posterior SBC targets inference conditional on specific observed data, not average performance over the prior.

For each preregistered fixed synthetic observation set y_obs:

1. obtain θ' draws from the approximate posterior p(θ | y_obs);
2. simulate posterior-predictive y' conditional on θ' and y_obs;
3. obtain new draws θ'' from the augmented posterior p(θ | y_obs, y');
4. compute PIT values comparing θ' against the augmented posterior draws, conditional on y_obs and y';
5. assess uniformity with suitable discrete-PIT handling, simultaneous confidence bands, and dependence-aware Monte Carlo procedures.

The same inference engine is evaluated on both posterior fits. Posterior SBC does not require access to the exact true posterior, but it is still a necessary-not-sufficient computational diagnostic. It does not establish physical-model adequacy; posterior predictive / discrepancy tests remain necessary.

The fixed y_obs fixtures should target:
- weakly informative likelihood and hierarchical funnel geometry;
- strong likelihood and narrow/high-curvature posterior;
- parameter/correlation near admissible covariance boundaries;
- model-discrepancy and multi-modal posterior states;
- support-stratum imbalance and influential observations;
- deliberately misspecified likelihoods and approximate solvers.

For discrete model indicators use randomized PIT or rank tie-breaking appropriate to the discrete posterior. For continuous covariance/discrepancy parameters include joint and folded/non-monotone test quantities where needed. Predeclare test power, multiplicity and compute budgets.

## 5. Paired policy inference and uncertainty decomposition

A policy contest must not compare overlapping marginal confidence intervals and infer that a difference exists or does not exist.

For each world w and paired replicate r, preserve each policy's outcome under shared potential-observation randomness where scientifically valid. For policies A and B, compute paired loss differences:

```text
Δ(w,r) = regret_A(w,r) - regret_B(w,r)
```

Report the mean/median and tail distribution of Δ, paired confidence intervals, probability of a practical improvement, and family-stratified results.

Uncertainty levels must remain distinct:

- replicate Monte Carlo error within one world;
- world-to-world variability within a mechanism family;
- between-family variability;
- data-group/site/specimen variability for later empirical evaluation.

Use cluster-aware or hierarchical resampling at the level of independent synthetic world and mechanism family. Never resample voxel, timepoint or repeated assay rows as though they were independent assignment units.

## 6. Required outputs

- nominal and worst-case decision regret;
- robust objective estimate and valid high-probability lower bound, with all assumptions;
- posterior calibration and predictive proper scores;
- prior and posterior SBC rank/PIT diagnostics;
- false discrepancy alerts and missed discrepancy rates;
- abstention and unsupported-domain rates;
- sensitivity to ambiguity radius, α, prior and discrepancy likelihood;
- paired policy-effect intervals and family-level uncertainty;
- measurement cost, simulator calls, runtime, and solver/estimator gap;
- every invalid, vacuous, failed or non-convergent run.

## 7. Proposed qualification gates (targets, not results)

- 100% hard feasibility, eligibility, cost and target-coverage compliance across all adversarial worlds.
- Exact reproduction of analytical maximin/Sibson reference values within preregistered numerical tolerances.
- Empirical coverage of PAC-Bayes lower bounds consistent with the stated confidence level under the generator assumptions; report vacuous bounds, never clip them into apparent success.
- Prior SBC and posterior SBC pass their separate reference controls; deliberate likelihood/sampler negative controls are detected at preregistered rates.
- Robust acquisition reduces worst-case regret against nominal EVSI on adversarial families, with paired uncertainty excluding zero, while retaining non-inferior behavior on in-family controls.
- No model receives credit for a narrow posterior under a false observation operator; predictive discrepancy, false-alert cost and calibration are separate endpoints.
- At least 10 mechanism families and 1,000 worlds for the mature paired policy campaign, with exact oracle gaps for small worlds.

These are research targets, not outcomes, field guarantees or universal tolerances.

## 8. Literature and source limitations

- Abdulsamad et al. (2026), *Maximin Robust Bayesian Experimental Design*, arXiv:2603.14094 / ICML: https://arxiv.org/abs/2603.14094. Uses a max–min adversarial formulation, KL-bounded perturbations, Sibson α-MI and PAC-Bayes policy lower bounds.
- Säilynoja et al. (2026), *Posterior SBC: simulation-based calibration checking conditional on data*, Statistics and Computing 36, 78: https://doi.org/10.1007/s11222-026-10825-9. Posterior SBC compares original posterior draws with augmented posterior draws after posterior-predictive simulation, conditional on observed data.
- Tang, Sloman & Kaski (2026), *Representative, Informative, and De-Amplifying*, AISTATS/PMLR 300: https://proceedings.mlr.press/v300/tang26d.html.
- López-Fidalgo & Wong (2026), *Optimal Designs for Correlated Data*, Annual Review of Statistics and Its Application 13:245–270: https://doi.org/10.1146/annurev-statistics-042324-012947. The review highlights that optimal design for correlated observations lags correlated-data estimation, with limited general algorithms and few tools to certify design optimality. This reinforces the need for exact small-case oracles and explicit approximation/solver gaps in our benchmark.
- Modrák et al. (2025), *Simulation-Based Calibration Checking for Bayesian Computation: The Choice of Test Quantities Shapes Sensitivity*: https://doi.org/10.1214/23-BA1404.

The methods are comparators and qualification inputs, not methods originated by this project.

## 9. Implementation order

1. Finish the finite continuous joint-likelihood posterior primitive in #5632.
2. Integrate conditional updates with retained correlated measurement histories and support metadata.
3. Implement exact discrete Posterior-SBC reference cases before continuous posterior samplers.
4. Implement Sibson α-MI and KL-adversarial reference cases where closed form is available.
5. Add a Monte Carlo/PAC-Bayes estimator with coverage tests and resource reporting.
6. Add paired replicate-level policy differences to #5633.
7. Extend to CT–tracer ambiguity families from #5643.
8. Reproduce published comparators faithfully before any claim of superiority.

## 10. Evidence and safety boundary

Synthetic mathematical/software work only. No physical experiment, land access, sample collection, CT scan, tracer injection, soil amendment or agronomic recommendation is authorized. All robustness claims are relative to an explicit ambiguity set and do not imply robustness to arbitrary unknown mechanisms or real-world conditions.


## 11. Implemented finite posterior-SBC control — 2026-11-01

A reference implementation of the conditional Posterior-SBC identity is now authored in `tools/sequential-bayes-design/src/posterior_sbc.rs` and registered only under `cfg(test)`.

It fixes an observed dataset, forms the exact conditional finite posterior, samples an original model state from that posterior, simulates an additional observation from the posterior predictive, and computes the exact augmented posterior. Randomized discrete PIT ranks are checked for both model identity and the joint log-likelihood test quantity over 4,096 replicates. The exact-update control must fall within a 7–13% per-bin reference frequency band; a deliberate likelihood-power-4 overconfidence negative control must violate at least one histogram.

This is an executable finite categorical Posterior-SBC reference test. It does not validate continuous covariance/discrepancy samplers or model adequacy, and it does not replace prior SBC or posterior predictive checks. Exact-head CI is required before calling it a passing test.


## 12. Implemented exact discrete Sibson reference — 2026-11-01

The Rust library now contains `tools/sequential-bayes-design/src/robust_information.rs` with:
- exact finite-discrete Shannon mutual information;
- exact finite-discrete Sibson α-mutual information for (0<\alpha<1);
- the corresponding α-tilted finite posterior (p_\alpha(M\mid x)\propto p(M)p(x\mid M)^\alpha);
- strict probability/identifier and alpha validation.

Six authored tests cover zero-information independence, convergence to Shannon MI as α approaches one, monotonicity over a preregistered alpha grid, CT–tracer correlation-only complementarity, posterior tempering, and invalid inputs. An independent formula check for the complementary discrete fixture gives Shannon MI 0.368064 nats and Sibson MI values 0.097772, 0.184551, 0.258413, 0.290396, 0.319208 and 0.356878 nats at alpha 0.2, 0.4, 0.6, 0.7, 0.8 and 0.95 respectively.

This is the exact deterministic discrete reference stage of #5648. It is not the full maximin game, ambiguity-radius-to-alpha optimizer, nested Monte Carlo estimator or PAC-Bayes stochastic design policy. Exact-head CI must establish compilation and tests.


## 13. Finite worst-case joint and KL-radius inversion — 2026-11-01

The exact finite-discrete robust-information module now implements the geometric-mixture worst-case joint distribution:

```text
q*(m,x) ∝ [p(m) p_alpha(x)]^(1-alpha) [p(m,x)]^alpha
```

It returns the alpha-tilted outcome marginal, q* model/outcome marginals and joint weights, plus (D_{KL}(q^*\|p)). It verifies the conditional identity (q^*(m\mid x)\propto p(m)p(x\mid m)^\alpha). A bisection routine numerically inverts the single-design KL cost to find alpha for a declared ambiguity radius, checking monotonicity across a grid and refusing radii beyond the configured minimum-alpha range.

Independent analytical calculations for the complementary CT–tracer finite fixture give KL radius 0.092332 nats at alpha 0.5, decreasing toward zero at alpha 1. This is a deterministic single-design reference, not the paper's average-radius stochastic-policy optimization. The paper's PAC-Bayes bound and nested estimator remain unimplemented; no robust policy superiority is claimed.


## 14. One-step finite robust design selector — 2026-11-01

The finite robust reference now includes `select_discrete_design`, comparing candidate actions under either nominal Shannon MI or Sibson-α MI. It rejects duplicate/empty action IDs, validates the complete candidate pool, enforces eligibility and action cost ≤ hard budget before objective ranking, uses a deterministic lexicographic action-ID tie break, and returns no selection when no action is feasible.

A synthetic ranking-reversal control has nominal Shannon MI 0.569802 vs 0.476391 nats for two candidates, while at α=0.3 the Sibson values are 0.292592 vs 0.329155 nats. The nominal selector chooses `nominal_sharp`; the robust selector chooses `robust_balanced`. This is a designed finite reference fixture, not evidence that Sibson universally improves real soil experiments.

It remains a deterministic one-step selector; stochastic policy optimization, adaptive multi-step robust trees, PAC-Bayes bounds and physical action/observation integration are not implemented.


## 15. Conditional Rényi identity cross-check — 2026-11-01

Added a second calculation path for the finite Sibson objective. For each nonzero-probability outcome, `conditional_renyi_information` computes (D_\alpha[p(M\mid x)\|p(M)]) using the **ordinary nominal posterior**, as in Proposition 3 of Abdulsamad et al. (2026). `sibson_from_conditional_renyi` reconstructs (I_\alpha^S) as the risk-sensitive exponential average of these conditional gains under the nominal outcome marginal.

An exact cross-check requires agreement with the direct Sibson formula to (10^{-10}) at alpha values 0.1, 0.2, 0.4, 0.6, 0.7, 0.8, 0.95 and 0.999. Additional controls cover uninformative outcomes, zero likelihood support and invalid indices/alpha values. This is an internal mathematical identity check, not independent physical validation.


## 16. Per-design KL-radius selector — 2026-11-01

Added `robust_information_at_kl_radius` and `select_discrete_design_at_kl_radius`. For every fixed finite candidate design, the implementation computes the q* ambiguity cost as a function of alpha, solves for alpha at a requested KL radius by bisection, then compares the resulting Sibson objective. It enforces the hard action budget and eligibility before selection and reports the requested/effective radius, alpha and whether the alpha range was clamped.

The zero-radius control exactly recovers Shannon MI (alpha=1). An uninformative design remains zero-information under positive requested radius because its q* divergence is zero. Tests also verify recovery of a known radius generated at alpha=0.5, feasibility handling and high-radius clamping.

**Important mathematical boundary:** this selector treats each candidate as its own fixed-design KL ambiguity problem. The paper's stochastic-policy formulation uses one shared *average* KL budget over the policy distribution and optimizes the common dual parameter with the corresponding (-\\beta\\rho) term. That stochastic-policy optimization is not implemented; this finite per-design selector must not be described as a faithful reproduction of it.


### Fail-closed robust-selector input validation — 2026-11-01

The fixed-radius and fixed-alpha selectors now validate objective parameters *before* checking whether any candidate is feasible. This closes a fail-open edge case where invalid alpha/radius inputs could be hidden by an empty or fully ineligible candidate pool. Zero-radius calls also validate the configured alpha search range and non-zero bisection budget before returning the nominal limit. Three negative-control tests cover these paths; the robust module now contains 26 authored tests.

## 17. Executable robust one-step bake-off — 2026-11-01

Added a Rust CLI binary at `tools/sequential-bayes-design/src/bin/robust-information-bakeoff.rs`, a synthetic action-pool fixture, and separate input/report JSON Schemas. It generates a SHA-256-bound machine-readable report comparing nominal Shannon selection, fixed-alpha Sibson selection and per-design KL-radius selection under identical actions and hard budget.

The five-action fixture includes: nominal-sharp vs robust-balanced ranking-reversal candidates, a zero-information repeated CT action, a perfectly discriminating but ineligible measurement, and a perfectly discriminating but over-budget measurement. Focused CI now parses both schemas, executes the binary, asserts expected selected IDs and budget/eligibility constraints, validates the report schema and uploads the artifact.

This creates an executable finite comparator. It is still one-step deterministic, not a continuous physical simulator, sequential stochastic-policy optimizer, PAC-Bayes bound, or full selector integration. Exact-head CI must verify the binary and assertions.


### Robust report provenance refinement — 2026-11-01

The executable report now records `minimum_alpha` and `alpha_bisection_iterations` alongside the requested KL radius and fixed-alpha objective. These search controls affect whether a candidate's ambiguity radius is attained or clamped, so they are part of the evidence record and required by the report schema.
