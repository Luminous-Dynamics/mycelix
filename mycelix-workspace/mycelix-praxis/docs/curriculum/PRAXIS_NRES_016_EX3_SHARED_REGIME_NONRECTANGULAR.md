# PRAXIS-NRES-016-EX3: Persistent shared-regime versus rectangular ambiguity

**Status:** synthetic public-development exact reference; not confirmatory or deployable.  
**Research issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5575.  
**Evaluator:** benchmarks/nres-capability-loop/src/adaptive_recourse.rs.  
**CLI:** cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --shared-regime-ex3.  
**Parent sequence:** EX0 exact expected-cost oracle -> EX1 nested mean-CVaR -> EX2 rectangular transition-TV -> EX3 shared persistent regime.

## 1. Question

EX2 assumes rectangular ambiguity: an adversary may choose the worst conditional transition law independently at every observable information state/action. This is computationally and dynamically convenient, but it may combine conditional distributions that cannot arise from any single persistent physical or economic regime.

EX3 asks how much risk value changes when the uncertainty source is a **single latent regime that persists across the full horizon** rather than an adversary that may switch among incompatible regimes at each node.

Examples of plausible persistent regimes include a regional weather pattern, a sustained port/supplier disruption, an extended workforce availability state, or a shared common-mode infrastructure condition. The regime is hidden from the policy unless it is genuinely revealed by an observation.

## 2. Two ambiguity contracts

Let theta belong to a finite set of persistent regimes Theta. For an observable history h and action a, the next-outcome law is (p_t(\cdot\mid h,a,\theta)). Each theta defines one complete family of conditional transition kernels across the horizon.

### Shared, non-rectangular regime

The controller commits to one policy pi that depends only on observable histories. The adversary chooses one theta for the entire trajectory:

[
J_{shared}(\pi)=\max_{\theta\in\Theta}J_{nested}(\pi\mid\theta),
\qquad
\pi^*=\arg\min_{\pi}J_{shared}(\pi).
]

The same theta must generate all transition probabilities encountered along a path, at every stage and observable node. It cannot be switched after seeing which branch occurs.

### Rectangular relaxation

The rectangular relaxation allows the adversary to select the worst regime-specific conditional kernel independently at each observable information state/action. It replaces the original joint family of coherent regime kernels with a product of local conditional ambiguity sets.

This enlarges the set of admissible path distributions. It can be a useful conservative upper bound, but it may combine locally worst kernels from different regimes into an impossible global scenario.

The evaluator reports the absolute price of rectangularity:

[
\Delta_{rect}=J_{rect}-J_{shared},
\]

and its relative value (\Delta_{rect}/J_{shared}) when the denominator is non-zero. Given aligned model, policy space and risk map, the rectangular relaxation should not be less conservative than the persistent-regime model. A negative gap is a conformance failure.

## 3. Risk and information contract

Both formulations use the same strictly monotone one-period mean-CVaR mapping:

[
\widehat\rho_{\alpha,\lambda}(Z)
=(1-\lambda)\mathbb E[Z]+\lambda\,CVaR_\alpha(Z),
\quad 0\leq\alpha<1,\quad0\leq\lambda<1.
]

Pure CVaR at lambda=1 is intentionally not admitted, for the strict-monotonicity reason documented in EX1 and Füllner & Rebennack (2025).

The policy is selected once as a map from observable information-state IDs to actions. The hidden regime ID is used only in the evaluator's adversarial comparison; it is never injected into observation history or passed as a policy input. The same policy is evaluated under every theta before the maximum is taken.

EX3 inherits EX0's contracts: explicit observation histories, transition-emitted observation equality, unique canonical histories, per-stage resource conservation, action feasibility, pathwise service floors and reachability.

## 4. Exact synthetic fixture

The fixture has two persistent regimes and a two-stage sequential hazard process. The regime is not observed by the controller.

| Transition | NORMAL | STRESS | Equal-weight reference |
|---|---:|---:|---:|
| First-stage failure | 0.05 | 0.01 | 0.03 |
| First-stage survival | 0.95 | 0.99 | 0.97 |
| Second-stage failure after first-stage survival | 0.01 | 0.05 | 0.03 |
| Second-stage survival | 0.99 | 0.95 | 0.97 |

Each failure incurs loss 1,000. A first-stage failure ends the hazard path; the second-stage hazard is encountered only after surviving the first stage. Parameters are alpha=0.90 and lambda=0.90.

The scalar outcome probability field in the fixture is the equal-weight average across regimes. The evaluator checks that equality and separately validates each regime's conditional probability mass at every action.

### Shared regime values

Under NORMAL, the second-stage conditional nested risk after survival is 91. The first-stage conditional risk becomes:

- Expected continuation loss: 136.45;
- CVaR90: 545.5;
- Mean-CVaR value: 504.595.

Under STRESS, the second-stage conditional nested risk after survival is 455. The first-stage conditional risk becomes:

- Expected continuation loss: 460.45;
- CVaR90: 509.5;
- Mean-CVaR value: 504.595.

The persistent shared-regime minimax value is therefore 504.595.

### Rectangular relaxation

The rectangular adversary can choose NORMAL at the first-stage node (failure probability 0.05) and STRESS at the second-stage node after survival (failure probability 0.05). That combination is not generated by either persistent regime.

Its nested values are:
- Second-stage robust continuation value: 455;
- First-stage expected value: 482.25;
- First-stage CVaR90: 727.5;
- Rectangular nested mean-CVaR: 702.975.

| Quantity | Shared regime | Rectangular relaxation |
|---|---:|---:|
| Root nested risk | 504.595 | 702.975 |
| Selected worst regimes | NORMAL or STRESS as a whole | NORMAL at first stage, STRESS at second stage |
| Price of rectangularity | — | 198.38 |
| Relative price | — | 39.3147% |

This is a deliberately small exact counterexample. It demonstrates that the rectangular bound can be materially more conservative when local worst-case kernels cannot coexist under a single persistent cause. It does not prove that persistent regimes are appropriate for any real system.

## 5. Exact evaluator mechanics

The shared-regime evaluator:
1. parses unique persistent regime IDs;
2. validates that every action/outcome declares exactly one probability per regime;
3. checks that each action's conditional probabilities sum to one separately under every regime;
4. checks that the scalar reference probability equals the unweighted regime mean;
5. enumerates complete non-anticipative policies over the finite information-state tree;
6. evaluates each fixed policy separately under each complete regime using nested mean-CVaR;
7. takes the worst complete regime for that policy, then minimizes over policies.

The rectangular evaluator:
1. computes continuation values recursively;
2. for each state/action, evaluates each regime-specific local transition kernel against the rectangular continuation values;
3. chooses the worst local regime independently at that node;
4. lets the controller choose the minimum-risk feasible action.

The output includes risk and expected cost by complete regime, the shared-regime minimax policy, the rectangular policy, the local worst-regime selection at every node, and both absolute and relative prices.

## 6. Conformance and tests

The module authors 36 Rust unit tests, including:
- shared regime values and rectangular price of rectangularity;
- proof by fixture that the local worst-regime choices switch across nodes;
- ensuring the policy does not contain hidden regime IDs;
- regime-specific probability normalization;
- missing-regime-map and nominal-average rejection;
- all earlier EX0/EX1/EX2 history, observation, resource and TV robustness controls.

The conformance artifact separately records 23 invalid-input controls, six boundary cases and eight round-trip requirements, including invalid regime maps, non-unit conditional probabilities, nominal-average mismatch, forbidden hidden-regime policy access and rectangularity-gap sign. Tests are authored, not confirmed passing until an actual Rust toolchain/CI run executes them.

## 7. Research positioning

Rectangularity is a foundational condition for dynamic decomposition. Shapiro (2016), *Rectangular Sets of Probability Measures*, Operations Research 64(2), 528–541, DOI https://doi.org/10.1287/opre.2015.1466, formalizes its relationship to multistage stochastic optimization, robust control, MDPs and dynamic risk measures.

Ma & Chen (2024), *Multi-stage distributionally robust convex stochastic optimization with Bayesian-type ambiguity sets*, Mathematical Methods of Operations Research 100, 553–600, DOI https://doi.org/10.1007/s00186-024-00872-1, propagates uncertainty about distribution parameters in a multistage setting and studies statistical properties and cutting-plane algorithms.

Yu & Basciftci (2026), *Distributionally robust optimization with multimodal decision-dependent ambiguity sets*, Mathematical Programming, DOI https://doi.org/10.1007/s10107-026-02337-1, treats decision-dependent mode probabilities and within-mode distributions in a two-stage DRO model. EX3 is a multistage finite persistent-regime comparison, not a reproduction of that work.

The contribution target is a measurable rectangularity diagnostic, not simply another robust solver: when does dynamic decomposition buy tractability at the cost of a large and potentially unrealistic risk bound?

## 8. Next steps

1. Add more than two persistent regimes and regime-specific mode transition matrices.
2. Introduce partial regime observations and posterior regime filtering, while ensuring the regime is never directly exposed to the policy.
3. Compare finite shared-regime minimax, Bayesian regime mixtures, and rectangular relaxation under identical observations.
4. Add latent Markov switching as a distinct model; a persistent regime is not a Markov-switching model.
5. Build a coupled continuous ambiguity version over shared parameters and compare it with both finite-regime and rectangular bounds.
6. Scale using scenario grouping/dual dynamic programming only after the exact micro-instances agree.

## 9. Non-claims

All regimes and values are synthetic. No real-world regime-identification, causal action-effect, calibration, coverage, deployment, readiness, emergency-command, certification or SOTA-superiority claim is made. All fixtures remain public-development-only and inactive.
