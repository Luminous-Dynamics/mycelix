# PRAXIS-NRES-016-EX2: Rectangular TV-robust adaptive recourse

**Status:** synthetic public-development exact reference; not calibrated, confirmatory or deployable.  
**Research issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5573.  
**Evaluator:** benchmarks/nres-capability-loop/src/adaptive_recourse.rs.  
**CLI:** cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --rectangular-tv-ex2.  
**Parent sequence:** NRES-016 protocol -> EX0 exact expected-cost oracle -> EX1 nested mean-CVaR -> EX2 rectangular transition ambiguity.

## 1. Question

EX0 proves the finite expected-cost recursion for a supplied transition law. EX1 adds a strictly monotone nested mean-CVaR risk mapping. EX2 moves uncertainty into the transition law itself: each action's next-observation distribution is allowed to vary inside a declared ambiguity set, and the policy is optimized against the worst admissible transition distribution at each observable information state.

This is the minimal distributionally robust adaptive-recourse increment. It is not yet a learned ambiguity set, a calibrated ambiguity set, nor a general non-rectangular multistage DRO solver.

## 2. Dynamic risk/ambiguity model

For observable history h and action a, let p(.|h,a) be the supplied nominal distribution over finite transition outcomes. EX2 assigns an action-specific total-variation radius eta(h,a) and ambiguity set:

[
\mathcal U(h,a)=\{q\in\Delta_K:\operatorname{TV}(q,p)\leq\eta(h,a)\},
\qquad
\operatorname{TV}(q,p)=\frac12\sum_i|q_i-p_i|.
]

The conditional robust risk map is:

[
\rho^{\mathrm{rob}}_{h,a}(Z)
=
\sup_{q\in\mathcal U(h,a)}
\left[(1-\lambda)\mathbb E_q[Z]+\lambda\,CVaR_\alpha^q(Z)\right]
]

with (0\leq\alpha<1), (0\leq\lambda<1).

The nested recursion is:

[
V(h)=\min_{a\in A(h)}
\left[
c(h,a)+
\sup_{q\in\mathcal U(h,a)}
\rho_{\alpha,\lambda}^{q}
\bigl(\ell(h,a,o)+V(h')\bigr)
\right].
]

The adversary acts after the candidate action is fixed, but before the next outcome is realized. EX2 assumes ambiguity sets are **rectangular across observable state/action transitions**: the adversary can select a conditional distribution independently at each decision node. This is the structural assumption that makes the nested Bellman recursion valid. It does not cover a shared unknown parameter, cross-stage coupled scenario law, or any non-rectangular ambiguity contract.

## 3. Exact finite-support TV maximizer

Let outcome losses be ordered (ell_1\leq\ell_2\leq\dots\leq\ell_K), and let the nominal probabilities be p. The reference constructs q* by:
1. choosing one maximum-loss receiver;
2. sorting all other outcomes from lowest loss to highest loss;
3. transferring probability mass from donors in that order to the receiver;
4. moving at most eta total mass.

Because TV is half-L1, moving m mass between states consumes exactly m TV radius. Therefore the algorithm never transfers more than eta.

For every threshold t below the maximum loss, q* increases the upper-tail probability by:

[
q^*(\ell>t)=p(\ell>t)+\min\{\eta,p(\ell\leq t)\}
=\min\{1,p(\ell>t)+\eta\}.
]

No other q in the TV ball can increase any upper-tail event by more than eta. Thus q* simultaneously maximizes all upper-tail probabilities, and first-order stochastically dominates every feasible q. Every increasing law-invariant risk measure—including expectation, upper CVaR and their mean-CVaR mixture—is maximized by q*. The evaluator still checks the simplex and TV-ball invariants at runtime.

This exact greedy construction depends on the discrete TV metric. It is not a replacement for weighted Wasserstein transport, where moving mass has state-dependent costs and a different inner optimization.

## 4. Synthetic decision-reversal fixture

The fixture reuses the EX1 tree but raises SAFE's cost to 460 and gives only the A/GAMBLE action a TV radius of 0.10. All other transition radii are zero. The parameters are alpha=0.90 and lambda=0.90.

Nominal gamble:
- good outcome cost 0 with probability 0.95;
- bad outcome cost 1,000 with probability 0.05;
- expected loss 50;
- CVaR90 500;
- mean-CVaR value 455.

Nominal nested mean-CVaR therefore selects GAMBLE because 455 < 460.

Worst-case TV law:
- transfer 0.10 mass from good to bad;
- worst-case probabilities become [0.85, 0.15];
- TV distance is exactly 0.10;
- worst-case mean is 150;
- CVaR90 is 1,000;
- robust mean-CVaR value is 915.

Robust nested mean-CVaR therefore selects SAFE because 460 < 915.

| Method | A-branch action | A-branch objective | Root robust/nested objective | Nominal expected path cost |
|---|---|---:|---:|---:|
| Nominal nested mean-CVaR | GAMBLE | 455 | 9,522.75 | 5,025 |
| Rectangular-TV robust nested mean-CVaR | SAFE | 460 | 9,523.00 | 5,230 |

The root value changes only slightly because branch B already dominates the root tail, but the locally admissible action changes. This is intentional: EX2 checks that the optimizer respects conditional transition uncertainty even when a global tail summary makes its effect look small.

## 5. Independent inner-problem verification

A separate LP was formulated for the two-outcome risk inner maximization. Let q be the adversarial probability and w the CVaR tail-weight vector:

- q is in the probability simplex and within the half-L1 TV radius;
- w is in the simplex;
- (0\leq w_i\leq q_i/(1-\alpha));
- maximize ((1-\lambda)q^T\ell+\lambda w^T\ell).

For p=[0.95,0.05], eta=0.10, losses=[0,1000], alpha=0.90 and lambda=0.90, the independent HiGHS LP returns:
- q=[0.85,0.15];
- CVaR tail weights w=[0,1];
- objective 915.

This matches the greedy TV construction. It is independent LP arithmetic, not Rust execution.

## 6. Invariants and conformance

- Every action has an explicit TV radius in [0,1]; a missing radius in older EX0/EX1 fixtures is interpreted as zero for backward compatibility.
- Nominal outcome probabilities sum to one; adversarial probabilities remain nonnegative and sum to one.
- (\operatorname{TV}(q^*,p)\leq\eta) is checked after construction.
- TV radius is half-L1, so transferred mass is bounded by eta, not 2*eta.
- Lambda remains in [0,1); lambda=1 is rejected.
- Rectangularity is explicit and reported as a modelling assumption.
- All EX0 history/observation binding, non-anticipativity, resource conservation, pathwise service-floor and reachability checks remain in force.
- The output includes selected-policy adversarial laws and per-action diagnostics, including rejected alternatives, to make the policy change auditable.

The module authors 29 Rust unit tests, and the EX2 conformance corpus records invalid-input, boundary and round-trip controls. These tests are authored, not confirmed passing until an actual Rust toolchain/CI run executes them.

## 7. Research positioning and next work

NRES-013 already supplies a finite-support Wasserstein DRO comparison for complete joint service states. NRES-014/015 address radius calibration, metric transfer, bulk-tail ambiguity and dependence-aware calibration. EX2 connects these ideas to a multistage policy by calibrating or specifying action-conditioned transition uncertainty at every observable decision node.

Current multistage emergency-resource literature includes robust multi-stage, multi-role allocation models (Scientific Reports 2026, https://doi.org/10.1038/s41598-026-48199-7) and time-coupled distributionally robust chance-constrained grid resilience models (Scientific Reports 2026, https://doi.org/10.1038/s41598-026-37197-4). These are candidate industry/research baselines, not results we have reproduced. EX2's differentiator is exact inner ambiguity verification, observation-level non-anticipativity, resource invariants and local policy-reversal diagnostics in one small benchmark.

Next:
1. compare the greedy TV adversary against independent LPs over generated supports up to at least 8 outcomes;
2. replace discrete TV with NRES-013 weighted-Wasserstein transition geometry;
3. use NRES-014/015 calibration contracts for radius selection and independent-family validation;
4. relax rectangularity by introducing shared latent parameters/coupled ambiguity and compare against a rectangular relaxation;
5. scale the policy evaluation using SDDP/dual dynamic programming, reporting bounds, gaps, wall time, memory and out-of-sample regret.

## 8. Non-claims

The eta=0.10 fixture radius is illustrative, not calibrated. Rectangularity is assumed, not established for real systems. There is no causal action-effect claim, no calibration/coverage guarantee, no confirmatory data, no deployment or certification authority, and no SOTA-superiority claim. All fixtures remain synthetic, public-development-only and inactive.
