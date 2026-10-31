# PRAXIS-NRES-016-EX4: Joint multimodal decision-dependent ambiguity

**Status:** protocol and exact-fixture contract; no solver or trained ambiguity set is claimed.  
**Research issue:** https://github.com/Luminous-Dynamics/mycelix/issues/5577.  
**Prerequisites:** EX0 exact information-state oracle; EX1 nested mean-CVaR; EX2 rectangular transition-TV; EX3 persistent shared-regime versus rectangular ambiguity.

## 1. Research question

Can a multistage decision process robustly optimize when an action changes both:
1. the probability of entering or remaining in an operating mode; and
2. the conditional outcome distribution within each mode?

Examples include hardening shifting the chance of a stressed infrastructure state, supplier diversification changing the mix of procurement regimes and their lead-time distributions, and maintenance changing both the likelihood of degraded operation and the recovery distribution conditional on degradation.

This requires more than putting one ambiguity ball around a collapsed mixture distribution. It requires preserving the **joint structure linking mode probabilities to within-mode distributions**.

## 2. Paradigm shift: uncertainty is a joint mode-and-kernel object

Let the latent operating mode be (m\in\mathcal M). At information history (h) and action (a), each ambiguity candidate is a complete tuple:

[
\zeta=(\pi,Q_1,\ldots,Q_{|\mathcal M|}),
\qquad
\pi\in\Delta_{|\mathcal M|},
\qquad
Q_m\in\Delta(\mathcal W_m).
]

The induced next-outcome law is:

[
P_\zeta(w\mid h,a)
=\sum_{m\in\mathcal M}\pi_m Q_m(w\mid h,a).
]

The action-specific ambiguity set is a set of complete tuples:

[
\mathcal D(h,a)\subseteq
\Delta_{|\mathcal M|}\times
\prod_{m\in\mathcal M}\Delta(\mathcal W_m).
]

The robust transition value is:

[
\sup_{\zeta\in\mathcal D(h,a)}
\widehat\rho_{\alpha,\lambda}^{P_\zeta}
\left(\ell(h,a,w)+V(h')\right),
\]

where the one-period risk map remains the strictly monotone mean-CVaR mixture from EX1:

[
\widehat\rho_{\alpha,\lambda}
=(1-\lambda)\mathbb E+\lambda\,CVaR_\alpha,
\quad 0\leq\alpha<1,\quad0\leq\lambda<1.
]

A candidate tuple is indivisible. Its mode weights and conditional laws must stay linked. A factorized relaxation that independently chooses a mode-probability vector and each within-mode law defines a larger set, not the original joint ambiguity set.

## 3. Decision dependence

The ambiguity family may depend on the action:

[
\mathcal D(h,a_1)\neq\mathcal D(h,a_2).
]

This permits hardening, diversification, maintenance, inspection or other actions to change the mode mixture and conditional within-mode transition distributions. The action effect is an explicit model input. It is not a causal conclusion unless identified by a separate design.

The benchmark freezes every action-to-ambiguity mapping before confirmatory evaluation. No hidden outcome, holdout score or confirmatory regime may tune the candidate set.

## 4. Persistent modes, transitioning modes and observations

EX4 starts with finite-mode candidates whose mode identity remains persistent over the short horizon. This is a joint extension of EX3: a tuple contains the mode probability and its conditional kernels, not just a regime ID with fixed conditional laws.

Future variants may permit a mode transition matrix:

[
T^a_{mm'}(h)=\Pr(M_{t+1}=m'\mid M_t=m,h,a).
]

That is a separate benchmark extension. Do not silently treat a fixed latent regime as a Markov-switching regime or assume mode independence across stages.

The policy still maps observable history to actions. Latent mode labels are not directly available. An observation may supply a noisy signal about the mode; its likelihood must be included in the same candidate tuple, and the posterior/ambiguity update may use only that emitted observation. EX4 must test for hidden-mode leakage in the observation channel.

## 5. Required comparators

Use the same information states, actions, costs, service/resource constraints, mode candidate family and compute budget.

1. **EX4 joint multimodal decision-dependent ambiguity:** maximize over complete tuples in (\mathcal D(h,a)).
2. **Factorized rectangular relaxation:** independently maximize over projections of the mode-weight and within-mode kernel sets. Explicitly report that this is a superset/relaxation.
3. **Collapsed one-mode ambiguity:** mix the mode-specific kernels and discard mode identity; this is an ablation whose lost dependence/tail information must be measured.
4. **EX3 persistent shared-regime model:** fixed regime-level kernels across the horizon.
5. **EX2 rectangular transition-TV model:** local total-variation uncertainty without multimodal mode structure.
6. **Nominal multimodal policy:** optimize the equal-weight or source-weighted reference tuple without robustification.
7. **Perfect-information/known-mode policy:** lower-bound or information-value diagnostic only; not a deployable policy unless the mode is observable.

## 6. Exact finite micro-instance

Start with:
- two modes (NORMAL/STRESS);
- two actions (e.g. HARDEN/DEFER);
- two or three transition stages;
- finite outcome support within each mode;
- a small finite list of joint candidates per action;
- one resource ledger and separate service floors.

Every candidate record contains:
- unique candidate ID;
- action and information-state applicability;
- mode-probability vector;
- one normalized within-mode transition distribution per mode;
- joint provenance/constraint label explaining why the weights and kernels belong together;
- optional observation likelihoods, when partial regime signals are modeled.

The exact oracle enumerates complete joint candidates and all feasible non-anticipative policies. It must not independently normalize or alter components of a joint candidate.

## 7. Price of factorization and mode collapse

Report three distinct values:

[
J_{joint}=\min_\pi\sup_{\zeta\in\mathcal D}J(\pi;\zeta),
]

[
J_{fact}=\min_\pi\sup_{(\pi_m,Q_m)\in
\operatorname{proj}(\mathcal D)}
J(\pi;\pi_m,Q_m),
]

and (J_{collapsed}), where mode identity is removed and a single mixed kernel is used.

The price of factorization is (J_{fact}-J_{joint}). It should be nonnegative when the factorized set is proven to contain the joint set and policy/risk contracts are identical. The price of mode collapse is reported as a separate difference; it may be positive or negative depending on what dependence information the collapse destroys.

Every claimed set containment must be verified at the candidate level. If the projections or coupling constraints do not establish containment, do not label the comparator a relaxation.

## 8. Required correctness and governance controls

- Each mode-probability vector is nonnegative and sums to one.
- Each within-mode conditional kernel is nonnegative and sums to one.
- Every candidate tuple is validated as an indivisible object.
- The action-dependent mapping is frozen before confirmatory evaluation.
- All outcome paths share one policy action for an identical observable history.
- Mode IDs are never direct action inputs unless an emitted observation actually reveals them.
- Observation likelihoods are attached to the same joint candidate as the mode and transition kernels.
- Resource conservation, transition timing, service floors and hard constraints remain unchanged.
- A factorized set must be shown to contain the complete joint candidate set before reporting a nonnegative relaxation gap as a theorem.
- The scalar mixed outcome law must reproduce the exact mode mixture, without silently dropping cross-mode structure.
- All failed, infeasible, vacuous and non-improving candidate families remain in the report.

## 9. Benchmark metrics

Report separately:
- joint robust objective and nominal expected cost;
- factorized robust objective and price of factorization;
- collapsed-mixture objective and price of mode collapse;
- policy changes by action and observable history;
- realized path regret and service-floor breaches by mode;
- per-mode conditional-tail risk and mode-mix sensitivity;
- worst-case mode weights and within-mode distributions;
- ambiguity-set width, candidate count and vacuity;
- action/transition feasibility and resource ledgers;
- non-anticipativity and hidden-mode leakage violations (must be zero);
- solve time, memory, enumeration count, optimality gap and out-of-sample regret.

Do not form one composite resilience score. Statistical inference must use independent source-family/regime trajectories, not multiple outcomes or seeds within one family as independent observations.

## 10. Research comparison

Yu & Basciftci (2026), *Distributionally robust optimization with multimodal decision-dependent ambiguity sets*, Mathematical Programming, DOI https://doi.org/10.1007/s10107-026-02337-1, jointly models uncertainty over mode probabilities and within-mode distributions in a two-stage decision-dependent setting. It is a direct conceptual comparator, not a reproduction target.

EX4's defensible research contribution is the multistage extension with observable-history policies, nested risk, persistent/transitioning mode semantics, explicit joint candidate integrity and exact micro-oracles. Before claiming any SOTA improvement, match the source family, action sets, uncertainty candidates, information structures, service/resource constraints and compute budgets used by the strongest comparator.

## 11. Proposed targets (not results)

- 100% probability-simplex and joint-candidate integrity checks.
- Zero hidden-mode leakage and non-anticipativity violations.
- Exact joint candidate enumeration agrees with an independent evaluator within 1e-10.
- 100% of claimed factorized supersets contain every declared joint candidate.
- Factorization price is nonnegative whenever containment is proved.
- On independent mode-shift trajectories, at least 5% paired regret improvement over EX2/EX3 baselines with a paired 95% CI excluding zero and no more than 2% nominal cost degradation; publish all non-improvement outcomes.

## 12. Non-claims

All initial fixtures remain synthetic, public-development-only and inactive. No regime probabilities or within-mode kernels are calibrated from real-world evidence. No causal intervention effect, deployment authority, emergency readiness, certification or SOTA superiority is claimed.
