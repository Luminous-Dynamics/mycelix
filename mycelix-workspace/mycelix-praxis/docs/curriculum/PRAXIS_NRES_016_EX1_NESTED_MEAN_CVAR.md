# PRAXIS-NRES-016-EX1: Nested mean-CVaR versus terminal mean-CVaR

**Status:** synthetic public-development semantic benchmark; not confirmatory or deployable.  
**Parent protocol:** PRAXIS_NRES_016_ADAPTIVE_RECOURSE_PROTOCOL.md.  
**Evaluator:** benchmarks/nres-capability-loop/src/adaptive_recourse.rs.  
**CLI:** cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --nested-risk-ex1.  
**Fixture:** nres-016-ex1-nested-cvar-fixture-001.json.

## Why EX1 exists

A multistage risk objective has to specify **when** risk is evaluated. These two formulations are not interchangeable:

1. **Terminal/precommitment risk:** apply one risk functional to total cost over the complete trajectory, then choose a policy at the root.
2. **Nested conditional risk:** recursively apply the conditional risk functional to each next-stage cost plus continuation value, optimizing again at each observable information state.

An optimizer can look acceptable under a terminal objective because another branch dominates the global tail, even when an action is locally unattractive under the conditional risk of the branch that has just been observed.

EX1 makes this distinction measurable on the same finite tree, costs, transitions, hard constraints, information histories and risk parameters. It is a semantic conformance benchmark, not a contest in which one risk preference is universally superior.

## Risk functional and strict-monotonicity correction

The one-period risk mapping is the convex combination:

[
\widehat\rho_{\alpha,\lambda}(Z)
=(1-\lambda)\mathbb E[Z]+\lambda\,CVaR_{\alpha}(Z),
\qquad 0\leq\lambda<1,\quad 0\leq\alpha<1.
]

For a finite distribution with losses (z_i) and probabilities (p_i), the upper-tail CVaR is evaluated by the Rockafellar-Uryasev representation:

[
CVaR_{\alpha}(Z)
=\min_{\eta\in\{z_i\}}
\left[
\eta+\frac{1}{1-\alpha}\sum_i p_i\max(z_i-\eta,0)
\right].
]

The exact finite-support minimum occurs at a support value, so EX1 checks every distinct support loss rather than approximating the quantile with a histogram.

**Important theory correction:** pure AVaR/CVaR is not strictly monotone in general. Therefore, nesting pure conditional AVaR does not automatically establish every strong time-consistency result. Füllner and Rebennack's 2025 SIAM Review distinguishes these notions and identifies the mapping ((1-\lambda)E+\lambda AVaR_\alpha) as strictly monotone for (\lambda\in[0,1)), which supports the nested dynamic-programming equivalence under their stated assumptions (Sections 12.1.1–12.1.3; DOI: https://doi.org/10.1137/23M1575093).

For this reason, EX1 rejects (\lambda=1). It does not call plain nested CVaR automatically time-consistent. The benchmark implements the strictly monotone mixture and retains the pure-CVaR helper only as a finite-distribution calculation component.

## Two compared objectives

The **nested** objective recursively applies the same one-period mapping at every observable information state:

[
V_t(h_t)=\min_{a_t\in A(h_t)}
\left\{
c_t(h_t,a_t)
+\widehat\rho_{\alpha,\lambda}
\left(
\ell_{t+1}+V_{t+1}(h_{t+1})
\mid h_t,a_t
\right)
\right\}.
]

For each action, the expectation and CVaR are computed over the same action-conditioned transition outcomes. Fixed action cost is translation-equivariant and is added outside the conditional risk mapping. Every positive-probability successor must admit a feasible continuation.

The **static terminal** comparator enumerates non-anticipative policies on the same finite information-state tree, computes the distribution of total terminal path cost for each policy, and minimizes:

[
(1-\lambda)\mathbb E[C_H]+\lambda\,CVaR_\alpha(C_H).
]

If the static objective is exactly tied within tolerance, it uses lower expected cost as a deterministic tie-break. This is a precommitment comparator, not a nested dynamic risk objective.

Both formulations share the same observable-history rules, pathwise service floors, action-conditioned transition probabilities and transition/resource invariants. Neither may condition an action on hidden scenario labels.

## Synthetic divergence fixture

Parameters: (\alpha=0.90), (\lambda=0.90). Root transitions to observable branches A and B with probability 0.5 each.

At A:
- SAFE: deterministic cost 150.
- GAMBLE: cost 0 with probability 0.95 and cost 1,000 with probability 0.05.

At B:
- HOLD: unavoidable deterministic cost 10,000.

The conditional A-branch values are:
- SAFE: mean 150, CVaR90 150, mixture value 150.
- GAMBLE: mean 50, CVaR90 500, mixture value 455.

Thus nested optimization selects SAFE after observing A.

The terminal cost distribution under the static GAMBLE policy is 0 with probability 0.475, 1,000 with probability 0.025 and 10,000 with probability 0.5. Under static SAFE it is 150 with probability 0.5 and 10,000 with probability 0.5. In either case, terminal CVaR90 is 10,000 because branch B dominates the worst 10% tail. The mean-CVaR mixture then selects GAMBLE by its lower expected cost.

| Formulation | A-branch action | Expected total cost | Terminal CVaR90 | Objective value |
|---|---|---:|---:|---:|
| Nested mean-CVaR | SAFE | 5,075 | 10,000 | 9,507.5 |
| Static terminal mean-CVaR | GAMBLE | 5,025 | 10,000 | 9,502.5 |

The numbers deliberately show a preference difference, not a global performance ranking. Static precommitment values a 50-unit reduction in mean cost because the A-tail event is hidden beneath the much larger B-branch tail. Nested conditional risk refuses that local gamble once A is observed. Which objective is appropriate depends on the decision contract and the organization's risk mandate.

## Exactness and constraints

- The finite-support CVaR inner minimization checks every distinct loss support point.
- Static policy enumeration uses explicit information states and combines compatible child policies; it does not assign a different action to hidden scenario copies of the same history.
- Nested dynamic programming optimizes one action per observable information state.
- All transitions are validated by the EX0 parser: emitted observations must match successor histories, resource ledgers must balance, probabilities must sum to one and every state must be reachable.
- Service floors are hard, pathwise constraints. If no feasible continuation exists, the associated action/policy is discarded.
- The objective is exact for this finite DAG and its supplied action-conditioned transition probabilities; it is not a large-scale solver.

## Unit tests and conformance

The module authors 20 Rust unit tests, including:
- finite-distribution CVaR hand calculation;
- static-versus-nested policy reversal and objective values;
- one action per observable information state;
- rejection of invalid confidence levels and (\lambda=1);
- explicit infeasibility under pathwise service floors;
- existing EX0 history, observation, resource and stage negative controls.

The conformance corpus separately records risk-level/lambda controls, objective semantics, policy non-anticipativity and terminal-distribution round-trip requirements. Authored tests are not passing tests until a Rust toolchain/CI run executes them.

## Research comparison and next steps

Füllner and Rebennack (SIAM Review 67(3), 2025, DOI https://doi.org/10.1137/23M1575093) review SDDP, nested risk measures, strict monotonicity and the distinction between end-of-horizon and nested risk. They characterize SDDP as a state-of-the-art solution approach for large-scale multistage stochastic linear programs and discuss risk-averse variants.

EX1 is not a competitor to SDDP. It is a tiny semantic oracle that future SDDP or decomposition implementations must reproduce before scale claims are made. Next increments:
1. independently formulate the same finite instance as a scenario-tree extensive-form LP with non-anticipativity constraints;
2. compare the LP against EX0 expected-cost and EX1 nested-risk objectives/policies;
3. add a rectangular stagewise ambiguity set and compare nested robust risk against the fixed-kernel mean-CVaR benchmark;
4. then move to larger scenario trees and SDDP-style decomposition, reporting optimality gaps, lower/upper bounds, runtime, memory, policy quality and out-of-sample regret.

## Non-claims

No real operational data, no calibration/coverage guarantee, no causal intervention estimate, no distributionally robust claim, no emergency-command authority, no readiness/certification claim and no SOTA-superiority claim. All fixtures are synthetic and inactive.
