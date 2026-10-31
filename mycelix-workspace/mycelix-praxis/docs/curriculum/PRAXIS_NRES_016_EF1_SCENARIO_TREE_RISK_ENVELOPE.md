# NRES-016 EF1 — Independent scenario-tree risk-envelope oracle

**Status:** research specification; implementation and Rust execution are pending  
**Split:** public development only  
**Activation:** inactive; synthetic fixtures only

## Purpose

EX0 and EX1 provide exact dynamic-programming references for finite-horizon expected cost and nested mean-CVaR semantics. EF1 adds an independent mathematical formulation: for each fixed non-anticipative policy, represent nested mean-CVaR as an adversarial probability-flow linear program on the scenario tree.

EF1 must not call or reuse EX0/EX1's recursive evaluator or CVaR support-scan helper. It exists to cross-check the semantics before a scalable method is attempted.

## Risk-envelope formulation

For nominal conditional outcome probabilities \(p_i\), confidence level \(0\leq\alpha<1\), and mixture weight \(0\leq\lambda<1\),

\[
\widehat\rho_{\alpha,\lambda}(Z)
=(1-\lambda)\mathbb E_p[Z]+\lambda\,CVaR_\alpha^p(Z).
\]

The upper-tail CVaR dual is

\[
CVaR_\alpha^p(Z)=
\max_{q\in\Delta}
\left\{\sum_iq_iZ_i:
0\leq q_i\leq p_i/(1-\alpha)\right\}.
\]

Thus the risk-adjusted conditional probabilities are
\(r_i=(1-\lambda)p_i+\lambda q_i\).

For a fixed deterministic policy, let \(m_n\) be the distorted arrival mass at information node \(n\), and \(x_{n,e}\) the outgoing mass on edge \(e\). The extensive-form adversary is:

\[
m_{\mathrm{root}}=1,\qquad
\sum_{e\in Out(n)}x_{n,e}=m_n,
\]

\[
(1-\lambda)p_{n,e}m_n
\leq x_{n,e}\leq
\left((1-\lambda)+\frac{\lambda}{1-\alpha}\right)
p_{n,e}m_n,
\]

\[
m_j=\sum_{(n,e):\,child(n,e)=j}x_{n,e}.
\]

Maximize the risk-adjusted total path cost

\[
\sum_n m_nc_n+\sum_{n,e}x_{n,e}d_{n,e},
\]

where \(c_n\) is the fixed action cost at node \(n\), and \(d_{n,e}\) is the transition cost on edge \(e\). The outer minimization enumerates feasible deterministic non-anticipative policies. Therefore this is an LP for each fixed policy, not one plain LP over the discrete action choices.

A first exact implementation may enumerate vertices of each small local risk-envelope polytope and propagate the corresponding scenario-tree flows. It must check the same constraints as the displayed LP. A separate HiGHS (or other LP backend) comparison is a later independent check.

The terminal/precommitment comparator is different: it uses one risk envelope over complete terminal-path costs. It must not be conflated with the node-wise nested risk envelope.

## Exact reference checks

The first EF1 corpus uses the existing EX0 and EX1 fixtures without changing their bytes.

| Fixture / formulation | Required action policy | Required objective |
| --- | --- | ---: |
| EX0, \(\lambda=0\) nominal expectation | PREVENT; USE-SPARE after quiet; REPAIR after storm | 2.3 |
| EX1, nested mean-CVaR | SAFE at branch A | 9507.5 |
| EX1, fixed nested GAMBLE policy | GAMBLE at branch A | 9522.75 |
| EX1, terminal/precommitment mean-CVaR | GAMBLE at branch A | 9502.5 |

The EX0 storm-branch DEFER action remains infeasible under its pathwise water-service floor. EX1 uses \(\alpha=0.90\), \(\lambda=0.90\).

These are synthetic arithmetic targets. The difference between nested and terminal risk is intentional: the two objectives encode different decision commitments.

## Correctness and negative controls

- Independently enumerate feasible deterministic policies; choose exactly one action per canonical observable history.
- Reject latent state, hidden regime, future observation, outcome label and oracle-label leakage into policy keys.
- Verify every nominal transition vector is finite, non-negative and normalized.
- Verify every risk-envelope flow satisfies its lower/upper bounds and conservation equations.
- Verify each child arrival mass equals its incoming edge flow.
- Verify all positive nominal transitions remain positive for \(\lambda<1\).
- Verify \(\lambda=0\) and \(\alpha=0\) both reduce to nominal expectation.
- Reject \(\lambda=1\) for this strict-monotonicity contract; pure CVaR is not strictly monotone in general.
- Verify invariance to outcome permutation and splitting an outcome into identical sub-outcomes whose probabilities sum to the original probability.
- Preserve service floors, resource conservation, action-conditioned transition semantics and emitted-observation/successor-history equality.
- Report infeasibility rather than repairing, clipping or silently dropping infeasible paths.

## Proposed acceptance targets (not results)

- EX0 and EX1 objective agreement within absolute error \(10^{-10}\).
- 100% selected-policy agreement on exhaustive tiny instances.
- Zero non-anticipativity, probability-flow, risk-envelope, resource or service-floor violations.
- Independent LP and vertex-enumerator agreement within \(10^{-10}\) on bounded micro-instances.
- Publish policy count, envelope-vertex count, explored combinations, numerical residuals, infeasibility, runtime and memory separately.
- No scaling claim until a scalable LP/MILP or SDDP-style method is compared on matched information sets, fixtures and compute budgets with certified bounds.

## Research positioning

Füllner & Rebennack, *Stochastic Dual Dynamic Programming and Its Variants: A Review*, SIAM Review 67(3), 2025, DOI [10.1137/23M1575093](https://doi.org/10.1137/23M1575093), review nested risk, strict monotonicity and time consistency. The expectation/CVaR mixture with \(\lambda<1\) is retained for the stated strict-monotonicity condition.

EF1 is a semantic oracle, not a competitor to SDDP. Future scalable methods must reproduce EF1 on tiny instances before performance comparisons.

## Non-claims

No real-world data, calibration guarantee, causal effect, operational readiness, emergency authority, certification or SOTA superiority is claimed.