# Space anytime confidence sequence reference

A dependency-free Rust reference for a one-sided **time-uniform upper confidence sequence** for a Bernoulli violation process under an explicit predictable conditional-probability model.

## Model and formula

Let X_t=1 denote a violation and Y_t=1-X_t denote a non-violation. For a candidate lower cap p0 on the conditional violation probability and a fixed, preregistered bet grid r_j in (0,1], define:

    M_t(p0) = (1/J) sum_j product_{i=1..t}
              [1 + (r_j/(1-p0)) (Y_i - (1-p0))]

Under the null

    P(X_t=1 | F_(t-1)) >= p0    for every t,

each bet component is a nonnegative supermartingale. Their fixed mixture is also a nonnegative supermartingale. Ville's inequality gives:

    P(sup_t M_t(p0) >= 1/delta) <= delta

Inverting this test over p0 gives an anytime-valid upper confidence sequence for the largest uniform lower cap on conditional violation probability. In the IID Bernoulli special case, the cap is the common violation probability p, so this is an upper confidence sequence for p.

## Very important estimand boundary

This is **not** an upper confidence bound on an arbitrary average across heterogeneous episodes. If conditional violation probabilities vary over time/history, the confidence sequence bounds their uniform lower cap under the declared filtration. It may be applied to dependent observations only when the conditional lower-bound null holds given all relevant prior information.

It does not automatically cover common latent episode risk, reused seeds, shared MCTS tree uncertainty, source-selection bias, distribution shift or a filtration that omits common causes. When the conditional model cannot be justified, the correct outcome is unsupported/indeterminate, not an IID fallback.

## Example

    cargo run --locked -- 0.05 00000000000000000000 conditional-violation-cap-v1 fixture://conditional-mean-model/v1

Every prefix produces a time-indexed bound. More non-violations provide evidence against a high violation-probability cap; new violations can widen the upper bound. The finite grid in the reference implementation is fixed and identified in each report.

## Relation to fixed-sample Clopper-Pearson

PR #6460 implements an exact fixed-sample IID Clopper-Pearson upper bound. It is appropriate only for a fixed sample size under IID Bernoulli trials. This confidence-sequence reference addresses repeated looks/optional stopping under its own conditional-mean null. The two methods have different assumptions and should be compared on matched processes.

## Claim ceiling

This crate qualifies statistical software behavior for the declared conditional process only. It does not establish orbit truth, Pc calibration, physical risk, independence of MCTS rollouts, maneuver safety, operator authority or flight readiness.
