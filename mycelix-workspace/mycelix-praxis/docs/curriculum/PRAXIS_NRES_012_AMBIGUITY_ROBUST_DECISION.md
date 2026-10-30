# PRAXIS-NRES-012: Ambiguity-robust decisions and minimax regret

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. It compares decision rules; it does not prescribe real preparedness action or certify any forecast distribution.  
**Fixture:** `examples/courses/civic-resilience-ambiguity-robust-decision-fixture-001.json`.  
**Conformance corpus:** `examples/courses/civic-resilience-ambiguity-robust-decision-conformance-001.json`.  
**Schema:** `schemas/nres/nres-ambiguity-robust-decision-v1.schema.json`.  
**Implementation:** `benchmarks/nres-capability-loop/src/robust_decision.rs`.

## 1. Why a point forecast is not enough

NRES-011 evaluates point-probability policies under three declared cost/loss profiles. But in a shifted environment the quoted probability itself may be uncertain. Choosing the action that minimizes expected loss at one point estimate can be brittle, especially when one critical input, dependency graph or source is uncertain.

The recent decision-focused literature studies this directly. Yamao et al. (UAI 2026) construct worst-case regret losses over uncertainty sets around measured objective coefficients and Wasserstein ambiguity sets around conditional empirical distributions, explicitly addressing observation error and deployment distribution shift: https://proceedings.mlr.press/v337/yamao26a.html. Ohnemus et al. (ICML 2026) study loss-aware trainable optimal-transport ambiguity sets, noting that decoupled ambiguity-set construction can produce excessive conservatism: https://proceedings.mlr.press/v306/ohnemus26a.html.

NRES-012 does **not** claim to implement those full methods. It implements a tractable first case: a Bernoulli event probability with a declared total-variation ambiguity set. For Bernoulli distributions, total-variation distance is the absolute difference between event probabilities, so the ambiguity set is an interval:
\[
\mathcal{P}_\rho(p)=\{q\in[0,1]:|q-p|\leq\rho\}
=[\max(0,p-\rho),\min(1,p+\rho)].
\]
The fixture's radii are synthetic stress parameters, not confidence levels or calibrated prediction intervals.

## 2. Three policies for each cost/loss profile

Let (C) be preparation cost, (L_0) event loss without preparation, and (L_1) residual event loss after preparation, with (L_0>L_1\). The point decision threshold is (p^*=C/(L_0-L_1)).

**Nominal point policy.** Choose `prepare` when the point forecast (p\geq p^*\), otherwise `no_action`.

**Worst-case expected-loss policy.** Choose the action with the lower worst expected loss over the ambiguity interval. Since both Bernoulli action-cost functions are increasing in (q\), their worst expected costs occur at the upper bound (q_U\):
\[
\overline C(\text{no action})=q_U L_0,
\qquad
\overline C(\text{prepare})=C+q_U L_1.
\]
Choose the lower value; exact ties select `no_action`. This is a conservative expected-loss criterion and can incur greater realized sample cost.

**Minimax-regret policy.** Regret compares the chosen action with the action that would have been selected if the true (q\) were known. The worst regret of each action over the interval is:
\[
R_U(\text{no action})=\max(0,q_U(L_0-L_1)-C),
\]
\[
R_U(\text{prepare})=\max(0,C-q_L(L_0-L_1)).
\]
Choose the action with the lower worst regret; exact ties select `no_action`. This may select a different action than worst-case expected-loss because the objectives are different.

An abstention keeps all probability interval fields null, invokes the separately supplied reviewer fallback and adds the declared deferral cost. Robust policies do not invent an interval when critical evidence is absent.

## 3. What the evaluator reports

For every regime and each cost/loss profile, NRES-012 reports:
- realized importance-weighted policy cost, relative economic value and regret versus the perfect-information lower bound for all three policies;
- weighted mean worst-case expected cost and worst-case regret on the issued-forecast subset, plus maximum single-case worst-case regret;
- interval width distribution, forecast coverage and effective sample size;
- action disagreements between nominal, worst-case expected-loss and minimax-regret policies;
- constant, training-climatology and perfect-information reference policy costs.

All profiles and regimes remain separate. No scalar robustness or resilience score is created.

## 4. Public fixture and observed demonstration

NRES-012 reuses 128 public synthetic NRES-011 cases, preserving the source fixture blob SHA and source case IDs. It assigns four declared ambiguity radii to the in-distribution, temporal-shift, structural-dependency-shift and evidence-degradation regimes. Five abstentions remain abstentions (four mandatory, one optional); no interval is emitted for them. The fixture is pinned to NRES-011's source blob SHA `a9f10ae28fe3c5031ed9ce0f88761e94ad6f4e51`, and the Rust evaluator checks all 128 copied source-case identities and copied forecast/evaluator fields.

The public conformance corpus contains **28 invalid-input negative controls** and **two policy-boundary cases**. The Rust reference evaluator contains **37 authored unit tests** covering the golden profile/regime vectors, exact policy tie-breaking, interval/radius consistency, source lineage, mandatory abstention, sampling design and prohibited aggregate scores.

On this illustrative sample, minimax regret lowers policy cost under the low-cost preparedness profile, is neutral under the moderate and high profiles, and worst-case expected-loss lowers cost for some profiles but increases it substantially for the high-cost profile. This is not paradoxical: worst-case expected cost is intentionally conservative, and the outcome observed in one synthetic sample need not coincide with the distributional worst case. The benchmark reports both quantities to expose that trade-off rather than hiding it behind one metric.

## 5. Research targets and comparison to SOTA

Compare identical scenario IDs, forecast digests, interval radii, cost profiles and target/proposal weights across nominal point decisions, worst-case expected loss, minimax regret, training climatology, constant policies and a perfect-information lower bound.

The next challenge set should use:
1. uncertainty radii estimated on a separate calibration split (not chosen after viewing the test outcomes);
2. multiple radii and misspecification levels, including radii too narrow and too wide;
3. scenario families where the decision threshold lies inside, below and above the uncertainty interval;
4. observation-error sets and Wasserstein ambiguity over multivariate service states;
5. decision-regret confidence intervals from repeated, independently generated scenario families;
6. paired comparisons of decision cost, worst-case regret, false negatives, unnecessary preparation and deferral cost.

The 2026 UAI work of Yamao et al. and ICML work of Ohnemus et al. are the relevant method-level comparison points. On the current 128-case public fixture, independent arithmetic reproduces **720 numeric golden fields** across 3 profiles × (1 overall + 4 regimes) with zero mismatches at 1e-12 tolerance. This is an arithmetic/source verification of the fixture, not a Rust build or execution. Praxis's opportunity is not to claim a better optimizer from a small fixture; it is to provide a deterministic, provenance-pinned, cross-regime conformance benchmark that makes the decision and ambiguity assumptions inspectable.

## 6. Boundaries and limitations

- This release uses a one-dimensional Bernoulli total-variation ball, not a Wasserstein ambiguity set over full service trajectories.
- The probability radius is a stress parameter, not a calibrated confidence interval.
- The synthetic cases are linked to NRES-011; they do not provide independent validation.
- Worst-case expected-loss, minimax regret and realized sample cost answer different questions.
- A successful local run validates deterministic arithmetic and fixture conformance only; it does not establish real-world operational safety, causal impact, forecast superiority or legal authority.

## 7. Reproducibility

From `mycelix-praxis`:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --robust-decision
```
