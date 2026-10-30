# PRAXIS-NRES-011: Forecast reliability, resolution and decision value

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. It does not prescribe real preparedness action, estimate real savings or allocate operational resources.  
**Fixture:** `examples/courses/civic-resilience-reliability-resolution-decision-value-fixture-001.json`.  
**Conformance corpus:** `examples/courses/civic-resilience-reliability-resolution-decision-value-conformance-001.json`.  
**Schema:** `schemas/nres/nres-reliability-resolution-decision-value-v1.schema.json`.  
**Implementation:** `benchmarks/nres-capability-loop/src/decision_value.rs`.

## 1. Research shift: score forecasts and assess what users can do with them

NRES-008 measures marginal exceedance probability; NRES-009 measures joint failure distributions; NRES-010 stresses rare-event tails and distribution shifts. These tests still leave a user-facing question unanswered:

**Does using a forecast improve a clearly specified decision for a user with a particular cost/loss structure?**

Forecast value is not a universal model property. It depends on the action threshold, cost of preparation, residual impact after preparation, deferral/review cost and the decision-maker's tolerated risk. The same forecast can rationally lead to different actions for different users.

The forecast-verification literature makes a related distinction. Murphy's Brier decomposition separates reliability, resolution and uncertainty: reliability measures calibration, resolution measures how well forecasts discriminate situations with different event rates, and uncertainty describes event variability. https://doi.org/10.1175/1520-0450(1973)012%3C0595%3AANVPOT%3E2.0.CO%3B2. Allen, Ferro and Kwasniok's conditional decomposition develops more detailed analysis of the sources of forecast information and their conditional biases: https://doi.org/10.1002/qj.4478.

Decision-oriented forecast evaluation adds a different lens. User-focused forecast evaluation recommends proper scoring rules for probability quality, while explicitly recognizing that the usefulness of a forecast depends on how it is used. https://journals.ametsoc.org/view/journals/wefo/39/8/WAF-D-23-0201.1.xml. Relative economic value compares forecast-based decisions with climatology and perfect-information references across cost/loss profiles; the optimal threshold varies with the user profile. https://doi.org/10.1175/WAF-D-19-0248.1.

NRES-011 combines these views without collapsing them into one score.

## 2. Fixed event definition and explicit decision profiles

The event remains the NRES-010 event: aggregate normalized service-deficit-hours over a 72-hour horizon strictly exceed 48. It reuses NRES-010's public synthetic cases and stable source case IDs. The source Git blob SHA and source-case count are pinned, and the evaluator compares every copied regime, sampling, timing, qualification, outcome and forecast field to the embedded NRES-010 source fixture. It is a linked benchmark increment, not independent confirmation data.

Each decision profile contains four declared quantities, all expressed in fictional normalized planning-loss units:
- (C): cost of a readiness/preparation action;
- (L_0): impact loss if an event occurs without preparation;
- (L_1): residual event loss if preparation is taken;
- (D): extra deferral/review cost when the model abstains.

Expected losses are:
\[
\mathbb{E}[\ell(\text{no action})]=pL_0
\]
\[
\mathbb{E}[\ell(\text{prepare})]=C+pL_1
\]

Preparation is selected when:
\[
p\geq p^*=\frac{C}{L_0-L_1},\quad L_0>L_1.
\]

An abstention is not quietly converted into a forecast. It invokes a separately supplied evaluator-owned human-review fallback action and adds the stated deferral cost (D). That review action is not model output and must not be used to train or score a person.

The public fixture has three profiles (low, moderate and high preparation cost). Their different thresholds make the user-dependence of forecast value visible.

## 3. Decision-cost accounting

For each case, regime and profile, the evaluator compares:

1. **Candidate forecast policy:** choose prepare or no action using the profile's frozen probability threshold; on abstention, use the separate human-review action and add (D).
2. **Always no action:** a constant lower-information baseline.
3. **Always prepare:** the complementary constant policy.
4. **Training climatology:** choose an action using only the declared 4% training-reference probability and the profile threshold. The test cohort does not determine this baseline.
5. **Perfect-information oracle:** an unattainable lower-bound comparator that chooses the lower realized loss after the event is known. It is never presented as an achievable policy.

Decision value is reported relative to training climatology and the perfect-information lower bound:
\[
EV=\frac{C_{\text{climatology}}-C_{\text{candidate}}}
{C_{\text{climatology}}-C_{\text{perfect information}}}.
\]

The denominator is reported; EV is undefined if it is non-positive. Negative EV is preserved: a forecast policy can be worse than the training-climatology decision. Values are reported independently for every cost/loss profile and shift regime. They are not averaged across different users or use cases.

## 4. Reliability, resolution and uncertainty decomposition

For the issued forecast subset, NRES-011 reports original importance-weighted Brier and log loss. It also groups issued probabilities in the frozen bins and calculates the classical fixed-bin Murphy decomposition for the *coarsened* forecast:
\[
BS_{\text{binned}}=Reliability-Resolution+Uncertainty.
\]

The three terms are:
\[
Reliability=\sum_k w_k(\bar p_k-\bar y_k)^2,
\]
\[
Resolution=\sum_k w_k(\bar y_k-\bar y)^2,
\]
\[
Uncertainty=\bar y(1-\bar y),
\]
where (w_k) is the importance-weighted mass in bin (k), (ar p_k) is its weighted mean forecast, (ar y_k) its weighted event rate, and (ar y) the weighted event rate overall.

Because within-bin forecast variation is discarded by coarsening, the evaluator also reports the original unbinned Brier score and the binned-minus-original residual. The decomposition is a fixed-bin diagnostic, not a corrected unbiased estimate. Ferro and Fricker (2012) show that the ordinary finite-sample Brier decomposition components can be biased; bias correction and uncertainty intervals require adequate samples and a stated estimator. https://doi.org/10.1002/qj.1924.

The forecast diagnostics always accompany issued-case count, raw and weighted coverage, importance-weight sum and effective sample size. The fixture reuses NRES-010's stress-enriched sample, so all decision costs are importance-weighted according to the frozen target/proposal stratum design.

## 5. Public fixture and limits

The 128 public synthetic cases span the same four NRES-010 regimes. Each case is evaluated under all three cost/loss profiles. The fixture carries an explicit four-regime catalog and a separate evaluator-owned review fallback action for the five abstentions (four mandatory, one optional). Its conformance corpus has 26 invalid-input negative controls and two valid-but-failing decision boundaries.

The data are deliberately linked to NRES-010, with copied stable source IDs, regimes, probabilities, observations and importance-sampling metadata. This permits repeatable integration testing but does not create independent evidence. All loss values are invented normalized units, not budgets, currency, or observed infrastructure impact.

The fixed-bin decomposition is small-sample descriptive. The 128 cases have low effective tail sample sizes; decision values and relative EV are illustrative. The perfect-information policy is a mathematical lower bound, not a deployable option.

## 6. Industry comparison and research protocol

Use the same hidden cases, forecast origin/horizon, event threshold, target/proposal sampling design and cost/loss profiles across:
- candidate Symtropy forecast policy;
- training-climatology policy;
- always-prepare and never-prepare constant baselines;
- candidate ablations and (when available) conventional frequency/GPD baselines;
- perfect-information oracle lower bound.

Compare every profile/regime cell separately. Report decision cost, EV, regret, false-negative and unnecessary-preparation counts, review frequency and cost, as well as calibration, resolution, coverage and ESS. A candidate that only helps a low-cost profile must not be described as globally superior. Report Pareto trade-offs across action cost, impact loss, deferral cost, forecast quality and profile value. Do not tune profile thresholds on the public fixture.

Confirmatory testing needs independently custodied scenarios, independently set cost/loss profiles with user/domain experts, frozen analysis, repeated scenario families and design-aware uncertainty intervals. Measure forecast value against real user requirements without turning a fictional profile into an operational prescription.

## 7. Cross-project responsibility

- **Symtropy:** emits forecasts, scenario/graph/model/build digests, and the declared sample-generation metadata.
- **Praxis/NRES:** computes forecast diagnostics and compares decisions under explicit profile rules; it does not create the candidate forecasts or select actual public-service actions.
- **Mycelix:** retains provenance, source qualifications and authority. Human-review fallbacks remain distinct from model predictions.
- **Sol-Atlas:** may provide evidence-qualified planning inputs; it does not turn mapped proximity into a proof of capacity or authority.

## 8. Reproducibility

From `mycelix-praxis`:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --decision-value
```

Passing the public fixture demonstrates deterministic score/cost calculation and contract checks only. It does not demonstrate causal intervention benefit, real municipal savings, user acceptance, legal authority, production readiness or SOTA superiority.
