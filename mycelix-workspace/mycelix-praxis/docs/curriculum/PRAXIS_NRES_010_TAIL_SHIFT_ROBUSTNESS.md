# PRAXIS-NRES-010: Rare-event tail calibration and distribution-shift robustness

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. It does not measure real outage likelihood, operational risk or public-service resilience.  
**Fixture:** `examples/courses/civic-resilience-tail-shift-robustness-fixture-001.json`.  
**Conformance corpus:** `examples/courses/civic-resilience-tail-shift-robustness-conformance-001.json`.  
**Schema:** `schemas/nres/nres-tail-shift-robustness-v1.schema.json`.  
**Implementation:** `benchmarks/nres-capability-loop/src/tail_shift.rs`.

## 1. Research motivation: overall forecast scores do not certify tail reliability

A model can perform well on ordinary days while systematically underestimating prolonged, multi-service disruption. NRES-008 and NRES-009 add marginal and joint forecast scores, but neither alone establishes that the *extreme tail* of a continuous impact distribution is calibrated.

Allen, Koh, Segers and Ziegel's 2025 *Tail Calibration of Probabilistic Forecasts* introduces occurrence and conditional excess-distribution diagnostics for tail reliability. Its central result is important for benchmark design: standard calibration does not imply tail calibration, and no proper scoring rule (or collection of them) guarantees that arbitrary tail misspecification is distinguishable by expected scores alone. A forecast can attain an expected score arbitrarily close to the ideal while retaining incorrect tail behavior. We therefore must pair proper scores with explicit tail diagnostics rather than simply adding another weighted score. https://doi.org/10.1080/01621459.2025.2506194; preprint: https://arxiv.org/abs/2407.03167.

The 2025 multivariate scoring review similarly argues for complementary, interpretable verification tools: propriety alone does not ensure that a score reveals all relevant defects, and dependence-targeted scores reveal different properties than marginal or overall scores. https://doi.org/10.5194/ascmo-11-23-2025.

## 2. A second important limit: robustness under arbitrary shifts is impossible in general

Henzi, Shen, Law and Bühlmann's *Invariant Probabilistic Prediction* studies distributional changes in probabilistic forecasting and shows that arbitrary shifts do not generally admit invariant, robust probabilistic predictions. Identifiability requires restricting the shift class and assumptions. NRES-010 therefore does not ask a model to be "robust to anything"; it preregisters four explicit regimes and evaluates them separately: https://doi.org/10.1093/biomet/asae063.

- **In-distribution:** reference regime.
- **Temporal shift:** changed temporal conditions / rates.
- **Structural dependency shift:** changed common-cause or service-dependency structure.
- **Evidence degradation:** missing or conflicting critical source inputs; mandatory abstention applies.

The temporal and structural regimes are deliberately forecast-eligible: these are the shifts being tested. Missing, conflicting or unsupported evidence is not converted into zero risk; the model must abstain.

## 3. Tail quantity and forecast contract

The target is aggregate normalized service-deficit-hours over a 72-hour horizon. It is the integrated shortfall across the declared services (a continuous severity measure), not a people score or a municipal readiness index.

The fixed tail threshold is (u=48). For each forecast-eligible scenario, the candidate emits:
1. (p_u=P(Y>u)), the probability of exceeding the severe-impact threshold; and
2. a generalized Pareto distribution (GPD) for the conditional excess (X=Y-u\mid Y>u), parameterized by scale \(\sigma>0\) and shape \(\xi\).

For excess (x\geq0), the conditional GPD CDF is:
\[
G_{\sigma,\xi}(x)=
\begin{cases}
1-(1+\xi x/\sigma)^{-1/\xi}, & \xi\neq0,\\
1-e^{-x/\sigma}, & \xi=0.
\end{cases}
\]
When \(\xi<0\), the finite upper endpoint is \(-\sigma/\xi\). An observation beyond this support is a zero-support forecast outcome; it must not be clipped or silently discarded.

## 4. Why stress-enriched sampling must be weighted

Rare catastrophic scenarios are expensive to obtain by naive random simulation. Evaluating only naturally rare observations leaves too few tail events to diagnose them; evaluating a stress-enriched set as though it were naturally sampled distorts event rates.

The fixture therefore declares a two-stratum design in each regime:
- routine: proposal mass (q=0.75), target mass (p=0.97);
- tail-enriched: proposal mass (q=0.25), target mass (p=0.03).

Each regime contains 24 routine and 8 tail-enriched cases. The frozen importance weight is:
\[
w_i=\frac{p(\text{stratum}_i)}{q(\text{stratum}_i)}.
\]
Every weighted average reports raw case counts, sum of weights and effective sample size:
\[
ESS=\frac{(\sum_i w_i)^2}{\sum_i w_i^2}.
\]

This reweights only the declared stratum mixture. It does not repair selection bias *within* a stratum, unknown target probabilities, or model misspecification. Confirmatory experiments must document how both proposal and target distributions were constructed and retain scenario-level likelihood ratios. Weight clipping is not allowed unless preregistered and accompanied by a bias analysis.

## 5. Evaluation outputs

### Exceedance occurrence

For forecasts that are issued, report:
- importance-weighted Brier score;
- importance-weighted logarithmic loss;
- a fixed climatology Brier baseline and Brier skill;
- weighted predicted exceedance rate and weighted observed exceedance rate;
- the occurrence calibration ratio (observed / predicted);
- reliability bins by forecast probability;
- case coverage, importance-weighted coverage, forecast ESS, observed tail count and tail-event ESS.

A zero assigned to an observed exceedance yields infinite log loss and is reported explicitly, not epsilon-clipped for the tail-support audit. The epsilon is only used for finite numerical log-loss calculations where the candidate probability is positive but near machine precision.

### Conditional tail severity

For each observed exceedance, compute the conditional GPD excess PIT (G_{\sigma,\xi}(Y-u)). Report the weighted empirical PIT deviation from uniformity as a diagnostic, conditional tail sample size, weight sum, ESS and zero-support count.

Also report weighted empirical coverage of the forecast's conditional 50%, 90% and 95% excess quantiles. These are tail diagnostics, not proper-score substitutes. A good exceedance probability cannot compensate for a badly under-dispersed conditional severity distribution.

The weighted PIT distance is descriptive; it is not a p-value or a formal goodness-of-fit test. Formal confidence intervals and hypothesis tests require design-aware resampling and substantially more independent tail cases.

### Shift robustness

Report every regime separately. Compare the Brier and log-loss degradation against the in-distribution reference, as well as tail PIT and quantile coverage by regime. Show worst-slice performance and coverage. Do not average away the structural-shift result inside the in-distribution mean.

The 2025 ECMWF memorandum on "Information and Noise" is another important comparison direction: it separates reliability from resolution, helping distinguish a calibrated-but-uninformative forecast from one that meaningfully discriminates events. Future NRES work should adopt a principled resolution decomposition alongside proper scores: https://www.ecmwf.int/en/elibrary/81662/forecast-verification-using-information-and-noise.

## 6. Industry comparison and benchmark targets

Compare identical scenario/seed/graph digests, event thresholds, service definitions and sampling weights across:
1. fixed training climatology;
2. a transparent regime-conditioned frequency baseline (trained on training data only);
3. a conventional severity model with fixed GPD fit;
4. candidate Symtropy forecast;
5. candidate ablations without dependency-shift or evidence-quality signals.

Proposed integrity gates:
- 100% valid range/support/proposal-weight checks;
- 100% mandatory abstention when critical inputs are missing/conflicting/unsupported;
- exact reproduction of weighted Brier/log-loss, occurrence and excess-PIT golden vectors;
- exact preservation of importance weights and null forecast states;
- explicit effective sample sizes for every regime and tail cohort;
- no composite score and no post-hoc threshold selection.

Predictive targets must be established from a larger, independently generated challenge set, not this public fixture. Pre-register:
- weighted Brier/log-loss improvement against both climatology and transparent baselines;
- no material degradation beyond preregistered uncertainty bounds in any critical shift regime;
- tail occurrence calibration intervals containing nominal calibration targets at adequate effective sample size;
- conditional excess PIT compatibility and 50/90/95% weighted quantile coverage intervals;
- zero silent zero-support events;
- fixed decision thresholds with separate false-negative / false-positive / deferral-cost reports.

Do not set universal numeric pass thresholds until event rates and attainable sampling precision are estimated. For rare events, raw case count is not enough: effective sample size and independent scenario families matter.

## 7. Relationship to NRES-007/008/009 and project boundaries

- NRES-007 scores service trajectories and recovery.
- NRES-008 scores individual-service breach probabilities and abstention.
- NRES-009 scores joint distributions and dependence.
- NRES-010 scores a continuous tail-impact forecast, conditional tail severity and performance across specified shifts.

Symtropy owns scenario generation and candidate ensemble/forecast output. Praxis independently verifies it. Mycelix retains provenance, evidence qualification and authority. The simulator must emit scenario/graph/model/build digests, forecast origin/horizon, sampling proposal probabilities and per-case likelihood weights. Praxis must not generate or self-certify the candidate forecast.

## 8. Reproducibility and limits

Run from `mycelix-praxis`:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --tail-shift
```

The public 128-case fixture exists to validate formulas, weighting, GPD support and abstention accounting. Its tail samples are too few for statistical conclusions. A successful local evaluator run would prove implementation consistency only—not real-world tail calibration, shift invariance, causal understanding, operational safety or SOTA superiority.
