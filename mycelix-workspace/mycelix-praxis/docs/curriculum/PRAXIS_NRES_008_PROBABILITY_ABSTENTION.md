# PRAXIS-NRES-008: Probability calibration and evidence-qualified abstention

**Status:** synthetic benchmark proposal and Rust reference evaluator for review. No real outage probability, public-service reliability or emergency readiness has been measured.  
**Fixture:** `examples/courses/civic-resilience-probability-abstention-fixture-001.json`.  
**Conformance corpus:** `examples/courses/civic-resilience-probability-abstention-conformance-001.json`.  
**Schema:** `schemas/nres/nres-probability-abstention-v1.schema.json`.  
**Implementation:** `benchmarks/nres-capability-loop/src/probability.rs`.

## 1. The next research shift: a model must know when it cannot know

NRES-007 measures an observed or simulated trajectory after its values exist. It does not answer whether a model's forecast about a future service-floor breach deserves confidence.

NRES-008 evaluates a different output: **the probability that a service will fall below its declared floor at least once in a fixed future horizon**. It scores probability quality against evaluator-owned outcomes, checks calibration, and makes abstention an explicit action.

The critical distinction is between:
- a qualified input on which a probability forecast may be emitted;
- missing critical input, conflicting sources, or unsupported distribution shift, for which the benchmark requires abstention; and
- an otherwise-qualified input on which a model voluntarily abstains because it cannot justify a forecast.

The evaluator-owned qualification state and observed minimum availability are kept separate from the candidate's action/probability output. Public examples expose both for reproducibility, but a confirmatory test must keep evaluator truth hidden from the candidate.

## 2. Compose with forecasting research and current preparedness practice

Proper scoring rules are the right foundation because they reward honest probability distributions in expectation. For binary breach events, NRES-008 uses the Brier score and logarithmic loss. Calibration diagnostics and forecast concentration/sharpness should be reported together; a narrow or confident forecast is not valuable if it is statistically unreliable. A 2025 review of multivariate probabilistic forecast verification discusses proper scores and the calibration/sharpness distinction: https://ascmo.copernicus.org/articles/11/23/2025/.

Fixed-bin expected calibration error (ECE) is included as a **diagnostic only**, not as the primary ranking score. ECE depends on binning and sample size and is not itself a proper scoring rule.

Conformal prediction can be useful for uncertainty sets, but its guarantees rely on assumptions such as exchangeability. Time-series dependence and distribution shift can break those assumptions; the 2025 tutorial explicitly warns against treating conformal coverage as assumption-free: https://journals.sagepub.com/doi/full/10.1177/25152459251380452. NRES-008 therefore makes no distribution-free coverage claim. Unsupported shift is a reason to abstain until a qualified method and validation regime exist.

This composes with established preparedness practice. FEMA HSEEP already provides objective-based exercise evaluation and corrective-action planning: https://training.fema.gov/programs/nsec/hseep/. WHO's 2026 national exercise guidance recommends sustained, progressive, system-wide simulation programmes rather than isolated drills: https://www.who.int/news/item/24-03-2026-new-who-guidance-helps-countries-to-institutionalize-simulation-exercises-to-strengthen-health-emergency-readiness. NRES-008 contributes an auditable probability/abstention measure to such programmes; it does not replace them.

## 3. Fixture and evaluator contract

The public fixture has 25 fictional cases:
- 20 forecast outputs on qualified inputs;
- 4 required abstentions: missing critical inputs (2), conflicting sources (1), and unsupported distribution shift (1);
- 1 optional model-uncertainty abstention on otherwise-qualified input.

The forecast horizon is 24 hours. The binary outcome is derived by comparing the evaluator-side observed minimum availability over that horizon with the service's frozen floor. A value strictly below the floor is a breach; equality is not.

The aggregate calibration-reference cohort has 20 synthetic observations and 10 breaches, giving a reference probability of 0.5. It is used solely to calculate the climatology Brier baseline in this public example. It is not a real-world base rate or a model-training claim.

The evaluator returns:
1. total and qualified-case forecast coverage;
2. abstention count, mandatory-abstention compliance and optional abstentions;
3. overall selective Brier score and logarithmic loss, using the frozen epsilon for numerical stability;
4. Brier skill score relative to the calibration-reference climatology on the exact same issued cases;
5. per-service Brier and log-loss summaries with explicit sample counts;
6. fixed reliability-bin counts, mean forecast probability and observed event rate;
7. fixed-bin ECE, labelled a diagnostic; and
8. a confidence-threshold risk/coverage curve.

A fixed confidence-threshold risk/coverage curve is also reported. Confidence is defined as \\( |2p-1| \\), with thresholds 0, 0.25, 0.5 and 0.75; at each point the evaluator reports selected-case count, population/eligible coverage and selective Brier score. Every service receives its own proper-score row so a weak service cannot disappear inside an overall average. Abstained cases contribute to the coverage and abstention denominators but do not receive invented probabilities and do not enter proper-score denominators. Every denominator is explicit. A predictor that abstains on every case has zero coverage and is not comparable to one that forecasts at useful coverage.

## 4. Non-negotiable rules

- A forecast must contain one finite probability in [0,1]. An abstention must contain a null probability and an explicit reason.
- Missing critical input, conflicting sources, and unsupported distribution shift require abstention.
- A qualified case may optionally abstain only with the declared model-uncertainty reason; this counts against eligible-case coverage.
- The candidate cannot redefine evaluator-owned input qualification or ground-truth outcomes.
- Compare systems on identical cases, floors, horizons, qualification labels and evaluation splits.
- Report proper scores and forecast coverage together. Never compare selective scores at unequal coverage without the complete risk/coverage curve.
- Report the climatology baseline and calibration cohort size. Do not infer event frequency in a real municipality from a synthetic reference cohort.
- ECE is diagnostic only. Do not use it as the sole optimisation target or rank systems by it.
- A shift flag is not a mathematical shift detector. The qualification process and its evidence must be independently specified and audited.
- Do not convert unknown, missing, conflicting or unsupported evidence into a zero probability.
- No composite resilience, political, loyalty, enlistment or personal-suitability score is permitted.

## 5. Benchmark targets and comparative study

### Conformance gates

- 100% of listed invalid forecast/abstention cases are rejected.
- 100% of mandatory-abstention cases are abstained on.
- 100% of abstentions carry null probability and a declared reason.
- All golden proper-score and coverage vectors reproduce within absolute tolerance (10^{-12}).
- Null probability, evaluator truth, input qualification, horizons and service units survive round trips exactly.

These are evidence-contract gates, not predictive-performance targets.

### Predictive comparison

Use at least these comparators on identical frozen cases:
1. **Climatology:** the event rate from the declared calibration cohort.
2. **Simple frequency model:** service-specific historical breach frequency, where adequate training data exist.
3. **Transparent rule baseline:** a versioned deterministic estimate from explicitly declared evidence.
4. **Candidate model:** the method under evaluation.
5. **Ablations:** remove source freshness, dependency completeness, or other candidate input groups one at a time.

Compare Brier score and log loss, then per-service scores, calibration bins and the fixed confidence-threshold risk/coverage curve. Where a candidate uses conformal prediction or selective deferral, include its declared calibration assumptions, deferral cost, coverage guarantee and behavior under each shifted slice; do not compare a nominal coverage guarantee with a shifted dataset as if the exchangeability assumption were automatic. For each system, publish uncertainty intervals over repeated scenario seeds and the entire risk/coverage curve. Use paired comparisons because every model sees the same scenario. Count invalid runs and abstentions; do not silently drop difficult cases.

A model should not be called superior merely because it improves mean Brier score while increasing false confidence under unsupported shift or reducing coverage drastically. Mandatory abstention compliance is a hard gate; probabilistic skill must be assessed at comparable coverage.

### Distribution-shift study

Separate at least four evaluation slices:
- same-generation / in-distribution;
- temporal shift;
- structural shift (changed service dependencies or supplier concentration);
- evidence degradation (missing, stale or conflicting inputs).

Keep scenario generation and model evaluation independent. Standard conformal coverage must not be claimed across these shifts unless the applicable assumptions and target guarantees have been demonstrated for that setting.

Do not set universal target probabilities or ECE thresholds from this synthetic example. First estimate event rates, variability, sample-size needs and calibration uncertainty using independently controlled cases. Report sample counts and confidence intervals. For rare breaches, avoid interpreting small-sample calibration bins as stable.

## 6. Relationship to NRES-005 through NRES-007

- NRES-005 records exercise-to-improvement evidence and independent retests.
- NRES-006 transports provenance and claim-state metadata.
- NRES-007 evaluates observed/simulated time-series service outcomes.
- NRES-008 evaluates forecasts made *before* those outcomes are observed, including whether the model properly abstains.

NRES-008 should consume NRES-007 service IDs, floors, units and horizon metadata but must not duplicate the cascade simulator. Symtropy generates candidate scenario forecasts; the evaluator independently scores those forecasts against hidden synthetic truth or approved observed outcomes. Mycelix retains source/claim provenance and authority. Simulated probabilities remain simulations, not official operational estimates.

## 7. Reproducibility

Run from the `mycelix-praxis` directory:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --probability
```

A successful result means the public synthetic fixture is structurally consistent and the declared scores were calculated. It does not establish predictive validity, calibrated real-world probabilities, emergency readiness, institutional legitimacy, legal compliance, causal benefit or SOTA superiority.

The public fixture is a development example, not a held-out evaluation set. Confirmatory work requires independent custody, frozen scoring code, preregistered hypotheses, multiple independent scenario families, and a third-party verifier.
