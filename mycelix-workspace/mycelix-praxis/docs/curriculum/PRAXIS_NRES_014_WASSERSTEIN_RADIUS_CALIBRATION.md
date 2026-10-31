# PRAXIS-NRES-014: Split-conformal calibration of Wasserstein ambiguity radii

**Status:** synthetic development benchmark and Rust reference evaluator for review.  
**Source forecast templates:** NRES-009 joint-service forecast fixture, Git blob SHA-1 799799f8a315d4c0f06a43b5dce5ab2e15ccefb5.  
**Fixture:** civic-resilience-wasserstein-radius-calibration-fixture-001.json.  
**Conformance corpus:** civic-resilience-wasserstein-radius-calibration-conformance-001.json.  
**Schema:** nres-wasserstein-radius-calibration-v1.schema.json.  
**Implementation:** benchmarks/nres-capability-loop/src/radius_calibration.rs.

## 1. Research question

NRES-013 uses fixed Wasserstein radii as sensitivity parameters. NRES-014 asks how to derive a radius from held-out calibration data without selecting it after viewing test outcomes.

The methodological shift is to calibrate at the **source-family block level**. Repeated observations from one scenario family produce one empirical joint-state distribution and therefore one nonconformity score. They do not become dozens of nominally independent calibration cases.

## 2. Block-level score and conformal radius

For calibration block b, let p_b be its frozen forecast distribution over the complete joint service-state support and let qhat_b be the empirical distribution of its observed joint states. Define:

    s_b = W_1(qhat_b, p_b)

using the predeclared weighted-Hamming ground metric from NRES-013.

For m exchangeable calibration blocks and target miscoverage alpha, compute:

    k = ceil((m + 1) * (1 - alpha)).

If k <= m, the calibrated radius is the k-th smallest calibration score (one-based indexing). If k > m, the mathematically correct threshold is +infinity; the serialized output uses null and explicitly states that no finite radius is available. We do not silently substitute the sample maximum.

The W1 score is computed as a finite balanced transportation problem using a successive-shortest-augmenting-path min-cost-flow solver with residual reverse arcs. The state space in this development fixture contains 16 complete joint states.

## 3. Exact guarantee boundary

Conditional on a separately fitted and frozen forecast model, state encoding, ground metric and block-size rule, if calibration source-family blocks and one future source-family block are exchangeable, the split-conformal order statistic gives:

    P(s_future <= radius) >= 1 - alpha,

where probability is joint over the calibration blocks and future block.

This is **marginal block-score coverage**, not coverage conditional on the particular realized calibration sample. The score concerns the future block's empirical outcome distribution relative to its forecast. It does not by itself establish that the unknown population distribution is contained in the ambiguity ball. Arbitrary temporal, structural or evidence shift can break exchangeability and invalidate the statement.

NRES-014's public development fixture does not prove block exchangeability or provide confirmatory coverage evidence. It demonstrates the rank calculation, exact finite-state transport, split separation and fail-closed conformance behavior.

## 4. Fixture design

- 19 synthetic calibration blocks, each with 12 observations.
- 12 separate synthetic development blocks, each with 12 observations.
- 16 joint service states and one frozen weighted-Hamming metric.
- Target miscoverage 0.10, target marginal coverage 0.90.
- Conformal rank: ceil((19+1)*0.90) = 18.
- Calibrated synthetic radius: 0.9211372893629722 weighted-Hamming units.
- Development diagnostic: 12/12 blocks fall within the radius (100% on this tiny fictional holdout).

The development diagnostic is not a coverage guarantee, a real-world estimate or a reason to change alpha. The 19+12 synthetic family identifiers are fixture bookkeeping, not independently custodied scenarios.

## 5. Why the 19-block minimum matters

At alpha=0.10, fewer than 9 blocks cannot even provide a finite standard split-conformal quantile. In fact, with 8 blocks the rank is ceil(9*0.90)=9, which exceeds the sample size. The evaluator returns an unbounded radius. This makes finite-sample resolution visible rather than hiding it behind an apparently precise threshold.

The calibrated radius is a single global radius for the declared block population. It is not conditionally valid for individual services, regimes, regions, or cost profiles. Those require separately predeclared strata and enough independent source families per stratum, or a justified weighted/dependence-aware calibration method.

## 6. Research frontier and comparison

- Ohnemus et al. (ICML 2026) learn loss-aware optimal-transport ambiguity sets through bilevel optimization and hypergradients; NRES-014 does not learn the metric or ambiguity-set shape. https://proceedings.mlr.press/v306/ohnemus26a.html
- Long et al. (2026) present a unified view of conformal prediction and Wasserstein DRO as finite-sample quantile corrections, distinguishing level inflation from value-space radius shifts. https://arxiv.org/abs/2608.29789
- Ramos et al. (PMLR 2026) analyze the calibration-conditional coverage distribution of split conformal and Wasserstein departures from its beta reference law, including clustered and dependent settings. https://proceedings.mlr.press/v329/r-ramos26a.html
- Shi et al. (UAI 2026) address conformal uncertainty under federated joint shift using privacy-preserving score-distribution sketches and importance weighting. https://proceedings.mlr.press/v337/shi26a.html
- Chen et al. (ICML 2026) propose bulk-calibrated credal ambiguity sets that bound bulk contamination and tail contribution separately. https://proceedings.mlr.press/v306/chen26e.html

The NRES research opportunity is to compare these uncertainty constructions on identical complete joint service-state forecasts and cost profiles, preserving independent scenario custody and evaluating decision regret as well as coverage.

## 7. Next research increments

1. Add block-dependence stress tests and compare ordinary split conformal with cluster/bootstrap or mixing-aware calibration; never assume row-level iid when scenario families share a source.
2. Add weighted/group-conditional calibration for known deployment strata, with effective sample size and an explicit unbounded-radius result when a stratum is too small.
3. Compare fixed-metric conformal W1 radii with loss-aware trainable transport metrics on separate training/validation data.
4. Add the bulk-plus-tail credal comparator to avoid pretending one Wasserstein radius is sufficient for heavy contamination or heavy-tail regimes.
5. Pass a calibrated radius into NRES-013 as a frozen artifact and report realized decision regret, empirical radius coverage and conservative upper bounds separately.
6. Open a separately custodied confirmatory challenge protocol; a public development fixture cannot certify the assumptions.

## 8. Reproducibility

From the mycelix-praxis package root:

~~~bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --calibrate-wasserstein-radius
~~~

A successful run verifies deterministic conformance to the synthetic fixture only. It does not prove the exchangeability assumption, population-law coverage, shift validity, operational readiness or SOTA performance.
