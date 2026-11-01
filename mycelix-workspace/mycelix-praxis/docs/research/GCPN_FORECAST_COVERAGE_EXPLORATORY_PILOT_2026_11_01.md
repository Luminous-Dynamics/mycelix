# Exploratory Known-DGP Coverage Pilot — 2026-11-01

**Status:** exploratory, separately coded simulation; not a Rust execution result and not confirmatory qualification.  
**Purpose:** check whether the declared bootstrap hierarchy produces visibly different uncertainty behavior before expanding Stage C.  
**Related:** #5644 and implementation PR #5652.

## 1. Frozen pilot DGP

The pilot independently re-expressed the first Rust DGP in a small NumPy experiment to explore finite-sample behavior. It used the same declared population model:

- high-risk independent-system probability: 0.20;
- two event IDs per system;
- one case per event in a low-risk system and four cases per event in a high-risk system;
- event probability: 0.10 in the low-risk state and 0.60 in the high-risk state;
- candidate probability: 0.20;
- reference probability: 0.35;
- binary paired Brier difference, candidate minus reference;
- analytical case-weighted truth: +0.0225.

The low-state expected paired difference is −0.0525; the high-state expected difference is +0.0975. Informative cluster size makes the system-weighted target −0.0225, distinct from the +0.0225 case-weighted target.

## 2. Pilot protocol

For each independent-system count (40, 80, 160, 320):
- 1,500 independently generated outer datasets;
- 500 paired percentile-bootstrap replicates per method and dataset;
- nominal 95% intervals;
- three resampling units: individual cases, event IDs, and complete independent systems;
- case-weighted ratio-of-sums estimator recomputed in each replicate;
- Wilson 95% binomial interval and Monte Carlo standard error reported for measured coverage.

The exploratory generator used NumPy PCG64 with seed `20261102 + independent_system_count`. This is a second implementation of the DGP, not an exact bit-for-bit reproduction of the Rust XorShift sequence.

## 3. Results

| Independent systems | Case bootstrap | Event bootstrap | Independent-system bootstrap |
|---:|---:|---:|---:|
| 40 | 83.27% | 88.60% | 93.07% |
| 80 | 81.87% | 88.27% | 93.47% |
| 160 | 81.73% | 89.00% | 94.13% |
| 320 | 81.67% | 89.13% | 94.27% |

Measured coverage MCSEs were approximately:
- case bootstrap: 0.96–1.00 percentage points;
- event bootstrap: 0.80–0.83 percentage points;
- independent-system bootstrap: 0.60–0.66 percentage points.

The Wilson Monte Carlo intervals for the independent-system method were:
- 40 systems: [91.67%, 94.25%];
- 80 systems: [92.10%, 94.61%];
- 160 systems: [92.83%, 95.21%];
- 320 systems: [92.97%, 95.33%].

These intervals quantify Monte Carlo uncertainty in the measured coverage, not uncertainty in a real forecast comparison.

## 4. Interpretation

The pilot supports three limited conclusions:

1. Resampling individual cases badly understates uncertainty under this nested shared-system DGP.
2. Resampling event IDs is better than resampling rows, but still under-covers because two event IDs within each system share the latent state.
3. Resampling independent systems is materially better, but the percentile interval is not demonstrated to achieve nominal 95% coverage at small cluster counts. At 40 and 80 systems, the Monte Carlo Wilson intervals are below 95%; at 160 and 320 systems they include 95%, but the pilot is too small to certify calibration.

The result reinforces the rule that `event_id` does not prove independence. It also shows that choosing the correct cluster is necessary but may not be sufficient for accurate finite-sample coverage.

## 5. Important limitations

- This used 1,500 outer trials and only 500 inner bootstrap replicates. It is a method-design pilot, not a confirmatory study.
- The pilot was separately coded from the Rust crate and used a different RNG. It does not establish that the Rust code compiles, tests pass or emits the same numerical results.
- Only one two-state DGP, one effect configuration and one nominal interval level were examined.
- The pilot does not prove universal undercoverage of case/event bootstrap; the result applies to this deliberately dependent DGP.
- No real meteorological observations or validated storm/system cluster assignments were used.

## 6. Required follow-on method work

Before qualification:
1. run the same DGP with 10,000 outer datasets and enough bootstrap replicates for stable 99% tail quantiles;
2. add null, candidate-better/worse, rare-event and heterogeneous-effect DGPs;
3. compare percentile, basic, studentized and BCa intervals, with explicit failure/abstention cases;
4. investigate small-cluster adjustments and wild-cluster bootstrap-t only with a method derived for this paired ratio estimand;
5. test cluster imbalance and informative cluster-size effects;
6. report 90%, 95% and 99% coverage, both tail miss rates, interval widths, Type-I error, power, MCSE and Wilson intervals;
7. independently reproduce the frozen pilot/confirmatory outputs from the Rust implementation.

No method is qualified by this note. The correct output of the pilot is a research direction, not a pass.

## 7. Non-claims

This is not real forecast verification, an interval calibration guarantee, statistical superiority, a SOTA comparison of weather models, meteorological cluster validation, operational warning qualification or a Rust/CI execution result.
