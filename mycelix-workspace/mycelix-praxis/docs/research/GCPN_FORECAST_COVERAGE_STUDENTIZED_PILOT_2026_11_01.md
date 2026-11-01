# Exploratory Studentized Cluster-Bootstrap Pilot — 2026-11-01

**Status:** exploratory, separately coded Python experiment; not a Rust execution and not confirmatory qualification.  
**Purpose:** compare the newly implemented independent-system studentized bootstrap-t candidate with percentile and reflected/basic intervals on the same known two-state DGP.  
**Related:** #5644, implementation PR #5652.

## 1. Frozen DGP

The pilot uses the v0.1 two-state cluster DGP:
- high-risk independent-system probability: 0.20;
- two event IDs per system;
- 2 cases in each low-risk system and 8 cases in each high-risk system;
- event probability 0.10 in low-risk systems and 0.60 in high-risk systems;
- candidate event probability 0.20 and reference probability 0.35;
- the case-weighted population paired Brier difference is exactly +0.0225;
- the system-weighted population target is −0.0225.

The system-level cluster score sum is recomputed from all cases in each system. Candidate and reference forecasts share the same outcome for every case.

## 2. Pilot protocol

For each independent-system count (40, 80, 160, 320):
- 1,200 independently generated outer datasets;
- 500 paired bootstrap replicates per outer dataset;
- nominal 95% intervals;
- complete independent systems resampled as units;
- case-weighted ratio-of-sums estimate in each bootstrap replicate;
- percentile interval;
- reflected/basic interval;
- studentized bootstrap-t interval.

The studentized statistic uses the cluster ratio-estimator standard error:

`SE(theta_hat) = sqrt(s^2(D_g - theta_hat*n_g) / (G * mean(n_g)^2))`.

The bootstrap-t replicate recomputes both the ratio estimate and the cluster standard error. A draw with zero/non-finite standard error is a degenerate replicate. The implementation requires at least 80% valid studentized draws; no outer dataset abstained in this pilot.

The exploratory simulation uses NumPy PCG64 with seed rule `20261102 + G`. It is separately coded from the Rust XorShift implementation and therefore is not bit-for-bit output reproduction of the Rust package.

## 3. Results

| Independent systems | Percentile coverage | Basic coverage | Studentized coverage | Studentized Wilson 95% |
|---:|---:|---:|---:|---:|
| 40 | 92.25% | 91.92% | 96.67% | [95.49%, 97.54%] |
| 80 | 93.08% | 93.25% | 95.67% | [94.36%, 96.68%] |
| 160 | 93.67% | 94.00% | 94.92% | [93.52%, 96.02%] |
| 320 | 95.08% | 94.75% | 95.33% | [93.99%, 96.39%] |

Studentized coverage Monte Carlo standard errors:
- 40 systems: 0.52 percentage points;
- 80 systems: 0.59 percentage points;
- 160 systems: 0.63 percentage points;
- 320 systems: 0.61 percentage points.

Mean interval widths (percentile/basic/studentized):
- 40 systems: 0.07036 / 0.07036 / 0.07923;
- 80 systems: 0.05000 / 0.05000 / 0.05224;
- 160 systems: 0.03553 / 0.03553 / 0.03624;
- 320 systems: 0.02519 / 0.02519 / 0.02542.

All 1,200 outer datasets emitted a studentized interval at each system count. That does not prove the 80% valid-replicate guard is appropriate for other DGPs.

## 4. Interpretation

This pilot suggests:
1. Studentization may improve small-cluster coverage for this particular ratio estimand and two-state DGP.
2. At 40 systems the studentized interval over-covers relative to nominal 95% in this pilot, and its Wilson Monte Carlo interval lies above 95%. More data are needed to distinguish a genuine finite-sample tendency from inner/outer Monte Carlo variation.
3. At 80, 160 and 320 systems, the studentized measured coverage is close to nominal 95%, with its Wilson interval containing 95%.
4. Studentized intervals are modestly wider than the percentile/basic intervals at the tested cluster counts.
5. The basic interval does not materially improve calibration over percentile in this DGP.

This is a method-design signal, not evidence that studentized bootstrap-t is generally superior. Studentization can be unstable when the cluster ratio standard error is degenerate, the number of independent clusters is small, cluster sizes are highly imbalanced, or the DGP violates assumptions.

## 5. Limitations

- 1,200 outer datasets is a pilot, not the 10,000-dataset confirmatory target.
- 500 inner bootstrap replicates do not fully characterize quantile Monte Carlo error, especially for 99% interval endpoints.
- Only one stationary two-state DGP and one nominal interval level were tested.
- No IID/event-level negative controls were included in this second run; the earlier exploratory pilot reported those under-clustering comparisons separately.
- The Python generator used a different RNG and draw order from Rust.
- No Rust tests, Cargo build, CLI or CI were executed.
- No meteorological data or validated weather-system clusters were used.

## 6. Next qualification steps

1. Freeze this pilot protocol and seeds separately from confirmatory seeds.
2. Measure inner-bootstrap quantile stability at 500, 1,000, 2,000 and 5,000+ replicates before setting confirmatory counts.
3. Run 10,000 outer datasets for null, better/worse, rare-event, informative-size and heterogeneous-effect DGPs.
4. Qualify nominal 90%, 95% and 99% intervals independently; report tail misses and coverage MCSE/Wilson intervals.
5. Test 80% studentized-replicate acceptance as a hyperparameter under degenerate and heavy-tailed regimes; it is not yet qualified.
6. Independently execute Rust and reproduce the population oracle, then compare against the separately coded Python reference.

No interval method is qualified by this note.

## 7. Non-claims

No Rust execution, confirmatory calibration guarantee, universal studentized-bootstrap superiority, real forecast comparison, meteorological cluster validation, SOTA model comparison or operational-warning qualification is claimed.
