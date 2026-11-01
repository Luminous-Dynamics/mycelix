# GCPN Spatiotemporal Ensemble and Extreme-Event Verification v0.1

**Status:** research specification and machine-readable score/DGP contract; executable package outstanding.  
**Parent issue:** #5662.  
**Baseline:** integrated GCPN evaluation v0.6.  
**Scope:** offline/synthetic forecast-output evaluation; no forecast production or operational decision.

## 1. Research basis and gap

WeatherBench 2 is an open evaluation framework for global medium-range physical and data-driven weather models. It publishes data, baselines, code and continuously updated scorecards, using metrics aligned with operational forecast verification.

The WeatherBench 2 paper explicitly explains that CRPS and spread/skill are univariate statistics: they do not capture spatial and temporal correlations and do not specifically focus on extreme weather. It also stresses that better CRPS does not automatically imply a more useful forecast, and that a range of application-specific metrics is needed.

A crucial fairness concern is verifying-data consistency. WeatherBench 2's published scorecard explains that operational models may be verified against IFS analysis while other models are verified against ERA5. That is transparent in the benchmark, but it means an evaluator must not turn values from different verification sources into a naïve direct leaderboard. GCPN should enforce exact verifying-source and version identity for every comparison.

GCPN's contribution is not another forecasting system. It is an auditable evaluation layer that combines marginal probabilistic scores, joint spatial diagnostics, extreme-event stratification, matched-case contracts and dependence-aware statistical uncertainty.

## 2. Paradigm shift: score the structure that matters

A single scalar metric can conceal failure modes:
- good marginal calibration but incorrect spatial correlation;
- good average performance but failure on rare severe events;
- accurate means but underdispersed uncertainty;
- good individual lead scores but incoherent trajectories across lead times;
- apparent superiority arising from different verifying analyses or case denominators.

The evaluator should expose each property as an independent measurement. It must not construct a weighted composite or automatically select a winning model.

## 3. Scoring definitions

### 3.1 Scalar ensemble CRPS

For ensemble members (x_1,ldots,x_M) and verifying observation (y):

`CRPS = (1/M) sum_m |x_m - y| - (1/(2M^2)) sum_m sum_n |x_m - x_n|`.

The finite-ensemble estimator, duplicate-member handling, missing-member policy, units and ensemble-member count must be explicit. Deterministic forecasts may be scored as degenerate distributions only where the comparison protocol explicitly permits that construction.

### 3.2 Threshold Brier score

For a predeclared event threshold `u`, transform ensemble members to event indicators and derive the ensemble probability `p_u`. Score `(p_u - o_u)^2` against the corresponding observed event indicator. The threshold and event definition must be supplied by a domain/scientific protocol before evaluation; the evaluator must not tune thresholds to favour a model.

Report Brier score, reference score/skill where defined, base rate, reliability, resolution, uncertainty, sample count, misses and false alarms separately.

### 3.3 Multivariate energy score

For ensemble members as full spatial vectors `X_m` and verifying spatial vector `y`:

`ES = mean_m ||X_m - y||_2 - 0.5 * mean_(m,n) ||X_m - X_n||_2`.

This is a joint multivariate proper-score candidate. Report field dimension, spatial mask, metric/scaling, ensemble size and missing-cell policy. Do not assume energy score alone is sensitive to every dependency error.

### 3.4 Variogram score

For a declared exponent `p` and nonnegative pair weights `w_ij`:

`VS_p = sum_(i<j) w_ij * ( |y_i-y_j|^p - mean_m |X_mi-X_mj|^p )^2`.

Pair weights, distance convention, exponent, field mask and normalization must be pinned. This score emphasizes pairwise spatial dependence. It complements rather than replaces energy score and marginal CRPS.

### 3.5 Calibration and ensemble structure

Preserve:
- reliability diagrams and scalar PIT/rank diagnostics;
- ensemble spread and spread-skill;
- rank histograms with explicit treatment of ties;
- calibration-in-large, resolution and sharpness where applicable;
- score distributions by lead, region, variable and predeclared event class.

Do not confuse sharpness with accuracy: concentrated probabilities are beneficial only when supported by verification.

## 4. Temporal and event dependence

Every record preserves:
- model and ensemble identity;
- issue/initialization time;
- valid time;
- lead time;
- variable and units;
- region/grid/mask;
- event ID;
- independently governed dependence-cluster ID where available;
- verification source, version and outcome vintage;
- partition and record-quality state.

Repeated updates, locations, grid cells and leads are not automatically independent samples. A known synthetic joint DGP must distinguish independent systems, nested events, spatially correlated cells and serially correlated time steps.

A bootstrap or block method must preserve the declared independent top-level unit. A row-wise bootstrap is an intentional negative control where spatial/temporal dependence exists.

## 5. Extreme-event evaluation

Use predeclared thresholds or event definitions from the domain protocol. For each threshold/event stratum, report:
- event base rate and denominator;
- Brier score and reference skill;
- reliability, resolution and uncertainty;
- false-alarm and miss counts/rates with denominators;
- score uncertainty and interval width;
- worst region/lead/event stratum;
- missing and excluded forecast counts.

Weighted proper scores can emphasize a declared target distribution, but weights must be frozen and justified. Publish unweighted scores next to weighted results. Do not let the evaluator choose impact thresholds or issue warning recommendations.

## 6. Fair-comparison contract

1. All compared models must use the exact same forecast issue times, valid times, case mask, grid/resolution policy, variables and held-out cases.
2. Every result must record verifying data source, version and outcome vintage.
3. Models with different verification sources cannot share one unqualified comparative ranking. Recompute against a common reference or publish separate panels.
4. Keep calibration/tuning, validation and confirmatory periods separate; use chronological/rolling-origin splits.
5. Preserve ensemble member count and method, post-processing and any deterministic-to-probabilistic conversion.
6. Do not interpolate/resample without recording the operator and its effect on verification.
7. Missing, late, invalid, duplicated, conflicting, out-of-domain and abstained forecasts remain visible.
8. Comparisons must be paired on identical cases. If exact matching is impossible, report an unpaired analysis with its own assumptions.
9. Thresholds, event definitions, weights and subgroup families must be frozen before confirmatory scoring.
10. Report proper scores and structural diagnostics separately. No single winner score.

## 7. Known-DGP matrix

| DGP family | Population truth | Intended diagnostic |
|---|---|---|
| Independent scalar ensembles | Known marginal distribution | CRPS oracle and rank diagnostics |
| Under/overdispersed ensemble | Known variance/spread | Spread-skill and calibration |
| Spatial Gaussian field | Known covariance | Energy/variogram response |
| Correct marginals, wrong spatial dependence | Identical marginal distributions, altered covariance | Marginal-score blind spot |
| Incorrect marginals, correct dependence | Controlled marginal shift | Separate calibration from dependence |
| Spatial displacement/phase shift | Known translated structure | Joint/field score sensitivity |
| Extreme mixture / rare events | Known tail rate and severity distribution | Threshold calibration and sample uncertainty |
| Multi-lead temporal process | Known cross-lead dependence | Trajectory coherence and block dependence |
| Regime shift | Known change point/window | Separate stationary windows |
| Data-integrity mutations | Known exact corruption | Matching, vintage and exclusion fail-closed |

The synthetic population score for each DGP must be analytically derived or independently checked. A finite simulation's own average is not its truth oracle.

## 8. Required measurements

Report independently:
- CRPS by variable/lead/region;
- threshold Brier and reference skill;
- reliability, resolution, uncertainty and sharpness;
- ensemble spread, spread-skill and rank histograms;
- energy score;
- variogram score and full weight/exponent specification;
- extreme-event scores, base rates, false alarms and misses;
- spatial/temporal dependence diagnostics;
- paired-case count and exact matching failures;
- verifying-source/version identity and mismatches;
- dependence-cluster count and exclusions;
- interval coverage, MCSE, width and tail miss rates from the separate known-DGP qualification programme;
- runtime and memory.

## 9. Comparison with WeatherBench 2 and operational practice

A later data adapter should reproduce a selected compatible subset of WeatherBench 2 headline metrics where dataset licensing, variables and evaluation definitions permit. It must pin the exact reference dataset and benchmark protocol.

The extension then adds:
- joint spatial energy/variogram diagnostics;
- explicit marginal-correct/spatial-dependence-wrong controls;
- extreme-event strata and uncertainty;
- strict common-verification-source comparisons;
- dependence-aware uncertainty qualification.

This is a comparison of evaluator breadth, auditability and statistical controls, not a claim that GCPN forecasts weather better than WeatherBench 2 models or ECMWF.

## 10. Proposed research targets (not current results or standards)

- 100% exact paired-case equality for comparisons labelled paired.
- 100% verifying-source/version identity per scored result.
- Zero unqualified cross-source comparative leaderboard rows.
- Exact synthetic CRPS, Brier, energy and variogram oracle reproduction within declared numeric tolerances.
- Correct detection of marginal-correct/spatial-dependence-wrong fixtures by the joint diagnostic family.
- No weighted or threshold-tuned comparison without a frozen, versioned protocol.
- Qualification of 90%, 95% and 99% interval coverage separately across declared DGP cells.
- Every missing/out-of-domain/abstained record visible in the denominator/exclusion report.

## 11. Implementation sequence

A:
- standalone Rust scoring core for CRPS, threshold Brier, energy and variogram scores;
- strict data structs, validation and exact synthetic fixtures;
- CLI and machine-readable report.

B:
- exact score oracles and mutation tests;
- fair-comparison source/version matching;
- marginal/dependence mismatch DGPs.

C:
- event/temporal dependence and extreme-event DGPs;
- integration with the independent known-DGP interval qualification engine.

D:
- independent statistical review;
- public benchmark adapter design and dataset/license review;
- only then consider a module-5 extension.

## 12. Sources

- Rasp et al., *WeatherBench 2: A Benchmark for the Next Generation of Data-Driven Global Weather Models*, Journal of Advances in Modeling Earth Systems (2024): https://agupubs.onlinelibrary.wiley.com/doi/10.1029/2023MS004019
- WeatherBench 2 current benchmark and scorecards: https://sites.research.google/gr/weatherbench/
- WMO, *Guidelines on the Verification of Hydrological Forecasts*, WMO-No. 1364 (2025): https://public.wmo.int/resources/meteoworld/meteoworld-june-2025/guidelines-streamflow-forecast-verification
- ECMWF, *Statistical Concepts — Probabilistic Data*: https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData

## 13. Non-claims

This is a forecast-verification research adapter, not a forecast model, weather service, hazard threshold authority, live early-warning service or SOTA forecast claim. It makes no operational model choice, warning decision or alert.
