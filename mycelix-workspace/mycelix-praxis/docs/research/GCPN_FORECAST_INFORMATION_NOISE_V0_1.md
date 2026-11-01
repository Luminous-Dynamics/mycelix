# GCPN Forecast Information, Noise and Post-Processing Attribution v0.1

**Status:** Stage A research specification; Stage B is a standalone deterministic Rust mathematical qualification package.  
**Parent issue:** #5676.  
**Scope:** offline synthetic verification diagnostics; no forecast-generation or operational calibration capability.

## 1. Research basis and motivation

Bonavita & Geer (2026) explain that traditional scores such as RMSE and anomaly correlation can mix intrinsic statistical resolution with bias and forecast activity. A smoother forecast can lower total error without adding predictive information. Their information/noise approach distinguishes the projection of forecast anomalies onto verifying anomalies from the orthogonal noise component; their information-error and noise-error decomposition reconstructs RMSE under the same vector definition.

GCPN should implement this as a transparent mathematical diagnostic and known-truth benchmark, while explicitly keeping bias, calibration/reliability, reference uncertainty and sampling uncertainty separate.

This is not a replacement for proper probabilistic scores, spatial scores, event verification or decision-value analysis.

## 2. Paradigm shift

Move from one leaderboard score to **attribution of forecast quality**:

- Total error: what the forecast product gets wrong.
- Bias: systematic offset relative to the verifying target.
- Intrinsic information: how much verifying-anomaly direction and amplitude is represented.
- Information error (IE): error along the verifying-anomaly direction.
- Orthogonal noise error (NE): forecast error orthogonal to the verifying-anomaly direction.
- Forecast activity: variability/amplitude relative to the verifying signal.
- Reliability/calibration: whether probabilities or values are statistically reliable after post-processing.
- Reference/observation uncertainty: whether the verifying data represent the target.
- Sampling uncertainty: finite independent-system uncertainty.

These are complementary dimensions, not interchangeable benefits. Calibration can be useful to decision-makers without constituting an increase in intrinsic resolution.

## 3. Frozen vector contract

Each case must declare:
- case ID and independent-system ID;
- forecast model ID and version;
- verification source ID and version;
- climatology/baseline ID;
- predictand ID and unit;
- temporal support ID;
- spatial support digest;
- as-of cutoff and recorded time;
- forecast, verifying and climatology vectors on identical support;
- positive finite area/cell weights.

The engine receives forecast values, verifying values, climatology values and weights. It computes the weighted forecast bias, then constructs the bias-adjusted forecast anomaly and the verifying anomaly under the frozen baseline.

The v0.1 weighted inner product is normalized by total weight. Let F be the bias-adjusted forecast-anomaly vector and T the verifying-anomaly vector:

- p = <F,T>_w / ||T||_w
- FI = p / ||T||_w when ||T||_w > 0
- IE = abs(||T||_w - p)
- NE = ||F - projection_T(F)||_w
- debiased_RMSE² = IE² + NE²

where projection_T(F) = (p / ||T||_w) T.

The engine reports raw bias, raw RMSE, debiased RMSE, forecast activity, truth activity, FI, IE and NE separately. It must preserve negative FI and FI above one. No clipping to [-1,1] is allowed.

If truth anomaly norm is zero, FI/IE/NE are undefined for the case and must be reported unavailable/degenerate; raw and debiased RMSE remain reportable.

## 4. DGP and diagnostic families

1. Perfect forecast.
2. Under-active and over-active forecast.
3. Orthogonal noise perturbation.
4. Smoothing / shrinkage that improves total RMSE while reducing intrinsic information.
5. Bias-only perturbation, reported in the bias channel.
6. Sign-reversed and negatively informative forecasts.
7. Heteroscedastic spatial activity and nonuniform positive weights.
8. Zero truth anomaly (degenerate projection).
9. Non-finite values, invalid weights, dimension mismatch and empty fields.
10. Climatology/baseline mismatch, support mismatch and source/version mismatch.
11. Forecast/observation temporal cutoff mismatch.
12. Calibration trained on confirmatory outcomes.
13. Earlier-training/held-out calibration with declared fit-data digest and cutoff.

Every synthetic fixture has independently calculated analytical oracles. The engine's observed vectors cannot serve as their own declared population truth.

## 5. Post-processing and calibration provenance

Any post-processing operation must declare:
- operation ID and version;
- input forecast model/version;
- fitting data digest;
- latest training time or a predeclared cross-fit fold ID;
- calibration/processing parameter digest;
- whether confirmatory outcomes were used;
- evaluation period start.

The engine must reject or mark a confirmatory intrinsic-skill comparison invalid when calibration uses evaluation outcomes or its training cutoff overlaps the evaluation period.

Report raw and processed results side by side. Do not claim calibration improvements equal intrinsic-information improvements. Do not infer that arbitrary nonlinear post-processing can never change rank-based statistical resolution; benchmark declared transformations and report their effects.

## 6. Negative controls and metamorphic invariants

- Perfect forecast produces zero raw and debiased RMSE, zero IE and NE, and FI=1 when truth norm is positive.
- Pure orthogonal perturbation leaves projection information unchanged while increasing NE.
- Under-activity and smoothing can lower total RMSE in a selected DGP while worsening IE relative to a more informative noisier forecast.
- Bias-only perturbation changes reported bias and raw error; post-bias projection diagnostics use only the declared bias correction.
- Negative FI remains negative; over-amplified forecasts can have FI greater than one.
- Zero truth norm yields explicit degenerate status, not NaN or fabricated FI.
- Reordering grid cells with corresponding weights preserves metrics.
- Multiplying all positive weights by the same positive constant preserves normalized metrics.
- Duplicate case IDs, mismatched case frames, sources, versions, climatologies and supports are rejected.
- Confirmatory calibration leakage is detected, never repaired by silently changing the cutoff.
- Case-level results are retained; pooling must not erase sign reversals or under/over-activity regimes.

## 7. Proposed qualification targets (not standards or achieved results)

| Dimension | Proposed target |
|---|---|
| Vector identity | 100% algebraic RMSE reconstruction within a fixed floating-point tolerance |
| Perfect control | Exact zero IE/NE and FI=1 |
| Projection domain | No clipping of negative FI or FI above one |
| Degeneracy | 100% explicit abstention for zero truth norm |
| Input integrity | 100% rejection of invalid dimensions, weights and source/support mismatches |
| Bias | 100% separate raw-bias reporting |
| Calibration leakage | Zero confirmatory fit/evaluation overlap accepted |
| Metamorphic stability | 100% invariance to paired cell/weight reorder and common positive weight rescaling |
| Reporting | IE, NE, activity, bias, raw/debiased RMSE and FI shown separately |
| Statistical inference | Population claims use independent-system sampling uncertainty from #5652 |

These are proposed research gates, not official standards or achieved results.

## 8. Relationship to GCPN

- #5652 estimates sampling uncertainty at the independent-system level.
- #5664 scores spatial ensemble distributions.
- #5667 evaluates decision value.
- #5668 qualifies selective outcome ascertainment.
- #5670 qualifies the observation-error process.
- #5672 evaluates interacting evidence-chain failures.

This track attributes deterministic forecast error and post-processing changes. It does not replace the others or collapse them into an overall score.

## 9. Research and industry positioning

Bonavita & Geer (2026) demonstrate the operational relevance of separating information from noise and show that smoothing, forecast activity, calibration and intrinsic resolution can affect traditional scores differently. ECMWF Technical Memorandum 927 (2025) explains the approach in an operational NWP context. WeatherBench 2 supplies common data, baselines and scorecards but also emphasizes that no single metric captures all dimensions of forecast quality.

GCPN's contribution is a dependency-free, known-truth executable implementation of the projection decomposition, strict provenance and leakage controls, metamorphic testing and integration with the broader evidence-qualification chain. It is not a claim of improved forecast skill.

## 10. Implementation boundary

Stage B is a standalone dependency-free Rust library and deterministic CLI. It computes per-case weighted vector diagnostics, validates provenance and support, checks post-processing fit windows, and emits typed degenerate/valid outputs. It does not train or apply calibrators, generate forecasts, fetch datasets, produce probabilistic scores or estimate sampling confidence intervals.

## 11. Research sources

- Bonavita, M. & Geer, A. (2026), *Forecast verification using information and noise*, Quarterly Journal of the Royal Meteorological Society, first published 8 February 2026. DOI: https://doi.org/10.1002/qj.70109
- ECMWF Technical Memorandum 927 (2025), *Forecast verification using Information and Noise*: https://www.ecmwf.int/en/elibrary/81662-forecast-verification-using-information-and-noise
- ECMWF science explainer (2026), *Separating the signal from the noise*: https://www.ecmwf.int/en/about/media-centre/science-blog/2026/separating-signal-noise
- Rasp et al. (2024), WeatherBench 2: https://agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2023MS004019

## 12. Non-claims

No real-world forecast improvement, universal skill metric, operational calibration quality, causal impact, sensor certification, SOTA superiority or operational readiness is claimed.
