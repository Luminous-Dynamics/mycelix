# GCPN Probabilistic Forecast Verification v0.2

**Status:** synthetic probabilistic scoring, calibration and chronological drift diagnostics.  
**Parent:** issue #5630; predecessor forecast benchmark PR #5620.  
**Package version:** 0.2.0.  
**Scope:** offline binary-event forecast verification only; no forecasts, warnings or actions are generated.

## 1. What changed from v0.1

v0.1 reports Brier score, logarithmic loss, Brier skill against a paired reference, calibration-in-the-large, fixed-bin reliability/ECE, mean forecast entropy, and per-hazard/region/lead breakdowns.

v0.2 adds two new research layers:

1. **Exact Murphy Brier decomposition by distinct forecast probability**: reliability, resolution, uncertainty and a numerical reconstruction residual.
2. **Chronological drift diagnostics**: non-overlapping rolling windows and a one-sided CUSUM of observed-minus-forecast residuals, with frozen parameters and no operational action.

The distinction matters: calibration quality can improve without increasing the intrinsic ability to distinguish higher-risk from lower-risk events. A sharp forecast can still be wrong.

## 2. Brier decomposition

For binary outcomes and issued probability p, the Brier score is decomposed into:

BS = REL - RES + UNC

where:
- REL (reliability) measures the squared difference between each probability category's mean forecast and observed event frequency;
- RES (resolution) measures how much each category's observed event frequency differs from the overall sample event frequency;
- UNC (uncertainty) is the sample climatological variance: base_rate × (1 - base_rate).

The evaluator groups *exactly equal issued probabilities* for this decomposition. It does not reuse the four fixed reliability-plot bins, because grouping different probabilities into the same broad bin would make this an approximate coarse-bin decomposition. The fixed four-bin ECE/reliability output remains a separate descriptive diagnostic.

The reconstruction residual is:

residual = BS - (REL - RES + UNC)

It should be within numerical tolerance of zero for each reported group. The test suite verifies this across every model/hazard/region/lead/partition group in the frozen scoring fixture.

This is the finite-sample Murphy decomposition for the empirical forecast-probability categories. It is not a causal attribution or a complete decomposition of information/noise in the underlying physical forecasting process.

## 3. Sequential calibration drift

The added drift fixture has three independent, 30-event synthetic chronological profiles:

- stable control: a calibrated 20% event frequency and 20% forecast probability;
- gradual calibration drift: forecast probability remains at 10% while the event frequency rises across consecutive blocks;
- abrupt calibration shift: forecast probability remains at 10% while outcomes switch from no events to sustained events.

Every record has a strictly increasing verification timestamp and a unique event ID. The parser rejects missing timestamps, non-monotonic time, skipped/reordered sequence numbers, repeated event IDs, mixed metadata within a profile and non-evaluation records.

### Rolling diagnostics

The profile is reported in non-overlapping windows of five observations. Each window includes:
- start and end sequence;
- sample count;
- Brier score;
- observed event frequency;
- calibration-in-the-large (mean forecast probability minus observed frequency).

These windows preserve chronology; they are not shuffled.

### CUSUM diagnostic

A one-sided CUSUM accumulates sustained positive forecast residuals (observed event minus forecast probability):

S_t = max(0, S_(t-1) + (y_t - p_t) - k)

Frozen v0.2 parameters:
- allowance k = 0.1;
- threshold h = 3.0;
- minimum observations before a flag = 5;
- reset to zero when the running statistic would become negative.

The synthetic oracle expects:
- stable control: no drift flag;
- gradual calibration-drift profile: first flag at sequence 28;
- abrupt calibration-shift profile: first flag at sequence 19.

These settings are fixed for the regression fixture. They are not fitted from the evaluation data and are not an alert threshold, a forecast invalidation rule, or an operational trigger. The flag means only that this synthetic residual sequence crossed a declared diagnostic boundary.

## 4. Synthetic regression values

The v0.1 65-row scoring fixture and 90-row drift fixture remain separate:
- scoring fixture: paired candidate/climatology/lead-reference results with calibration and evaluation partitions;
- drift fixture: three evaluation-only chronological candidate series.

The previously documented candidate aggregate improvement over climatology and adverse heat/72-hour subgroup result remain unchanged. The exact Brier decomposition should reconstruct the Brier score for every scored group. The CUSUM profile oracle distinguishes stable, gradual and abrupt synthetic patterns.

No claim is made about real weather models, real forecast calibration, operational drift detection accuracy, or warning performance. The fixture is a software regression suite, not representative weather data.

## 5. Proper scores and calibration remain separate

The package continues to report:
- Brier score;
- logarithmic loss with probability floor 1e-15;
- Brier skill against a paired reference forecast;
- calibration-in-the-large;
- fixed-bin ECE and reliability-bin counts;
- mean Bernoulli entropy as a sharpness descriptor;
- Brier reliability, resolution and uncertainty decomposition.

A model with higher sharpness is not automatically better. A low Brier score can coexist with poor calibration in a subgroup, and calibration can be adjusted without adding predictive resolution. All subgroup and baseline metrics remain visible.

## 6. Reproduction

From repository root:

    cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-forecast-calibration/Cargo.toml
    cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-forecast-calibration/Cargo.toml

The executable prints:
- data-quality counters and scoring oracle;
- overall and subgroup score/decomposition rows;
- rolling drift windows and CUSUM summary by synthetic profile;
- drift parameters and the drift-fixture oracle.

The v0.1 scoring fixture remains pinned by its existing FNV-1a regression hash. The drift fixture is pinned by exact row count, profile count, ordered sequences, strict timestamps, frozen rolling outputs and expected first-crossing sequences.

## 7. Industry/SOTA comparison protocol

Use a compatible event set and independently sourced observations. Compare:
- climatology;
- persistence or lead-conditioned reference;
- the candidate system;
- post-processed/calibrated candidate, if used, as a separate system version.

Report proper scores, Murphy decomposition, reliability diagrams, sharpness, hazard/horizon/region strata and chronological drift. Use held-out or rolling-origin partitions. Keep calibration data separate from evaluation; never let evaluation outcomes tune calibration or drift parameters. For uncertainty intervals, use paired resampling at the event or event-cluster level, not independent resampling of correlated forecast rows.

Forecast quality, calibration, resolution, uncertainty, coverage, abstention, latency and compute cost remain distinct. The two-case heat/72-hour subgroup in v0.1 is a synthetic negative control, not a statistically powered comparison.

## 8. Sources and technical context

- ECMWF probabilistic forecast verification guide: https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData
- Bonavita & Geer, information/noise diagnostics (2025/2026): https://rmets.onlinelibrary.wiley.com/doi/10.1002/qj.70109
- WMO, Guidelines on the Verification of Hydrological Forecasts (2025): https://primarynewssource.org/wp-content/uploads/WMO-1364-2025_en.pdf
- Waghmare & Ziegel (2026), proper scoring rules review: https://www.annualreviews.org/content/journals/10.1146/annurev-statistics-042424-050626
- NOAA, NDFD verification score definitions: https://vlab.noaa.gov/documents/10440810/0/NDFD%2BVerification%2BScore%2BDefinitions%2BV2.0.pdf/f7b15dba-7d62-c781-aaa0-0d22dd674840

## 9. Non-claims and safety boundary

This package does not generate forecasts, set hazard thresholds, issue public warnings, invalidate live forecasts, monitor current hazards, or authorize action. It uses synthetic binary outcomes and probabilities only. Real-world forecast evaluation requires competent forecasters, authoritative observations, defensible event definitions, representative samples, appropriate uncertainty intervals and local/national governance.
