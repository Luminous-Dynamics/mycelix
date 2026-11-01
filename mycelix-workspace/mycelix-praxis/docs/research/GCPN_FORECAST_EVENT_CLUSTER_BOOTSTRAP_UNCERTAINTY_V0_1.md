# GCPN Paired Event-Cluster Forecast Uncertainty Benchmark v0.1

**Status:** deterministic synthetic statistical diagnostic; non-operational.  
**Parent:** issue #5637.  
**Package:** benchmarks/gcpn-forecast-uncertainty.  
**Dependency:** local gcpn-forecast-calibration v0.2.0.  
**Source fixture:** 65 frozen forecast records; 12 complete paired evaluation event IDs.

## 1. Why this layer is necessary

The forecast verifier already reports Brier score, log loss, Brier skill, reliability, resolution, uncertainty, calibration, sharpness, subgroup metrics and chronological drift. Those point diagnostics do not by themselves quantify sampling uncertainty in differences between systems.

ECMWF's probabilistic forecast guide explicitly notes that probability forecasts require samples for verification; its Brier framework separates reliability, resolution and outcome uncertainty. ECMWF's newer information/noise research also emphasizes separating intrinsic predictive information from calibration or post-processing effects. This benchmark adds paired uncertainty to the current score comparisons; it does not replace proper scores or Murphy decomposition.

## 2. Paired event-cluster resampling

The v0.1 synthetic fixture has forecast rows for three paired model series:
- candidate;
- climatology;
- lead_reference.

For each accepted evaluation event, the evaluator requires exactly one accepted forecast from every model and a common outcome/metadata record. It then stores the three Brier losses together in one EventScores record.

Each bootstrap replicate draws **12 complete event records with replacement**. Candidate and baseline scores always travel together. It never independently resamples forecast rows, models or outcomes.

The evaluated comparison is the mean paired Brier-loss difference:

candidate minus baseline

A negative difference means the candidate's Brier loss is lower on this synthetic fixture; a positive difference means it is higher.

## 3. Eligibility and exclusions

The evaluator consumes the v0.2 forecast parser and only includes records that are:
- in the held-out evaluation partition;
- valid and issued before the event window begins;
- paired with an independently verified binary outcome;
- finite, in-range forecast and reference probabilities;
- not part of a partition-leaked event;
- not part of an event with metadata disagreement;
- complete across candidate, climatology and lead-reference models.

Exact duplicate rows are deduplicated before resampling. Conflicting duplicate model records exclude the entire event. Missing or invalid records cannot silently become zero-loss observations.

This yields 12 complete canonical event records. The fixture also contains deliberate data-quality and leakage controls. Their exclusion is part of the regression oracle, not evidence about any real forecasting system.

## 4. Reproducibility contract

- Algorithm: paired percentile bootstrap.
- Resampling unit: event_id in this fixture only.
- Replicates: 20,000.
- Seed: 0x6a09e667f3bcc909.
- PRNG: explicit deterministic XorShift64* implementation.
- Percentile interpolation: linear interpolation at empirical 2.5th and 97.5th percentiles.
- Source fixture: 65 rows, FNV-1a 0x81af0b6779196770.

No p-value is emitted. The percentage of bootstrap replicates with a positive or negative result is not described as a probability that one model is superior. Both intervals are read against zero as descriptive uncertainty intervals under the stated resampling assumption.

## 5. Synthetic results

| Paired comparison | Mean candidate-minus-baseline Brier | 95% percentile interval |
|---|---:|---:|
| Candidate − climatology | −0.078317 | [−0.240000, 0.164208] |
| Candidate − lead reference | +0.131683 | [−0.030000, 0.374208] |

Both intervals include zero. Thus:
- the candidate's point estimate is lower than climatology in this fixture, but the small-sample interval does not establish superiority;
- the candidate's point estimate is worse than the lead-conditioned reference, but this interval also includes zero;
- the two extreme synthetic heat/72-hour cases contribute to the wide uncertainty and must not be hidden by aggregate metrics.

The numerical results are regression oracles on invented test data. They are not real-world forecast estimates, confidence claims about a weather service, or SOTA comparisons.

## 6. Critical dependence caveat

The source fixture carries event_id, not an independently validated dependence_cluster_id. Treating each event ID as the resampling unit is therefore a **synthetic exercise assumption**, not proof that these are independent meteorological events.

Actual forecasts may be dependent across locations, time steps, lead times, hazards and multiple observations from the same storm or synoptic system. A real benchmark must obtain a defensible dependence-cluster identifier from its event-definition/observation protocol and resample that unit. It must not assume every station-time row or event label is independent.

This limitation is particularly important because bootstrap coverage depends on whether the resampling unit preserves the material dependence structure. The present 12-cluster exercise is too small to validate interval coverage.

## 7. Proposed qualification programme

Before calling an interval method production-qualified, add a separate simulation suite with known data-generating processes:

1. Null skill difference, candidate better, candidate worse, heavy-tailed and rare-event profiles.
2. Independent events, within-storm spatial dependence, temporal autocorrelation, overlapping forecast windows and hierarchical region/storm clusters.
3. Coverage evaluation for nominal 90%, 95% and 99% intervals.
4. Paired interval width, bias, Monte Carlo error and finite-sample sensitivity.
5. Compare percentile, basic, studentized and BCa intervals where their assumptions and sample sizes permit.
6. Vary the number of independent dependence clusters (e.g. 12, 25, 50, 100, 250, 500) and report the point at which inference becomes stable.
7. Use rolling-origin partitions and independently sourced outcomes for any real forecast comparison.

A proposed qualification target is empirical 95% interval coverage between 93% and 97% under each sufficiently replicated, predeclared simulation regime, with Monte Carlo uncertainty reported. This is a future research target—not a v0.1 result, guarantee, or external standard.

## 8. Industry/SOTA comparison

This v0.1 work does **not** beat operational numerical-weather-prediction centres or establish superior forecasts. It strengthens the audit protocol that should be applied to any candidate:
- proper scores and baselines remain paired;
- calibration and intrinsic resolution remain separate;
- subgroup failure remains visible;
- time drift remains a distinct diagnostic;
- sampling uncertainty is exposed;
- model claims are withheld when the independent cluster sample is too small or the intervals cross zero.

ECMWF's published information/noise work motivates separating the forecast's intrinsic signal from noise and calibration effects. Our present addition addresses a different, complementary question: how uncertain the paired score difference is when the resampling unit is made explicit.

## 9. Evidence ladder and non-claims

| Evidence level | v0.1 status |
|---|---|
| Research contract | authored |
| Rust implementation | authored |
| Source fixture / parser | inherited from local forecast package v0.2 |
| Static event-cluster audit | independently recomputed |
| Bootstrap numerical oracle | independently recomputed with same seed, replicate count and percentile convention |
| Rust compilation and unit tests | outstanding; no cargo/rustc in authoring environment |
| Real dependence-cluster validation | not performed |
| Real forecast evaluation | not performed |
| Operational decision qualification | not performed |

No live forecasts, hazard monitoring, alert thresholds, model auto-selection, public-warning generation or action authorization are present.

## 10. Sources

- ECMWF, Forecast User Guide: Statistical Concepts — Probabilistic Data (updated May 2026): https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData
- Bonavita & Geer, Forecast verification using Information and Noise, ECMWF Technical Memorandum 927 (May 2025): https://www.ecmwf.int/en/elibrary/81662-forecast-verification-using-information-and-noise
- Bonavita & Geer, Forecast verification using information and noise, Quarterly Journal of the Royal Meteorological Society (2026): https://rmets.onlinelibrary.wiley.com/doi/10.1002/qj.70109
- Waghmare & Ziegel, Proper Scoring Rules for Estimation and Forecast Evaluation (2026): https://www.annualreviews.org/content/journals/10.1146/annurev-statistics-042424-050626
