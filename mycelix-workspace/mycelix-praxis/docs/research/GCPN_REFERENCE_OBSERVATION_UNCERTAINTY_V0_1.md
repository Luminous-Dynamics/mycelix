# GCPN Reference-Observation Uncertainty and Predictand Representativeness v0.1

**Status:** Stage A research specification; implementation and confirmatory experiments outstanding.  
**Parent issue:** #5670.  
**Scope:** offline synthetic qualification of measurement and representativeness uncertainty in forecast verification.

## 1. Why another verification layer is needed

A verified outcome record is not necessarily an error-free truth value. It may be noisy, biased, censored, spatially or temporally unrepresentative, or measured on a different support from the forecast predictand.

Ferro (2017) showed that proper scoring rules applied to observations containing error can assess forecasts of the observation process rather than forecasts of the true predictand. ECMWF Technical Memorandum 865 (2020) quantified large effects of spatial representativeness error on ensemble reliability and skill. Jolliffe (2017) demonstrated how observation error changes the probability forecast that minimizes expected Brier score.

This track is deliberately distinct from #5668:
- **Ascertainment:** whether an eligible outcome was observed and available by an as-of cutoff.
- **Reference uncertainty:** conditional on an observation existing, how it relates to the latent target quantity.
- **Sampling uncertainty:** uncertainty from finite independent systems/cases, handled by the forecast interval qualification track.

The three must remain separately inspectable.

## 2. Paradigm shift: declare the predictand and observation operator

Every verification comparison must freeze:
1. predictand identity and operational definition;
2. unit and scale;
3. temporal support and aggregation interval;
4. spatial support (point, cell, area, basin, region or other declared object);
5. threshold/event semantics;
6. the latent target variable or event;
7. observation source, version, calibration epoch and observation-operator lineage;
8. the measurement and representativeness error model, or explicit bounded-error assumptions.

A missing support definition is not a minor metadata issue: it can make two forecasts and observations incomparable.

## 3. Error taxonomy

Keep separate:
- **Random measurement error:** stochastic instrument error conditional on true local state.
- **Systematic bias:** calibration offset, scale distortion or source-specific bias.
- **Heteroscedastic error:** error magnitude changes with state, location, season, hazard intensity or sensor conditions.
- **Representativeness error:** a point measurement is used for an area mean or a local measurement for a different spatial support.
- **Temporal support error:** instantaneous measurement compared with an interval mean/maximum/accumulation.
- **Censoring/quantization:** instrument floors, ceilings, rounding and detection thresholds.
- **Event-label error:** sensitivity/specificity or classification confusion when a continuous latent state is turned into an event.
- **Correlated/common-mode error:** shared sensor calibration, telemetry, power, processing, retrieval model or reference dataset lineage.
- **Reference-network drift:** observation quality or composition changes over the comparison period.

These are not interchangeable noise terms. The evaluation contract must name which are represented and which remain unknown.

## 4. Estimands

Primary estimand: expected forecast score against the declared latent target under the declared target population and support.

For a known-truth DGP, compute:
- oracle score against latent truth;
- naive score against the measured observation;
- error-aware score estimate under a specified observation model;
- bias and RMSE of each estimator relative to the latent-truth oracle;
- coverage of uncertainty intervals for the latent-truth score;
- forecast ranking reversals between latent-truth and observation-space comparisons.

Do not call the score against an uncertain measurement a latent-truth skill score. It is an observation-space score unless a valid error-aware method establishes the target.

## 5. Candidate method families

Stage A should compare, not prematurely canonize:
- naive point-observation scoring as a negative control;
- observation-likelihood / latent-state methods with known measurement models;
- probabilistic representativeness-error models;
- error-aware categorical verification with known sensitivity/specificity matrices;
- bounded-error and partial-identification approaches when a probability law is not supported;
- robust sensitivity envelopes over declared error-model parameters.

The method must state its assumptions and target. It must not simply substitute the posterior mean as truth or add observation variance to forecast variance without a derivation appropriate to the score and estimand.

## 6. Known-truth DGP families

1. No observation error (oracle control).
2. Additive unbiased Gaussian measurement error.
3. Heavy-tailed, skewed and censored measurement error.
4. Systematic bias and calibration drift.
5. Heteroscedastic noise across event severity and access strata.
6. Spatial point-to-area representativeness error with known covariance.
7. Temporal instantaneous-to-interval aggregation mismatch.
8. Threshold/event misclassification with known sensitivity and specificity.
9. Correlated source errors from shared calibration, power, telemetry, retrieval model or reference dataset.
10. Time-varying observation network composition and quality.
11. Correctly specified versus misspecified observation-error models.
12. Mixed measurement error and outcome ascertainment/missingness, with explicit composition of #5668 selection mechanisms.
13. Rare/extreme events where the error model is weakest and tail behavior matters.

Each synthetic DGP must expose a separately coded latent-truth oracle. Never use the same observation sample as both estimator input and ground truth.

## 7. Negative controls and invariants

- A perfect forecast is perfect against latent truth in the no-error control.
- No result is labelled latent-truth skill when it only scores against noisy observations.
- Common-mode sensor lineage cannot be represented as independent corroboration.
- Support mismatch is rejected or explicitly mapped through a declared observation operator.
- Known calibration bias is not reclassified as zero-mean random noise.
- Heteroscedasticity is not collapsed to a single variance without a declared approximation.
- Missing outcomes remain governed by #5668 and are not treated as measurement noise.
- Source-quality changes over time are reported separately from forecast changes.
- Misspecified/unknown error models trigger sensitivity analysis or abstention, not false precision.

## 8. Proposed qualification targets (not standards or results)

| Dimension | Proposed target |
|---|---|
| Predictand contract | 100% of paired comparisons declare units, support, time window and event semantics |
| Observation lineage | 100% identify source/version/calibration epoch and observation-operator profile |
| Error semantics | Zero silent conversion of uncertain observations to exact truth |
| Known-truth oracle | Exact reproduction on deterministic fixtures |
| Error-aware estimation | Bias, RMSE and interval coverage assessed against latent truth |
| Common-mode independence | Zero false independence claims for shared fault domains |
| Support mismatch | 100% diagnose/reject undeclared point/area/time support mismatch |
| Tail behavior | 90%, 95% and 99% coverage, MCSE and tail diagnostics by DGP cell |
| Unknown error law | Explicit bounds/sensitivity/abstention; no invented distribution |
| Uncertainty separation | Measurement, representativeness, ascertainment and sampling components published separately |

These are proposed gates, not official standards or achieved results.

## 9. Relation to current practice and competitive contribution

This track complements:
- WMO/ECMWF forecast verification and reference-data quality control;
- WeatherBench 2 common-truth and scorecard discipline;
- ECMWF's published representativeness-error work;
- proper scoring-rule research under observation error;
- GCPN's existing matched-source/version, spatial score, bitemporal evidence and independent-system bootstrap tracks.

Potential contribution: reproducible experiments where the latent truth, observation operator and common-mode error mechanisms are known, allowing institutions to measure whether a verification method tracks forecast skill or merely tracks the quality of the observation network.

It is not a claim that GCPN has a better real-world observation model or better forecast skill.

## 10. Integration boundary

The evidence graph records the exact observation operator, source, version and calibration epoch. The ascertainment package describes which target cases have observations. This track qualifies the relationship between observed values and the predictand. The forecast interval package estimates sampling uncertainty at the declared independent-system level. Spatial forecast metrics evaluate joint forecast structure.

Do not collapse those concerns into a single quality number or one undifferentiated confidence interval.

## 11. Research sources

- Ferro, C. A. T. (2017), *Measuring forecast performance in the presence of observation error*, Quarterly Journal of the Royal Meteorological Society 143(708), 2665–2676. https://rmets.onlinelibrary.wiley.com/doi/pdf/10.1002/qj.3115
- Ben Bouallègue, Z. (2020), *Accounting for representativeness in the verification of ensemble forecasts*, ECMWF Technical Memorandum 865. https://www.ecmwf.int/en/elibrary/81173-accounting-representativeness-verification-ensemble-forecasts
- Jolliffe, I. T. (2017), *Probability forecasts with observation error: what should be forecast?*, Meteorological Applications 24(2), 276–278. https://rmets.onlinelibrary.wiley.com/doi/full/10.1002/met.1626
- Brown et al. (2024), *Challenges of Operational Weather Forecast Verification and Evaluation*, Bulletin of the American Meteorological Society. https://journals.ametsoc.org/view/journals/bams/105/4/BAMS-D-22-0257.1.xml
- *Explicitly Accounting for Observation Error in Categorical Verification of Forecasts* (2006), Monthly Weather Review 134(6). https://journals.ametsoc.org/view/journals/mwre/134/6/mwr3138.1.xml

## 12. Non-claims

No operational sensor calibration, universal observation-error distribution, identified latent truth for arbitrary datasets, forecast-generation capability, real forecast superiority, SOTA superiority or operational readiness is claimed.
