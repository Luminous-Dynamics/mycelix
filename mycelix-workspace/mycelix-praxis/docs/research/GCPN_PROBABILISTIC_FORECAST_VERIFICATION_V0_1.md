# GCPN Probabilistic Forecast Calibration and Verification Benchmark v0.1

**Status:** deterministic synthetic forecast-verification benchmark; non-operational.  
**Parent:** issue #5615.  
**Package:** `benchmarks/gcpn-forecast-calibration`.  
**Models:** climatology reference, simple lead-conditioned reference, candidate.

## 1. Research objective

The SADC MHEWS capacity programme explicitly includes responsible AI/ML, hazard monitoring, risk analysis, impact forecasting and decision support. A useful evidence layer must test whether probabilistic forecasts are calibrated and skilful before a competent forecaster or authority uses them.

This package evaluates forecast records. It does not generate forecasts, set alert thresholds, issue warnings, or authorize action.

## 2. Why a single accuracy number is insufficient

A probabilistic forecast needs several distinct evaluation attributes:
- **proper score / accuracy:** how close the stated probabilities are to observed outcomes;
- **calibration / reliability:** whether events forecast at probability (p) occur at approximately frequency (p) over a sufficiently large sample;
- **resolution / discrimination:** whether forecasts distinguish higher-risk from lower-risk cases;
- **sharpness:** how concentrated the issued probabilities are, without confusing confidence with correctness;
- **skill relative to a suitable baseline:** whether the model adds value over a declared reference;
- **subgroup robustness:** whether average results hide failures by hazard, lead time, region or population of interest.

ECMWF's forecast-verification guidance describes the Brier score and reliability diagrams and cautions that calibration and sharpness are different properties. WMO's 2025 hydrological verification guidance further separates reliability, resolution, discrimination and sharpness in probabilistic forecast evaluation. NOAA's score definitions also warn that Brier scores can vary with event base rate, so comparisons need matched samples or a suitable reference forecast.

References:
- ECMWF probabilistic verification and reliability diagrams: https://confluence.ecmwf.int/spaces/FUG/pages/673551875/Section%2B12.B%2BStatistical%2BConcepts%2B-%2BProbabilistic%2BData
- WMO, Guidelines on the Verification of Hydrological Forecasts (2025): https://primarynewssource.org/wp-content/uploads/WMO-1364-2025_en.pdf
- NOAA, NDFD Verification Score Definitions: https://vlab.noaa.gov/documents/10440810/0/NDFD%2BVerification%2BScore%2BDefinitions%2BV2.0.pdf/f7b15dba-7d62-c781-aaa0-0d22dd674840
- National Academies, probabilistic forecast attributes: https://www.nationalacademies.org/read/11699/chapter/5

## 3. Frozen synthetic experiment

The fixture contains 65 forecast records associated with 26 event IDs:
- 18 base events across flood, cyclone and extreme-heat hazard labels;
- 24-hour and 72-hour lead times;
- calibration and held-out evaluation partitions;
- synthetic calibration/evaluation regions;
- three forecast model IDs: `climatology`, `lead_reference`, and `candidate`;
- explicit outcome provenance and event-window timestamps;
- deliberate negative controls for data quality and leakage.

The simple `lead_reference` comparator is a synthetic reference that uses only the lead-time bucket in this constructed dataset. It is not a deployed persistence system, a meteorological model, or a claim about real forecast skill. Outcomes were deliberately constructed to make scoring arithmetic transparent; this fixture is a software-verification dataset, not a scientifically representative weather sample.

The calibration partition contains six base events per model. The held-out evaluation partition contains twelve base events per model. The evaluator reports both partitions separately and never pools them into one score.

## 4. Scoring definitions

For binary outcome (y_i in {0,1}) and forecast probability (p_i in [0,1]):

**Brier score** (lower is better):

[
BS = rac{1}{N}sum_{i=1}^{N}(p_i-y_i)^2
]

**Logarithmic loss** (lower is better):

[
LL = -rac{1}{N}sum_{i=1}^{N}
left[y_iln(p_i)+(1-y_i)ln(1-p_i)ight]
]

For numerical stability, the log-loss implementation clips probabilities to ([10^{-15}, 1-10^{-15}]). Brier score uses the original valid probability without clipping.

**Brier skill score** against the paired reference probability (p_i^{ref}):

[
BSS = 1 - rac{BS_{model}}{BS_{reference}}
]

A zero reference Brier score yields no defined BSS; the implementation returns `None` instead of inventing a value.

**Calibration-in-the-large:**

[
CIL = ar p - ar y
]

Positive values indicate average overforecasting; negative values indicate average underforecasting.

**Expected calibration error (ECE):** fixed four-bin weighted absolute difference between mean forecast probability and observed event frequency in each bin. It is a descriptive, bin-sensitive diagnostic, not a proper scoring rule.

**Sharpness descriptor:** mean Bernoulli entropy of forecast probabilities. It describes probability concentration; it must never be interpreted as accuracy or reliability on its own.

Reliability bins use ([0,.25)), ([.25,.50)), ([.50,.75)), and ([.75,1]), and always report sample counts.

## 5. Frozen negative controls and data-quality handling

The fixture includes:
- one duplicate event/model/lead record;
- one forecast issued after the event window has started;
- one unverified outcome;
- one missing outcome;
- one invalid probability above 1;
- one out-of-domain record;
- one abstention;
- one event ID leaking across calibration and evaluation partitions;
- one event ID with inconsistent hazard metadata between model records.

The expected evaluator totals:
- 65 raw records;
- 54 scored records;
- one duplicate;
- one late forecast;
- one unverified outcome record;
- one missing outcome record;
- one invalid probability;
- one out-of-domain record;
- one abstention;
- one calibration/evaluation partition leak;
- one event metadata mismatch.

Records affected by event-level metadata mismatch or partition leakage are excluded from scoring. Abstention and out-of-domain records remain visible as coverage/status counts, not silently discarded.

## 6. Synthetic regression results

On the 12 held-out evaluation events per model:

| Model | Brier score | Brier skill vs climatology |
|---|---:|---:|
| Climatology | 0.250000 | 0.000000 |
| Lead-conditioned reference | 0.040000 | 0.840000 |
| Candidate | 0.171683 | 0.313267 |

This shows that the candidate improves over climatology in aggregate but is worse than the simple lead-conditioned reference on this deliberately constructed sample.

The candidate also has a severe hazard/horizon failure:

| Candidate subgroup | Samples | Brier | Brier skill vs climatology |
|---|---:|---:|---:|
| Heat, 72-hour lead, evaluation partition | 2 | 0.980100 | -2.920400 |

The overall candidate Brier score alone would not communicate the seriousness of that subgroup failure. This is the key negative-control lesson of the fixture.

The candidate calibration partition produces a Brier score of 0.01 on six synthetic cases; it is reported separately from the held-out score. No fitted calibrator is applied in v0.1.

## 7. Industry and SOTA comparison protocol

This package implements standard forecast-verification concepts; it does not claim a new scoring rule or superiority over operational weather services.

A fair next-stage comparison should:
1. use identical event IDs, outcome definitions, observation sources, issue cutoffs and lead-time windows across models;
2. compare climatology and suitable persistence/lead-conditioned reference baselines alongside candidate models;
3. use event-held-out partitions or rolling-origin tests—never random splits that leak an event across train/calibration/evaluation;
4. report Brier and log loss, reliability diagrams, calibration-in-the-large, resolution/discrimination diagnostics, sharpness and baseline skill;
5. stratify by hazard class, forecast horizon, region and relevant data-quality conditions;
6. use paired bootstrap or another justified interval method at the event or event-cluster level, preserving dependence between paired models;
7. disclose missing observations, abstentions, out-of-domain cases, late forecasts and invalid probabilities;
8. report compute cost, forecast latency and data requirements separately from forecast quality.

The current 2-case heat/72-hour subgroup is intentionally too small for statistical conclusions. It is a deterministic code-level regression test. No operational model ranking should be attempted from this fixture.

## 8. Reproduction

From repository root:

```bash
cargo test --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-forecast-calibration/Cargo.toml
cargo run --manifest-path mycelix-workspace/mycelix-praxis/benchmarks/gcpn-forecast-calibration/Cargo.toml
```

The frozen fixture is pinned with an FNV-1a regression fingerprint. The fingerprint is a file-change detector, not a cryptographic signature or data provenance system.

## 9. Future research

Next increments should add:
- rolling-origin forecast evaluation with event-cluster bootstrap intervals;
- multicategory Brier / ranked probability score;
- continuous-distribution CRPS for precipitation, streamflow and temperature distributions;
- reliability decomposition into calibration, resolution and uncertainty;
- cost-loss / decision-value analysis with explicit stakeholder-supplied cost profiles, without the benchmark choosing alert thresholds;
- sequential calibration drift and change-point detection;
- forecast lineage, version, post-processing and model update tracking;
- comparison with public, appropriately licensed historical forecast archives and independently sourced observations.

## 10. Safety and non-claims

This is a synthetic verification tool, not a hazard prediction system. It does not ingest real-time forecast feeds, define operational thresholds, generate alerts or choose actions. Real-world evaluation requires competent forecasters, authoritative observations, careful event definitions, representative samples, appropriate uncertainty intervals and national/local governance.
