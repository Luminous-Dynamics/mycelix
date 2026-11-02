# GCPN common-source paired forecast comparison v0.1

## Research objective

Establish whether forecast outputs can be compared on a common frozen set of cases, reference data and scoring contract before accepting aggregate scores.

GCPN does not replace WeatherBench 2, WeatherBench-X, ECMWF forecast verification, operational NWP or national meteorological services. This work is an evidence-contract and denominator layer.

## Paradigm shift

Move from accepting published aggregate scores to validating **the comparison itself**:

- the same predictand, unit and temporal support;
- the same spatial grid, mask, coordinate convention and area weights;
- the same climatology and evaluation window;
- the same initialization/valid-time/lead case frame;
- one explicit reference source/release/artifact/observation lineage;
- the same as-of cutoff and availability policy;
- the same metric definition, scoring engine/configuration and deterministic/ensemble input requirements;
- a frozen training cutoff and post-processing provenance for intrinsic-skill claims.

Candidate, reference and verification availability are separate per-case facts. Unavailable forecasts and verification records stay typed and visible. Excluded cases stay visible but do not enter the eligible denominator.

## Evidence-backed motivation

WeatherBench 2 exposes multiple metrics and baselines and publishes scorecards for data-driven and physics-based forecasts. Its official scorecards explicitly note that certain operational models are evaluated against IFS analysis while other models are evaluated against ERA5. Those results remain useful in their stated context; however, direct cross-system rankings require the verification-source distinction to remain visible.

A common-source comparison cohort should therefore be reported separately from any model-native verification cohort. The common-source cohort improves strict comparability; it does not make one reference dataset universally correct or remove reference-observation uncertainty.

ECMWF's 2026 forecast information/noise work further motivates preserving raw bias/RMSE and intrinsic information/noise diagnostics separately rather than treating one lower aggregate error as a complete explanation of skill.

## Stage A frozen contract

1. Dataset release: ID/version, locator/citation, retrieval timestamp, artifact digest, license and terms URI, declared redistribution policy.
2. Verification: exact source/release/artifact digest, observation-lineage ID, processing epoch and as-of cutoff.
3. Spatial support: support ID, grid/mask digests, coordinate convention and area-weight scheme.
4. Comparison: comparison ID, predictand, units, temporal/spatial support, climatology, evaluation interval, as-of cutoff and claim type.
5. Model outputs: model ID/version, archive release/digest, predictand/unit, temporal/spatial support, grid/mask, climatology, deterministic/ensemble kind/member count, training cutoff and post-processing provenance.
6. Metric: definition/version, scoring engine/version, configuration digest and input requirements.
7. Matched case frame: unique case key, initialization/valid time/lead, eligibility/exclusion status, candidate/ref availability, common-verification availability and typed missing reasons.
8. Report: eligible/excluded denominators, candidate/reference marginal availability, paired forecasts, verification counts, marginally verifiable cases and paired-verifiable cases.

## Required negative controls

- verification source/release/artifact or as-of mismatch;
- grid/mask, spatial support, unit, predictand, climatology or temporal support mismatch;
- forecast initialization/valid-time/lead mismatch;
- duplicate case key and conflicting duplicate output;
- candidate/reference forecast availability on different case subsets;
- missing verifying analysis and late-retrieved verification;
- deterministic/ensemble type mismatch and member-count requirements;
- model training or calibrator fitting overlapping the evaluation period;
- post-processing using confirmatory evaluation outcomes;
- unknown redistribution terms are not treated as permission.

## Proposed targets (not standards or achieved results)

- 100% exact rejection of incompatible contracts;
- 100% accounting of the frozen eligible case frame;
- zero silent missingness coercion or case dropping;
- zero false paired comparisons;
- zero acceptance of duplicate case keys;
- zero acceptance of evaluation-outcome leakage for intrinsic-skill claims;
- byte-identical deterministic report reproduction.

## Implementation boundary

The Stage B engine validates metadata and per-case availability. It does not fetch/redistribute data, independently hash external bytes, calculate RMSE/CRPS/Brier/FSS, provide legal advice, or establish forecast superiority.

An ingestion adapter must hash exact downloaded bytes, capture immutable artifact metadata and license provenance. A scoring adapter must run the pinned metric engine over exact paired-verifiable cases and publish raw score values with uncertainty. These are separate qualification stages.

## Benchmark programme and SOTA comparison

WeatherBench 2 remains the relevant open model benchmark for global medium-range forecasting, with multi-metric scorecards and a maintained evaluation framework. GCPN should be judged on strict comparability, source/denominator integrity, missingness accounting and independent reproducibility—not on whether its metadata validator has lower RMSE.

A meaningful model comparison would report both:
- common-reference, matched-case scores over the exact common subset; and
- model-native/operational verification scores, clearly labelled as a different cohort.

Do not subtract scores across incomparable references or claim that a reference change is meteorological model improvement.

## References

- WeatherBench 2 official scorecards: https://sites.research.google/gr/weatherbench/
- WeatherBench 2 paper: https://research.google/pubs/weatherbench-2-a-benchmark-for-the-next-generation-of-data-driven-global-weather-models/
- ECMWF (25 March 2026), Separating the signal from the noise: https://www.ecmwf.int/en/about/media-centre/science-blog/2026/separating-signal-noise
- SADC 2026 MHEWS capacity-strengthening scope: https://www.sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and

## Non-claims

No claim of score superiority, operational readiness, legal permission, national validation or disaster-impact reduction is made.
