# GCPN paired probabilistic FSS adapter v0.1

## Objective

Connect the qualified common-source case-frame/availability engine to pFSS scoring without allowing the scoring layer to rebuild, shrink, or silently alter the paired denominator.

The adapter is an evidence-constrained score runner. It is not a forecast engine, data archive, warning authority or operational deployment.

## Paradigm shift: validate the score cohort before the score

A candidate and reference must be compared only on the exact intersection of:
- frozen eligible case;
- candidate forecast artifact available by the as-of cutoff;
- reference forecast artifact available by the same cutoff;
- common verifying observation available by the same cutoff;
- valid matching initialization, valid time and lead;
- identical grid, mask, weights, threshold, neighbourhood and missingness policy.

The scoring layer may never derive an alternative case set from whichever arrays happen to be present. The upstream common-source report is the denominator authority for this adapter.

## Input contracts

### Common-source frame
Consume a validated ComparisonManifest and its ComparisonReport from gcpn-common-source-comparison. Preserve:
- comparison ID and frozen case-frame ID;
- verification source, release/version, artifact digest, observation lineage and processing epoch;
- grid/mask/spatial-support and climatology identity;
- evaluation window and as-of cutoff;
- every eligible and excluded case;
- candidate/reference forecast availability and typed missing reasons;
- verification availability and typed missing reasons.

Do not treat paired_forecasts as the score denominator. The score denominator is the narrower paired_verifiable case subset.

### Per-case scored artifacts
For each paired-verifiable case, supply:
- exact case key (ID, initialization, valid time, lead);
- candidate forecast artifact digest, initialized-at time, recorded-at time and event block ID;
- reference forecast artifact digest, initialized-at time, recorded-at time and event block ID;
- common verification artifact digest and recorded-at time;
- candidate and reference arrays;
- observation arrays, valid mask and area weights;
- support identity and dimensions.

### Shared score configuration
Candidate and reference use one immutable FssConfig, including:
- threshold and inequality convention;
- unit;
- odd neighbourhood width;
- boundary policy;
- missing-data policy.

Both forecasts must be evaluated against the exact same observation, mask, grid geometry and weights. The adapter rejects differences instead of silently aligning or regridding them. Any regridding must be a separately versioned preprocessing step before this adapter.

## Denominator accounting

The report must expose and preserve:
- eligible cases;
- excluded cases;
- candidate-only forecast outputs;
- reference-only forecast outputs;
- both forecasts missing;
- paired forecasts without verification;
- candidate/reference/verification missing reasons;
- paired-verifiable cases;
- paired cases actually scored;
- every expected paired-verifiable case missing a grid artifact (fatal adapter error);
- every unmanifested grid artifact (fatal adapter error).

Required invariant:

scored_case_ids == common_source.paired_verifiable_case_ids

and their order must be canonicalized only for deterministic serialization, never for inclusion/exclusion.

## Paired uncertainty

Use paired_block_bootstrap, not two separate block_bootstrap runs.

- Candidate and reference share identical event-block IDs for every case.
- Every replicate samples one common set of event blocks and applies it to both systems.
- The reported estimand is candidate pFSS minus reference pFSS on the same common cases.
- The report names FBS/WFBS component pooling as the score aggregation policy.
- Degenerate resamples remain counted; they are not coerced to zero or removed without accounting.
- Grid cells are not bootstrap resampling units.
- A difference in two independent marginal intervals is not a paired confidence interval.

## Required negative controls

- candidate artifact digest mismatch against common-source manifest;
- reference artifact digest mismatch;
- verification artifact digest mismatch;
- forecast initialized-at, valid-time or lead mismatch;
- late candidate, reference or observation artifact;
- changed observation array for candidate vs reference;
- changed dimensions, grid digest, mask digest, valid-mask cells or area weights;
- threshold, operator, units, neighbourhood or boundary-policy mismatch;
- candidate-only, reference-only and both-missing rows;
- forecast pair present but verification missing;
- excluded rows must never enter score denominator;
- duplicated or unmanifested case artifacts;
- missing grid for a paired-verifiable manifest case;
- two independent bootstrap seeds must not be used for the paired comparison;
- event-block label differs between candidate and reference;
- same seed reproduces exactly the same paired delta interval;
- as-of cutoff and upstream common-reference version mismatch.

## Benchmark targets (proposed, not achieved)

- 100% exact match between scored IDs and common-source paired-verifiable IDs.
- Zero false paired comparisons.
- Zero silent denominator changes.
- Zero observation, mask, weight, lead or cutoff drift between candidate and reference.
- 100% exact digest binding for candidate, reference and verification artifacts.
- 100% use of identical event-block draws in paired bootstrap.
- Byte-identical repeated output for identical manifests, arrays, configurations, seed and source versions.
- Every unscored case accounted for in a typed availability category.
- Report scale/threshold/lead/region strata separately; no single compensatory score.

## Implementation stages

A. Specification and synthetic fixtures (this document).

B. Standalone Rust adapter with local path dependencies on:
- gcpn-common-source-comparison;
- gcpn-probabilistic-fss.

C. Exact-head tests:
- common-source manifest and report validation;
- paired artifact and array identity;
- paired denominator preservation;
- paired event-block bootstrap;
- strict Clippy and deterministic output.

D. Permissioned WeatherBench 2 replay adapter:
- immutable retrieval, actual byte hashing, source and licensing metadata;
- common-reference matched case cohort;
- one common scoring engine for candidate/reference;
- independent reproduction.

No benchmark data or large artifacts are copied into the public repository.

## Industry comparison

WeatherBench 2 remains the forecasting benchmark for model skill. GCPN's contribution is a strict evidence boundary around which cases and verifying source a comparison actually uses. It should publish:
1. a model-native verification cohort, clearly labelled;
2. the common-reference, paired-verifiable cohort;
3. exact case-count differences and typed missing reasons;
4. paired pFSS and event-block uncertainty;
5. CRPS/Brier/reliability/calibration scores separately.

A lower or higher pFSS in one slice does not alone establish calibration, user decision value or disaster impact.

## Non-claims

Passing synthetic tests is not evidence of operational weather superiority, disaster-impact reduction or national MHEWS readiness. Common-reference comparisons do not make the verifying analysis universally true; observation uncertainty remains an independent qualification.