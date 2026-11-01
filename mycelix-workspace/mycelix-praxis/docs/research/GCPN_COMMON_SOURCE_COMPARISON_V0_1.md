# GCPN Common-Source Forecast Comparison Manifest v0.1

**Status:** Stage A specification; Stage B is a standalone Rust metadata qualification library.  
**Parent issue:** #5679.  
**Scope:** metadata/provenance and paired-frame validation for external forecast benchmark comparisons; no data fetching or forecast scoring.

## 1. Research question

Can independent evaluators compare forecast products on exactly the same cases, predictand, verification reference, grid, mask, climatology, lead time and scoring definition, while preserving missingness and provenance?

WeatherBench 2 provides data, baselines and scorecards for data-driven weather prediction. GCPN's intended contribution is to enforce comparability before accepting aggregate comparisons and to make all eligible/unavailable denominators explicit.

## 2. Paradigm shift

Move from comparing aggregate score values to qualifying the **comparison contract** first.

A pair of scores is not directly comparable just because they share a metric label. They need the same target semantics, verification reference and version, temporal/spatial support, grid/mask, climatology, initialization/valid times, lead time, eligible case frame and metric configuration.

Unavailable outputs must remain visible. The paired-case count is distinct from each model's marginal availability count.

## 3. Required immutable manifests

### 3.1 Dataset/release manifest

- dataset ID and release/version;
- source locator and DOI/citation;
- retrieval timestamp;
- artifact or manifest digest;
- license identifier;
- terms URL;
- explicit redistribution policy: allowed, prohibited or unknown.

An unknown redistribution policy is not permission. The adapter validates metadata only; it does not give legal advice.

### 3.2 Predictand and temporal contract

- predictand ID and unit;
- temporal aggregation/support ID;
- initialization time, valid time and lead duration;
- forecast issuance time;
- climatology/baseline ID;
- evaluation cutoff.

### 3.3 Spatial support contract

- grid ID and grid digest;
- cell-mask digest;
- coordinate convention;
- spatial aggregation/support ID;
- area-weight scheme and identity.

### 3.4 Verification/reference contract

- verification source ID and version;
- reference release/artifact digest;
- observation/analysis lineage ID;
- calibration or reference-processing epoch;
- as-of cutoff.

### 3.5 Forecast-model contract

- model ID and version;
- forecast archive/release ID;
- forecast artifact digest;
- deterministic or ensemble kind;
- ensemble member count where applicable;
- post-processing operation ID/version;
- post-processing fit-data digest and cutoff.

### 3.6 Scoring-engine contract

- metric name and mathematical definition/version;
- score package ID/version;
- configuration digest;
- required input contract (deterministic values, ensemble members, area weights, thresholds, etc.).

A metric name alone is insufficient to establish equivalence.

## 4. Canonical paired case frame

Each case key is derived from frozen metadata: predictand, initialization time, valid time, lead duration, region/grid, cell mask, climatology and verification contract.

The candidate and reference must both declare a record for every eligible case. Availability is one of:
- available with forecast artifact digest;
- missing with a typed reason (not produced, archive gap, invalid output, access unavailable, unknown);
- excluded by a predeclared rule and reason.

Do not silently drop missing model outputs or reference outcomes. The evaluator reports:
- eligible cases;
- candidate available;
- reference available;
- both available (paired);
- candidate-only available;
- reference-only available;
- both missing;
- excluded/invalid.

The paired score population is exactly the both-available intersection after the same common verification eligibility contract. Marginal model score summaries must remain separately labelled.

## 5. Compatibility rules

Reject direct paired-comparison qualification when any of the following differ:
- predictand or unit;
- initialization/valid time or lead duration;
- temporal or spatial support;
- grid or cell-mask digest;
- climatology/baseline;
- verification source/version/artifact;
- metric definition/configuration/score engine version;
- eligible case-frame digest.

Ensemble comparisons may have different ensemble member counts if the metric supports them, but the counts and sampling convention must be explicitly reported. If a metric's contract requires equal member count or common randomization, enforce that requirement rather than infer it.

## 6. Licensing/provenance boundary

Dataset/model license, terms URL and redistribution policy are provenance fields. The adapter:
- does not assume that public access implies permission to redistribute;
- does not download or bundle external data;
- does not decide whether a license is legally compatible;
- emits an explicit unknown/unsupported policy state for human/legal review.

## 7. Negative controls and metamorphic invariants

- Different verification release or reference digest is rejected.
- Different grid/mask or climatology is rejected.
- Different lead/initialization/valid time is rejected.
- Different metric definition/config digest is rejected.
- Candidate/reference case frame mismatch is rejected.
- A missing model output changes typed paired/marginal counts, never the case frame denominator.
- Reordering case records does not change canonical report or digest.
- Consistent opaque case-ID renaming changes the case-frame digest but not counts.
- Duplicate case IDs and conflicting rows are rejected.
- Unknown license policy never maps to allowed.
- Confirmatory post-processing leakage is rejected.

## 8. Proposed qualification targets (not standards or results)

| Dimension | Proposed target |
|---|---|
| Contract compatibility | 100% exact accept/reject reproduction on deterministic fixtures |
| Paired fairness | 100% same eligible case frame and verification lineage |
| Denominator accounting | 100% of eligible records classified in a typed availability category |
| Missingness | Zero silent drop or implicit success conversion |
| Spatial comparability | 100% identical grid/mask/support for direct paired score comparison |
| Temporal comparability | 100% same initialization, valid time and lead |
| Metric comparability | 100% same mathematical definition/configuration/engine version |
| Provenance | 100% dataset/model/reference/score-engine digest and version metadata |
| License | Zero unknown-to-allowed promotion |
| Reproducibility | Deterministic canonical manifest and digest under record reordering |

## 9. Relationship to GCPN

This is an interoperability gate for downstream evaluations:
- #5652 supplies independent-system uncertainty for paired comparisons.
- #5664 supplies spatial ensemble score primitives.
- #5667 supplies decision-value analysis.
- #5668 qualifies outcome ascertainment.
- #5670 qualifies reference-observation uncertainty.
- #5676 qualifies information/noise decomposition.
- #5672 audits interacting evidence-chain failures.

This adapter checks whether model score artifacts can be compared at all. It does not compute CRPS, Brier score, energy/variogram score, decision value, information/noise decomposition or forecast uncertainty.

## 10. Research basis

- Rasp et al. (2024), WeatherBench 2: https://agupubs.onlinelibrary.wiley.com/doi/full/10.1029/2023MS004019
- WeatherBench 2 repository and scorecards: https://github.com/google-research/weatherbench2
- ECMWF TM 927 (2025), information/noise diagnostics: https://www.ecmwf.int/en/elibrary/81662-forecast-verification-using-information-and-noise

## 11. Non-claims

No forecast superiority, data completeness, license permission, operational readiness or full WeatherBench 2 replication is claimed.
