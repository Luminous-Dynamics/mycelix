# FSRE cohort outcome envelope v0.1

**Status:** Deterministic read-only composition reference over synthetic cohort lanes. It is not a source authenticator, population register, equity-weighting policy, physical simulator or operational controller.

## Why this exists

The existing \`fsre-service-trajectory\` evaluator computes safe service, quality non-compliance, unknown time, deficit and recovery metrics for one declared service lane. This crate applies that evaluator independently to multiple cohort lanes and then reports their unweighted outcome distribution without collapsing the underlying cohort records.

It addresses a specific failure mode: an aggregate or average recovery metric can look acceptable while a declared cohort remains below its service floor or experiences a longer unsafe episode.

## What it checks

- All lane files share a service, service family, cohort-set ID, time horizon, observation-hold policy, recovery-stability policy, quality policy and population-denominator unit.
- Cohort IDs are unique; input ordering does not alter output ordering.
- Each lane is evaluated by the existing \`fsre-service-trajectory\` reference.
- Unknown observation time remains unknown and prevents an \`ESTIMABLE_FOR_DECLARED_LANES\` disposition.
- Quality non-compliance is reported separately from service quantity deficit.
- Worst-cohort, nearest-rank p50/p90 and outcome-gap summaries are reported alongside every per-cohort row.
- Population-weighted aggregation and social-priority weights are deliberately not inferred.

## Input format

Supply one lane file per protected cohort. Each file uses the existing three-column observation format:

    at_seconds,service_fraction,quality_compliant

The required metadata are:

- \`service_id\`
- \`service_family_id\`
- \`cohort_set_id\`
- \`cohort_id\`
- \`population_denominator\` (positive integer or \`UNKNOWN\`)
- \`population_unit\` (for example \`persons\` or \`connections\`)
- \`horizon_start_seconds\`, \`horizon_end_seconds\`
- \`minimum_safe_service_fraction\`
- \`max_observation_hold_seconds\`
- \`recovery_stability_seconds\`
- \`quality_required\`

Each lane has its own minimum safe-service fraction. Other horizon and policy fields must be shared to make cohort comparisons meaningful. Population denominators are preserved for disclosure but are not used as weights in v0.1.

Run:

    cargo test --locked --offline --manifest-path tools/fsre-cohort-outcome-envelope/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-cohort-outcome-envelope/Cargo.toml -- \
      tools/fsre-cohort-outcome-envelope/examples/hospital.csv \
      tools/fsre-cohort-outcome-envelope/examples/households.csv \
      tools/fsre-cohort-outcome-envelope/examples/remote-community.csv

## Output semantics

- \`ESTIMABLE_FOR_DECLARED_LANES\` means every submitted lane has no unknown time in the declared horizon. It does **not** establish that the submitted cohort set covers the actual affected population.
- \`NOT_ESTIMABLE\` means at least one lane has unknown observation time. Known-only deficit and percentile values remain explicitly labelled as such.
- Quantiles use the nearest-rank convention and are unweighted across cohort lanes.
- Quality non-compliance is not silently converted into a service quantity deficit.
- Denominators are reported per lane. No population-weighted aggregate or normative priority is computed.

The report explicitly states:
- \`cohort_set_completeness=NOT_ESTABLISHED_BY_THIS_TOOL\`
- \`population_weighted_aggregation=NOT_COMPUTED\`
- \`population_coverage=NOT_ESTABLISHED\`
- \`operational_authority=NONE\`

## Limitations and next research

- The service fraction and quality flag are only as trustworthy as the supplied observations; this tool does not authenticate them.
- It does not establish that a lane represents an entire neighbourhood, municipality or vulnerable group.
- It does not define what an equitable allocation should be. Distributional and restorative equity require transparent, legitimate, owner-reviewed policy rather than weights inferred from labels.
- No uncertainty interval is estimated in v0.1.
- Future work should add cohort-set manifests with source-backed denominator lineage, independently adjudicated cohort mappings, uncertainty and missingness analysis, and matched comparison against unweighted, population-weighted, threshold-gap and restorative-equity baselines.
