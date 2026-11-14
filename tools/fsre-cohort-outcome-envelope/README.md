# FSRE cohort outcome envelope v0.1

**Status:** Deterministic read-only composition reference over synthetic cohort lanes and a declared cohort manifest. It is not a source authenticator, population register, equity-weighting policy, physical simulator or operational controller.

## Why this exists

The existing `fsre-service-trajectory` evaluator computes safe service, quality non-compliance, unknown time, deficit and recovery metrics for one declared service lane. This crate applies that evaluator independently to multiple cohort lanes and then reports their unweighted outcome distribution without collapsing the underlying cohort records.

It addresses a specific failure mode: an aggregate or average recovery metric can look acceptable while a declared cohort remains below its service floor or experiences a longer unsafe episode.

## What it checks

- All lane files share a service, service family, cohort-set ID, outcome snapshot, time horizon, observation-hold policy, recovery-stability policy, quality policy and population-denominator unit.
- Cohort IDs are unique; input ordering does not alter output ordering.
- Each lane carries separate cohort-identity, delivery, laboratory-quality and denominator source owner/reference pairs.
- Each lane is evaluated by the existing `fsre-service-trajectory` reference.
- Unknown observation time remains unknown and prevents an `ESTIMABLE_FOR_DECLARED_LANES` disposition.
- Quality non-compliance is reported separately from service quantity deficit.
- Worst-cohort, nearest-rank p50/p90 and outcome-gap summaries are reported alongside every per-cohort row.
- Population-weighted aggregation and social-priority weights are deliberately not inferred.

## Input format

Supply one lane file per protected cohort. Each file uses the existing three-column observation format:

    at_seconds,service_fraction,quality_compliant

The required metadata are:

- `service_id`
- `service_family_id`
- `cohort_set_id`
- `cohort_id`
- `identity_source_owner`, `identity_source_ref`
- `population_denominator` (positive integer or `UNKNOWN`)
- `population_unit` (for example `persons` or `connections`)
- `outcome_snapshot_id`
- `delivery_source_owner`, `delivery_source_ref`
- `quality_source_owner`, `quality_source_ref`
- `denominator_source_owner`, `denominator_source_ref`
- `horizon_start_seconds`, `horizon_end_seconds`
- `minimum_safe_service_fraction`
- `max_observation_hold_seconds`
- `recovery_stability_seconds`
- `quality_required`

Each lane has its own minimum safe-service fraction. Other horizon and policy fields must be shared to make cohort comparisons meaningful. Population denominators are preserved for disclosure but are not used as weights in v0.1.

### Cohort manifest

The CLI requires a frozen manifest followed by one lane file per cohort. The manifest declares `manifest_id`, `manifest_snapshot_id`, `manifest_owner`, `manifest_ref`, service/family/cohort-set IDs, outcome snapshot and population unit. It also declares the study horizon and a source-use assertion: `data_use_status`, `data_use_owner`, `data_use_ref`, `data_use_purpose`, `data_use_valid_from_seconds` and `data_use_valid_until_seconds`. Each `COHORT` row declares the expected cohort ID, denominator, identity-source owner/reference, denominator-source owner/reference and half-open membership validity interval `[valid_from_seconds, valid_until_seconds)`.

The evaluator compares exact cohort-set membership and checks every lane against its manifest row. It requires the declared membership interval to cover the full analysis horizon, exact denominator and source-reference agreement, and exact service/snapshot/horizon scope agreement. Missing lanes, extra lanes, expired membership, or denominator/identity-source drift prevent a completeness claim.

Before any lane is parsed or evaluated, the CLI checks the source-use status. `REVIEW_REQUIRED`, `NOT_AUTHORIZED`, and `AUTHORIZED_FOR_DECLARED_PURPOSE` whose validity interval does not cover the complete study horizon return a blocked report with `service_metrics=NOT_COMPUTED`. A permitted status also needs an allowed state value; unknown states are rejected.

Run:

    cargo test --locked --offline --manifest-path tools/fsre-cohort-outcome-envelope/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-cohort-outcome-envelope/Cargo.toml -- \
      tools/fsre-cohort-outcome-envelope/fixtures/cohort-manifest.psv \
      tools/fsre-cohort-outcome-envelope/examples/hospital.csv \
      tools/fsre-cohort-outcome-envelope/examples/households.csv \
      tools/fsre-cohort-outcome-envelope/examples/remote-community.csv

## Output semantics

- `ESTIMABLE_FOR_DECLARED_MANIFEST` means every lane in the supplied manifest is present exactly once, lane scope/denominator/source bindings agree, manifest membership covers the full horizon, and no lane has unknown time. It is complete only relative to that declared manifest.
- `NOT_ESTIMABLE` means at least one lane has unknown observation time or the submitted cohort set does not exactly reconcile to the manifest. Known-only deficit and percentile values remain explicitly labelled as such.
- Quantiles use the nearest-rank convention and are unweighted across cohort lanes.
- Quality non-compliance is not silently converted into a service quantity deficit.
- Denominators are reported per lane. No population-weighted aggregate or normative priority is computed.

The report preserves separate identity, delivery, laboratory-quality and population-denominator owner/reference pairs for each cohort, and declares one shared retrospective outcome snapshot. The manifest has a separate owner, opaque reference and snapshot identifier. These references are lineage aids, not authenticated or content-hashed receipts. Manifest completeness establishes coverage relative to the manifest only; it does not establish that the register includes every affected population.

**Critical authorization limitation:** `AUTHORIZED_FOR_DECLARED_PURPOSE` is a supplied assertion. The current reference checks its status and time scope but does not verify a signature, legal instrument, source-owner identity or the underlying permission document. An accepted output reports `data_use_authorization=ASSERTED_NOT_AUTHENTICATED_BY_THIS_TOOL` and `source_use_gate=UNVERIFIED_AUTHORIZATION_ASSERTION_SCOPE_ACCEPTED`. This is an engineering preflight, not a legal authorization system. Do not use it to promote external Green Drop or other restricted data into production.

The report explicitly states:
- `cohort_set_completeness=COMPLETE_FOR_DECLARED_MANIFEST` only when exact manifest reconciliation passes; otherwise `INCOMPLETE_OR_MISMATCHED_TO_DECLARED_MANIFEST`.
- A complete manifest does not make unknown service observations estimable; the two dispositions are independent.
- `population_weighted_aggregation=NOT_COMPUTED`
- `population_coverage=NOT_ESTABLISHED`
- `operational_authority=NONE`

## Limitations and next research

- The service fraction and quality flag are only as trustworthy as the supplied observations; this tool does not authenticate them or recompute content digests for the source references.
- It does not establish that a lane represents an entire neighbourhood, municipality or vulnerable group.
- It does not define what an equitable allocation should be. Distributional and restorative equity require transparent, legitimate, owner-reviewed policy rather than weights inferred from labels.
- No uncertainty interval is estimated in v0.1.
- Future work should replace the positive authorization assertion with a content-bound, owner-signed source-use receipt and independently trusted verification policy; then add independently adjudicated cohort mappings, external population-coverage audits, uncertainty and missingness analysis, and matched comparison against unweighted, population-weighted, threshold-gap and restorative-equity baselines.
