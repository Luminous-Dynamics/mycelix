# FSRE frame dependency audit v0.1

**Status:** Dependency-free structural audit of declared source-frame ancestry and collection-process identity. It does not authenticate source provenance or prove statistical independence.

## Run

    cargo test --locked --offline --manifest-path tools/fsre-frame-dependency-audit/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-frame-dependency-audit/Cargo.toml -- tools/fsre-frame-dependency-audit/examples/common-root.psv

## Input

Metadata uses strict key=value lines. Table header:

    record_type|frame_id|upstream_frame_id|frame_role|owner_id|collection_process_id|snapshot_id|valid_from_seconds|valid_until_seconds|evidence_owner|evidence_ref|assessment

FRAME rows declare a CAPTURE or UPSTREAM frame. CAPTURE assessment is SEPARATE_DESIGN_DECLARED, DEPENDENCE_DECLARED or UNKNOWN. UPSTREAM frames must use UNKNOWN. Frame IDs, collection-process IDs and snapshot IDs must be stable and source-qualified; snapshot IDs should be globally unique or content-addressed so equal values identify the same snapshot.

DEPENDENCY rows declare that frame_id depends on upstream_frame_id. Each edge requires evidence owner/reference and validity covering the complete study horizon. All frame and dependency endpoints must exist; duplicate edges and cycles are rejected.

## Interpretation

- Shared collection-process IDs and common upstream ancestry are explicit structural dependence signals.
- A shared owner is only a review signal, not proof of dependence.
- Separate-design declarations with no detected common dependency are still not authenticated independence.
- The report always states that dual-system and multiple-systems estimator admissibility is NOT_ESTABLISHED.

No population coverage estimate or estimator is computed by this crate. Source-use status other than AUTHORIZED blocks before frame rows are parsed.
