# aero-qif-external-reference

Fail-closed normalized resolver for the QIF 3.0 Results → external Plan xId → STEP UUID trace.

## Run

    cargo fmt --manifest-path tools/aero-qif-external-reference/Cargo.toml -- --check
    cargo test --manifest-path tools/aero-qif-external-reference/Cargo.toml
    cargo clippy --manifest-path tools/aero-qif-external-reference/Cargo.toml --all-targets -- -D warnings

## Entry points

- parse_trace_case_json: strict nested JSON parsing.
- validate_trace_case: normalized QPId/local-ref/xId/STEP-UUID graph checks.
- verify_and_resolve_trace_case: exact Results, Plan and STEP payload byte verification followed by trace resolution.

The crate reuses aero-adapter-run-receipt ContentArtifactV1 and verify_artifact_bytes; it does not create a new content-addressing system.

This is not a QIF or STEP parser. The checked-in fixture is synthetic; no official CAx-IF archive data is redistributed.
