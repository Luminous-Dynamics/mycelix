# aero-qif-external-reference

Fail-closed normalized resolver for the QIF 3.0 Results → external Plan xId → STEP UUID trace.

## Run

    cargo fmt --manifest-path tools/aero-qif-external-reference/Cargo.toml -- --check
    cargo test --manifest-path tools/aero-qif-external-reference/Cargo.toml
    cargo clippy --manifest-path tools/aero-qif-external-reference/Cargo.toml --all-targets -- -D warnings

## Entry points

- parse_trace_case_json: strict nested JSON parsing.
- validate_trace_case: normalized QPId, numeric QIF ID, ResultSet membership, local-reference/xId/CharacteristicItem/CharacteristicNominal/EntityExternalId/STEP UUID graph checks.
- verify_and_resolve_trace_case: exact Results, Plan and STEP payload byte verification followed by trace resolution.

## Identifier path

The resolver preserves each QIF namespace separately:

    Results CharacteristicItemId local reference id + @xId
      -> ExternalQIFDocumentReference -> external Plan QPId
      -> Plan CharacteristicItem -> CharacteristicNominalId
      -> CharacteristicNominal EntityExternalIds
      -> DigitalModel Entities Entity/@id -> Entity/EntityId STEP UUID
      -> exact AP242 source artifact

A nominal may map to multiple STEP entities. Failed measurement records remain traceable. QPIds and STEP UUIDs are case-insensitive; QIF document-local ids remain scoped to their containing document.

## Dependencies and limits

This crate consumes aero-adapter-run-receipt ContentArtifactV1 and verify_artifact_bytes, plus the shared aero-evidence-types ArtifactRef. It does not create a new content-addressing system.

This is not a QIF or STEP parser, QIF schema validator, GD&T evaluator, or metrology acceptance tool. The checked-in fixture is synthetic; no official CAx-IF archive data is redistributed.
