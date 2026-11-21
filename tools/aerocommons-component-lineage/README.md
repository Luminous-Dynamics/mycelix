# AeroCommons component-lineage verifier

A small, standalone Rust verifier for a synthetic non-flight-critical build packet.

## Run

From the repository root:

    cargo test --manifest-path tools/aerocommons-component-lineage/Cargo.toml

The crate consumes the typed AeroEvidence primitives from the stacked AeroCommons evidence-types branch. It is deliberately outside the large Holochain workspace so the focused test target does not require building unrelated zomes.

## Result semantics

`parse_packet_json` is the strict admission boundary: it rejects unknown and missing fields at the packet, reference, artifact, calibration and nested evidence levels. `validate_packet` then checks structural identity, edge roles, exact serialized-unit/configuration binding, lifecycle, calibration validity windows and event chronology. Its result is STRUCTURALLY_COMPLETE only; calibration timestamps are declared replay metadata, not proof of clock accuracy.

`verify_artifact_bytes` recomputes byte length and SHA-256. `verify_packet_artifact_payloads` requires the exact four artifact payloads and verifies each against its content-addressed reference. A full-packet result reports `artifact_payloads_verified = true` only after these byte checks succeed.

It does not retrieve artifact bytes, verify their SHA-256 values against remote storage, authenticate signatures, establish independent review, qualify a process/material/instrument, decide whether a part passed inspection or test, establish conformity or airworthiness, or authorize fabrication/use/flight.

## Interoperability

The packet references STEP AP242 for the product definition and QIF for the inspection result. The checked-in payloads are deliberately tiny synthetic text/XML samples; they are not normative AP242 or QIF documents. The verifier checks identity, byte length and SHA-256 only. It does not parse, schema-validate, semantically validate or claim conformance to those standards.
