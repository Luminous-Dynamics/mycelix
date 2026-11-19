# AeroCommons component-lineage verifier

A small, standalone Rust verifier for a synthetic non-flight-critical build packet.

## Run

From the repository root:

    cargo test --manifest-path tools/aerocommons-component-lineage/Cargo.toml --locked

The crate consumes the typed AeroEvidence primitives from the stacked AeroCommons evidence-types branch. It is deliberately outside the large Holochain workspace so the focused test target does not require building unrelated zomes.

## Result semantics

A successful report is STRUCTURALLY_COMPLETE only. It means that required artifact references and evidence records are present, internally cross-linked, bound to the same component/configuration where required, and active under the declared record lifecycle.

It does not retrieve artifact bytes, verify their SHA-256 values against remote storage, authenticate signatures, establish independent review, qualify a process/material/instrument, decide whether a part passed inspection or test, establish conformity or airworthiness, or authorize fabrication/use/flight.

## Interoperability

The packet references STEP AP242 for the product definition and QIF for the inspection result. These are opaque external artefact references in this increment; the crate does not parse those formats or claim standards conformance.
