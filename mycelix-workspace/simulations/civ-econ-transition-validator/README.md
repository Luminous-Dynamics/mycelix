# CIV-ECON-001A transition validator

This dependency-free Rust reference model checks invariants defined in
`docs/research/civ/CIV-ECON-001A_REGIME_PROFILE_AND_TRANSITION_PROTOCOL_V1.md`.

It operates on a **normalized typed projection** of a transition manifest.
A JSON-Schema-to-Rust adapter is not implemented in this crate yet, and the
reference model does not parse the schema file. It checks:

- versioned policy-profile, economic-instrument, and unit identities as distinct types;
- source-scope coverage with exactly one typed mapping per item;
- exact identity and atomic-quantity preservation for Preserve and ReReserve;
- explicit conversion profiles, authority decisions, novation acceptance, and discharge receipts;
- source-origin preservation across conversion and novation;
- lifecycle progression and frozen manifest-digest binding;
- simulation run receipt, output digest, scenario-corpus digest, and environment binding;
- evaluator binding to the exact simulation output and scenario corpus, with a separate evaluation report digest and receipt;
- rights-floor, resource-reservation, external-obligation, cutover, and reconciliation gates;
- idempotent effect identity, duplicate replay, and payload-conflict rejection.

## Run

From this directory:

    cargo test --offline
    cargo fmt --check
    cargo clippy --offline --all-targets -- -D warnings

The crate has an isolated Cargo workspace and no third-party dependencies.

## Claim ceiling

This is a reference validator, not a source of authority. It does not hash
canonical manifests, verify signatures, query live state, establish authority
validity/currentness, prove physical capacity, prove counterparty acceptance,
verify conversion arithmetic from source data, establish legal discharge, or
establish external settlement finality. Production work still needs the JSON
adapter, live evidence bindings, and independent qualification.

The 34 unit tests in this source are authored regression tests, not a PASS claim
until the exact-head CI run executes and succeeds. Even then, passing them proves
only the local invariants actually tested, not end-to-end transition correctness
or production readiness.
