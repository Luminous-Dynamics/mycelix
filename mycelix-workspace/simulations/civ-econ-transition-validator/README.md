# CIV-ECON-001A transition validator

This dependency-free Rust reference model composes with the transition contract:
docs/research/civ/CIV-ECON-001A_REGIME_PROFILE_AND_TRANSITION_PROTOCOL_V1.md

It checks typed manifest completeness, one-to-one scope mapping coverage,
mapping-specific requirements, authority-role overlap declarations, lifecycle
progression, exact digest binding across simulation/evaluation/approval/
reconciliation, cutover guardrails, and idempotent effect request identity.

## Run

From this directory:

    cargo test --offline
    cargo fmt --check
    cargo clippy --offline --all-targets -- -D warnings

The crate has an isolated Cargo workspace and no third-party dependencies.

## Claim ceiling

This is a reference validator, not a source of authority. It does not parse or
validate the companion JSON Schema, hash canonical manifests, verify signatures,
query live state, prove legal authority, prove physical capacity, establish
counterparty acceptance, or establish external settlement finality. Callers
must provide canonical digest values and qualified evidence; production
integration must bind these through the existing Economic Fabric, Finance,
governance, and identity owners.

Passing its tests would establish only the local invariants that were tested,
not end-to-end transition correctness or production readiness.
