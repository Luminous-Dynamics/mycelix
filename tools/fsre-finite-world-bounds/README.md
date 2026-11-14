# FSRE finite-world bounds v0.1

**Status:** Dependency-free exact reference oracle over an explicitly enumerated finite set of admissible worlds. It is intended for small-instance verification of future LP/MILP backends, not as a scalable causal inference engine.

## Run

    cargo test --locked --offline --manifest-path tools/fsre-finite-world-bounds/Cargo.toml
    cargo run --quiet --locked --offline --manifest-path tools/fsre-finite-world-bounds/Cargo.toml -- tools/fsre-finite-world-bounds/examples/water-restoration.psv

## Input

Metadata is strict key=value text. Required table header:

    world_id|action_id|decision_value|safety_value

Every world must have one row per action. All outcome units are signed integer fixed-point values. Missing potential outcomes are not imputed.

Required gates:
- source_use_status=AUTHORIZED
- authority_status=ESTABLISHED
- world_set_completeness=DECLARED_COMPLETE

The first two are assertions supplied by the caller, not cryptographic or legal verification. The third is an assumption about the finite model set, not proof that the model set covers reality. A failed source-use/authority gate blocks numeric evaluation. An unknown/incomplete world set returns NOT_IDENTIFIED without bounds.

## Outputs

Per-action decision and safety bounds, robust service-floor status, best/worst regret bounds for robustly safe actions, robust dominance, rank invariance and minimax-regret actions among robustly safe candidates.

A successful result is conditional on the declared finite world set. It is not a real-world causal identification claim, source authentication or population completeness finding.
