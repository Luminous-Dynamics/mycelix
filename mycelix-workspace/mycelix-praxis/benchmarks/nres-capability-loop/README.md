# Praxis NRES Capability-Loop Evaluator

A small standalone Rust reference evaluator for the public NRES-005 exercise-to-improvement fixture.

Run from the `mycelix-praxis` directory:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml
```

The CLI emits a machine-readable report with separate measures for objective-evidence coverage, per-service-floor attainment, budget and responder-hour utilization, authority-evidence completeness, accessibility-check coverage, verified corrective actions, and recurrence of previously failed conditions.

The evaluator rejects budget overruns, service-floor drift from the NRES-002 source fixture, accepted observations without evidence, corrective-action closure without completion evidence / independent verification / a successful service-floor retest, and attempts to add aggregate/leaderboard/personal-suitability scores.

## Scope

- Both input fixtures are public synthetic development data.
- A successful evaluator result means internal fixture consistency only.
- The evaluator does not determine legal authority, establish real-world readiness, issue operational instructions, certify first aid, or estimate educational effect.
- No participant identity, political belief, enlistment interest, loyalty, or personal suitability is scored.
- This crate is isolated in its own Cargo workspace and is not part of Praxis's production application or active curriculum.

Tests are authored in `src/lib.rs`. They have not been run in the current environment.
