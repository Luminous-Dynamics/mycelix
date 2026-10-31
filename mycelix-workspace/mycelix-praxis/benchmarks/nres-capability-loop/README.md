# Praxis NRES Capability-Loop Evaluator

A small standalone Rust reference evaluator for the public NRES-005 exercise-to-improvement fixture.

Run from the `mycelix-praxis` directory:

```bash
cargo test --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --envelope
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --trajectory
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --probability
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --joint
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --tail-shift
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --decision-value
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --robust-decision
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --wasserstein-joint
cargo run --manifest-path benchmarks/nres-capability-loop/Cargo.toml -- --calibrate-wasserstein-radius
```

The `--calibrate-wasserstein-radius` mode evaluates NRES-014 block-level split-conformal calibration of a finite-support Wasserstein radius, recomputes block empirical distributions and exact W1 scores with min-cost flow, enforces disjoint source-family IDs across calibration/development splits, and reports the conformal rank, a non-confirmatory development diagnostic, and a separate finite-state population sampling-error correction with an explicit vacuity flag. The `--wasserstein-joint` mode evaluates NRES-013 nominal, exact finite-support Wasserstein worst-case expected-loss and Wasserstein minimax-regret policies over complete joint service states, with source-lineage checks, a weighted-Hamming transport metric, resource-feasible actions, radius sensitivity and separate profile/radius reports. The `--robust-decision` mode evaluates NRES-012 nominal point, worst-case expected-loss and minimax-regret decisions over declared Bernoulli total-variation ambiguity intervals, per profile and shift regime. The `--decision-value` mode evaluates NRES-011 weighted reliability/resolution/uncertainty diagnostics and profile-specific preparedness decision value, including human-review deferral costs and a perfect-information lower bound. The `--tail-shift` mode evaluates NRES-010 importance-weighted rare-event exceedance probabilities, GPD conditional tail severity, tail PIT, quantile coverage and performance across declared shift regimes. The `--joint` mode evaluates NRES-009 complete joint breach distributions with multiclass Brier, joint log loss, energy score, variogram score, per-service marginals, pairwise co-breach scores and a same-marginal independence-projection ablation. The `--probability` mode evaluates NRES-008 binary breach probabilities, Brier/log scores, per-service coverage and proper scores, abstention reasons, fixed reliability bins and a risk/coverage curve. The `--trajectory` mode evaluates NRES-007 matched-shock service trajectories, exact piecewise-linear deficit areas, sustained recovery, dependency metadata and resource limits. It reports every service separately and produces only descriptive scenario contrasts. The default CLI mode emits a machine-readable report with separate measures for objective-evidence coverage, per-service-floor attainment, budget and responder-hour utilization, authority-evidence completeness, accessibility-check coverage, verified corrective actions, and recurrence of previously failed conditions. The `--envelope` mode exports a JSON-LD 1.1 envelope using W3C PROV vocabulary, framework crosswalk metadata, privacy boundaries and an explicit unsigned-demo state.

The evaluator rejects budget overruns, service-floor drift from the NRES-002 source fixture, accepted observations without evidence, corrective-action closure without completion evidence / independent verification / a successful service-floor retest, invalid probability outputs, and attempts to add aggregate/leaderboard/personal-suitability scores.

## Scope

- Both input fixtures are public synthetic development data.
- A successful evaluator result means internal fixture consistency only.
- The evaluator does not determine legal authority, establish real-world readiness, issue operational instructions, certify first aid, or estimate educational effect.
- No participant identity, political belief, enlistment interest, loyalty, or personal suitability is scored.
- This crate is isolated in its own Cargo workspace and is not part of Praxis's production application or active curriculum.

Tests are authored in `src/lib.rs`, `src/interchange.rs`, `src/trajectory.rs`, `src/probability.rs`, `src/joint.rs`, `src/tail_shift.rs`, `src/decision_value.rs` and `src/robust_decision.rs` and `src/wasserstein.rs` and `src/radius_calibration.rs`. They have not been run in the current environment. The envelope profile and public conformance corpus are documented in [PRAXIS-NRES-006](../../docs/curriculum/PRAXIS_NRES_006_EVIDENCE_INTERCHANGE.md) and [PRAXIS-NRES-007](../../docs/curriculum/PRAXIS_NRES_007_CASCADE_RECOVERY.md) and [PRAXIS-NRES-008](../../docs/curriculum/PRAXIS_NRES_008_PROBABILITY_ABSTENTION.md) and [PRAXIS-NRES-009](../../docs/curriculum/PRAXIS_NRES_009_JOINT_SERVICE_FORECAST.md) and [PRAXIS-NRES-010](../../docs/curriculum/PRAXIS_NRES_010_TAIL_SHIFT_ROBUSTNESS.md) and [PRAXIS-NRES-011](../../docs/curriculum/PRAXIS_NRES_011_RELIABILITY_RESOLUTION_DECISION_VALUE.md) and [PRAXIS-NRES-012](../../docs/curriculum/PRAXIS_NRES_012_AMBIGUITY_ROBUST_DECISION.md) and [PRAXIS-NRES-013](../../docs/curriculum/PRAXIS_NRES_013_WASSERSTEIN_JOINT_DECISION.md) and [PRAXIS-NRES-014](../../docs/curriculum/PRAXIS_NRES_014_WASSERSTEIN_RADIUS_CALIBRATION.md).
