# Fabrication Monitor Core

A deterministic, Holochain-independent Rust core for the first Cincinnati process-monitoring implementation.

## Contract

- Input: a source-timestamped sensor frame, exact design/recipe/equipment/material/environment/calibration profile, monitoring-session ID, and a versioned detector configuration.
- Baseline: robust medians and median absolute deviations (MAD) fit only from explicitly selected healthy reference frames. A per-signal minimum scale prevents division by zero.
- Output: per-signal robust-z deviations plus a disposition explaining whether the frame was evaluated, rejected, uncalibrated, reordered, or separated by a data gap.
- Determinism: no wall clock, randomness, network access, Holochain dependency, mutable global state, or machine I/O.
- Safety boundary: advisory only. It cannot pause/abort a print, adjust a recipe, establish defect probability, produce a product quality score, or certify part conformity.

## Why profile identity is exact

The baseline is invalid for a frame from another design, process recipe, equipment configuration, material lot/profile, declared environment, or calibration profile. The core returns ProcessProfileMismatch rather than silently applying baseline statistics outside their declared applicability. Reference fitting also rejects duplicated session sequence/timestamp identities so duplicated samples cannot inflate the reference count. Adapters must preserve units, source timestamps, sequence, missing-data state and measurement-system identity.

## Metrics and limitations

The engine reports robust-z deviations, not defect diagnoses. Temperature/current/vibration/tension changes can have multiple physical causes, so a deviation must not be treated as a proven defect. The configured minimum scales and threshold are profile-bound values; no unit-independent defaults are supplied.

Optional filament tension can be omitted during fitting. If a later observation contains tension without reference coverage, the report explicitly marks it uncalibrated while retaining results for the other covered signals. Missing sequence numbers or excessive sample intervals are not scored.

## Frozen synthetic corpus

- Corpus ID: CINCINNATI-MONITOR-SYNTHETIC-V1
- Fixture: tests/fixtures/cincinnati_v1.tsv
- SHA-256: e1206515e6a7c03b9e6fbdd077b7ab4ca470bb9abb02beb78e17241231739eaa
- 15 cases: nominal trace, four signal deviations, uncalibrated optional signal, profile mismatch, invalid input, sequence/timestamp gaps, ordering violations and session mismatch.
- The independent integration evaluator in tests/frozen_corpus.rs checks disposition and signal-level expected outputs against the frozen fixture.

This hash identifies the TSV bytes only. The corpus remains synthetic and is not evidence of physical defect sensitivity, production yield, safety, cost improvement or generalized cross-process performance.

## Run tests

From mycelix-workspace/happs/fabrication:

    cargo test -p fabrication_monitor_core
    cargo fmt --all -- --check
    cargo clippy -p fabrication_monitor_core --all-targets -- -D warnings

These commands are intended acceptance checks; a command being listed here is not evidence that it has passed. Each result must be bound to the tested commit and reported separately.

## Next steps

1. Independently review and run the crate tests.
2. Add frozen positive and negative synthetic traces with corpus manifest and exact counts.
3. Add an adapter from the existing Fabrication common SensorSnapshot type without changing existing hApp contracts.
4. Compare this detector with fixed engineering limits and a conventional statistical-process-control baseline.
5. Evaluate against recorded, process-appropriate data before making any physical quality or savings claim.
6. Keep monitoring in shadow/recommendation mode until control authority and hardware safety have a separate verified design.
