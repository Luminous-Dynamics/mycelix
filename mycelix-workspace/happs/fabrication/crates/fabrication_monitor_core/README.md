# Fabrication Monitor Core

A deterministic, Holochain-independent Rust core for the first Cincinnati process-monitoring implementation.

## Contract

- Input: a source-timestamped sensor frame, exact design/recipe/equipment/material/environment/calibration profile, monitoring-session ID, and a versioned detector configuration with an exact reference dataset/corpus identifier.
- Baseline: robust medians and median absolute deviations (MAD) fit only from explicitly selected healthy reference frames. A per-signal minimum scale prevents division by zero.
- Output: per-signal robust-z deviations plus a disposition explaining whether the frame was evaluated, rejected, uncalibrated, reordered, or separated by a data gap.
- Determinism: no wall clock, randomness, network access, Holochain dependency, mutable global state, or machine I/O.
- Safety boundary: advisory only. It cannot pause/abort a print, adjust a recipe, establish defect probability, produce a product quality score, or certify part conformity.

## Why profile identity is exact

The baseline is invalid for a frame from another design, process recipe, equipment configuration, material lot/profile, declared environment, or calibration profile. Every report carries the reference dataset identifier and detector version. The core returns ProcessProfileMismatch rather than silently applying baseline statistics outside their declared applicability. Reference fitting also rejects duplicated session sequence/timestamp identities so duplicated samples cannot inflate the reference count. Adapters must preserve units, source timestamps, sequence, missing-data state and measurement-system identity.

## Metrics and limitations

The engine reports robust-z deviations for hotend/bed/ambient temperature, X/Y/Z/extruder current, vibration, filament tension and relative humidity, not defect diagnoses. Motor-current inputs remain signed when the source sensor uses signed current; any magnitude/RMS transformation must be explicit in the source adapter and measurement profile. Changes can have multiple physical causes, so a deviation must not be treated as a proven defect. The configured minimum scales and threshold are profile-bound values; no unit-independent defaults are supplied.

Optional filament tension, ambient temperature and humidity can be omitted during fitting. If a later observation contains a value without reference coverage, the report explicitly marks that signal uncalibrated while retaining results for other covered signals. Missing sequence numbers or excessive sample intervals are not scored. Relative humidity is constrained to 0–100%.

## Frozen synthetic corpus

- Corpus ID: CINCINNATI-MONITOR-SYNTHETIC-V1
- Fixture: tests/fixtures/cincinnati_v1.tsv
- SHA-256: 10abc8e1839c232790dd6e3adb975bf326ca2ea9516d743886c3242148d330c9
- 24 cases: nominal trace, temperature/current/vibration/environment deviations, optional-sensor coverage, design/recipe/equipment/material/environment/calibration mismatch, invalid input, signed-current handling, sequence/timestamp gaps, ordering violations and session mismatch.
- The public-API integration test in tests/frozen_corpus.rs checks disposition and signal-level outputs against expected values frozen separately from detector code. A separately built, independent verifier is still a follow-on.

This hash identifies the TSV bytes only. The corpus remains synthetic and is not evidence of physical defect sensitivity, production yield, safety, cost improvement or generalized cross-process performance.

## Run tests

From mycelix-workspace/happs/fabrication:

    cargo test -p fabrication_monitor_core
    cargo fmt --all -- --check
    cargo clippy -p fabrication_monitor_core --all-targets -- -D warnings

These commands are intended acceptance checks; a command being listed here is not evidence that it has passed. Each result must be bound to the tested commit and reported separately.

## Next steps

1. Independently review and run the crate tests against the exact PR head.
2. Add an adapter from the existing Fabrication common SensorSnapshot type without changing existing hApp contracts.
3. Compare this detector with fixed engineering limits and a conventional statistical-process-control baseline.
4. Evaluate against recorded, process-appropriate data using run- or batch-level holdouts before making any physical quality or savings claim.
5. Keep monitoring in shadow/recommendation mode until control authority and hardware safety have a separate verified design.
