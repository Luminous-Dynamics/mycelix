# FSRE service-trajectory reference evaluator

A small, dependency-free Rust reference implementation for a narrow FSRE benchmark metric: the time trajectory of one declared service lane.

It measures availability against a declared minimum safe service floor and full-service target. It separately reports quality-noncompliance, safe degradation, known/unknown observation duration, cumulative fractional service deficit and stable recovery times.

This crate is **not** a physical water/power/network simulator, causal engine, dependency graph, evidence ledger, identity/authority service or source-authenticity verifier. It does not determine whether the supplied observations are true. It calculates declared metrics from supplied time-series observations.

## Run

From the repository root:

```sh
cargo test --locked --manifest-path tools/fsre-service-trajectory/Cargo.toml
cargo run --locked --manifest-path tools/fsre-service-trajectory/Cargo.toml -- \
  tools/fsre-service-trajectory/examples/synthetic-water-service.csv
```

No third-party Rust dependencies are used.

## Input format

The input is a narrow CSV-like format with key/value metadata comments and exactly three comma-separated fields per observation. It intentionally does not support quoted CSV fields.

Required metadata:

```text
# service_id=synthetic-household-water
# horizon_start_seconds=0
# horizon_end_seconds=100
# minimum_safe_service_fraction=0.50
# max_observation_hold_seconds=60
# recovery_stability_seconds=10
# quality_required=true
```

Observation rows:

```text
# Columns: at_seconds,service_fraction,quality_compliant
0,1.0,true
20,0.2,true
40,0.6,true
70,1.0,true
```

The horizon is half-open: `[horizon_start_seconds, horizon_end_seconds)`. Observation timestamps must be strictly increasing and fall within the horizon. Service fractions must be finite and within `[0,1]`.

## Time and unknown semantics

Each observation applies from its timestamp until the next observation, but never beyond `max_observation_hold_seconds`. Any uncovered portion of the horizon is **unknown**. The evaluator does not interpolate, extrapolate or treat a stale observation as healthy or failed.

The first time below the service floor or in required quality non-compliance is the first known breach. Safe recovery is the beginning of the first period that remains above the minimum floor and satisfies required quality for the full declared stability window. Full recovery uses the same stability window at 100% service. A recovery that is too short does not count.

The service-deficit integral is:

`sum((1 - service_fraction) * known_interval_seconds)`

It is reported in fraction-seconds. Quality non-compliance is reported separately and does not get disguised as a low availability fraction. Unknown time is reported separately and excluded from the deficit integral; a result with substantial unknown coverage must not be described as a complete service outcome.

Metrics are computed for one service/cohort lane at a time. Do not average across unrelated services, geographic areas or cohorts.

## Synthetic example

The included fixture is made up solely to exercise metric semantics. It is not data from a municipality, utility, asset, network or operating system.

Expected deterministic values:

| Metric | Value |
|---|---:|
| Horizon | 100 seconds |
| Known / unknown | 100 / 0 seconds |
| Full service | 50 seconds |
| Safe degraded service | 30 seconds |
| Below-minimum service | 20 seconds |
| Quality non-compliance | 0 seconds |
| Longest unsafe episode | 20 seconds |
| Service deficit | 28 fraction-seconds |
| First breach | 20 seconds |
| Stable safe recovery starts | 40 seconds |
| Stable full recovery starts | 70 seconds |

The expected values are hand-derived from the fixture semantics. Run the Rust tests and CLI on a Rust-equipped host before treating the implementation as locally qualified.

## Qualification boundary

The unit tests cover healthy service, safe degradation, outage/recovery, quality requirements, stale and unknown intervals, recovery stability, timestamp/input validation and service-accounting identities. A bounded exhaustive test enumerates 1,296 observation trajectories across three observation-hold limits, both quality policies and four recovery-stability windows (31,104 evaluations) and checks mathematical accounting invariants. The path-specific CI workflow also runs the CLI against two committed golden reports: the nominal water-service example and an unknown-gap/recovery case. These checks do not test production telemetry, physical process correctness, legal authority, causal attribution, field operations or independent service impact.

The standalone CI workflow compiles, tests, formats and lints this crate when its files change. A skipped or pending hosted job is not a pass.
