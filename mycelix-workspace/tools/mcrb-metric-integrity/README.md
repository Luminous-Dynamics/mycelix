# MCRB Metric Integrity Verifier

This dependency-free Rust library validates measurement batches before the Mycelix Coordination and Recovery Benchmark (MCRB) emits aggregate performance metrics.

## Local duration contract

The verifier does **not** accept a caller-supplied `duration_ms` as the source of truth.

Each `DurationSample` carries:

- peer ID and logical operation ID;
- monotonic clock ID;
- monotonic clock epoch / boot generation;
- raw monotonic `start_ticks` and `end_ticks`.

Each peer's `ClockObservation` carries the same clock ID/epoch and its declared tick frequency. The verifier checks identity and epoch equality, rejects reversed ticks and zero frequency, and derives elapsed nanoseconds using integer arithmetic before converting to milliseconds for percentile calculation.

The supplied duration is not trusted, so the benchmark cannot make an operation appear faster by submitting a smaller duration value than the raw ticks imply.

## Invariants

- Local durations are derived from raw monotonic start/end ticks.
- The sample clock ID and epoch must match the peer's declared clock.
- A sample cannot cross a clock epoch / reboot boundary.
- Reversed ticks and zero tick frequency invalidate the batch.
- Missing peer observations or incomplete counters make the batch incomplete.
- Unknown peers, duplicate peer clocks, empty operation IDs and duplicate per-peer operation IDs are rejected.
- Outliers inside the declared duration window are preserved and reported by their original sample indexes. They are not trimmed, clipped or silently excluded.
- Aggregate p50/p95/p99 values are emitted only when the full batch is valid and complete.
- Global and per-peer distributions are calculated from the derived durations, not from a caller-provided duration field.

## Use

The API accepts a workload-specific maximum duration, outlier threshold and wall-clock uncertainty limit. These are explicit experiment parameters, not universal Mycelix defaults.

```rust
let report = mycelix_mcrb_metric_integrity::validate_batch(&config, &batch);
if report.aggregate_eligible {
    let percentiles = report.percentiles.expect("valid batches have percentiles");
    // Bind percentiles, raw tick samples and report to the run receipt.
} else {
    // Preserve all raw ticks and issues; do not publish a performance claim.
}
```

## Status meanings

- `Valid`: all required peers, clock identities/epochs, derived durations and counters are coherent.
- `Incomplete`: coverage or expected observations are missing; no aggregate is eligible.
- `Invalid`: an impossible or out-of-contract measurement was observed; no aggregate is eligible.

## Remaining limitation: cross-host convergence timing

The current crate now independently derives **local** durations from monotonic ticks. It does not yet pair a publisher's record event with each peer's first valid observation by record digest, nor derive uncertainty-bounded cross-host latency intervals from both hosts' wall-clock calibration.

Therefore this crate can support local per-peer duration distributions once exact-head validation passes, but it must not yet be used to claim independently qualified cross-host DHT convergence latency.

Follow-up: [MCRB-TIME-001 #6335](https://github.com/Luminous-Dynamics/mycelix/issues/6335).

## Validation state

Thirteen unit tests are authored for local tick derivation, preserved outliers, reversed ticks, clock identity/epoch mismatch, invalid frequency, duration bounds, peer coverage, clock uncertainty metadata, counter mismatch, wall-clock declaration, duplicate operation IDs, multi-peer observation and empty runs.

No exact-head Cargo test or Clippy pass is claimed until the GitHub workflow completes successfully. Do not use this crate as an external qualification verifier until its implementation and run-receipt binding are independently reviewed.
