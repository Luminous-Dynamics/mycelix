# MCRB Metric Integrity Verifier

This dependency-free Rust library validates raw measurement batches before the Mycelix Coordination and Recovery Benchmark (MCRB) emits aggregate performance metrics.

## Invariants

- Per-host duration measurements must use monotonic clocks.
- Clock source, synchronization method, offset and uncertainty are explicit.
- Negative, non-finite and out-of-declared-window durations invalidate the batch.
- Missing peer observations or incomplete counters make the batch incomplete.
- Counter overflow, unknown peers, duplicate peer clocks and excessive clock uncertainty invalidate the batch.
- Outliers inside the declared measurement window are preserved and reported by their original sample indexes. They are not trimmed, clipped or silently excluded.
- Aggregate p50/p95/p99 values are emitted only when the full batch is valid and complete.

## Use

The API accepts a workload-specific maximum duration, outlier threshold and maximum clock uncertainty. These are explicit experiment parameters, not universal Mycelix defaults.

```rust
let report = mycelix_mcrb_metric_integrity::validate_batch(&config, &batch);
if report.aggregate_eligible {
    let percentiles = report.percentiles.expect("valid batches have percentiles");
    // Bind percentiles, raw sample digest and report to the run receipt.
} else {
    // Preserve all raw samples and issues; do not publish a performance claim.
}
```

## Status meanings

- `Valid`: all required peers, clocks, durations and counters are coherent.
- `Incomplete`: coverage or expected observations are missing; no aggregate is eligible.
- `Invalid`: an impossible or out-of-contract measurement was observed; no aggregate is eligible.

This library does not synchronize clocks, collect telemetry, infer a network's true topology or qualify application semantics. The caller must supply raw peer observations and pin the verifier version/digest in the MCRB run receipt.

## Current validation state

The unit tests are authored, but no exact-head Cargo run is claimed until the `MCRB Metric Integrity` workflow completes successfully. Do not use this crate as an external qualification verifier until its implementation and receipt binding are independently reviewed.


## Known limitation: raw clock evidence follow-up

The current `DurationSample` contains a caller-supplied `duration_ms` value and the current `ClockObservation` contains caller-supplied clock metadata. The verifier therefore checks structural consistency of submitted samples, but it does **not yet independently recompute local durations from raw monotonic start/end ticks**.

It also does not yet pair cross-host publish and first-valid-observation events by record digest or calculate uncertainty-bounded convergence intervals. Do not use the current verifier to claim independently qualified cross-host DHT convergence latency.

Follow-up: [MCRB-TIME-001 #6335](https://github.com/Luminous-Dynamics/mycelix/issues/6335).

The benchmark manifest now records this limitation. MCRB-16 requires raw monotonic ticks, clock-epoch identity, matched publish/observe event pairs and uncertainty intervals before cross-host latency claims can qualify. The existing basic sample-integrity output remains useful as an intermediate gate, not as a complete clock-evidence verifier.
