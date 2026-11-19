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
