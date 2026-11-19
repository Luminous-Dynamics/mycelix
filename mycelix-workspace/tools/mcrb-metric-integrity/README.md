
# MCRB Metric Integrity and QOPS

Rust-first validation primitives for the Mycelix Coordination and Recovery Benchmark.

This crate contains three independent measurement gates:

1. local per-peer duration integrity from raw monotonic ticks;
2. cross-host publish/first-valid-observation timing as uncertainty-bounded intervals;
3. Qualified Operations Per Second (QOPS) accounting from complete offered-operation IDs and terminal outcomes.

It is a measurement/verifier component, not a runtime adapter, workload generator, clock authority, source-attestation service or external-effect observer.

## Local duration integrity

The verifier derives durations from raw monotonic start/end ticks and requires the sample clock ID and clock epoch to match the peer's declared clock observation.

It rejects reversed ticks, zero tick frequency, clock ID/epoch mismatches, wall-clock duration declarations, missing peer clock observations, incomplete peer/counter coverage, duplicate per-peer operation IDs, unexpected peers, non-finite or beyond-window durations, and invalid clock uncertainty values.

Outliers inside the declared valid duration window are preserved and reported by their original sample indexes. They are not trimmed, clipped, winsorized or silently excluded. Aggregate p50/p95/p99 values are emitted only when the full local batch is valid and complete. Per-peer distributions are retained so a slow peer is not hidden by pooling.

## Cross-host publish/observe timing

The cross-host validator requires an exact expected tuple: logical operation ID, record digest, publisher peer and observer peer.

It requires one calibration per participating peer, checks calibration identity and validity windows, rejects missing/duplicate/unexpected event pairs, and uses both clock offsets with combined uncertainty to derive a latency interval.

A point estimate is emitted only when combined uncertainty is within the declared limit. A local-only batch reports cross-host timing as NotEvaluated, not PASS. Cross-host incomplete/invalid state is separate from local per-peer duration status.

The helper validates calculations and structural consistency only. It does not authenticate the clock source, calibrator, event origin, record digest or run identity. These inputs must be bound to signed/raw artifacts in the run receipt before a cross-host result can support a benchmark claim.

## Qualified Operations Per Second (QOPS)

QOPS counts only outcomes classified as QualifiedSuccess under the scenario-specific contract:

    QOPS = qualified_successes / fixed_measurement_window_seconds

Every expected offered operation must have exactly one terminal outcome. The reducer reports qualified success, correct denial, unexpected denial, known failure, unknown outcome and duplicate external effect.

Correct denials are not business successes. Unknown and failed outcomes remain in the offered-operation denominator. An unoffered operation cannot enter the numerator. Missing, duplicate or unexpected samples suppress the rate. A duplicate external effect, failed hard-correctness gate or non-Valid metric-integrity report suppresses all performance rates.

The reducer consumes the actual IntegrityReport, not a caller-provided metric-valid boolean. The offered-operation ID list must still be bound to an independently generated append-only offer ledger, workload source digest, seed and measurement-window identity; the reducer cannot prove a caller did not fabricate a smaller expected set.

## Public API

    let report = compute_qops(&input, &metric_integrity);
    if report.status == QopsStatus::Eligible {
        let qops = report.qualified_operations_per_second.expect("eligible QOPS");
        // Bind QOPS and all outcome counts to the exact run receipt.
    } else {
        // Preserve issues and withhold performance rates.
    }

Build the verifier from the standalone workspace:

    cargo test --manifest-path mycelix-workspace/tools/mcrb-metric-integrity/Cargo.toml
    cargo clippy --manifest-path mycelix-workspace/tools/mcrb-metric-integrity/Cargo.toml --all-targets -- -D warnings

## Status and claim limits

- Local Valid means required local peer clocks, durations and counters are coherent.
- Local Incomplete means expected observations/samples/counters are missing.
- Local Invalid means an impossible or out-of-contract measurement was observed.
- Cross-host NotEvaluated is not PASS.
- QOPS Eligible means the expected offered-ID set is fully classified, hard correctness gates passed, and the supplied metric-integrity report is Valid.
- QOPS Incomplete means outcome coverage is incomplete.
- QOPS Invalid means malformed/duplicate/unoffered outcomes or a non-compensatory gate failed.

Neither cross-host metric integrity nor QOPS alone proves that source events, run metadata, workload manifests, clock calibrations or external effects are authentic. The receipt verifier must bind those sources independently.

## Tests and execution state

The source contains 36 authored Rust unit tests across local duration, cross-host interval and QOPS logic. This count is source inventory, not an execution result.

No exact-head Cargo test, rustfmt, Clippy, live conductor, DHT convergence, provider-effect observation or production qualification is claimed until the corresponding exact-head workflow and runtime evidence complete successfully.


## Scenario-specific remote-visibility requirement

QOPS input includes an explicit cross-host-measurement requirement sourced from the frozen scenario manifest:

- If the scenario is local/offline-first and its contract does not require a remote observation, the report may keep cross-host timing as NotEvaluated.
- If the scenario requires peer visibility/convergence, QOPS is invalid unless the metric-integrity report has CrossHostTimingStatus::Valid and cross_host_aggregate_eligible=true.

The caller must not decide this dynamically after observing the run. The requirement is part of the scenario's versioned contract and is copied into the run receipt.


The QOPS input includes cross_host_measurement_required, sourced from the frozen scenario manifest. When true, the reducer requires cross-host status Valid and cross_host_aggregate_eligible=true. When false, local-first workflows may leave cross-host timing NotEvaluated.
