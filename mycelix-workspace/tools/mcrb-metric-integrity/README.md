# MCRB Metric Integrity, Offer Ledger and QOPS

Rust-first validation primitives for the Mycelix Coordination and Recovery Benchmark.

This standalone crate contains four related gates:

1. local per-peer duration integrity from raw monotonic ticks;
2. cross-host publish/first-valid-observation timing as uncertainty-bounded intervals;
3. canonical append-only offer-ledger verification;
4. Qualified Operations Per Second (QOPS) from a verified offer ledger and terminal outcomes.

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

## Canonical offer ledger

The offer ledger establishes the workload denominator before operations are dispatched.

Each ordered offer binds:
- run and scenario identity;
- contiguous sequence number and unique operation ID;
- request SHA-256 digest;
- optional unique idempotency key, with a mandatory key when the offer contract declares an external effect;
- scheduled monotonic tick, clock ID and boot epoch;
- operation class;
- exact workload manifest, generator and parameter-set digests.

The envelope binds the clock source, measurement window, tick frequency, entry count, profile digests and a canonical SHA-256 root. The canonical encoding is domain-separated, uses length-prefixed strings and fixed-width big-endian integers, and commits the offer entries in sequence order. Ordinary unordered JSON serialization is not used as the digest input.

Use the verifier before QOPS:

```rust
let ledger: OfferLedgerEnvelope = serde_json::from_slice(&raw_ledger_bytes)?;
let verified = verify_offer_ledger(&ledger)?;
let report = compute_qops(&verified, &qops_input, &metric_integrity);
```

The returned VerifiedOfferLedger is opaque; callers cannot construct it or replace its operation-ID set. It supplies the denominator and measurement window directly to QOPS.

A digest proves internal consistency, not generator honesty. The raw ledger artifact, generator binary/source, scenario manifest, runner/orchestrator identity and evidence custody must be separately bound into the run receipt. Do not call a run independently authentic just because its ledger is hash-consistent.

## Qualified Operations Per Second (QOPS)

```
QOPS = qualified_successes / verified_offer_ledger_measurement_window_seconds
```

QOPS counts only outcomes classified as QualifiedSuccess under the scenario-specific contract. Every verified offered operation must have exactly one terminal outcome.

Correct denials are not business successes. Unknown and failed outcomes remain in the offered denominator. An unoffered operation cannot enter the numerator. Missing, duplicate or unexpected samples suppress the rate. A duplicate external effect, failed hard-correctness gate or non-Valid metric-integrity report suppresses all performance rates.

The QOPS API no longer accepts a caller-supplied expected-operation-ID vector, caller-supplied measurement duration or caller-supplied cross-host requirement. It consumes the opaque verified ledger and derives these values itself. The cross-host requirement is copied from the selected scenario contract into the offer-ledger envelope and committed by the canonical digest, so a caller cannot downgrade a remote-visibility scenario to local-only after the run.

Scenario-specific remote visibility is frozen in the scenario manifest:
- local/offline-first scenarios may keep cross-host timing as NotEvaluated when their contract does not require remote visibility;
- scenarios requiring DHT/peer visibility cannot earn QOPS unless cross-host timing is Valid and the aggregate is eligible.

## Tests and execution state

The source currently contains 48 authored Rust unit tests across local duration, cross-host interval, offer-ledger and QOPS logic, plus 3 artifact-contract integration tests. These counts are source inventory, not execution results.

No exact-head Cargo test, rustfmt, Clippy, live conductor, DHT convergence, provider-effect observation or production qualification is claimed until the corresponding exact-head workflow and runtime evidence complete successfully.

