# Pulse delivery protocol: benchmark and fault-qualification plan

This plan defines **targets to test**, not measured results. Keep protocol correctness gates separate from performance measurements: a fast incorrect delivery path fails qualification.

## P0 — release-blocking correctness gates

| ID | Measure | Target | Required evidence |
|---|---|---|---|
| B-DEL-01 | False `Delivered` claims | **0** | Adversarial receipts, wrong message/recipient/commitment, stale verifier version, transport-only ACK, timeout, tampered bytes |
| B-DEL-02 | Duplicate application-visible inbox commits | **0** | Same key across routes, attempts, concurrent handlers, event replay, restart and recovery |
| B-DEL-03 | Silently lost accepted durable outbox entries | **0** | Crash/reopen at every write boundary under the explicitly declared storage durability model |
| B-DEL-04 | Stale worker authoritative writes / send authorizations | **0** | Pause old worker, advance epoch, resume old worker at each mutation boundary |
| B-DEL-05 | New sends authorized after deadline or applicable revocation | **0** | Virtual-clock boundary at expiry - 1 tick, expiry, expiry + 1 tick; revocation before/at/after the authorization linearization point |
| B-DEL-06 | Conflicting reuse of logical/event/attempt/idempotency identity accepted as success | **0** | Same key + same commitment; same key + conflicting commitment; unknown key/version |
| B-DEL-07 | Critical model mutants surviving | **0** | 100% of critical safety mutants detected; any surviving mutant invalidates the run |
| B-DEL-08 | Unbounded retry/history growth | **0** | For every declared bound N, test N-1, N, N+1 and overflow recovery |
| B-DEL-09 | Recovery-induced trusted-state divergence | **0** | Replay identical event history 1x, 2x, 10x, 100x and shuffled/reordered allowed delivery sequences; compare canonical authoritative-state digest |
| B-DEL-10 | Unaccounted terminal outcomes | **0** | Every test case ends in a defined evidence-backed success, explicit terminal failure, expired/revoked state, or visible unknown/ambiguous state |

Do not convert a missing observation to an observed absence. Unknown, unavailable, conflict, expired, revoked, and terminal failure must remain distinguishable.

## P1 — fault campaign coverage

Enumerate fault points around every protocol linearization boundary:

1. before and after outbox write/commit;
2. before transport invocation, after invocation, and after remote acceptance but before local response handling;
3. before and after receiver deduplication/inbox commit and application acceptance;
4. before/after receipt creation, persistence, transmission, sender verification, and terminal-state commit;
5. during restart/replay, lease expiry/renewal, policy-generation update, journal-cap enforcement, and schema migration.

For every point, run: process crash, delayed response, lost response, duplicate event, reorder, concurrent duplicate receiver, and stale-worker resumption where applicable. Every discovered counterexample must become a minimized deterministic Rust regression fixture. Report denominator and result per fault point; aggregate percentages must not hide uncovered boundaries.

Use deterministic virtual time and seeded schedules. Record the seed, action trace, expected invariant ID, observed state, source SHA, model/tool versions, schema/config digests, and fixture digest. Do not include message plaintext, direct recipient identity, precise location, or secrets in report artifacts.

## P2 — initial scale matrix

Exercise the pure delivery reducer and test storage seam at the following sizes; these are test dimensions, not claims that a given production scale is supported:

- candidate routes: 1, 2, 8, 32, 64, 256;
- outbox entries / replay events: 1, 10, 100, 1,000, 10,000;
- duplicate deliveries per logical key: 1, 2, 10, 100;
- worker epochs: 1, 2, 5, 10;
- injected crash points per lifecycle: every enumerated boundary, not a random subset only.

Capture p50/p95/p99 latency, throughput, bytes per outbox/inbox/event record, peak memory, journal growth, replay time, recovery time, duplicate suppression count, route availability assumptions, and the count of ambiguous outcomes correctly left unknown. Separate pure reducer time, local durable-store time, Holochain operation time, and end-to-end transport time; never compare different layers as if they were equivalent.

## Provisional performance goals (not release gates until baselined)

- Pure state reduction and invariant evaluation: p95 <= 10 ms at 64 route candidates on a named reference machine.
- Replay of 1,000 compact events: <= 1 s on the same reference machine, with canonical output digest identical to single-application replay.
- Retry/recovery overhead: report p50/p95/p99 and keep ordinary-path p95 regression <= 10% versus the same build with fault instrumentation disabled.
- Memory/history: strict declared caps; no hidden unbounded retry queues. Report bytes and allocations per logical message rather than only process RSS.
- Scale-out: publish the first point where latency or memory becomes superlinear, with an explainable profile rather than a single peak-throughput number.

These thresholds are hypotheses to validate against the first reproducible baseline. If the baseline disproves feasibility, publish the results and revise targets explicitly—never silently relax correctness gates to improve a chart.

## Benchmark protocol / provenance

Each result must bind to exact source commit SHA, tree or working-tree cleanliness, target triple, compiler/toolchain, build flags, CPU/RAM/storage, dataset and seed digests, model/config digests, and exact command. Store raw machine-readable summaries plus minimized failure traces. Label runs independently as queued, running, passed, failed, skipped, or unavailable. A queued run, a skipped workflow, a stale-head result, or a missing receipt is **not a pass**.

The synthetic corpus must include negative controls (tampered receipt, conflicting identity, duplicate cross-route delivery, stale epoch, late/out-of-context receipt, journal limit+1). Keep synthetic assumptions separate from measured traces. Use only consented, minimized traces for real-world calibration; synthetic benchmarks do not establish production reliability or independent route failure domains.
