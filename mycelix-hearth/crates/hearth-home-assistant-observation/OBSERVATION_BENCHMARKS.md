# Home Assistant observation benchmarks (HA-0.2A / HA-0.2B)

**Status: proposed acceptance targets, not measured results.** The pure protocol machine and projection tests do not establish transport performance, freshness under reconnect, or live Home Assistant conformance. Publish measurements only with exact source SHA, hardware/runtime profile, Home Assistant version, input corpus digest, raw sample artifact, and test-run identity.

## Paradigm shift: qualify claims, not features

The benchmark unit is not “Home Assistant connected” or “supports N devices.” It is the smallest reproducible claim: a specific protocol event, received under a known session/subscription identity, either produces the exact permitted semantic observation or fails closed without mutating the projection cache. Performance work may not relax allowlisting, metadata binding, freshness, epoch, or read-only invariants.

The native `subscribe_entities` stream is preferred because Home Assistant sends an initial compact snapshot followed by incremental changes under one filtered subscription. The stream still has separate protocol messages and event frames; the protocol machine gates correlation/order and the projector enforces semantic minimization. See the [official WebSocket API](https://developers.home-assistant.io/docs/api/websocket/) and the [Home Assistant Core handler](https://github.com/home-assistant/core/blob/dev/homeassistant/components/websocket_api/commands.py).

## Proposed release targets

| ID | Benchmark | Proposed acceptance threshold | Status |
|---|---|---|---|
| HAOBS-001 | Protocol transition safety | Zero invalid sequences reach `Live`; run 1,000,000 deterministic event sequences covering wrong IDs, wrong event kinds, stale epochs, duplicate snapshots, terminal replay and out-of-order responses | Not measured |
| HAOBS-002 | Allowlist escape | Zero unallowlisted entities emitted as normalized observations in 1,000,000 generated snapshot/delta cases | Not measured |
| HAOBS-003 | Projection atomicity | Zero partial cache commits on every malformed-state, profile-metadata-drift, group-overlap, or duplicate-add negative vector; pre/post cache digest identical on rejection, including when a later sorted entity invalidates a multi-entity delta or an `a` addition replays an already-cached ID | Not measured |
| HAOBS-004 | Non-semantic freshness | Zero emitted observation frames for 100,000 attribute/context/time-metadata-only updates or exact duplicate state replays when semantic state and admitted metadata are unchanged | Not measured |
| HAOBS-005 | Sensitive-field leakage | Zero seeded attribute/context markers in normalized observations and redacted diagnostics; no credentials enter protocol action, observation profile, receipt, or DHT fields | Not measured |
| HAOBS-006 | Single-delta latency | p95 ≤ 10 ms from an already-decoded one-entity delta through projection and typed mapping on a named Raspberry Pi 5 profile; report p50/p95/p99 | Blocked on transport/runtime measurement |
| HAOBS-007 | Initial snapshot latency | p95 ≤ 100 ms to project and map a 1,024-entity snapshot on the same named hardware, with the message byte ceiling recorded | Blocked on transport/runtime measurement |
| HAOBS-008 | Projector memory ceiling | ≤ 8 MiB retained projector state at 1,024 admitted entities using declared maximum string/metadata bounds; measure allocated/retained bytes separately from JSON parser buffers | Not measured |
| HAOBS-009 | Reconnect currentness | Zero frames accepted from stale session epochs; zero transition to `Live` until the exact subscription's initial snapshot is applied | Not measured |
| HAOBS-010 | Capability/version fallback | If `subscribe_entities` is unsupported, 100% of attempts return explicit unsupported status and do not silently switch to a legacy protocol | Blocked on transport/runtime measurement |

The protocol test harness now enumerates exactly 1,000,000 length-six histories over ten event classes. Focused tests separately cover stale epochs and malformed initial-snapshot ordering. This is bounded executable coverage, not formal proof, and remains unmeasured until the exact-head workflow executes successfully.

All thresholds are engineering hypotheses. Revise them only with recorded baseline evidence, not to paper over a failing test. The safety thresholds are invariants and may not be relaxed for performance.

## Reproducible method

1. Freeze synthetic vector corpus and its SHA-256. Include positive controls, every named negative control, unknown/unavailable sources, malformed compact groups, allowlist misses, stale epochs, and command/subscription ID mismatches.
2. Run deterministic correctness/mutation tests before performance sampling. A mutant that removes an epoch check, profile allowlist, atomic staging, expected unit/device-class comparison, or metadata-only freshness suppression must be killed by a named test. In particular, a downstream mapper rejection must not be allowed to mask a cache mutation that already committed invalid metadata.
3. For each performance workload, perform 10 warm-up iterations followed by at least 100 timed samples per run and 5 independent runs. Report sample distributions rather than only the fastest result.
4. Separate JSON parsing, transport, projector, codec/mapping, and end-to-end latency. Never attribute the pure projector's time to a full WebSocket path.
5. Record hardware model, OS/kernel, Rust toolchain, build profile, allocator, Home Assistant server version, profile size, event size, entity count, source SHA, corpus SHA, run ID, raw output and any excluded samples.
6. Keep payload-size and JSON-depth limits in the transport contract. Projector-side caps do not replace pre-parse frame limits.

## Claim ceiling

A future PASS may establish these assertions only for the tested source, corpus, server version and hardware. It does not prove Home Assistant is trustworthy, prove physical device state, qualify physical actuation, establish household authority, or replace Mycelix Hearth's adapter-admission, exclusive-lease, durable-journal and reconciliation path. No benchmark is marked measured until the executable harness emits exact-head evidence.
