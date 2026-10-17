# Regional Policy Kernel — Measurement Plan

## Scope of the first benchmark

The native benchmark reports per-operation p50/p95/p99 for the pure, already-verified policy decision path, including exact profile binding, one verified bilateral agreement with two distinct party approvals, and one verified classification claim. It intentionally excludes cryptographic verification of profiles/agreements/classification/consent, revocation-provider lookups, storage, Holochain DHT/gossip, network latency, logging, and adapter work. Those measurements must be reported separately rather than mixed into a single opaque latency number.

## Initial targets (proposed, not measured)

| Measure | Initial target | Qualification rule |
|---|---:|---|
| Pure policy decision latency | p95 < 5 ms | Document CPU, OS, rustc, build profile, batch size and commit |
| Fail-closed policy corpus | 100% seeded negative cases denied for the expected reason | Run positive controls and negative controls together |
| Unknown schema / unrecognized scope | 100% denied | Do not silently map to a default region |
| Profile/consent tamper and replay | 100% of seeded cases rejected by the verifier + kernel composition | Kernel-only test mocks do not count as cryptographic verification |
| Unauthorized restricted-data egress | 0 in integrated end-to-end adversarial runs | Exercise storage, gossip, transport, cache, indexes, backups, logs, telemetry and adapters |
| Offline continuation | 72 hours of declared pilot tasks | Reconnect, reconcile, and verify no policy bypass or lost authorized records |
| Low-bandwidth recovery | Measured by device/network strata | Record artifact size, interrupted-transfer behaviour and successful resume rate |
| Human outcomes | Establish partner-owned baselines before setting targets | Cost/time saved, service access, appeal completion, privacy incidents and accessibility |

## Reproduction

    cd mycelix-workspace/crates/mycelix-region-policy
    cargo bench --bench policy_eval --locked

Record the exact 40-character commit SHA, toolchain, OS/kernel, processor, power/performance mode, compiler profile, total operations, and full stdout. A queued or compiled-only benchmark is not a measurement. Do not compare results across machines without their environment manifests.

## Next benchmark expansion

1. Add verifier microbenchmarks after the production profile-signature adapter and consent/revocation adapter exist.
2. Add placement integration tests that instrument every write/replication/egress boundary and prove restricted classes never enter disallowed paths.
3. Add fault injection: stale or rolled-back profile versions, revoked keys, unavailable trust roots, wall-clock rollback, dropped network, partial federation rollout, cache poisoning, and backup restore.
4. Add a deterministic public fixture corpus with the profile digests, source/destination scopes, requested action, valid-time window, consent evidence state, expected decision, and expected reason codes.
5. Measure end-to-end latency and reliability by data class, destination region, network quality, hardware tier, and payload size. Report p50/p95/p99 plus failure and denial rates.
6. Add effect-lease issue/consume microbenchmarks including fresh reevaluation, TTL rejection, target mismatch, and replay rejection; report the reference ledger separately from any durable/distributed adapter.