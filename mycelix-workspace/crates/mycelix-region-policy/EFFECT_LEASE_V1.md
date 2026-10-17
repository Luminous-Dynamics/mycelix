# Effect Lease v1 — decision-to-side-effect qualification

**Status:** Draft reference contract. The local Rust model is not a production authorization service.

## Paradigm shift

A policy result is not itself proof that a later write, replica, backup, telemetry event, or disclosure is still authorized. The boundary must bind policy to the exact effect, make freshness explicit, and treat uncertain outcomes as non-retryable until reconciled.

This tranche adds a small Rust reference gate with five properties:

1. **Last-boundary re-evaluation:** a fresh timestamp and current verified wrappers are required when consuming the lease.
2. **Exact-effect binding:** action, resource ID, content digest, request ID, recipient, and purpose must match the lease and the independently observed sink target.
3. **Policy-version binding:** agreement and both profile IDs, versions, and digests must still match the lease.
4. **Single use:** the nonce enters the in-memory consumed set before the effect callback is called.
5. **Uncertain outcome is not replayed:** if the callback errors, the nonce stays consumed.

The lease is opaque inside the process and has a hard maximum TTL of 5,000 ms. The adapter supplies a fresh 256-bit random lease ID encoded as 64 lowercase hexadecimal characters; the crate deliberately does not pretend to provide a cryptographic random generator.

## Explicit limits

The reference ledger is process-local, volatile, and single-instance. It does not survive crashes, coordinate between nodes, authenticate a serialized token, or make a remote side effect atomic. Its callback ordering is a safety model, not exactly-once distributed execution.

Production requirements before claiming the stronger guarantee:

- Reconstruct the verified profile, agreement, classification, consent, and current revocation evidence through trusted adapters at the effect boundary. Passing old verified wrappers does not refresh external evidence.
- Persist lease consumption and the effect in one storage transaction when possible. Otherwise implement a sink-native monotonic fencing/version protocol and document the unavoidable uncertainty window.
- Store the nonce with a uniqueness constraint that is enforced across retries, processes, and nodes.
- On crash/restart or timeout after sending, report OutcomeUnknown and reconcile with the destination. Never resend solely because the caller did not receive a response.
- Independently derive the observed target from the real sink operation; do not copy it from the request object and call that independent observation.
- Instrument every sink, including DHT/gossip, replicas, caches, indexes, backups, logs, telemetry, export adapters, and retry/recovery paths.

## Conformance cases

| ID | Probe | Required outcome |
|---|---|---|
| EFX-001 | Current allowed request and exact observed target | Effect callback executes once |
| EFX-002 | Resource, action, digest, recipient, purpose, or request ID mismatch | Deny before callback |
| EFX-003 | Decision timestamp differs from trusted boundary time | Deny |
| EFX-004 | Lease expiry instant is reached (now >= expires_at) | Deny |
| EFX-005 | Profile or agreement version/digest changes before consume | Deny; require a newly issued lease |
| EFX-006 | Fresh boundary evaluation is denied due to expiry or revoked/invalid evidence | Deny before callback |
| EFX-007 | Same lease ID is used twice in one ledger | Second use denied |
| EFX-008 | Effect callback returns an error or timeout/unknown result | Nonce remains consumed; no automatic replay |
| EFX-009 | TTL is zero, too large, or would overflow the clock | Deny |
| EFX-010 | Invalid or non-lowercase/non-hex lease ID | Deny |

## Proposed benchmarks — not measured

| Metric | Initial target | Qualification boundary |
|---|---:|---|
| Local issue + consume/re-evaluation overhead | p95 < 1 ms | Native release build; report host, rustc, OS, workload, and exact commit |
| Expired/mismatched/replayed lease rejection | 100% seeded cases denied | Include positive controls to prove the harness is sensitive |
| Replay after callback error | 0 automatic second effects | Inject error before, during, and after simulated commit; no silent retry |
| Crash/restart/multi-node replay | 0 accepted duplicate effects | Requires the future durable/distributed adapter; in-memory tests do not count |
| Restricted-data egress | 0 unauthorized observed egress | Requires instrumented end-to-end sink coverage, not kernel tests |
| Decision-to-effect drift | 100% profile/agreement changes rejected | Mutate IDs, versions, and digests one field at a time |

## Research alignment

- [NIST SP 800-207 Zero Trust Architecture](https://csrc.nist.gov/pubs/sp/800/207/final) separates policy decision and enforcement roles.
- [Zanzibar](https://research.google/pubs/zanzibar-googles-consistent-global-authorization-system/) treats authorization consistency and causal ordering as first-class requirements, not merely fast decision lookup.
- [OPA external data](https://www.openpolicyagent.org/docs/external-data) documents the trade-offs between cached/replicated state and request-time freshness.

These are design references, not evidence that this crate implements their guarantees. No cryptographic adapter, durable nonce ledger, storage transaction, fencing token, end-to-end egress proof, or benchmark result is claimed by this tranche.
