# Durable Effect Journal v1 — lifecycle and fault-injection contract

**Status:** Research/engineering contract. No durable backend implementation or distributed guarantee is claimed.

## Paradigm shift: classify uncertainty instead of retrying it

A caller timeout does not prove that a remote side effect failed. The protected sink may have committed while the response was dropped. Replaying a new operation can duplicate disclosure, replication, payment, backup, or governance mutation.

The journal lifecycle makes that uncertainty explicit:

`Prepared → Submitted → Committed`
`Prepared → Rejected` (only if dispatch never began)
`Submitted → OutcomeUnknown → Committed | Rejected` (only after authenticated sink reconciliation)
`Submitted → Rejected` (only with definitive authenticated sink rejection)

`OutcomeUnknown` is never permission to submit the effect again. Reconciliation queries the destination by the original idempotency key / operation ID and exact target digest. This is the core shift from retry logic to evidence-driven recovery.

## Rust reference model

`src/effect_journal.rs` provides a deterministic in-memory state machine, opaque `VerifiedSinkOutcome`, a verifier adapter trait, stable transition errors, and twelve unit tests. It does not persist state or dispatch network requests. Test adapters are mocks, not cryptography.

Production requirements:

1. **Durable uniqueness:** enforce a unique constraint on (authority domain, idempotency key), with exact target and policy-binding digests. Reuse with different parameters is a hard conflict, never a new operation.
2. **Write-ahead intent:** commit `Prepared` and the stable downstream operation ID before network dispatch.
3. **Submit fencing:** atomically compare-and-swap the record revision before dispatch or use sink-native idempotency/fencing; no stale worker may submit after a successor takes over.
4. **Reconcile unknown:** query the authoritative destination with the same key and target; accept only authenticated evidence tied to that operation and digest.
5. **Atomic terminal receipt:** durably record the verified receipt/result before reporting success to the caller.
6. **Retention and privacy:** scope and expire keys deliberately; persist only opaque operation identifiers and digests, never record plaintext, personal identifiers, consent bodies, or raw credentials.
7. **Observability:** count `OutcomeUnknown`, reconciliation latency, conflicts, replays, and rejected stale revisions without emitting private targets.

An idempotency key by itself is not exactly-once delivery. The destination must participate in the idempotency/result lookup contract. Without it, a crash between remote commit and local journal update cannot be solved by local memory alone.

## Fault-injection corpus

| ID | Fault | Expected behavior |
|---|---|---|
| EJ-001 | New identity | Create Prepared revision 0 |
| EJ-002 | Submit from Prepared | Persist stable operation ID before dispatch |
| EJ-003 | Attempt second submit from Submitted or OutcomeUnknown | Reject transition |
| EJ-004 | Crash after Prepared but before dispatch | Safe to abort/restart preparation under durable CAS |
| EJ-005 | Crash/timeout after dispatch | Mark OutcomeUnknown; do not resend |
| EJ-006 | Lost reply after destination commits | Reconcile original key; record Committed from verified evidence |
| EJ-007 | Destination definitively rejects | Record Rejected from verified evidence |
| EJ-008 | Wrong key, target digest, or operation ID | Reject proof |
| EJ-009 | Missing, stale, or invalid proof | Fail closed; remain unresolved |
| EJ-010 | Two workers race on same revision | Exactly one CAS transition succeeds |
| EJ-011 | Same key is reused for a different target or policy digest | Hard conflict |
| EJ-012 | Crash after sink commit but before local terminal write | Reconcile and persist same verified receipt; no duplicate effect |
| EJ-013 | Duplicate delivery after terminal commit | Return the same stored result; do not dispatch again |
| EJ-014 | Journal storage unavailable | Do not dispatch a protected effect |
| EJ-015 | Key expiry/tombstone cleanup races with a late retry | No key reuse while the old effect can still be replayed |
| EJ-016 | Sink proof is replayed under a different policy-binding digest | Reject proof and do not resolve the attempt |

Current pure source tests cover the core prepared/submitted/unknown/terminal transitions, matching proof checks, policy-binding replay rejection, invalid proof, operation-ID mismatch, and snapshot restore invariants. The crash, multi-worker, durable-store, destination integration, privacy, and idempotent-result semantics remain external qualification gates.

## Proposed targets (not measured)

- Zero duplicate downstream effects across at least 100,000 deterministic injected crash/timeout schedules.
- 100% of submitted-but-unresolved attempts recover as OutcomeUnknown; zero blind resend.
- 100% of stale-revision/CAS race schedules admit no more than one winner.
- 100% of wrong-key/target/operation proofs rejected.
- p95 local transition overhead below 1 ms in a documented release-build environment; report durable-store latency separately.
- Zero raw plaintext, consent body, personal identity or credential material in journal persistence and diagnostics.
- Every terminal success returned to callers is backed by a durable, destination-verified receipt for the exact operation key and target digest.

## Research

- [AWS Well-Architected — make mutating operations idempotent](https://docs.aws.amazon.com/wellarchitected/2025-02-25/framework/rel_prevent_interaction_failure_idempotent.html) describes token-based result reuse and the difficulty of exactly-once effects in distributed systems.
- [Google's Zanzibar paper (USENIX ATC 2019)](https://www.usenix.org/conference/atc19/presentation/pang) shows why authorization needs causal consistency and must avoid applying stale authorization to newer content; Mycelix still must supply that consistency boundary.
- [IETF Idempotency-Key draft -07](https://datatracker.ietf.org/doc/html/draft-ietf-httpapi-idempotency-key-header-07) provides relevant HTTP vocabulary, but the cited draft expired on 18 April 2026 and is not treated here as a current final RFC.

These sources motivate the design. They are not independent validation of this implementation.
