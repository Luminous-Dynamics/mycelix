# Durable Effect Journal v1 — lifecycle and fault-injection contract

**Status:** Research/engineering contract. No durable backend implementation or distributed guarantee is claimed.

## Paradigm shift: classify uncertainty instead of retrying it

A caller timeout does not prove that a remote side effect failed. The protected sink may have committed while the response was dropped. Replaying a new operation can duplicate disclosure, replication, payment, backup, or governance mutation.

The journal lifecycle makes that uncertainty explicit:

`Prepared → Reserved → DispatchStarted → Committed`
`Prepared | Reserved → Rejected` (only if dispatch never began)
`DispatchStarted → OutcomeUnknown → Committed | Rejected` (only after authenticated sink reconciliation)
`DispatchStarted → Committed | Rejected` (only with definitive authenticated sink evidence)

`Reserved` means no dispatch has begun; it can be safely aborted if authority is revoked before dispatch. Once `DispatchStarted` is durably recorded, timeout/crash uncertainty becomes `OutcomeUnknown`; neither `OutcomeUnknown` nor a local error authorizes a new logical operation. Reconciliation queries the destination by the original idempotency key / operation ID and exact target digest. This is the core shift from retry logic to evidence-driven recovery.

## Composition boundary — reuse the existing durable-effect line

This regional state machine is a **conformance/reference model**, not a second production outbox or commit classifier. Before production wiring, it must compose with the existing [PR #780 runtime claim commit-classification tranche](https://github.com/Luminous-Dynamics/mycelix/pull/780) and [AGENT-009 composition tracker #803](https://github.com/Luminous-Dynamics/mycelix/issues/803).

Preserve the existing shared persistence verdicts:

- `Committed`
- `DefinitelyNotCommitted`
- `IndeterminateCommit`

Do not classify a transaction using returned error text/code. Do not treat `DefinitelyNotCommitted` as retry authorization; fresh regional policy and current authority must independently allow the next attempt. Treat `IndeterminateCommit` as reconciliation-only. Keep local persistence truth separate from external provider effect truth. Production integration should join these existing qualified semantics rather than copy their transaction/witness machinery into this crate.

## Rust reference model

`src/effect_journal.rs` provides a deterministic in-memory state machine, opaque `VerifiedSinkOutcome`, a verifier adapter trait, stable transition errors, and seventeen unit tests. It does not persist state or dispatch network requests. Test adapters are mocks, not cryptography.

Production requirements:

1. **Durable uniqueness:** enforce a unique constraint on (authority domain, idempotency key), with exact target and policy-binding digests. The reference identity and destination evidence both include the authority domain to prevent cross-community key collisions/replay. Reuse with different parameters is a hard conflict, never a new operation.
2. **Write-ahead intent:** durably commit `Prepared`, then `Reserved` plus the stable downstream operation ID before any network dispatch.
3. **Dispatch fencing:** atomically compare-and-swap `Reserved → DispatchStarted` before network I/O or use sink-native fencing; no stale worker may dispatch after a successor takes over.
4. **Reconcile unknown:** query the authoritative destination with the same key and target; accept only authenticated evidence tied to that operation and digest.
5. **Atomic terminal receipt:** durably record the verified receipt/result before reporting success to the caller.
6. **Retention and privacy:** scope and expire keys deliberately; persist only opaque operation identifiers and digests, never record plaintext, personal identifiers, consent bodies, or raw credentials.
7. **Observability:** count `OutcomeUnknown`, reconciliation latency, conflicts, replays, and rejected stale revisions without emitting private targets.

An idempotency key by itself is not exactly-once delivery. The destination must participate in the idempotency/result lookup contract. Without it, a crash between remote commit and local journal update cannot be solved by local memory alone.

## Fault-injection corpus

| ID | Fault | Expected behavior |
|---|---|---|
| EJ-001 | New identity | Create Prepared revision 0 |
| EJ-002 | Reserve from Prepared | Persist stable operation ID without dispatching |
| EJ-003 | Attempt to dispatch from Prepared, or restart dispatch from OutcomeUnknown | Reject invalid transition; unknown is reconciliation-only |
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
| EJ-017 | Sink proof is replayed across authority domains using the same key/target | Reject proof and do not resolve the attempt |

Current pure source tests cover the prepared/reserved/dispatch-started/unknown/terminal transitions, matching proof checks, policy-binding replay rejection, invalid proof, operation-ID mismatch, and snapshot restore invariants. The crash, multi-worker, durable-store, destination integration, privacy, and idempotent-result semantics remain external qualification gates.

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


## Machine-readable fault corpus

[`EFFECT_JOURNAL_FAULT_CORPUS_V1.json`](EFFECT_JOURNAL_FAULT_CORPUS_V1.json) is the 25-case fault inventory; [`EFFECT_JOURNAL_FAULT_CORPUS_V1.schema.json`](EFFECT_JOURNAL_FAULT_CORPUS_V1.schema.json) defines its closed JSON structure. `implemented_reference_model` and `implemented_reference_test` are source-level availability labels only, not executed PASS evidence. The remaining rows are explicit integration gates.


## Snapshot dispatch-history invariant

Restored records include a monotonic `dispatch_started` marker. A `Rejected` snapshot that crossed dispatch must carry a verified sink outcome digest; a caller-provided local timeout/reason cannot turn a dispatched-but-uncertain effect into a terminal rejection. This closes an ambiguity that a revision number alone cannot resolve. Test anchor: `restore_rejects_postdispatch_rejection_without_sink_evidence`.

The companion `src/effect_journal_model.rs` contains six source-level model tests. The bounded model explorer checks all 2,520 order-preserving four-worker load/reserve schedules and also enumerates reachable abstract lifecycle states to depth 8 and checks effect-count/dispatch-count invariants, including both forms of `OutcomeUnknown` (destination may or may not have applied the effect). This remains abstract state exploration, not real storage proof.


## Next qualification rung: real concurrency and crash behavior

The current four-worker enumerator explores all 2,520 order-preserving **logical load/reserve event schedules**. It does not create actual threads or validate a database transaction.

When concrete synchronization or durable-store adapter code is introduced, advance the ladder:

1. Use [Loom](https://github.com/tokio-rs/loom) to permute actual Rust concurrent executions around the adapter's synchronization/CAS boundary, with state reduction to keep the search tractable.
2. Use the existing integration-runtime transaction classifier and exact-store witness machinery; inject failures at reservation, dispatch-start commit, sink call, and receipt commit boundaries.
3. For a SQLite-backed adapter, build a controlled crash/fault campaign modeled on [SQLite's documented atomic-commit crash tests](https://www.sqlite.org/atomiccommit.html#testing_atomic_commit_behavior), which simulate write-loss/corruption at varied points and verify recovery. A process-kill test alone is not an adequate replacement for the database's durability/fault boundaries.
4. Test destination idempotency with independent sink observations; keep local commit classification separate from provider outcome.

No Loom thread model, SQLite crash harness, actual durable adapter, or provider simulation is implemented in this tranche.
