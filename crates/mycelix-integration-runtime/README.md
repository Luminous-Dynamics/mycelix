# mycelix-integration-runtime

`mycelix-integration-runtime` is the durable causal-state layer for the Mycelix integration plane.

It is deliberately **not** a connector, provider SDK wrapper, authority oracle, provider capability, or domain-state engine.

## v0.1 reference boundary

The reference implementation uses SQLite to make failure, restart, multi-handle, and storage-repair semantics executable before distributed infrastructure or provider SDKs are introduced.

The public crate root is `src/runtime.rs`:

```text
src/runtime.rs         stable public boundary + cross-handle/atomic fences
      ↓
src/storage_guard.rs   lock-first admission + reconstructable storage enforcement + checkpoint CAS
      ↓
src/v31.rs             storage-semantics v3.1 facade
      ↓
src/lib.rs             preserved v2 SQLite causal engine
```

The layered boundary preserves the already-frozen v2 crash/retry/history machinery while adding the semantic and concurrency safeguards required before INT-04:

- durable semantic-producer/profile identity separate from SQLite schema identity;
- fail-closed migration when legacy state meaning is underdetermined;
- lock-first validation of semantic profile **and structural schema version** before repair;
- durable monotonic provider-operation identity, including late historical evidence;
- typed, subject-bound reconstruction of derived provider-operation indexes from append-only history;
- orphan-history rejection;
- per-entry Mycelix runtime causal-time monotonicity;
- cross-handle freshness checks plus atomic SQLite causal-time write fences;
- monotonic connector-wide reconciliation checkpoints with equal-time conflict detection;
- explicit enforcement-profile identity plus atomic trigger replacement on reopen;
- lease expiry that independently removes current completion power before recovery runs;
- entry-local quarantine so one poisoned recovery record cannot globally block unrelated work.

## Causal execution vocabulary

```text
AuthorityChecked
      +-- policy/authority denial ---------> AuthorityDenied -> Finalized
      |
      v
Approved
      v
OutboxCommitted
      v
AttemptPrepared
      |  lease expires before dispatch
      +-------------------------------> OutboxCommitted
      |
      |  INT-04 fresh execution authority
      |  + qualified provider profile
      |  + exact payload materialization
      v
DispatchStarted
      +-- proven pre-commit rejection ----> RejectedBeforeCommit -> Finalized
      +-- confirmed effect ---------------> Confirmed ----+
      +-- uncertain / timeout ------------> Ambiguous ----+--> Reconciled -> Finalized
```

The distinctions are semantic, not cosmetic:

```text
AuthorityDenied != RejectedBeforeCommit
Claimed != Dispatched
RejectedBeforeCommit != CommitUnknown
Timeout != NotCommitted
Ambiguous != RejectedBeforeCommit
Reversible != SafeBlindRetry
Compensatable != SafeBlindRetry
IdempotencyKeyPresent != EndpointIdempotencyGuarantee
```

## Storage semantics v3.1

SQLite `PRAGMA user_version` describes structural storage compatibility. It does **not** identify which integration semantics produced durable records.

The v3.1 layer persists:

```text
mycelix-integration-runtime/semantic-profile-v3.1
```

in `integration_runtime_semantics` for file-backed stores.

```text
same table shape
    != same semantic meaning

same integer stage
    != same semantic cause

syntactic readability
    != semantic equivalence
```

The producer identity changed from the earlier draft v3 marker when provider-operation identity became first-class durable state. The newer `runtime.rs`/`storage_guard.rs` layers do not reinterpret that durable history; they strengthen admission, reconstruction, and enforcement of the already-declared v3.1 rules.

Pre-split INT-02 represented both authority denial and provider rejection with one generic `Rejected` state. An untagged non-empty legacy runtime therefore cannot be promoted into current semantics merely because its rows remain structurally readable. v3.1 fails closed and requires an explicit provenance-bearing migration/import path.

Unknown or tampered semantic producer identities also fail closed on reopen.

## Lock-first structural admission

For an already initialized file-backed runtime, `storage_guard.rs` acquires a SQLite `IMMEDIATE` transaction before it trusts or repairs durable enforcement state.

Admission first checks the expected semantic producer identity and then the structural schema version. The v3.1 runtime currently requires:

```text
PRAGMA user_version = 2
```

Only after both checks succeed may startup validate history, rebuild derived indexes, replace security-critical triggers, and record the enforcement profile.

```text
semantic-profile-v3.1
+ unsupported structural schema
    -> fail closed
    -X-> repair derived indexes
    -X-> replace triggers
    -X-> rewrite enforcement metadata
```

The hostile structural-admission regression deliberately sets `user_version = 99`, weakens a causal trigger, and tampers enforcement metadata. Reopen must reject before repair, leaving those deliberately corrupted derived artifacts untouched. That test demonstrates ordering of admission and repair; it is not a claim that hostile storage should normally remain corrupted.

## Exact attempt fencing and finite attempt generation

Each claim receives a distinct `ExecutionAttemptId` derived from the outbox entry and monotonically increasing attempt generation. Reusing a worker ID is insufficient to complete another attempt.

The v0.1 reference profile admits at most **1024 attempts per outbox entry**. Attempt generation is durable across restarts and fails closed with `AttemptBudgetExceeded` before attempt 1025 can be minted.

Exhausted older commands are skipped while unrelated eligible work exists, so bounded retry does not become head-of-line denial of service.

## Lease expiry independently removes current completion power

A fence can stop being current even when the recovery sweep has not run yet.

```text
now >= lease_until
    -> current completion power is gone
    -> post-dispatch state preserves Ambiguous
    -> exact late provider result remains historical evidence
    -X-> direct current Confirmed / RejectedBeforeCommit
```

Expiry changes current workflow authority, not evidentiary existence.

## Provider-operation identity: evidence first, derived index second

`ExternalOperationRef.provider_operation` may begin as `None` before a provider exposes a stable operation ID. Once accepted evidence establishes `Some(A)`, that exact identity becomes part of the durable historical record.

```text
None -> Some(A)      may refine
Some(A) -> Some(A)   compatible
Some(A) -> Some(B)   conflict / reject
Some(A) -> None      cannot erase the binding
```

The append-only execution and reconciliation histories are the historical source material. `integration_runtime_operation_binding` is a **derived enforcement index**, not an independent source of truth.

On file-backed reopen, the runtime admits the structural/semantic substrate under an SQLite `IMMEDIATE` transaction, rejects orphan history, deserializes execution/reconciliation rows as exact INT-02 types, validates their embedded command/connector subject against the owning outbox entry, rebuilds the derived binding table, atomically replaces the binding/causal triggers, records the expected enforcement profile, commits, and only then reloads the v3.1 runtime caches from the repaired database.

The inherited INT-02 ID types also validate through Serde, so typed history cannot smuggle identifiers that their public constructors would reject.

```text
valid-looking JSON
    != typed INT-02 history

typed history
+ wrong outbox subject
    -> reject

missing derived row
    != no historical binding

wrong stale derived row
    -> rebuild from consistent history

history establishes only Some(A)
    -> derived index becomes Some(A)

history establishes Some(A) and Some(B)
    -> fail closed
    -X-> choose a winner
```

This closes cache-loss, stale-cache, malformed-history, foreign-subject, and orphan-history cases, including identities learned only from late non-applying historical evidence after finalization.

Provider-operation identity is **not** provider authority, success, reconciliation, or a physical-world postcondition.

## Enforcement machinery has its own identity

Reconstructable SQLite admission/reconstruction/trigger machinery is labeled separately from semantic history:

```text
mycelix-integration-runtime/enforcement-profile-v3
```

A trigger name alone is not accepted as proof that the installed SQL implements the expected safety theorem. Reopen replaces the known security-critical trigger definitions inside the same `IMMEDIATE` startup transaction used for typed derived-state reconstruction.

Enforcement profile v3 includes lock-first structural admission in addition to the typed reconstruction, exact subject binding, orphan rejection, and trigger/index repair introduced by the preceding hardening work.

```text
same trigger name
    != same trigger semantics

semantic producer identity
    != enforcement implementation identity

structural schema identity
    != semantic producer identity
```

This protects against stale/drifted local enforcement definitions. It is **not** a claim that SQLite is cryptographically tamper-proof against an attacker with unrestricted storage access.

## Per-entry causal time and atomic fencing

Runtime transition/observation time is locally monotonic for each outbox entry:

```text
claim >= durable creation
DispatchStarted >= claim
runtime result observation >= dispatch
reconciliation >= prior runtime transition
finalization >= reconciliation
```

For file-backed stores, `runtime.rs` enforces this twice:

1. a fresh durable-frontier read provides an early deterministic rejection for stale handles;
2. SQLite `BEFORE` triggers fence the actual write transaction, closing the race between that read and the write.

The atomic fences cover:

- `integration_outbox.updated_at_ms` updates;
- `integration_execution_observation.observed_at_ms` inserts;
- `integration_reconciliation_history.recorded_at_ms` inserts.

```text
fresh precheck
    != atomic write guarantee

fresh precheck + database trigger
    -> stale concurrent write fails closed
```

This is a per-entry Mycelix causal coordinate, not a global clock. Remote provider source timestamps remain separate evidence and are not forced into this ordering.

## Reconciliation checkpoint currentness

Connector-wide reconciliation checkpoints use a Mycelix local transition timestamp as their anti-rollback coordinate. Provider cursor values remain opaque.

```text
no stored checkpoint
    -> insert

new_time < stored_time
    -> reject rollback

new_time == stored_time
+ exact same cursor + commitment
    -> idempotent

new_time == stored_time
+ different cursor or commitment
    -> conflict / fail closed

new_time > stored_time
    -> may advance
```

The file-backed implementation performs the read/compare/write under one SQLite `IMMEDIATE` transaction, so two open writers cannot race an older or equal-time conflicting checkpoint into storage. In-memory stores implement the same public contract.

`load_reconciliation_checkpoint_snapshot()` returns both the `ReconcileCursor` and `updated_at_ms` so callers do not have to confuse “a cursor exists” with “this is the locally current checkpoint.”

The runtime does **not** lexically or numerically sort arbitrary provider cursor strings. A later Mycelix checkpoint may legitimately contain a cursor whose textual representation sorts before the prior one.

```text
checkpoint currentness
    != provider completeness

checkpoint timestamp
    != provider event timestamp
```

## Historical evidence is append-only

Current state transition and historical evidence admission remain separate operations.

Exact late same-attempt provider observations may remain appendable after `Ambiguous`, `Reconciled`, or `Finalized` as non-applying evidence. Exact-operation reconciliation after resolution/finalization is likewise preserved without rewriting terminal state.

```text
historical observation append
    != current-state transition

Finalized
    != history closed to new evidence
```

Wrong-operation and wrong-attempt evidence remain rejected.

## Bounded history and entry-local quarantine

Provider observations and internally generated crash ambiguity share the same per-entry execution-observation budget. Internal code has no unbounded bypass.

If a stale post-dispatch entry cannot record its ambiguity observation because the observation budget is exhausted, v3.1 isolates that entry in durable `integration_runtime_quarantine` state and removes the stale lease from the normal recovery scan.

```text
entry A safety failure
    -> A quarantined / unresolved / inspectable

entry B independent eligible work
    -> still progresses
```

`quarantine_reason(entry_id)` exposes the durable reason. Quarantine never means success, no-effect, reconciliation, or permission to retry.

## Authority and payload-materialization boundary

`DurableOutboundIntent.authority_commitment` is durable provenance only. It cannot recreate fresh execution authority after restart.

`ExecutionClaim` intentionally omits `command_bytes` and the durable authority commitment. Claiming queue work is not enough to obtain a provider-ready payload.

The intended INT-04 composition remains:

```text
DurableOutboundIntent
      v
ExecutionClaim / AttemptLease       INT-03
      + fresh CurrentExecutionAuthority
      + qualified provider execution/replay profile
      + exact command/target/attempt binding
      v
provider payload materialization    INT-04
      v
mark DispatchStarted
      v
external dispatch
```

Therefore:

```text
OutboxCommitted != executable capability
ExecutionClaim != CurrentExecutionAuthority
stored authority commitment != current authority
stored side-effect class != qualified replay safety
queue presence != permission to act
```

## Provider idempotency remains external

The runtime may persist an idempotency key, but key presence does not prove provider support, scope, retention, payload equivalence, failover behavior, or query/reconciliation capability.

Those properties belong to a versioned qualified provider profile. Crash recovery therefore defaults conservatively to `ManualReview` rather than inferring idempotency support from persisted data.

## Qualification target

Promotion requires one exact-head hosted qualification establishing the legacy engine, v3.1 semantics, stable public concurrency boundary, and reconstructable storage guard:

1. formatting, compilation, full tests, and Clippy with warnings denied;
2. provider-neutral normal transitive dependency closure;
3. inherited INT-02 identifier-deserialization validation;
4. non-materializing claims and exact attempt fencing;
5. pre-dispatch safe reclaim and post-dispatch ambiguity;
6. bounded persistent attempts plus unrelated-work liveness;
7. append-only bounded observation/reconciliation histories;
8. fail-closed legacy semantic migration;
9. durable/reopen-stable semantic producer identity and tamper rejection;
10. lock-first structural-schema-v2 admission before any repair mutation;
11. expired post-dispatch fence demotion before completion;
12. provider-operation substitution rejection, including historical-only binding across reopen and cross-handle visibility;
13. typed exact-subject reconstruction plus orphan-history rejection;
14. missing/stale derived provider-operation index reconstruction from append-only history;
15. conflicting provider-operation history fails closed rather than selecting a winner;
16. per-entry causal-time rollback rejection;
17. atomic database rejection of backdated state/observation/reconciliation writes;
18. reconciliation-checkpoint CAS, equal-time conflict rejection, cross-handle rollback rejection, and reopen stability;
19. weakened same-name security trigger replacement plus enforcement-profile-v3 restoration;
20. poison-entry isolation with durable quarantine visibility;
21. no provider execution API in INT-03.

A green hosted run proves only that the exact source satisfied that workflow profile. It does **not** establish provider correctness, current execution authority, exactly-once effects, global-clock correctness, provider-history completeness, institutional legitimacy, storage tamper-proofness, or physical-world postconditions.
