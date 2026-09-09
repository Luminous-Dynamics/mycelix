# mycelix-integration-runtime

`mycelix-integration-runtime` is the durable causal-state layer for the Mycelix integration plane.

It is deliberately **not** a connector, provider SDK wrapper, authority oracle, provider capability, or domain-state engine.

## v0.1 reference boundary

The reference implementation uses SQLite to make failure and restart semantics executable before distributed infrastructure or provider SDKs are introduced.

The active crate root is `src/v31.rs`. It wraps the preserved v2 causal engine in `src/lib.rs` and adds the semantic safeguards required before INT-04:

- durable semantic-producer/profile identity separate from SQLite schema identity;
- fail-closed migration when legacy state meaning is underdetermined;
- durable monotonic provider-operation identity, including bindings learned only from late historical evidence;
- per-entry Mycelix runtime causal-time monotonicity;
- lease expiry that independently removes current completion power before recovery runs;
- entry-local quarantine so one poisoned recovery record cannot globally block unrelated work.

The legacy engine remains intentionally intact underneath the facade so its frozen crash, retry, bounded-history, attempt-budget, and state-machine regressions continue to run.

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

The active facade therefore persists:

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

The semantic producer identity changed from the earlier draft v3 marker when provider-operation identity became first-class durable state. The runtime does not silently change durable semantics while retaining the old producer identity.

Pre-split INT-02 represented both authority denial and provider rejection with one generic `Rejected` state. An untagged non-empty legacy runtime therefore cannot be promoted into current semantics merely because its rows remain structurally readable. v3.1 fails closed and requires an explicit provenance-bearing migration/import path.

Unknown or tampered semantic producer identities also fail closed on reopen.

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

## Provider-operation identity is durable and monotonic

`ExternalOperationRef.provider_operation` may begin as `None` before a provider exposes a stable operation ID. Once an accepted observation establishes `Some(A)`, that identity becomes durable protocol state in `integration_runtime_operation_binding`.

```text
None -> Some(A)      may refine
Some(A) -> Some(A)   compatible
Some(A) -> Some(B)   conflict / reject
Some(A) -> None      cannot erase the binding
```

For file-backed stores, SQLite triggers establish/check the binding in the same transaction that appends provider execution or reconciliation evidence. This prevents a crash between evidence admission and identity binding from weakening the invariant.

The binding survives finalization and reopen, including when `Some(A)` was learned only from late non-applying historical evidence. A later `Some(B)` remains a mismatch after restart.

Provider-operation identity is **not** provider authority, success, reconciliation, or a physical-world postcondition.

## Per-entry runtime causal time

Runtime transition/observation time is locally monotonic for each outbox entry:

```text
claim >= durable creation
DispatchStarted >= claim
runtime result observation >= dispatch
reconciliation >= prior runtime transition
finalization >= reconciliation
```

Clock rollback fails closed instead of strengthening an old fence or creating impossible causal order.

This does **not** impose Mycelix clock order on remote provider source timestamps. Provider clocks remain separate evidence and may differ because of skew or transport delay.

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

Promotion requires one exact-head hosted qualification establishing the legacy engine invariants and the active v3.1 facade:

1. formatting, compilation, full tests, and Clippy with warnings denied;
2. provider-neutral normal transitive dependency closure;
3. non-materializing claims and exact attempt fencing;
4. pre-dispatch safe reclaim and post-dispatch ambiguity;
5. bounded persistent attempts plus unrelated-work liveness;
6. append-only bounded observation/reconciliation histories;
7. fail-closed legacy semantic migration;
8. durable/reopen-stable semantic producer identity and tamper rejection;
9. expired post-dispatch fence demotion before completion;
10. provider-operation substitution rejection, including historical-only binding across reopen;
11. per-entry causal-time rollback rejection;
12. poison-entry isolation with durable quarantine visibility;
13. no provider execution API in INT-03.

A green hosted run proves only that the exact source satisfied that workflow profile. It does **not** establish provider correctness, current execution authority, exactly-once effects, global-clock correctness, institutional legitimacy, or physical-world postconditions.
