# mycelix-integration-runtime

`mycelix-integration-runtime` is the durable causal-state layer for the Mycelix integration plane.

It is deliberately **not** a connector, provider SDK wrapper, authority oracle, provider capability, or domain-state engine.

## v0.1 reference boundary

The reference implementation uses SQLite to make failure and restart semantics executable before distributed infrastructure or provider SDKs are introduced.

The active crate root is now `src/v3.rs`. It wraps the earlier v2 engine in `src/lib.rs` and adds the semantic safeguards that must exist at the public boundary before INT-04:

- durable semantic-producer/profile identity separate from SQLite schema identity;
- fail-closed migration when legacy rejection cause is underdetermined;
- provider-operation identity refinement that cannot substitute `Some(B)` for established `Some(A)`;
- per-entry Mycelix runtime causal-time monotonicity;
- lease expiry that independently removes current completion power even before recovery runs;
- entry-local quarantine so one poisoned recovery record cannot globally block unrelated work.

The legacy v2 engine remains intentionally intact underneath the facade so its already-frozen crash, retry, bounded-history, attempt-budget, and state-machine regressions continue to run.

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

## Storage semantics v3

SQLite `PRAGMA user_version` describes structural storage compatibility. It does **not** identify which integration semantics produced a durable row.

The public v3 facade therefore persists:

```text
mycelix-integration-runtime/semantic-profile-v3
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

This matters because pre-split INT-02 represented both authority denial and provider rejection with one generic `Rejected` state. An old generic rejection cannot truthfully be relabeled as either `AuthorityDenied` or `RejectedBeforeCommit` without additional provenance.

The v3 open path therefore fails closed on underdetermined legacy rejection rows and on unknown/tampered semantic producer IDs. New v3 stores persist the producer identity and prove it across reopen.

## Exact attempt fencing and finite attempt generation

Each claim receives a distinct `ExecutionAttemptId` derived from the outbox entry and monotonically increasing attempt generation. Reusing a worker ID is insufficient to complete another attempt.

The v0.1 reference profile admits at most **1024 attempts per outbox entry**. Attempt generation is durable across restarts and fails closed with `AttemptBudgetExceeded` before attempt 1025 can be minted.

Exhausted older commands are skipped while unrelated eligible work exists, so the safety bound does not become a head-of-line denial-of-service mechanism.

## Lease expiry is independently authoritative for currentness

An exact attempt fence can cease to be current even if the recovery sweep has not run yet.

For a post-dispatch attempt:

```text
now >= lease_until
    -> current completion power is gone
    -> state is conservatively demoted to Ambiguous
    -> exact late provider result remains historical evidence
    -X-> direct current Confirmed / RejectedBeforeCommit
```

Expiry affects current workflow authority, not whether the later response is worth preserving.

## Provider-operation identity refines monotonically

`ExternalOperationRef.provider_operation` may begin as `None` before the provider exposes a stable operation ID. Once established, it cannot be silently replaced or erased:

```text
None -> Some(A)      may refine
Some(A) -> Some(A)   compatible
Some(A) -> Some(B)   conflict / reject
Some(A) -> None      cannot erase the binding
```

Command + connector equality is not enough to treat two different provider operation IDs as the same exact operation.

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

If a stale post-dispatch entry cannot record the ambiguity observation because its execution-observation budget is already exhausted, v3 isolates that entry in durable `integration_runtime_quarantine` state and removes it from the normal stale-recovery scan.

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

Promotion requires one exact-head hosted qualification that establishes both the legacy engine invariants and the active v3 facade, including:

1. formatting, compilation, full tests, and Clippy with warnings denied;
2. non-materializing claims and exact attempt fencing;
3. pre-dispatch safe reclaim and post-dispatch ambiguity;
4. bounded persistent attempts plus unrelated-work liveness;
5. append-only bounded observation/reconciliation histories;
6. fail-closed legacy semantic migration;
7. durable/reopen-stable semantic producer identity and tamper rejection;
8. expired post-dispatch fence demotion before completion;
9. provider-operation substitution rejection;
10. per-entry causal-time rollback rejection;
11. poison-entry isolation with durable quarantine visibility;
12. no provider execution API in INT-03.

A green hosted run proves only that the exact source satisfied that workflow profile. It does **not** establish provider correctness, current execution authority, exactly-once effects, global-clock correctness, institutional legitimacy, or physical-world postconditions.
