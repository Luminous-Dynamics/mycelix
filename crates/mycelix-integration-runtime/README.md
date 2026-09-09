# mycelix-integration-runtime

`mycelix-integration-runtime` is the durable causal-state layer for the Mycelix integration plane.

It is deliberately **not** a connector, provider SDK wrapper, authority oracle, provider capability, or domain-state engine.

## v0.1 reference boundary

The reference implementation uses SQLite to make failure and restart semantics executable before distributed infrastructure or provider SDKs are introduced.

It owns:

- durable normalized inbound insertion;
- duplicate-vs-identity-collision detection;
- durable outbound intent storage;
- prepared execution attempts with bounded leases;
- exact attempt/fence identities;
- an explicit local `DispatchStarted` boundary;
- conservative crash recovery;
- exact-operation provider outcomes;
- append-only execution observations;
- append-only reconciliation history, including post-resolution evidence;
- reconciliation checkpoints;
- reconciliation-before-finalization for effect-bearing or uncertain outcomes;
- direct closure for proven no-effect outcomes;
- bounded payload/history/attempt admission;
- fail-safe storage-schema migration.

It does **not** own provider APIs, webhook authentication, Holochain calls, institutional authority, capability minting, provider idempotency claims, provider payload materialization, domain acceptance, or physical postcondition truth.

## Distinct denial, claim, dispatch, and outcome states

The runtime consumes the provider-neutral causal model from INT-02:

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

A worker crash before `DispatchStarted` proves this runtime never crossed its local external-call boundary, so the attempt can be reclaimed even for an irreversible command. After `DispatchStarted`, absence of a response does not prove the provider did nothing; the result becomes `Ambiguous` and remains non-executable until qualified reconciliation resolves it.

A provider response is `RejectedBeforeCommit` only when a qualified provider profile can establish that the exact operation did not commit an external effect. Ordinary transport failures or ambiguous provider errors must remain `Ambiguous`.

## Exact attempt fencing and explicit attempt budget

Each claim receives a distinct `ExecutionAttemptId` derived from the outbox entry and monotonically increasing attempt generation. Reusing a worker ID is insufficient to complete another attempt.

The v0.1 reference profile admits at most **1024 attempts per outbox entry**. Attempt generation is durable across restarts and fails closed with `AttemptBudgetExceeded` before attempt 1025 can be minted. Exhaustion does not mean success, rejection, ambiguity, or fresh authority.

```text
AttemptBudgetExceeded
    != provider outcome
    != reconciliation
    != execution authority
```

The attempt budget is a declared reference-profile resource bound, not an accidental integer ceiling.

## Historical evidence is append-only

Current state transition and historical evidence admission are separate operations.

An exact late provider result for the same attempt may arrive after `Ambiguous`, `Reconciled`, or `Finalized`. The runtime appends it to execution-observation history as non-applying evidence and does not rewrite current state.

Likewise, an exact-operation reconciliation result that arrives after `Reconciled` or `Finalized` is appended to reconciliation history without mutating the terminal state, even when it contradicts the earlier resolution.

```text
historical observation append
    != current-state transition

Finalized
    != history closed to new evidence
```

Wrong-operation and wrong-attempt evidence remain rejected. Later contradictory evidence is therefore preserved for a future review/reopen/impact layer instead of being silently discarded or allowed to mutate history.

## Shared bounded evidence budgets

Both ordinary provider observations and internally generated crash ambiguity consume the same per-entry execution-observation budget. Internal recovery has no unbounded side channel.

The v0.1 SQLite profile bounds:

- inbound normalized payload bytes;
- durable command bytes;
- serialized provider outcomes;
- serialized reconciliation results;
- execution-observation history;
- reconciliation history;
- execution attempt generations;
- worker identifiers and claim batch size.

Budget exhaustion fails closed and does not promote epistemic or execution state.

## Unknown is preserved

`ReconciliationDisposition::StillAmbiguous` is a reconciliation observation, not a resolution. It stays in `OutboundStage::Ambiguous`, cannot enable `Finalized`, and is appended to history.

The outbox row may cache the latest applicable reconciliation for inspection, but append-only histories are the durable causal record. Post-resolution observations do not overwrite that cached historical decision.

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

The runtime may persist an idempotency key, but key presence does not prove the provider honors it, scopes it correctly, retains it long enough, exposes it for reconciliation, or returns semantically equivalent replay results.

Those properties belong to a versioned qualified provider profile. Crash recovery therefore defaults to `ManualReview` rather than inferring idempotency support from persisted data.

## Storage evolution

Runtime schema v2 adds prepared-attempt/dispatch state and append-only histories.

The original reference schema had no `PRAGMA user_version` and encoded numeric stage `4` as `Executing`. In v2, numeric `4` means `AttemptPrepared`. Reinterpreting an old stage-4 row as merely prepared would be unsafe because the old representation cannot prove whether dispatch occurred, so migration maps legacy `Executing` conservatively to v2 `Ambiguous`.

Existing v2 numeric meanings are preserved as the vocabulary becomes more precise: the former definite provider-rejection slot maps to `RejectedBeforeCommit`; the newly distinguished `AuthorityDenied` receives a new numeric slot rather than reinterpreting old stored rows.

Unknown future runtime schema versions fail closed.

## Qualification target

Before INT-03 is promoted, exact-head hosted qualification should establish at least:

1. inbound duplicate/id-collision semantics;
2. immutable command/connector binding;
3. non-materializing execution claims;
4. pre-dispatch safe reclaim and post-dispatch ambiguity;
5. exact attempt fencing, including same-worker reuse;
6. exact-operation binding for every provider outcome;
7. distinct `AuthorityDenied` and `RejectedBeforeCommit` semantics;
8. shared bounded execution-observation history, including crash-generated ambiguity;
9. `StillAmbiguous` preservation and append-only reconciliation;
10. post-`Reconciled`/`Finalized` exact evidence remains appendable without state rewrite;
11. the 1024-attempt budget persists across SQLite reopen and attempt 1025 fails with `AttemptBudgetExceeded`;
12. `Confirmed`/`Ambiguous` require conclusive reconciliation before finalization, while proven no-effect paths can close directly;
13. idempotency-key presence does not create provider capability semantics;
14. legacy `Executing` storage migrates to `Ambiguous`, not `AttemptPrepared`;
15. persisted state survives reopen under the versioned schema.

A green hosted run establishes execution of that exact source under the workflow profile. It does not establish provider correctness, current execution authority, exactly-once effects, or physical-world postconditions.
