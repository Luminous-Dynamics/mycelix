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
- exact-operation execution outcomes;
- append-only execution observations;
- append-only reconciliation history;
- reconciliation checkpoints;
- reconciliation-before-finalization;
- bounded payload/history admission;
- fail-safe storage-schema migration.

It does **not** own:

- provider HTTP/API clients;
- webhook authentication;
- Holochain zome calls;
- institutional authority;
- capability minting;
- provider idempotency claims;
- provider payload materialization;
- domain acceptance;
- physical postcondition truth.

## Claim is not dispatch

The runtime deliberately separates acquiring durable work from crossing the external-effect boundary:

```text
OutboxCommitted
      |
      v
AttemptPrepared
      |
      |  lease expires before dispatch
      +-------------------------------> OutboxCommitted
      |
      |  INT-04 fresh execution authority
      |  + qualified provider profile
      |  + exact payload materialization
      v
DispatchStarted
      |
      +-- confirmed receipt ----------> Confirmed
      +-- explicit rejection ---------> Rejected
      +-- uncertain / timeout --------> Ambiguous
      +-- worker disappears ----------> Ambiguous
                                          |
                                          | conclusive reconciliation
                                          v
                                      Reconciled
                                          |
                                          v
                                       Finalized
```

This distinction matters because a worker crash before `DispatchStarted` proves that this runtime never crossed its local external-call boundary. The attempt can therefore be reclaimed even for an irreversible command.

After `DispatchStarted`, absence of a response does **not** prove the provider did nothing. The result becomes `Ambiguous` and stays out of the executable queue until a conclusive reconciliation record exists.

```text
Claimed != Dispatched
Timeout != NotCommitted
Ambiguous != Rejected
Reversible != SafeBlindRetry
Compensatable != SafeBlindRetry
IdempotencyKeyPresent != EndpointIdempotencyGuarantee
```

## Exact attempt fencing

Each claim receives a distinct `ExecutionAttemptId` bound to the outbox entry and monotonically increasing attempt count.

Execution completion must present the exact attempt identity. Reusing a worker ID after restart is insufficient.

A stale attempt therefore cannot attach its result to a later attempt. A late result for the same attempt after timeout is preserved as an execution observation but does not silently rewrite an `Ambiguous` case into a terminal state.

## Unknown is preserved

`ReconciliationDisposition::StillAmbiguous` is a reconciliation observation, not a resolution.

It therefore:

- remains in `OutboundStage::Ambiguous`;
- cannot enable `Finalized`;
- is appended to reconciliation history;
- may be followed by additional reconciliation attempts.

The runtime may cache the latest reconciliation on the outbox row for inspection, but the append-only history is the authoritative durable record of reconciliation attempts.

## Authority and payload-materialization boundary

`DurableOutboundIntent.authority_commitment` is durable provenance only. It cannot recreate fresh execution authority after restart.

Likewise, `ExecutionClaim` intentionally omits `command_bytes` and the durable authority commitment. Claiming queue work is not enough to obtain a provider-ready payload.

The intended INT-04 composition is:

```text
DurableOutboundIntent
      |
      v
ExecutionClaim / AttemptLease       INT-03
      |
      + fresh CurrentExecutionAuthority
      + qualified provider execution/replay profile
      + exact command/target/attempt binding
      v
provider payload materialization    INT-04
      |
      v
mark DispatchStarted
      |
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

The runtime can persist an idempotency key, but the existence of that key says nothing about whether a provider:

- honors it;
- scopes it correctly;
- retains it long enough;
- exposes it for reconciliation;
- returns the same semantic result on replay.

Those properties belong to a versioned, qualified provider profile. Consequently crash recovery defaults to `ManualReview` rather than inferring `IdempotencyKey` reconciliation support from stored data.

## Storage evolution

Runtime schema v2 adds prepared-attempt and dispatch state plus append-only histories.

The original reference schema had no `PRAGMA user_version` and encoded numeric stage `4` as `Executing`. In v2 numeric stage `4` is `AttemptPrepared`.

Reinterpreting an existing stage-4 row as merely prepared would be unsafe because the old database cannot prove whether external dispatch occurred. The migration therefore maps legacy `Executing` conservatively to v2 `Ambiguous`, clears its old worker/lease ownership, and requires reconciliation.

```text
legacy Executing
    != v2 AttemptPrepared

legacy Executing
    -> v2 Ambiguous
```

Unknown future runtime schema versions fail closed rather than being interpreted under v2 semantics.

## Bounded reference profile

The SQLite implementation applies explicit v0.1 limits to inbound payloads, command material, serialized outcomes/reconciliations, and per-entry observation/reconciliation histories. These are reference implementation admission bounds, not immutable universal protocol constants.

A future PostgreSQL/distributed implementation should satisfy the same behavioral contract while declaring its own compatible admission profile rather than redefining the causal semantics.

## Qualification target

Before INT-03 is promoted, exact-head hosted qualification should establish at least:

1. identical inbound redelivery is idempotent, while content or normalization substitution is rejected;
2. one command ID cannot be rebound to another connector or command representation;
3. claims expose attempt metadata but not executable command bytes;
4. pre-dispatch crash safely returns the work to the durable queue;
5. post-dispatch crash becomes `Ambiguous` regardless of reversible/compensatable labeling;
6. stale attempts cannot complete later attempts, including same-worker reuse;
7. late results are preserved without erasing ambiguity;
8. rejection/outcome/reconciliation records bind the exact logical operation;
9. `StillAmbiguous` remains ambiguous and reconciliation history is append-only;
10. finalization requires conclusive reconciliation;
11. presence of an idempotency key does not create provider capability semantics;
12. legacy `Executing` storage migrates to `Ambiguous` rather than `AttemptPrepared`;
13. persisted state survives reopen under the versioned schema.

A green hosted run establishes execution of that exact source under the workflow profile. It does not establish provider correctness, current execution authority, exactly-once effects, or physical postconditions.
