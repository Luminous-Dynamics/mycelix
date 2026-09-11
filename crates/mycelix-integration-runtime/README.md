# mycelix-integration-runtime

`mycelix-integration-runtime` is the durable causal-state layer for the Mycelix integration plane.

It is deliberately **not** a connector, provider SDK wrapper, authority oracle, provider capability, or domain-state engine.

## Reference boundary

The v0.1 reference implementation uses SQLite to make crash, restart, concurrency, bootstrap, migration, reconciliation, and storage-repair semantics executable before provider SDKs or money-moving connectors are introduced.

The active public crate root is `src/runtime.rs`:

```text
src/runtime.rs
      |
      +-> bootstrap_guard.rs
      |      bootstrap-profile-v1
      |      unversioned ownership + zero-history qualification
      |
      +-> storage_guard.rs
      |      one-transaction initialized-store hardening
      |             |
      |             +-> schema_manifest.rs
      |                    structural-manifest-v2
      |
      +-> v31.rs
      |      storage-semantics v3.1 facade
      |
      v
src/lib.rs
      preserved v2 SQLite causal engine
```

The layered boundary preserves the established crash/retry/history machinery while making durable meaning explicit and fail-closed.

## Five independent storage identities

INT-03 does not use one version marker as proof of every other property.

```text
semantic producer
  = mycelix-integration-runtime/semantic-profile-v3.1

declared SQLite generation
  = PRAGMA user_version = 2

concrete SQLite structure
  = mycelix-integration-runtime/structural-manifest-v2

bootstrap admission rules
  = mycelix-integration-runtime/bootstrap-profile-v1

enforcement / reconstruction machinery
  = mycelix-integration-runtime/enforcement-profile-v5
```

Therefore:

```text
schema emptiness != bootstrap authority
same schema version != same concrete structure
same structure != same semantic producer
bootstrap eligibility != initialized-store enforcement
same trigger name != same enforcement semantics
valid storage != CurrentExecutionAuthority
```

The inherited INT-02 identifier types also validate through Serde, so typed historical records cannot smuggle identifiers that their public constructors would reject.

## Fresh bootstrap is not database annexation

`bootstrap-profile-v1` distinguishes a genuinely fresh database from an unversioned or partially initialized database that merely looks convenient to reuse.

Before the legacy classifier may touch a `user_version = 0` database, `bootstrap_guard::prepare_unversioned_file_store()` acquires a SQLite `IMMEDIATE` transaction and checks for user schema objects.

```text
user_version = 0
+ no user table/index/trigger/view
    -> may stamp structural-v2 bootstrap intent

user_version = 0
+ any user schema object
    -> do not mutate
    -> strict admission / fail closed
```

The check excludes SQLite-owned `sqlite_%` objects but treats every user table, index, trigger, or view as evidence that the file is not a pristine Mycelix bootstrap target.

This closes the earlier ambiguity where `no integration_outbox` could accidentally mean “safe to adopt.” An unversioned historical Mycelix fragment and an unrelated empty SQLite application database both remain unmodified; their `user_version` stays 0 and no Mycelix semantic singleton is created.

## Partial-v2 bootstrap recovery is schema-qualified

A structural-v2 database is not automatically a Mycelix bootstrap candidate either. Before semantic bootstrap metadata may be completed, the existing user schema must contain only recognized Mycelix bootstrap objects.

Recognized bootstrap surfaces are the managed runtime tables, the expected runtime indexes, and the expected managed trigger names. User views and foreign tables/indexes/triggers are not bootstrap-qualified.

```text
user_version = 2
+ only recognized Mycelix bootstrap objects
+ zero durable runtime rows
+ no relevant prior AUTOINCREMENT activity
+ semantic producer absent/empty OR exact v3.1
    -> partial bootstrap may resume

user_version = 2
+ foreign user schema
    -> no semantic adoption
    -> strict admission / fail closed
```

The zero-history qualifier also checks the known durable runtime tables and relevant `sqlite_sequence` activity. An empty outbox alone is therefore insufficient.

```text
schema marker exists != schema complete
outbox empty != runtime empty
zero live rows != no prior sequence history
bootstrap recovery != migration authority
bootstrap recovery != foreign-database adoption authority
```

These checks are local admission rules, not cryptographic proof that storage was never modified.

## Concrete structural-manifest-v2

`PRAGMA user_version = 2` is only a declaration. Before initialized-store reconstruction, INT-03 qualifies the actual SQLite structures it relies on.

`structural-manifest-v2` checks the relevant table columns/order/types/nullability/defaults/PK positions, rowid/AUTOINCREMENT behavior, exact singleton checks, UNIQUE and primary-key index sets, foreign-key relationships/actions, required named indexes, managed-trigger identities, optional quarantine shape, and post-hardening foreign-key integrity.

It also rejects unqualified structural modifiers that change equality or write semantics, including unexpected `COLLATE`, table-level `ON CONFLICT`, deferrable constraints, partial UNIQUE indexes, expression indexes, unknown managed-table triggers, and unqualified CHECK constraints.

Representative same-version attacks are:

```text
user_version = 2 + wrong same-name claim index
    -> reject

user_version = 2 + extra UNIQUE constraint
    -> reject

user_version = 2 + missing required history index
    -> reject

user_version = 2 + unknown trigger on a managed table
    -> reject before repair
```

This is structural qualification, not cryptographic storage attestation.

## Atomic startup hardening — enforcement profile v5

For an initialized file-backed runtime, the critical storage hardening section is one SQLite `IMMEDIATE` transaction:

```text
BEGIN IMMEDIATE
    -> structural-manifest-v2 PRE qualification
    -> semantic-profile-v3.1 verification
    -> structural schema-v2 verification
    -> orphan-history rejection
    -> exact INT-02 typed history validation
    -> exact owning command + connector subject validation
    -> provider-operation conflict detection
    -> deterministic derived-binding reconstruction
    -> security-trigger replacement
    -> enforcement-profile-v5 recording
    -> structural-manifest-v2 POST qualification
COMMIT
```

The pre- and post-manifest checks run on the **same transaction object** as reconstruction and trigger replacement. A competing SQLite writer cannot commit a schema mutation between the qualified pre-state and repair/post-qualification phases.

`hardening_transaction_atomicity.rs` additionally forces a post-manifest failure after repair has begun and proves the attempted trigger/profile updates roll back with the transaction.

Process-local v3.1 caches are loaded only after the durable hardening transaction commits successfully.

```text
atomic startup hardening != lifetime storage integrity
SQLite write exclusion != cryptographic tamper resistance
```

## Typed history is evidence; derived indexes are machinery

Append-only execution and reconciliation histories are the historical source material. `integration_runtime_operation_binding` is a derived enforcement index.

Reconstruction requires exact INT-02 deserialization and exact subject binding:

```text
execution bytes
  -> ExternalExecutionOutcome
  -> constructor-valid nested identifiers
  -> operation.command_id + connector_instance
  -> exact owning outbox subject

reconciliation bytes
  -> ReconciliationResult
  -> constructor-valid nested identifiers
  -> operation.command_id + connector_instance
  -> exact owning outbox subject
```

Malformed, foreign-subject, or orphan history fails closed. Missing or stale derived rows are rebuilt from consistent history. Conflicting provider identities do not get a winner selected.

```text
None -> Some(A)      may refine
Some(A) -> Some(A)   compatible
Some(A) -> Some(B)   conflict
Some(A) -> None      cannot erase established identity
```

Provider-operation identity is not provider authority, success, reconciliation, or a world postcondition.

## Causal execution vocabulary

```text
AuthorityChecked
      +-- local policy denial -------------> AuthorityDenied -> Finalized
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
      |  future INT-04 authority/materialization boundary
      v
DispatchStarted
      +-- proven pre-commit rejection ----> RejectedBeforeCommit -> Finalized
      +-- confirmed provider effect ------> Confirmed ----+
      +-- commit uncertainty -------------> Ambiguous ----+--> Reconciled -> Finalized
```

Important distinctions remain:

```text
AuthorityDenied != RejectedBeforeCommit
Claimed != Dispatched
Timeout != NotCommitted
RejectedBeforeCommit != CommitUnknown
Reversible != SafeBlindRetry
Compensatable != SafeBlindRetry
IdempotencyKeyPresent != EndpointIdempotencyGuarantee
```

## Attempt fencing, liveness, and history

Each claim receives an exact `ExecutionAttemptId`. Worker identity alone cannot complete another attempt. The v0.1 profile permits at most 1024 attempts per outbox entry; attempt generations survive restart and are never reused.

Exhausted older commands do not head-of-line block unrelated eligible work. Post-dispatch expiration preserves ambiguity instead of manufacturing a retry-safe result.

Execution and reconciliation histories are bounded and append-only. Exact late evidence may remain appendable after resolution/finalization without rewriting terminal state; wrong-attempt or wrong-operation evidence remains rejected.

A poisoned recovery entry can be quarantined without turning one local safety failure into a global queue outage.

## Per-entry causal time

Runtime transition/observation time is locally monotonic per outbox entry. File-backed stores combine fresh durable-frontier reads with SQLite `BEFORE` triggers at the actual write boundary.

The atomic fences cover outbox `updated_at_ms`, execution observation `observed_at_ms`, and reconciliation `recorded_at_ms`.

This is a Mycelix-local causal coordinate, not a global or provider clock. Provider source timestamps remain separate evidence.

## Reconciliation checkpoint currentness

Connector-wide checkpoints use an atomic compare-and-set contract under one `IMMEDIATE` transaction:

```text
no checkpoint                  -> insert
new_time < stored_time         -> reject rollback
same time + identical content  -> idempotent
same time + different content  -> conflict
new_time > stored_time         -> advance
```

The provider cursor itself remains opaque. INT-03 does not infer lexical/numeric cursor order or provider-history completeness.

`load_reconciliation_checkpoint_snapshot()` exposes both the cursor and its local `updated_at_ms` coordinate.

## Authority and payload-materialization boundary

Durable authority provenance cannot recreate fresh execution authority after restart.

`ExecutionClaim` intentionally omits executable command bytes and the durable authority commitment. Claiming queue work is not enough to materialize a provider-ready payload.

The intended INT-04 boundary remains:

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
valid database != permission to act
```

## Qualification target

Promotion requires one **exact-head hosted** qualification of the current source. The workflow includes formatting, compile-without-run, the full runtime corpus, Clippy with warnings denied, provider-neutral dependency closure, inherited INT-02 identifier validation, and named adversarial families covering bootstrap, semantic migration, structural manifests, attempts/fences, operation identity, typed history, causal time, checkpoints, reconstruction, enforcement repair, and poison isolation.

Bootstrap qualification explicitly freezes:

- concurrent pristine first-open convergence;
- unversioned historical Mycelix state remains unstamped and untagged;
- an unrelated unversioned schema remains unstamped even when empty;
- an unrelated structural-v2 schema receives no Mycelix semantic identity;
- partial-v2 recovery for empty/exact semantic metadata;
- durable-history and prior-AUTOINCREMENT rejection;
- foreign semantic-profile preservation.

The workflow separately ratchets the atomic v5 initialized-store ordering:

```text
BEGIN IMMEDIATE
  < pre-manifest
  < first repair mutation
  < enforcement-profile-v5 record
  < post-manifest
  < COMMIT
```

A green hosted run proves only that the exact source satisfied that workflow profile. It does **not** establish current execution authority, provider correctness, exactly-once external effects, global-clock correctness, provider-history completeness, institutional legitimacy, lifetime storage integrity, cryptographic tamper-proofness, or physical-world postconditions.
