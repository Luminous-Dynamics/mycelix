# WEB-LEASE-001B — Durable CAS Store + Commit Reconciliation Corpus v0.1

Status: **FROZEN CONTRACT CORPUS / NOT IMPLEMENTED / NOT PASS**

Tracks issue #2735.

Exact parent semantic product:

`8490c8816e05bc99f2ead9361036be6ce1930b46` — WEB-LEASE-001A pure transition planner / PR #2731.

Profiles frozen by this corpus:

- store contract: `mycelix:web-attempt-lease-store:v1`
- transition commitment: `mycelix:web-attempt-lease-transition-commitment:sha256:v1`
- durable record commitment: `mycelix:web-attempt-lease-record-commitment:sha256:v1`

## Why this layer exists

The pure planner can say what transition is semantically allowed, but that is not evidence that storage changed.

```text
TransitionPlanV1 accepted
!= durable state changed
!= CAS succeeded
!= token may be minted
```

There is also a harder ambiguity:

```text
storage transaction committed
+
caller never received acknowledgement
```

is possible in real systems.

Therefore:

```text
storage call returned error
!= mutation definitely did not commit
```

A safe one-shot network authority cannot treat every error as retryable.

## Mutation identity

Every attempted persistent transition carries an exact `mutation_id`.

Its reconciliation identity binds at least:

```text
authority_key
expected_store_revision
mutation_id
next_record
store_contract_profile
```

A deterministic transition commitment MUST bind those semantic inputs using a separately versioned canonical encoding.

```text
same mutation_id
+ different transition commitment
-> MutationIdentityConflict
```

The mutation ID is not by itself an authority token and is not sufficient evidence of a commit.

## CAS request

Conceptually:

```text
CompareAndSwapRequestV1 {
    authority_key,
    expected_store_revision,
    mutation_id,
    next_record,
    transition_commitment,
    store_contract_profile,
}
```

For `Absent -> Prepared`, the request uses explicit create-if-absent semantics rather than pretending `Absent` is revision -1.

## Commit result algebra

A backend adapter must preserve distinctions at least as strong as:

```text
Committed
AlreadyCommittedSameMutation
RevisionConflict
MutationIdentityConflict
IndeterminateStorageOutcome
CorruptState
```

Backend-specific errors may refine this algebra but may not collapse ambiguous and known-not-committed outcomes into one retryable error.

### `Committed`

The exact requested mutation durably committed according to the named backend durability profile.

### `AlreadyCommittedSameMutation`

The exact mutation was already committed. This is reconciliation, **not a second write**.

### `RevisionConflict`

The requested expected revision does not match current durable state and the mutation is not exact reconciliation of an already-committed mutation.

### `MutationIdentityConflict`

The mutation ID is already associated with a different authority key or different transition commitment.

### `IndeterminateStorageOutcome`

The caller cannot safely determine whether the transaction committed.

This result emits **no execution authority** until the same mutation is reconciled.

### `CorruptState`

Current state/history cannot be interpreted under the backend profile. Corruption never becomes `Absent`, `Prepared`, or retry authority.

## Token minting theorem

The store itself does not grant arbitrary network access. A separately typed post-commit adapter may mint the pure planner's named effect only after exact confirmation of:

```text
Committed(exact mutation)
OR
AlreadyCommittedSameMutation(exact mutation)
```

Never after:

```text
IndeterminateStorageOutcome
RevisionConflict
MutationIdentityConflict
CorruptState
```

For `Started`, this means:

```text
indeterminate durable result
-> no Started authorization
-> no socket operation
```

## Lost-ack reconciliation

After an ambiguous result the caller MUST retain the original mutation identity and reconcile it.

```text
lookup_mutation(authority_key, mutation_id, transition_commitment)
```

The caller must not invent `mutation_id + 1` and semantically retry the transition.

A reconciliation API needs outcomes at least as strong as:

```text
CommittedExact
NotObserved
Conflict
CorruptOrIndeterminate
```

`CommittedExact` may permit recovery of the exact post-commit effect.

`NotObserved` is deliberately **not universally defined as safe-to-retry**. Whether absence is final enough to retry depends on the chosen backend's transaction/durability theorem and must be qualified there.

This keeps the abstract contract honest across local databases, remote stores, replicated stores, and future backends.

## Crash after commit, before token construction

Token creation must be recoverable from durable state + exact mutation binding rather than from ephemeral process memory.

```text
commit confirmed
+ process crashes before token constructed
-> restart
-> reconcile exact mutation
-> recover exact post-commit effect
```

This does not mean a token is persistently replayable forever. Token lifetime/consumption semantics remain governed by lease state, exact revision, epoch and downstream connector composition.

## Concurrency theorem

For distinct mutations M1 and M2 against the same authority key and expected revision N:

```text
M1(expected=N) || M2(expected=N)
-> at most one commits revision N+1
```

The loser must observe conflict or exact reconciliation, never an independent parallel history.

Create-if-absent has the same theorem:

```text
Absent
+ concurrent create M1/M2
-> exactly one Prepared history
```

## Revision integrity

The durable contract rejects:

- revision regression;
- skipped/incorrect next revisions under the named planner/store profile;
- authority-key substitution;
- mutation-ID reuse with different transition bytes;
- corrupt current-state encoding;
- corrupt history;
- current/history mismatch when the backend profile promises both.

`store_revision` remains distinct from `lease_epoch`.

## Current state + mutation history

A first durable backend SHOULD preserve both:

```text
current state index
+
append-only committed mutation history
```

because this makes lost-ack reconciliation and forensic recovery materially stronger than an overwritable current row alone.

This corpus does not dictate SQL tables or a particular database.

## Commitment boundaries

### Transition commitment

Used to prove that a repeated mutation ID refers to the exact same intended CAS transition.

### Durable record commitment

Identifies the resulting durable record under a named storage/canonicalization profile.

Neither is interchangeable with the B1 admission commitment:

```text
admission commitment
!= lease transition commitment
!= durable record commitment
```

Likewise:

```text
valid record commitment
!= storage engine actually flushed to hardware
```

The backend durability profile must say what a successful commit establishes.

## Crash / failure qualification matrix

A concrete backend must independently test at least:

1. crash before transaction begin;
2. crash after history staging but before current-state staging;
3. crash after current-state staging but before transaction commit;
4. commit succeeds but acknowledgement is dropped;
5. acknowledgement succeeds but process crashes before post-commit token construction;
6. exact mutation reconciliation after restart;
7. competing mutation after an ambiguous result;
8. disk-full / I/O failure;
9. current-state corruption;
10. mutation-history corruption;
11. current/history mismatch;
12. concurrent create-if-absent;
13. concurrent CAS from the same revision.

A transactional backend may make some intermediate states impossible. Qualification must demonstrate that rather than infer it from API documentation alone.

## Machine-readable corpus

The fixture contains 24 contract cases covering:

- create-if-absent and exact replay;
- normal CAS and exact replay;
- mutation-ID substitution;
- stale revisions;
- concurrent mutation races;
- lost acknowledgement after commit;
- exact mutation reconciliation;
- prohibition on inventing a new mutation after ambiguity;
- indeterminate `Started` with no socket authority;
- post-restart token recovery;
- `NotObserved` conservatism;
- current/history corruption;
- revision regression;
- authority-key substitution;
- known pre-commit storage error.

Exact SHA-256 over the re-read UTF-8 fixture bytes:

`5c9b7dc399fb6b0934c9433e5cc69ab50c5aca5067892f28a53426112c2bed9e`

This digest identifies only the authored corpus and does not qualify an implementation.

## Backend split

The intended development sequence is:

```text
WEB-LEASE-001A
pure transition planner
        ↓
WEB-LEASE-001B
this abstract CAS/reconciliation contract
        ↓
WEB-LEASE-001C
first concrete durable backend
        ↓
WEB-LEASE-001CQ
backend crash/concurrency qualification
        ↓
post-commit token adapter qualification
        ↓
WEB-CONNECT-001
```

Do not add a public socket connector simply because this contract exists.

## Backend selection

SQLite may be a good first local backend because a capture worker needs local durable transactions more than distributed consensus. But this corpus intentionally does not choose SQLite merely because SQL tooling exists somewhere in the workspace.

The concrete backend must be selected and qualified on its actual transaction/durability properties.

## Nonclaims

This corpus does not establish:

- a database or filesystem implementation;
- CAS atomicity in any real store;
- durability across power loss;
- global consensus;
- B0/B1/B2 correctness;
- qualified token minting;
- connector ordering;
- network safety;
- TLS/source identity;
- HTTP correctness;
- source authenticity;
- content truth;
- EPI admission.

## Qualification discipline

```text
store contract frozen
!= backend implemented
!= atomicity demonstrated
!= crash durability demonstrated
!= token minting qualified
!= connector authorized
!= PASS
```
