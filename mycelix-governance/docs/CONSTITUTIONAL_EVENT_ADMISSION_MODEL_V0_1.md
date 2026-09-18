# Constitutional Event Admission Model v0.1

Status: **pure reference model / experimental / unqualified**  
Tranche: **MYC-CONST-003D1D-E1B**  
Tracks: **#1648**  
Parent: **MYC-CONST-003D1D-E1A** semantic head `249753017c3781f7a3259061aae16d7e0075e984`

## Purpose

E1A defined the cross-lineage predicates a future constitutional-event DHT provider must satisfy. E1B makes those predicates executable under adversarial write sequences without opening a Holochain persistence surface.

The model answers:

> If a malicious caller could directly propose event entries or action-key links, what state transitions must the integrity layer accept, reject, preserve, or halt?

It is intentionally a pure Rust state machine. It performs no HDK/HDI host calls and registers no Holochain entry or link types.

## Core state

The reference state contains only constitutional admission truth:

```text
events_by_action[action_id] -> DurableConstitutionalEvent
action_index[action_id]     -> event_commitment
halted_actions[action_id]    -> IntegrityFault
```

The stable uniqueness key is always the D1C/E0 `action_id`.

There is no timestamp-generated provider identity and no last-write-wins event slot.

## Explicit admission evidence

Admission never accepts an `authorized: bool` argument.

A candidate must carry an `AdmissionEvidence` object containing:

```text
claim_binding_reference
claim_target_digest
claim_payload_digest
qualification evidence for:
    003B4
    003CR1
    003D1C
    003D1D-E0
    003D1D-E1A
admission evidence commitment
```

Each dependency names the exact semantic subject the model understands. A semantic-head change requires a new model revision rather than silently inheriting compatibility.

Qualification state is explicit:

```text
Qualified { receipt_id }
Pending
Failed { evidence_id }
```

Admission requires the exact dependency census and requires every dependency to be `Qualified` with a non-empty retained receipt reference.

`Pending`, `Failed`, missing, duplicate, extra, or wrong-subject evidence fails closed.

The receipt references in tests are deliberately test-only placeholders. E1B does not claim that the currently queued repository workflows have passed.

## Evidence binding

The admission-evidence commitment is domain-separated and covers:

```text
schema version
event commitment
ClaimBinding reference
ClaimBinding target digest
ClaimBinding payload digest
sorted dependency IDs
exact semantic heads
qualification state
receipt/failure evidence IDs
```

This means a qualification receipt set cannot be transplanted from one semantic event onto a different event while retaining the same evidence commitment.

## Exact target derivation

E1B implements the E1A `EventTargetDescriptorV1` convention directly:

```text
domain = MYCELIX-CONSTITUTIONAL-EVENT-TARGET\0V1\0

fields in exact order:
    EmitEvent
    operation_id
    action_id
    proposal_id
    event_name
```

Each field is encoded as `u64 big-endian length || UTF-8 bytes` and the descriptor receives a self-describing BLAKE3-256 commitment.

Admission requires:

```text
claim_target_digest
    == derived EventTargetDescriptorV1 commitment
```

The caller cannot supply a target descriptor and ask the model to trust it.

## Exact payload binding

Admission also requires:

```text
claim_payload_digest
    == DurableConstitutionalEvent.payload_commitment
```

The E0 payload commitment already covers deterministic canonical JSON.

Changing event parameters therefore requires a different authorized payload digest.

## Author binding

The admission boundary receives an authenticated author DID separately from the event body.

It requires:

```text
authenticated_author_did
    == event.authority.publisher_did
```

An unauthenticated publisher claim inside the event is insufficient.

A spoofed direct write is rejected before it can create or halt canonical event state.

## First admission

For an absent action identity:

```text
qualified exact evidence
        +
matching authenticated author
        +
matching target/payload/ClaimBinding
        |
        v
store immutable event
        |
        v
store canonical action_id -> event_commitment index
        |
        v
Created
```

The reference transition creates the logical event and canonical index together.

A state containing an event without its canonical index, an index without its event, or a mismatched index target violates the model invariant.

## Duplicate admission

For an already-admitted action:

```text
same action_id
same event semantic commitment
valid exact evidence
    -> ExistingSame
```

The model does not create a second event or second logical index.

Wall-clock retry time remains outside event identity through E0.

## Conflicting event

For the same action identity with different event semantics:

```text
preserve original event
preserve original canonical index
record ConflictingEvent fault
mark action IntegrityHalted
reject candidate
```

The candidate does not overwrite constitutional history.

The halt is monotonic in E1B. Even replaying the original good event afterward is rejected until a future, separately governed recovery protocol exists.

This is intentional. E1B does not invent recovery authority.

## Action-key link attacks

The logical action index is part of the constitutional uniqueness contract, not merely a query optimization.

A direct index write is valid only when:

```text
an admitted event already exists for action_id
and
target == that event's canonical commitment
```

Repeating that exact index is idempotent.

An alternate target:

```text
preserves canonical index
records ConflictingActionIndex fault
halts action
rejects alternate target
```

An index cannot be created before a durable event exists.

Deleting the canonical index is always rejected in this model.

## Immutability

Constitutional event updates are always rejected.

A changed constitutional fact must be a new authorized action identity, not an update to the prior action's event.

This is separate from ordinary mutable bridge events elsewhere in the system.

## Projection isolation

E1B reuses E0's projection semantics in tests:

```text
admitted event
    + failed subscriber signal
    -> event remains admitted
```

Projection attempts are not included in `EventAdmissionState` at all. They cannot create, replace, delete, or halt constitutional event truth.

## State invariants

`validate_state()` independently requires:

1. every admitted event validates under E0;
2. map key equals the event provider/action key;
3. every event has exactly the canonical logical index target;
4. every logical index has an admitted event;
5. index target equals the event commitment;
6. a conflicting-event halt preserves the original event commitment;
7. a conflicting-index halt preserves the canonical event commitment;
8. fault action identity equals the key under which the halt is stored.

The model never repairs an invalid state by selecting the latest record.

## Adversarial test surface

The crate tests at least:

- first qualified event commit;
- pending qualification rejection;
- failed qualification rejection;
- missing qualification dependency rejection;
- wrong semantic qualification subject rejection;
- spoofed publisher/direct-write rejection;
- wrong ClaimBinding reference rejection;
- wrong target digest rejection;
- wrong payload digest rejection;
- admission-evidence commitment tampering;
- exact duplicate idempotency;
- conflicting payload halt with original preservation;
- independently authenticated conflicting publisher halt;
- post-conflict replay blocked;
- event update rejection;
- canonical index deletion rejection;
- duplicate exact index idempotency;
- alternate index target halt;
- index-before-event rejection;
- signal projection failure not altering admission truth.

## Relationship to current queued evidence

E1B's `Qualified` test fixtures are hypothetical model inputs. They are not statements about current CI.

At the time E1B was authored, the relevant exact hosted qualifiers remained queued, including B4 R1, CR1, D1C, E0, and E1A.

Runtime activation therefore remains blocked regardless of whether this reference model later compiles and passes its own tests.

## Future Holochain refinement

A future E1 runtime tranche should map these logical reference operations onto HDI/HDK validation:

```text
admit_event
    -> validate CreateEntry<ConstitutionalEvent>

canonical action_index
    -> validate RegisterCreateLink<ActionToConstitutionalEvent>

attempt_update_event
    -> reject UpdateEntry<ConstitutionalEvent>

attempt_delete_action_index
    -> reject RegisterDeleteLink<ActionToConstitutionalEvent>
```

The concrete Holochain implementation must then prove that its distributed operation semantics refine this reference behavior, including malicious coordinator bypass and concurrent/conflicting writes.

That last concurrency/refinement proof is intentionally not claimed by E1B.

## Non-claims

E1B does not establish:

- B4 qualification;
- CR1 qualification;
- D1C qualification;
- E0 qualification;
- E1A qualification;
- real retained qualification receipt IDs;
- Holochain persistence;
- Holochain concurrency behavior;
- a Holochain entry or link type;
- a callable write extern;
- recovery authority from `IntegrityHalted`;
- live `GovernanceAction::EmitEvent` repair;
- deployment currentness.

It establishes only an executable reference contract for the admission semantics that later runtime code must preserve.
