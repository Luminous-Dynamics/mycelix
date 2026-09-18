# Constitutional Event Admission Crosswalk v0.1

Status: **PendingQualifiedCrosswalk / activation blocked / unqualified**  
Tranche: **MYC-CONST-003D1D-E1A**  
Tracks: **#1639**  
Parent: **MYC-CONST-003D1D-E0** semantic head `36a4fcffb6ca806570ebf439f9c36cf76401e7b2`

## Purpose

E0 froze durable constitutional event semantics without opening a Holochain write surface. E1A answers the next question without contaminating that evidence boundary:

> What exactly must a future integrity zome prove before an `EmitEvent` action may become durable constitutional truth?

E1A is a **cross-lineage admission specification**. It binds exact Git objects from four otherwise separate lines of evidence:

- B4 ClaimBinding semantics;
- D1C constitutional effect-ledger identity;
- E0 durable event semantics;
- CR1 refinement-crosswalk scaffold.

It deliberately does **not** merge those branches or inherit their qualification states.

## Why this remains separate from runtime code

The E0 lineage does not contain the B4 ClaimBinding crate. That is intentional evidence hygiene rather than a missing dependency.

Pulling an unqualified B4 branch into E0 merely to reuse the Rust type would create a mixed lineage before the B4 and CR1 receipts have passed inspection.

E1A therefore works from exact immutable Git objects:

```text
B4 semantic head 037f61c1...
        |
        | exact source blob
        v
ClaimBinding semantics

D1C semantic head 47d1d764...
        |
        v
operation/action identity

E0 semantic head 36a4fcff...
        |
        v
durable event semantics

CR1 scaffold 4edb56bd...
        |
        v
refinement status / pending extensions
```

The independent validator resolves those objects directly with Git and fails on blob drift.

## Admission model

A future DHT constitutional event is admissible only if all of these layers agree:

```text
Qualified constitutional authority
        |
        v
exact B4 ClaimBinding
        |
        +------ target_digest ------> exact event target
        |
        +------ payload_digest -----> exact canonical event payload
        |
        v
D1C ConstitutionalOperation
        |
        v
D1C ActionIntent[EmitEvent]
        |
        v
E0 DurableConstitutionalEvent
        |
        v
DHT author / immutable entry / action-key index
```

No individual layer is allowed to substitute for the others.

## Source-bound identity continuity

The following relationships are already source-visible in D1C + E0 and therefore classified `LocallySourceBound`:

```text
E0 operation_id
    == D1C ConstitutionalOperation.operation_id

E0 proposal_id
    == D1C ConstitutionalOperation.proposal_id

E0 action_id
    == D1C ActionIntent.action_id

E0 action_commitment
    == D1C ActionIntent.action_commitment

E0 claim_binding_commitment
    == D1C ConstitutionalOperation.claim_binding

E0 provider key
    == D1C ActionIntent.action_id
```

The E0 integration test constructs authority directly from those D1C fields and explicitly rejects an action commitment that does not match the operation plan.

E1A therefore forbids regenerating any of these identities at the provider boundary.

## B4 semantics that must survive admission

B4 ClaimBinding covers every security-relevant component of the constitutional claim:

```text
claim_id
envelope_digest
nonce
use_index
jurisdiction
matter.namespace
matter.stable_id
target_digest
payload_digest
budget_id
```

Its canonical bytes are domain-separated and include the exact target and payload digests.

E1A does not reduce this to `claim_id` or a generic proposal reference.

A future event admission must remain bound to the exact B4 claim body.

## Event target refinement

B4 defines `target_digest` semantically as:

> commitment to the exact target/resource authorized by the envelope

E1A defines the required `EmitEvent` target convention as a future refinement:

```text
EventTargetDescriptorV1 {
    provider_lane = "EmitEvent",
    operation_id,
    action_id,
    proposal_id,
    event_name,
}
```

The descriptor uses:

```text
domain = "MYCELIX-CONSTITUTIONAL-EVENT-TARGET\0V1\0"
encoding = u64-length-prefixed UTF-8 fields
field order = provider_lane, operation_id, action_id, proposal_id, event_name
digest = blake3-256:<64 lowercase hex>
```

Required future relationship:

```text
ClaimBinding.target_digest
    == EventTargetDescriptorV1.commitment
```

This mapping is currently **PendingQualifiedCrosswalk**.

It is an E1 convention. It is not retroactively claimed to be part of B4, whose target digest is deliberately algorithm-agnostic.

## Event payload refinement

B4 defines `payload_digest` as:

> commitment to the exact effect payload/parameters

E0 already produces a deterministic canonical JSON payload and a self-describing BLAKE3-256 payload commitment.

The required future relation is therefore:

```text
ClaimBinding.payload_digest
    == DurableConstitutionalEvent.payload_commitment
```

That binds constitutional authorization to the exact canonical event parameters rather than merely the event name.

This relationship also remains **PendingQualifiedCrosswalk** until an exact-head refinement tranche proves it.

## ClaimBinding reference refinement

D1C currently carries:

```text
ConstitutionalOperation.claim_binding: String
```

E0 carries the same opaque value as:

```text
EventAuthorityBinding.claim_binding_commitment
```

B4 provides stable canonical ClaimBinding bytes but deliberately leaves hash/signature algorithm selection outside its crate.

Therefore E1A does **not** invent and silently bless a hash convention for this field.

A future qualified integration profile must freeze the exact relationship between:

```text
B4 ClaimBinding.canonical_bytes()
        <->
D1C ConstitutionalOperation.claim_binding
        <->
E0 EventAuthorityBinding.claim_binding_commitment
```

before activation.

## DHT integrity boundary

Coordinator checks are insufficient for constitutional truth because a malicious or buggy caller may bypass a coordinator and attempt a direct DHT write.

E1 activation must enforce at integrity validation:

```text
entry.publisher_did
    == DID(action.author)
```

and must reject:

```text
publisher mismatch
unqualified ClaimBinding/refinement evidence
wrong operation/action identity
wrong target digest
wrong payload digest
mutable constitutional event updates
conflicting event under the same action key
unvalidated action-key link
link deletion that removes canonical lookup
```

The action-key index must not merely be an optimization. It is part of the uniqueness/admission contract.

## Duplicate semantics

E0's event semantics carry forward unchanged:

```text
same action identity
same qualified semantic event
    -> ExistingSame

same action identity
different target / payload / authority / event semantics
    -> IntegrityConflict
```

There is no constitutional:

```text
Overwrite
Replace
LastWriteWins
```

A conflicting duplicate is evidence of a fault, not a newer version of the event.

## Projection remains outside admission truth

Signal delivery remains downstream projection:

```text
admitted durable constitutional event
        |
        v
emit_signal / subscriber projection
```

Signal success, loss, retry, duplicate delivery, or unknown delivery outcome cannot create or revoke constitutional completion.

Operational projection diagnostics may be audited, but they are not authority evidence.

## Current qualification ceiling

The frozen E1A profile records these exact states at observation time:

```text
003B4 R1   508e81ea... run 35344734164  queued
003CR1     651eb0c0... run 35319268716  queued
003D1C     c4397f4b... run 35349600389  queued
003D1D-E0  55c9e1d5... run 35357940683  queued
```

Therefore:

```text
activation_allowed = false
status = PendingQualifiedCrosswalk
activation_status = Blocked
```

A later PASS does not mutate this historical profile into qualified evidence. A successor exact-head profile must incorporate inspected retained receipts.

## Independent validator

`tools/formal/validate_constitutional_event_admission.py` independently verifies:

1. exact source-binding census;
2. declared and actual Git blob identities;
3. B4 ClaimBinding contains target/payload and all security-relevant fields;
4. D1C carries the required operation/action identities;
5. E0 carries the exact authority fields, action-key provider identity, payload commitment and projection independence;
6. E0's D1C bridge test maps fields without regeneration;
7. the frozen CR1 scaffold still classifies ClaimBinding as pending qualification;
8. target descriptor field census/order and digest convention;
9. payload refinement relation;
10. integrity-bound author, immutability and action-key requirements;
11. non-claim ceiling;
12. activation remains blocked.

Its self-tests must reject mutants that:

- prematurely enable activation;
- remove `action_id` from the target descriptor;
- weaken payload binding to something other than the exact payload commitment;
- replace integrity author binding with coordinator-only authorization;
- drift a bound source blob;
- allow provider-key regeneration;
- allow last-write-wins conflicts;
- mark a dependency qualified without a receipt;
- omit a B4 security-relevant field.

## Required successor before E1 activation

E1A is still not the DHT provider.

The next activation-bearing tranche should happen only after the dependent receipts are available and should add, in order:

```text
1. qualified target/payload refinement profile
2. dedicated immutable ConstitutionalEvent entry
3. integrity-bound author check
4. integrity-validated ActionId -> event index
5. delete/update rejection
6. coordinator publish-or-return-existing path
7. signal projection only after durable commit
8. direct-DHT adversarial tests
9. exact-head qualification
10. only then redirect GovernanceAction::EmitEvent
```

## Non-claims

E1A does not establish:

- a Holochain entry type;
- a Holochain link type;
- a live write extern;
- B4 qualification;
- CR1 qualification;
- D1C qualification;
- E0 qualification;
- qualified target/payload refinement;
- active constitutional-event admission;
- live `EmitEvent` repair;
- deployment currentness.

Those omissions are the safety boundary of this tranche, not unfinished claims.
