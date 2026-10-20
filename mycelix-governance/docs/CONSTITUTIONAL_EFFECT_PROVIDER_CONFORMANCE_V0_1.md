# Constitutional Effect Provider Conformance v0.1

Status: **ObservedSourceBound / experimental / unqualified**  
Tranche: **MYC-CONST-003D1D0**  
Tracks: #1623  
Runtime subject: `15b9c89adf0ac3c6c5a73681614d6bfcd368820a`  
D1C semantic predecessor: `47d1d764323dbfaf991b5574cfde83abb7a3e4a4`

## Purpose

D1A established the abstract crash-consistent outbox. D1B separated constitutional operation atomicity from physical batch atomicity. D1C made the append-only execution evidence contract concrete in Rust.

D1D0 answers the next question:

> Do the providers currently targeted by `GovernanceAction` actually satisfy the contracts needed to inhabit that model?

For the exact source subject audited here, the answer is **not yet**.

This tranche freezes that fact without changing production behavior. Each provider lane is independent. No property earned by one provider can be inherited by another.

## Source boundary

The profile binds exact Git blobs for:

- the execution coordinator;
- governance bridge finance routing;
- governance bridge durable event routing;
- legacy transfer input validation;
- the finance treasury coordinator;
- the constitution parameter coordinator;
- constitution integrity validation.

The machine-readable profile lives at:

`specs/constitutional-effect-provider-conformance.v1.json`

and is checked by:

`tools/formal/validate_effect_provider_conformance.py`

The validator re-hashes the bound source files and independently checks key source-level control-flow observations. Its self-tests reject capability inflation.

---

## Lane 1 — TransferCredits / finance

### Current execution path

The current `GovernanceAction::TransferCredits` builds the legacy payload:

```text
from
to
amount
```

and dispatches it to:

```text
governance_bridge::transfer_credits
```

The bridge-side validation visible for this shape checks structural properties such as non-empty accounts and positive finite amount. That is not an idempotency, authorization, or reconciliation contract.

### A second finance route exists

The governance bridge also exposes `execute_approved_transfer`, whose request contains:

```text
proposal_hash
recipient_did
amount_sap
purpose
```

and routes to:

```text
finance role
  -> treasury
      -> execute_governance_transfer
```

That is a better *shape* for governed execution because it carries proposal context, but D1D0 does not infer capability from a request field.

In the exact bound treasury coordinator, no `execute_governance_transfer` extern is observed. The bridge-to-provider contract is therefore not established for this source subject.

### Existing allocation path is useful but not crash-safe

Treasury does expose a stronger primitive:

```text
Allocation.id
  -> indexed lookup
  -> execute_allocation(allocation_id)
```

The allocation identity exists before physical execution and remains queryable. That is exactly the kind of stable provider object a future adapter should prefer.

However, current `execute_allocation` orders its steps as:

```text
read Approved allocation
        ↓
debit_treasury(...)
        ↓
update Allocation -> Executed
```

If the debit becomes durable and the later allocation update fails or execution terminates in between, the observable allocation can remain `Approved` even though the balance changed.

A retry can then pass the same `Approved` precondition and debit again.

Therefore the existence of a stable allocation ID does **not** establish idempotency.

### DKG path has the same ordering class

`execute_dkg_allocation` verifies its threshold signature and then:

```text
debit_treasury(...)
        ↓
construct timestamp-derived allocation id
        ↓
create Executed allocation record
```

The authoritative effect identity is therefore not durably established before the debit.

### Finance classification

```text
Conformance                 Incompatible
Stable effect identity      NotEstablished
Idempotency                 NotEstablished
Authoritative outcome query NotEstablished
Receipt verification        NotEstablished
Provider atomicity          NotEstablished
Authorization binding       NotEstablished
```

### Strong repair direction

A successor finance adapter should use one pre-existing durable identity—preferably a qualified allocation/action identity—before debit occurs.

Conceptually:

```text
Constitutional ActionIntent
      action_id
      exact finance payload commitment
      authorization / ClaimBinding
             ↓
FinanceEffectIntent(action_id)
      status = Pending
             ↓
provider debit using SAME action_id
             ↓
FinanceEffectObservation(action_id)
      KnownSuccess | KnownNoEffect | UnknownOutcome
```

The provider must expose an authoritative lookup by `action_id`. Repeated execution of the same `action_id` must either return the already-observed result or prove no effect occurred before retry.

A transport error is never `KnownNoEffect`.

---

## Lane 2 — UpdateParameter / constitution

### Request-schema mismatch

The execution coordinator currently constructs:

```text
{
  parameter,
  value
}
```

`UpdateParameterInput`, however, also contains:

```text
proposal_id: Option<String>
```

The missing field deserializes to `None`.

The provider's `set_parameter` path explicitly rejects changing an existing parameter without proposal authorization:

```text
existing parameter + proposal_id = None
    -> reject
```

Yet when the named parameter does not already exist, the same request can enter the creation path.

So the current execution action is not merely “an update with weak metadata.” It has asymmetric semantics:

```text
existing governed parameter -> normally denied
missing parameter           -> creation path available
```

### Value encoding mismatch

Integrity validation requires `GovernanceParameter.value` to parse as JSON.

The execution action accepts a plain Rust `String` and forwards it without canonical JSON encoding. Therefore provider compatibility depends on the caller having already supplied a JSON lexical representation.

For example, conceptually:

```text
42       -> valid JSON number
true     -> valid JSON boolean
"hello"  -> valid JSON string
hello    -> not valid JSON
```

D1D0 does not treat that implicit convention as a qualified request schema.

### Integrity authority is not sufficient

The coordinator-level `proposal_id` check is not mirrored by the bound integrity rules for `GovernanceParameter`.

The exact integrity functions accept `_action: Create` / `_action: Update` without using the action author for parameter authorization. Their pure checks validate primarily:

- non-empty name on creation;
- valid JSON value.

The parameter index link is also accepted without a provider-specific governance-authority predicate.

Therefore a future adapter must not claim constitutional authorization merely because the coordinator accepted its request. The authority invariant must survive direct DHT validation paths.

### Repetition is not idempotency

`set_parameter` creates another parameter entry and another parameter-index link. `get_parameter` selects the latest linked record by timestamp.

Repeating the same logical update is therefore not source-visibly the same as observing an existing action result. It can create another historical write.

That may be perfectly acceptable for a versioned parameter ledger, but only if revision identity and action identity are explicit. Timestamp ordering alone is not an exactly-once effect contract.

### Phi config is a derived projection

After the constitutional parameter write, `update_parameter` best-effort synchronizes selected Phi parameters into the governance bridge.

Failure of that sync does not fail the constitutional parameter update.

D1D0 therefore treats bridge Phi config as a **derived projection**, not part of the constitutional parameter effect's completion predicate.

That is desirable once made explicit: the source of truth can remain the durable constitutional parameter record, while caches are reconciled independently.

### Parameter classification

```text
Conformance                 Incompatible
Stable effect identity      NotEstablished
Idempotency                 NotEstablished
Authoritative outcome query Partial
Receipt verification        NotEstablished
Provider atomicity          NotEstablished
Authorization binding       Incompatible
```

### Strong repair direction

Use a typed request bound to the exact constitutional action:

```text
ConstitutionalParameterUpdate {
    operation_id,
    action_id,
    claim_binding,
    proposal_id,
    parameter,
    canonical_value,
    expected_prior_revision,
}
```

and persist a version/revision that is queryable by `action_id`.

Integrity validation should prove the immutable linkage among:

```text
action identity
proposal authority
claim binding
parameter name
prior revision
new canonical value
```

A duplicate submission of the same action identity should return the same committed revision. A different payload under the same action identity must halt or reject as an integrity conflict.

---

## Lane 3 — EmitEvent / signaling

### Current path is not a durable effect

The execution coordinator currently performs:

```text
let _ = emit_signal(...)
return "... [emitted]"
```

The return from `emit_signal` is discarded.

This is useful as a client notification mechanism, but it cannot establish constitutional completion because no durable provider record or authoritative outcome query is created by that action.

### A durable event source already exists

The governance bridge already exposes `broadcast_governance_event`.

That path:

```text
construct GovernanceBridgeEvent
        ↓
create durable event entry
        ↓
link into recent events
        ↓
return stored Record
```

This is a much better authority direction.

Its current event identity is timestamp-derived, so D1D0 still does not call it idempotent. A successor should bind the durable event to the constitutional `operation_id` / `action_id` before client projection.

### Event classification

`EmitEvent` is therefore classified:

```text
ProjectionOnly
```

not as a physical provider effect requiring an exactly-once delivery theorem.

The safer architecture is:

```text
Durable constitutional event/action observation
                ↓
     committed source of truth
                ↓
         emit_signal(...)
                ↓
       best-effort projection
```

Signal loss or redelivery then changes UI freshness, not constitutional history.

---

## Adapter contract added by D1D0

Every future provider adapter must establish all of the following before it is eligible for live D1C execution wiring:

1. request schema is compatible with the provider;
2. authorization survives the integrity boundary, not only the coordinator;
3. stable action identity exists before physical effect;
4. durable intent exists before physical effect;
5. `KnownNoEffect` comes only from authoritative provider evidence;
6. timeout/transport failure maps to `UnknownOutcome`;
7. repeated same action identity cannot repeat a physical effect unless replay safety has been independently qualified;
8. completion yields machine-readable receipt or query evidence;
9. derived projections cannot define constitutional completion.

## Recommended successor split

D1D should continue as separate provider repair lanes:

```text
003D1D-F  Finance allocation adapter
003D1D-P  Constitutional parameter adapter
003D1D-E  Durable event + signal projection adapter
```

Each lane should receive its own semantic head and exact-head verifier. They should not be merged into a common “provider safe” claim.

Only after a provider lane qualifies should its D1B capability snapshot move from `Unknown` / `NotEstablished` to a positive capability.

## Non-claims

D1D0 does not establish:

- that any provider is repaired;
- that any provider is qualified;
- deployment currentness;
- finance idempotency;
- parameter governance authority;
- signal delivery receipts;
- physical exactly-once execution;
- runtime D1C wiring.
