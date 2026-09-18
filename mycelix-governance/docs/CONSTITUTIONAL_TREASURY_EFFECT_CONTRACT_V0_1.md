# MYC-CONST-003D1D-F0 — Constitutional Treasury Effect Contract v0.1

F0 is an **inert provider-effect reference contract**. It exists to freeze semantic identity, replay and reconciliation behavior before any Treasury debit path or external settlement adapter is activated.

It does not create a Holochain entry/link type, does not call Treasury, does not move value, and does not authorize public-fund execution.

## Why this tranche exists

The frozen D1D0 source exposes more than one finance-facing route:

```text
GovernanceAction::TransferCredits
        -> governance_bridge::transfer_credits

newer governance finance bridge
        -> finance/treasury::execute_governance_transfer
```

The exact bound Treasury coordinator does not expose `execute_governance_transfer`.

At the same time the historical Treasury surface stores value as free-form `currency: String` plus integer quantities, while newer Finance work is separately qualifying canonical SAP value semantics.

F0 therefore must not pick an old endpoint and call it "the provider", and it must not turn legacy strings/raw integers into new constitutional value authority.

## Corrected local-transaction claim

F0 deliberately corrects an earlier overly broad repair framing.

The fact that historical `execute_allocation()` calls `debit_treasury(...)` before writing `AllocationStatus::Executed` is **not by itself** evidence of a local partial-commit crash bug. Local Holochain writes made within one zome invocation have a different transactional boundary from cross-cell or external effects.

The relevant successor theorem is:

```text
local transactional mutation
!= one-shot authorization consumption across calls
!= deterministic cross-boundary effect identity
!= provider reconciliation
!= physical exactly-once settlement
```

F0 models the latter boundaries only.

## Semantic identities

F0 keeps these identities separate:

```text
authorization_id
capacity_allocation_commitment
execution_id
attempt_id
external_receipt_id
```

A retry never mints new constitutional authority and never receives a fresh `execution_id`.

### Exact request subject

`TreasuryEffectSubject` binds opaque exact references to:

- D1C operation ID;
- D1C action ID;
- proposal ID;
- ClaimBinding reference/commitment;
- D1C action commitment;
- one-shot authorization identity and exact authorization-subject commitment;
- Treasury descriptor commitment;
- allocation-subject commitment;
- approval-projection commitment;
- optional exact reserved-capacity allocation commitment;
- recipient DID and recipient commitment;
- value profile ID and exact value-authority commitment;
- policy revision commitment;
- effect-adapter profile ID;
- effect-target commitment.

Those objects are intentionally opaque to F0. Their legitimacy is established elsewhere.

The request commitment therefore means only:

```text
these exact provider-effect inputs were bound together
```

not:

```text
every input is qualified/current/authorized
```

## Deterministic execution identity

`execution_id` is derived from the semantic request and stable one-shot identities.

It excludes:

- timestamps;
- attempt counters;
- worker IDs;
- queue IDs;
- transport request IDs;
- random retry nonces;
- external receipt IDs.

So:

```text
same exact authorized effect + transport retry
        -> same execution_id
```

while changing Treasury, recipient, value subject, authorization, capacity allocation, adapter or effect target changes the request/execution identity.

## Uniqueness domains

The pure registry maintains independent indexes for:

```text
action_id       -> execution_id
authorization_id -> execution_id
capacity allocation commitment -> execution_id
```

This prevents a caller from bypassing one-shot semantics merely by changing another identifier.

The decisions are deliberately conservative:

```text
same exact request
    -> ExistingSame

same action + changed request
    -> IntegrityConflict

same one-shot authorization + second effect
    -> IntegrityConflict

same exact capacity allocation + second effect
    -> IntegrityConflict
```

The first F0 reference model halts the registry on those cross-effect identity contradictions. Recovery/scope narrowing is intentionally not invented here.

## Effect state machine

```text
Pending
   |
   | Initial dispatch
   v
InFlight
   |\
   | \ provider success
   |  +--------------------> KnownSuccess
   |
   | transport timeout / provider unknown
   v
UnknownOutcome
   |\
   | \ authoritative no-effect reconciliation
   |  +--------------------> KnownNoEffect
   |
   | qualified replay capability
   +------------------------> InFlight

KnownNoEffect
   |
   | retry citing exact no-effect observation
   +------------------------> InFlight
```

Contradictory provider evidence enters `IntegrityHalted`.

## Unknown outcome is not failure

A transport timeout means only:

```text
caller does not know the external outcome
```

It does not mean:

```text
provider definitely performed no effect
```

Therefore `record_transport_failure()` always produces `UnknownOutcome`.

Blind retry from that state is rejected.

A retry can proceed only when the reference model is given one of two bases:

1. separately qualified provider replay/idempotency evidence; or
2. exact authoritative `KnownNoEffect` reconciliation evidence.

The `RetryBasis` enum itself is **not** proof that such evidence is authentic or qualified. F1 must obtain that result from a separately qualified provider boundary.

## Provider observation horizon

Provider observations carry:

```text
covers_through_attempt_ordinal
```

This avoids allowing an old query result to overwrite a newer in-flight retry.

For example:

```text
attempt 1 -> UnknownOutcome
attempt 2 -> InFlight
provider says "no effect through attempt 1"
```

The observation remains historical evidence and cannot turn attempt 2 into `KnownNoEffect`.

## Known success

`KnownSuccess` requires provider evidence bound to the exact:

```text
execution_id
request_commitment
effect_commitment
external receipt identity
receipt commitment
```

F0 does not authenticate provider origin. It only models the exact semantic binding required of future qualified evidence.

A successful effect is terminal for dispatch retry under the same execution identity.

Two success observations with the same semantic effect commitment may coexist as additional evidence. Different effect commitments for the same execution cause an integrity halt.

A no-effect observation covering the same or later attempt horizon as an already observed success is also an integrity conflict.

## D1C continuity

The integration test consumes the existing D1C structures and requires F0 to reuse exactly:

```text
ConstitutionalOperation.operation_id
ConstitutionalOperation.proposal_id
ConstitutionalOperation.claim_binding
ActionIntent.action_id
ActionIntent.action_commitment
```

It also keeps D1C provider replay/observability claims at `Unknown` in the fixture.

Structural identity reuse is not inherited qualification.

## Value semantics boundary

F0 contains no authoritative field named simply `amount: u64`, `amount: f64` or `currency: String`.

Instead it consumes:

```text
value_profile_id
value_authority_commitment
```

For the future SAP public-funds path those are expected to come from the independently qualified `SapAmount` lineage.

This prevents the provider layer from creating a competing money type or silently upgrading legacy Treasury records.

## Capacity boundary

`capacity_allocation_commitment` is optional in the pure contract because F0 is provider-generic. A future public-funds F1 profile that requires reserved Treasury capacity must make it mandatory through that profile's own admission theorem.

If supplied in F0, it is one-shot indexed and cannot back a second semantic effect.

## Compensation

F0 exposes no rollback/compensation state transition.

Compensation is a new forward effect that must receive its own authorization, capacity/value semantics and execution identity. A reverse transfer is not automatically the inverse of the original world state.

## Legacy source findings

The source-bound validator retains these observations from exact historical objects:

- `TransferCredits` dispatches `{from,to,amount}` to `governance_bridge::transfer_credits`;
- the newer bridge calls `treasury::execute_governance_transfer`;
- that extern is absent from the exact bound Treasury coordinator;
- historical Treasury/allocation records use `currency: String` and `u64` quantities;
- ordinary allocation execution has a stable allocation ID lookup and checked subtraction;
- large-value/DKG authorization remains owned by its separate assurance lane;
- F0 does not infer a local partial-commit bug from source statement order alone.

## F1 activation boundary

F1 may be considered only after the exact consumed subjects qualify.

At minimum it must establish:

- qualified constitutional operation/action identity;
- qualified exact action-bound authorization;
- qualified SAP value semantics for SAP execution;
- qualified Treasury/allocation/approval/capacity subjects required by the selected profile;
- one authoritative provider adapter;
- persistent intent before the declared irreversible dispatch boundary;
- provider-origin reconciliation by the same execution ID;
- exact provider replay/idempotency theorem before automatic retry under uncertainty;
- exact receipt binding;
- concurrent duplicate/conflicting-request qualification;
- exact-head hosted qualification before live routing.

## Claim ceiling

Even a future F0 PASS may establish only:

```text
deterministic provider-effect request identity
+ deterministic execution identity
+ one-shot semantic uniqueness in the pure registry
+ explicit retry/reconciliation state semantics
+ mutation-sensitive reference behavior
```

It does **not** establish:

- public-fund authority;
- `SapAmount` qualification;
- Treasury migration/current capacity;
- provider origin/truth;
- physical exactly-once execution;
- external settlement finality;
- accounting/legal finality;
- deployment currentness.
