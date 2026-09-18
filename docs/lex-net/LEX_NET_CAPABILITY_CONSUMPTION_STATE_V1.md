# LEX-NET-040 — Local Capability Consumption State v1

Status: executable research contract, R4. Language-neutral, deterministic, zero-network, backend-neutral, and closed-schema.

## Governing theorems

`append-only evidence != authoritative latest consumption state`

`caller says current != authoritative current`

`same authoritative prior state + competing transitions -> at most one authoritative successor`

`idempotent retry != new authority consumption`

`authority use committed != effect attempted != effect accepted != effect committed != effect final`

`040 state-machine PASS != backend atomicity PASS`

`genesis registration is single-assignment`

`re-registering genesis != restoring authority budget`

`unknown authority field != harmless metadata`

`commitment-valid extension != accepted v1 authority product`

LEX-NET-040 qualifies a synthetic local authority-state transition model for bounded-use capability consumption. It does not make caller-supplied evidence authoritative and does not claim an external effect happened.

## Qualified parent and lineage

Direct qualified parent: LEX-NET-025 R3-Q3 exact head `a539a2e0f5db4a1933534acc3303e2dc709c0023`.

Parent evidence:
- run `35284293688`
- attempt 1
- job `105413071448`
- conclusion `success`

Historical pre-execution LEX-NET-040 subjects remain frozen:
- R1 `3ecdf23608871702cda48786bc94b4656ebc58b4`
- R2 `abc38ed288bc0e8c6543689b72d9367d11a0c976`
- R2-Q2 `0b9257ae0b521a84cac5de526d2e3f619237e4bf`
- R3 `544a5cfe63c5b25a06cc04eea94553b6dc928e9e`

R4 does not rewrite them.

## Composition boundary

Qualified LEX-NET-025 supplies the immutable local `LocalCapabilityLeaseEnvelope`.

LEX-NET-040 owns authoritative local use-state for that envelope.

LEX-NET-035 owns downstream external effect attempt, acceptance, commit, finality, compensation, and reconciliation.

`028 evidence/mint binding -> 025 lease envelope -> 040 local authority consumption -> 035 external effect/finality`

No effect state flows backward into capability authority except through a new explicit local authority transition under separately frozen policy.

## Product domains

Lease envelope:
`LEX-NET/AUTHORITY/LEASE-ENVELOPE/v1 :: LocalCapabilityLeaseEnvelope`

Consumption state:
`LEX-NET/AUTHORITY/CONSUMPTION-STATE/v1 :: CapabilityConsumptionState`

Invocation:
`LEX-NET/INVOCATION/v1 :: CapabilityInvocation`

Authority-transition evidence:
`LEX-NET/EVIDENCE/v3 :: AuthorityTransitionEvidence`

Transition identifier:
`LEX-NET/AUTHORITY-TRANSITION-ID/v1`

These product identities are domain-separated.

## Closed v1 schemas

R4 makes the v1 authority surface closed.

The exact key sets for the lease envelope, consumption state, invocation, and invocation record are frozen by the executable profile.

Unknown fields fail closed even when the object commitment recomputes successfully.

`unknown authority field != harmless metadata`

`commitment-valid extension != accepted v1 authority product`

This rule applies at first registration, ordinary consumption, and CAS successor validation.

Future extensions require a new explicitly versioned profile and qualification. R4 does not infer that an unknown field is safe, non-authoritative, or ignorable.

## Canonical genesis

The exact genesis state is a deterministic function of the exact qualified lease envelope.

Registration must equal that one deterministic `genesis(lease)` object and commitment.

A merely commitment-valid or structurally plausible version-0 object is insufficient.

This prevents alternate genesis commitments for one lease and prevents extension-smuggling into the first authoritative state.

Genesis registration is single-assignment per exact lease-envelope commitment. Once a lease has an authoritative current-state pointer, `register()` returns `AlreadyRegistered` and leaves the pointer and stored lineage unchanged.

Re-registering deterministic genesis cannot restore spent budget, erase invocation history, or move the current pointer backward.

## Synthetic AuthorityStateStore

The reference oracle models:
- registered exact lease envelopes;
- committed exact consumption states;
- one current-state commitment per exact lease;
- one compare-and-swap transition.

`compare_and_swap(lease_commitment, expected_current_state_commitment, proposed_successor_state)`

The store rejects:
- stale expected state;
- wrong lease or domain;
- malformed/extended state;
- skipped version;
- wrong prior commitment;
- budget drift;
- rewritten prior invocation record;
- non-monotonic consumption index;
- duplicate transition ID;
- invalid transition-ID derivation;
- more than one newly added invocation record.

Prior invocation records are immutable across successors.

`consumption_indices = {1, ..., uses_consumed}`

The synthetic store is only the reference oracle. It does not establish database transaction atomicity, backend access-control correctness, durable crash recovery, Holochain/DHT uniqueness, consensus correctness, or production availability.

## CapabilityConsumptionState

The exact state binds:
- local domain;
- lease-envelope commitment;
- version;
- prior-state commitment;
- exact lease use budget;
- uses consumed;
- current-until ceiling;
- replay domain;
- revocation-handle commitment;
- policy-epoch commitment;
- bounded exact invocation-record map;
- state commitment.

V1 requires:

`version = uses_consumed`

`0 <= uses_consumed <= lease_use_budget <= 64`

`invocation_record_count = uses_consumed`

`consumption_indices = {1, ..., uses_consumed}`

Each record has exactly:
- invocation commitment;
- transition ID;
- consumption index.

No extra record fields are accepted.

## Invocation

The exact invocation binds:
- invocation ID;
- subject;
- purpose;
- resource;
- action;
- replay domain;
- payload commitment;
- invocation commitment.

V1 bounds:
- use budget <= `64`;
- invocation ID <= `128` UTF-8 bytes;
- payload commitment <= `256` UTF-8 bytes.

Unknown invocation fields fail closed.

## Deterministic transition ordering

`ConsumeCapability(lease, supplied_state, store, invocation, evaluation_time, local_policy_context)` performs:

1. verify exact lease schema and commitment;
2. verify exact state schema/commitment and exact lease/domain binding;
3. require the store current pointer to equal the supplied state commitment;
4. verify exact invocation schema and commitment;
5. resolve existing invocation IDs before new-use budget/currentness checks:
   - same ID + same invocation commitment -> `IdempotentReplay`;
   - same ID + different invocation commitment -> `InvocationConflict`;
6. for a new invocation, verify currentness, revocation, policy epoch, exact scope, replay domain, and budget;
7. construct one exact-schema successor;
8. CAS validates prior history conservation, one version increment, one consumption increment, and the new transition ID;
9. only CAS success emits `AuthorityTransitionEvidence(disposition = AuthorityUseCommitted)`.

## Idempotency, concurrency, and replay bounds

Exact retry after a committed use is historical lookup, not new authority.

`same invocation ID + same invocation commitment -> IdempotentReplay`

`same invocation ID + different invocation commitment -> InvocationConflict`

For two distinct invocations racing a one-use capability from the same prior state, only one proposed successor may become authoritative.

For two identical invocations racing from one prior state, one may commit and the loser resolves idempotently after reload.

Invocation records are added only for committed new uses and cannot exceed the lease use budget or profile cap.

`replay safety != permission for unbounded invocation history`

## Authority-transition evidence

A successful new use emits append-only `AuthorityTransitionEvidence` binding the exact lease, invocation, prior state, successor state, transition ID, consumption index, local domain, and disposition `AuthorityUseCommitted`.

It also binds:

`effect_attempted = false`

`effect_confirmed = false`

`grants_local_authority = false`

`grants_external_effect_authority = false`

Evidence cannot become, recreate, replace, or roll back authoritative state.

## Effect boundary

A committed authority transition consumes or reserves local authority budget.

`AuthorityUseCommitted != EffectAttempted`

Missing effect confirmation does not silently restore budget.

Any release, retry authorization, or restoration is a new explicit local authority transition under separately frozen policy.

LEX-NET-035 remains downstream owner of effect attempt, unknown timeout, acceptance, commit, finality, compensation, and reconciliation.

## Reference outcomes

- `TransitionCommitted`
- `IdempotentReplay`
- `AlreadyRegistered`
- `StateInvalid`
- `StateStoreMissing`
- `StaleState`
- `LeaseMismatch`
- `DomainMismatch`
- `LeaseNotCurrent`
- `Revoked`
- `PolicyEpochMismatch`
- `ScopeMismatch`
- `ReplayDomainMismatch`
- `BudgetExhausted`
- `InvocationConflict`
- `InvocationInvalid`

## Golden/adversarial corpus

The frozen 48-fixture corpus includes all R3 cases plus:
- extra field on lease -> rejected;
- extra field on invocation -> rejected;
- extra field on state -> rejected;
- extra field on invocation record -> rejected;
- alternate commitment-valid genesis with an extra field -> rejected;
- commitment-valid successor with an extra state field -> rejected.

These directly prove the closed-schema and canonical-genesis refinements.

## Nonclaims

This tranche does not establish legal authority.

This tranche does not establish identity truth.

This tranche does not establish fairness.

This tranche does not establish database transaction atomicity.

This tranche does not establish backend access-control correctness.

This tranche does not establish durable crash recovery.

This tranche does not establish distributed consensus correctness.

This tranche does not establish Holochain-specific uniqueness or serialization guarantees.

This tranche does not establish availability.

This tranche does not establish external effect attempt or success.

This tranche does not establish external finality.

This tranche does not establish forward compatibility with unknown authority-state extensions.

This tranche does not establish production security.

A PASS establishes only the frozen synthetic local capability-consumption state, closed-schema/canonical-genesis, single-assignment registration, history-conservation, and CAS-transition theorem.
