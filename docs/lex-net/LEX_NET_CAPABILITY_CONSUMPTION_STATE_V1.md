# LEX-NET-040 — Local Capability Consumption State v1

Status: executable research contract. Language-neutral, deterministic, zero-network, and backend-neutral.

## Governing theorems

`append-only evidence != authoritative latest consumption state`

`caller says current != authoritative current`

`same authoritative prior state + competing transitions -> at most one authoritative successor`

`idempotent retry != new authority consumption`

`authority use committed != effect attempted != effect accepted != effect committed != effect final`

`040 state-machine PASS != backend atomicity PASS`

LEX-NET-040 qualifies a synthetic local authority-state transition model for bounded-use capability consumption. It does not make caller-supplied evidence history authoritative and does not claim that an external effect happened.

## Qualified parent

Direct qualified parent: LEX-NET-025 R3-Q3 exact head `a539a2e0f5db4a1933534acc3303e2dc709c0023`.

Parent evidence:
- run `35284293688`
- attempt 1
- job `105413071448`
- conclusion `success`

## Composition boundary

Qualified LEX-NET-025 supplies the immutable local `LocalCapabilityLeaseEnvelope`.

LEX-NET-040 owns authoritative local use-state for that envelope.

LEX-NET-035 owns downstream external effect attempt, acceptance, commit, finality, compensation, and reconciliation.

`028 evidence/mint binding -> 025 lease envelope -> 040 local authority consumption -> 035 external effect/finality`

No effect state flows backward into capability authority except through a new explicit local authority transition under a separately frozen policy.

## Product domains

Lease envelope:

`LEX-NET/AUTHORITY/LEASE-ENVELOPE/v1 :: LocalCapabilityLeaseEnvelope`

Consumption state:

`LEX-NET/AUTHORITY/CONSUMPTION-STATE/v1 :: CapabilityConsumptionState`

Invocation:

`LEX-NET/INVOCATION/v1 :: CapabilityInvocation`

Authority-transition evidence:

`LEX-NET/EVIDENCE/v3 :: AuthorityTransitionEvidence`

These product identities are domain-separated.

## Synthetic AuthorityStateStore

The v1 oracle models an explicit local state store with:

`current_state_commitment_by_lease`

and a compare-and-swap transition:

`compare_and_swap(lease_commitment, expected_current_state_commitment, proposed_successor_state)`

The store keeps the registered lease and committed states and rejects malformed successors, wrong prior-state bindings, skipped versions, budget drift, and stale expected commitments.

A caller-supplied string named `current` is not authority.

The synthetic store is only the reference oracle. It does not establish database transaction atomicity, Holochain/DHT uniqueness, crash durability, distributed consensus, or production availability.

## CapabilityConsumptionState

A state binds:
- exact local domain;
- exact lease-envelope commitment;
- monotonic state version;
- prior-state commitment (`null` only at genesis);
- exact lease use budget;
- uses consumed;
- lease current-until ceiling;
- replay/idempotency domain;
- revocation-handle commitment;
- policy-epoch commitment;
- bounded invocation records;
- authority-state commitment.

V1 requires:

`version = uses_consumed`

`0 <= uses_consumed <= lease_use_budget <= 64`

`invocation_record_count = uses_consumed`

Every new committed use creates exactly one successor version and one invocation record.

## Genesis

Genesis is registered for exactly one structurally valid local lease envelope.

It has version `0`, zero uses consumed, no prior state, and an empty invocation record map. Budget, currentness, replay, revocation, policy, domain, and lease identity are copied exactly from the lease.

Foreign consumption state cannot be registered as destination-domain authority.

## Invocation

A committed invocation binds:
- invocation ID;
- exact subject/purpose/resource/action;
- replay domain;
- payload commitment;
- invocation commitment.

V1 bounds:
- use budget <= `64`;
- invocation ID <= `128` UTF-8 bytes;
- payload commitment <= `256` UTF-8 bytes.

Oversized or malformed invocations fail closed.

## Deterministic transition ordering

`ConsumeCapability(lease, supplied_state, store, invocation, evaluation_time, local_policy_context)` performs:

1. verify lease structure and commitment;
2. verify state structure/commitment and exact lease/domain binding;
3. require the store's authoritative current pointer to equal the supplied state commitment;
4. resolve existing invocation IDs before new-use budget/currentness checks:
   - same ID + same invocation commitment -> `IdempotentReplay`, with no new consumption;
   - same ID + different invocation commitment -> `InvocationConflict`;
5. for a new invocation, verify lease/state currentness, revocation, policy epoch, exact scope, replay domain, and remaining budget;
6. construct one successor whose version and uses-consumed increment by exactly one;
7. require the store CAS to validate the successor against the registered prior and lease;
8. only on CAS success emit `AuthorityTransitionEvidence(disposition = AuthorityUseCommitted)`.

CAS failure emits no committed successor and no external effect claim.

## Idempotency and concurrency

Exact retry after a committed use is historical lookup, not new authority.

`same invocation ID + same invocation commitment -> IdempotentReplay`

`same invocation ID + different invocation commitment -> InvocationConflict`

For two distinct invocations racing a one-use capability from the same prior state, only one proposed successor may become authoritative. The loser becomes stale; after reloading current state it receives the result implied by that state.

For two identical invocations racing from one prior state, one may commit. The loser reloads and resolves idempotently.

## Bounded replay state

Invocation records are created only for committed new uses and cannot exceed the lease use budget or profile cap.

V1 does not prune invocation history while claiming exact idempotency.

`replay safety != permission for unbounded invocation history`

Any future accumulator/compaction profile must separately qualify equivalent conflict/idempotency semantics.

## Authority-transition evidence

A successful new local use emits append-only `AuthorityTransitionEvidence` binding:
- lease-envelope commitment;
- invocation ID and invocation commitment;
- prior state commitment/version;
- successor state commitment/version;
- consumption index;
- local domain;
- disposition `AuthorityUseCommitted`;
- `effect_attempted = false`;
- `effect_confirmed = false`;
- `grants_local_authority = false`;
- `grants_external_effect_authority = false`.

This evidence records the local authority-state transition. It cannot become, recreate, replace, or roll back authoritative state.

## Effect boundary

A committed authority transition consumes/reserves bounded local authority budget. Missing later effect confirmation does not silently restore the budget.

`AuthorityUseCommitted != EffectAttempted`

If release, retry, or restoration is permitted, that is a new explicit local authority transition under separately frozen policy. It is never inferred from timeout, missing receipt, process crash, external rejection, or compensation request.

LEX-NET-035 owns downstream effect/finality/reconciliation semantics.

## Reference outcomes

- `TransitionCommitted`
- `IdempotentReplay`
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

The frozen 38-fixture corpus covers genesis, first use, budget exhaustion, stale-state replay, distinct and identical races, rollback, state forgery, wrong lease/domain, foreign-state substitution, exact idempotency/conflict, scope/replay mismatch, expiry, revocation/policy changes, successor-version/prior binding, transition-evidence binding, evidence/state separation, omitted or duplicate evidence, caller-current substitution, skipped versions, bounded invocation history, oversized invocation fields, profile budget cap, historical idempotent lookup after expiry, unknown/rejected external effects without budget restoration, synthetic-store absence, state commitment tampering, direct successor-version jumps, and wrong-prior CAS attempts.

## Nonclaims

This tranche does not establish legal authority.

This tranche does not establish identity truth.

This tranche does not establish fairness.

This tranche does not establish database transaction atomicity.

This tranche does not establish durable crash recovery.

This tranche does not establish distributed consensus correctness.

This tranche does not establish Holochain-specific uniqueness or serialization guarantees.

This tranche does not establish availability.

This tranche does not establish external effect attempt or success.

This tranche does not establish external finality.

This tranche does not establish production security.

A PASS establishes only the frozen synthetic local capability-consumption state and CAS-transition theorem.
