# MYC-CONST-003D1D-F1B0 — Constitutional Payments Provider Journal v0.1

## Status

This tranche is an **inert semantic contract**. It adds no Holochain entry or link types, no payments-zome externs, no Treasury or Governance routing, no provider query endpoint, and no replay-capability minting.

Parent semantic subject: `MYC-CONST-003D1D-F1A` at `4879c83c4ef21c15e124730a1518ec68812d1611`.

## Why F1B0 exists

F1A established deterministic provider-operation identity and the outcome vocabulary `KnownSuccess | KnownNoEffect | UnknownOutcome`. It did not yet define how that truth survives crashes, lost acknowledgements, retries, contradictory provider evidence, or persistence.

F1B0 freezes the persistence semantics before any live DHT surface exists.

The key rule is:

```text
provider status is not an authoritative mutable field
```

Instead, one provider operation has a hash-linked append-only journal:

```text
OperationIntent
    ↓
DispatchAttempt(1)
    ↓
Observation(... horizon 1 ...)
    ↓
DispatchAttempt(2)         only after KnownNoEffect through attempt 1
    ↓
Observation(... horizon 2 ...)
```

`KnownSuccess`, `KnownNoEffect`, `UnknownOutcome`, `InFlight`, and `IntegrityHalted` are derived by replaying the journal.

## Durable record identity

Each journal record binds:

- exact F1A `provider_operation_key`;
- contiguous sequence number;
- predecessor record commitment;
- exact record kind and semantic payload.

`recorded_at_unix_ms` is audit metadata and is excluded from semantic record identity.

The journal genesis record is always the exact F1A `PaymentProviderIntent`.

## Dispatch attempts

Dispatch attempts bind the exact F1A execution ID and request commitment. Attempt IDs must be unique and ordinals contiguous.

F1B0 intentionally has **no qualified replay-capability consumption path**. Therefore:

```text
IntentCommitted
    → first dispatch allowed

UnknownOutcome
    → blind redispatch forbidden

KnownNoEffect through complete current attempt horizon
    → next attempt allowed

KnownSuccess
    → terminal

IntegrityHalted
    → dispatch forbidden
```

A later tranche may add a verifier-owned replay-capability consumption record. F1B0 does not represent a receipt ID, boolean, or caller string as replay authority.

## Attempt-horizon observations

Every provider observation states the dispatch horizon it covers.

This closes an ambiguity that F1A intentionally did not model:

```text
attempt 1 → authoritative KnownNoEffect through 1
attempt 2 → success through 2
```

is coherent, while:

```text
attempt 1 → authoritative KnownNoEffect through 1
later receipt → success through 1
```

is contradictory evidence and derives `IntegrityHalted`.

`KnownSuccess` cannot cover horizon zero. `KnownNoEffect` and `UnknownOutcome` may cover horizon zero when an authoritative provider query occurs before any dispatch is observed.

### Evidence monotonicity

A stale observation cannot roll the state machine backwards across a newer dispatch horizon.

For example:

```text
KnownNoEffect through attempt 1
    ↓
DispatchAttempt(2)
    ↓
stale KnownNoEffect/UnknownOutcome through attempt 1
```

must remain `InFlight(attempt 2)`.

Likewise, a later `UnknownOutcome` cannot demote `KnownSuccess`.

Once contradictory evidence derives `IntegrityHalted`, that halt is sticky. Additional observations may be retained as forensic facts, but they cannot clear the halt and no later dispatch may occur.

## Lost acknowledgement recovery

A provider success is an append-only observation. If the caller loses the response after the success observation is committed, a later query replays the journal and still returns `KnownSuccess` with the payment ID and provider receipt evidence.

A lost acknowledgement therefore does not require generating a new payment or provider operation identity.

## Conflict semantics

The journal fails closed on:

- one observation ID carrying different evidence;
- success contradicting a prior no-effect statement covering the same-or-later horizon;
- no-effect contradicting a prior success at or before the claimed no-effect horizon;
- incompatible success receipts for one provider operation.

Exact duplicate observations are idempotent and do not append a second record.

Unknown evidence arriving after a terminal success is historical-only and cannot demote the success.

## Provider operation index

The pure registry binds:

```text
F0 execution_id → exactly one F1A provider_operation_key → exactly one journal
```

The provider query can resolve by provider operation key or F0 execution ID and returns the current journal-head commitment with the derived result.

This is the semantic target for the future DHT index. It is not itself DHT persistence.

## F1B1 boundary

The next tranche may refine these records into **inactive HDI/DHT types** only if it preserves:

1. separate F0 execution identity, provider operation key, historical `Payment.id`, attempt ID, and receipt ID namespaces;
2. append-only journal semantics;
3. exact predecessor and operation-key binding;
4. attempt-horizon reconciliation;
5. stale-evidence monotonicity across newer dispatch attempts;
6. sticky integrity halt with no post-halt dispatch;
7. `UnknownOutcome` without failure laundering;
8. receipt binding to the exact provider operation and F0 request;
9. no live payment dispatch or Governance routing activation.

Only after that persistence refinement is independently qualified should a concrete payments adapter, authoritative provider query, and replay-capability mint path be considered.

## Non-claims

F1B0 does **not** establish Holochain persistence, payments-zome correctness, provider truth, replay qualification, public-fund authority, physical exactly-once settlement, external settlement finality, live Governance routing, deployment currentness, or qualification.
