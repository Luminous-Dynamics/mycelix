# MYC-CONST-003D1D-F0C — Treasury Provider Replay Capability Gate

## Purpose

F0 established deterministic Treasury effect identity, guarded reconciliation history,
and a fail-closed `UnknownOutcome` state. Its private kernel still models qualified
provider replay with a string evidence identifier. F0C removes that trust surface from
the **public API**.

F0C does not qualify any provider. It defines the type boundary that a future exact
provider qualifier must cross before automatic replay from uncertainty can be enabled.

## Public theorem

Public callers cannot construct automatic replay authority from:

- a boolean;
- a receipt ID;
- an arbitrary evidence string;
- an adapter/profile name;
- a transport success/failure;
- a deserialized token.

The public retry surface is:

```text
Initial
| ProviderReplayQualified(QualifiedProviderReplayCapability)
| ReconciledNoEffect { exact observation_id }
```

`QualifiedProviderReplayCapability` is deliberately:

- non-serializable;
- non-deserializable;
- non-Clone;
- non-Copy;
- field-private;
- constructor-private;
- process-local.

No production minting path exists in F0C.

Therefore, until a later provider qualification tranche adds a verifier-owned minting
path, public automatic replay from `UnknownOutcome` is impossible. Exact authoritative
`KnownNoEffect` reconciliation remains available and still retries the same semantic
`execution_id`.

## Effect- and horizon-bound capability

A future qualified capability must bind:

```text
provider_profile_id
provider_profile_commitment
qualification_receipt_id
qualification_receipt_commitment
execution_id
request_commitment
covers_through_attempt_ordinal
```

The capability is accepted only when:

```text
capability.provider_profile_id
    == effect.adapter_profile_id

capability.execution_id
    == current effect.execution_id

capability.request_commitment
    == current effect.request_commitment

current state
    == UnknownOutcome(capability.covers_through_attempt_ordinal)
```

This prevents capability transplantation between providers, requests, executions or
uncertainty horizons.

## Retry identity remains unchanged

A capability authorizes only the decision to attempt the **same** semantic effect again.
It does not mint a new effect:

```text
execution_id before retry == execution_id after retry
```

Attempt IDs and timestamps remain operational diagnostics and do not become authority.

## Attempt audit surface

F0C also stops leaking the private F0 retry enum through `EffectAttempt`.

Public attempt history is exposed as a read-only snapshot:

```text
AttemptRetryEvidence::Initial
AttemptRetryEvidence::ProviderReplayQualified { capability_evidence_id }
AttemptRetryEvidence::ReconciledNoEffect { observation_id }
```

This is descriptive evidence only. It cannot be supplied back to authorize a retry.

## Required future provider qualification

A future provider-specific tranche may add a minting path only after exact qualification
of all of the following.

### 1. Provider operation key

The provider must expose a deterministic operation/idempotency key bound to the F0
`execution_id` or to a separately qualified deterministic mapping from it.

Required properties:

```text
same execution + same request -> same provider operation key
changed request + same key     -> conflict
random retry nonce             -> forbidden as semantic provider identity
```

### 2. Duplicate semantics

The provider must establish:

```text
same key + same request
    -> same semantic effect or same authoritative receipt

same key + different request
    -> conflict

concurrent same-key submissions
    -> one semantic effect or explicit conflict
```

A provider that simply accepts duplicate requests cannot earn automatic replay authority.

### 3. Authoritative query/reconciliation

The provider must support a query keyed by the qualified provider operation key and
produce machine-classifiable outcomes:

```text
KnownSuccess
KnownNoEffect
UnknownOutcome
```

Transport failure is never `KnownNoEffect`.

### 4. Receipt schema

Authoritative receipts must be machine-readable and bind at least:

```text
provider_operation_key
execution_id
request_commitment
provider_outcome
provider_receipt_identity
provider_receipt_commitment
```

A human-readable message or status string is insufficient.

### 5. Replay policy

Automatic replay from `UnknownOutcome` requires a provider idempotency/replay theorem.
Without that theorem, the system must reconcile first.

Authoritative `KnownNoEffect` may permit retry of the same `execution_id` because it
establishes that the prior covered attempt horizon produced no provider effect.

Conflicting provider evidence must halt rather than silently choose one history.

## Qualification boundary

The machine-readable profile remains:

```text
qualification_state = Pending
activation_allowed   = false
public_mint_path      = false
minting_path          = AbsentUntilExactProviderQualification
```

A future provider PASS must produce a new exact receipt and a new semantic child that
adds the verifier-owned minting path. It must not rewrite this pending profile into a
qualified one.

## Non-claims

F0C does **not** establish:

- provider qualification or provider truth;
- external settlement finality;
- physical exactly-once execution;
- public-fund authorization;
- qualified `SapAmount` semantics;
- Treasury V2 migration/currentness;
- reserved-capacity qualification;
- a live Treasury adapter;
- live governance routing;
- deployment currentness.
