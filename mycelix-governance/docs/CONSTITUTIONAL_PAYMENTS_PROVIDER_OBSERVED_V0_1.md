# MYC-CONST-003D1D-F0P0 — Payments Provider Replay Eligibility

## Purpose

F0C makes automatic replay authority impossible to assert through an arbitrary string.
The next question is whether an existing Finance provider is already strong enough to
mint `QualifiedProviderReplayCapability`.

This tranche source-binds the exact `finance::payments::send_payment` implementation on
runtime subject:

`15b9c89adf0ac3c6c5a73681614d6bfcd368820a`

It is observation/evidence only. It changes no Finance, Governance or Holochain runtime
code.

## Result

The payments zome contains useful provider-like primitives, but the frozen source does
**not** establish F0C automatic-replay eligibility.

```text
Payments provider candidate
    + indexed payment ID lookup
    + receipt lookup
    + immutable receipt entry
    + sender-author binding on Payment creation
    + sender-agent signature creation

    - deterministic operation key derived from F0 execution_id
    - retry-stable provider key
    - exact same-key duplicate theorem
    - changed-request/same-key conflict theorem
    - authoritative KnownNoEffect query
    - provider UnknownOutcome query
    - receipt binding to execution_id
    - receipt binding to request_commitment
    - peer-reproducible receipt signature verification
    - qualified SapAmount input
    - current governance routing

=> automatic_replay_eligible = false
=> QualifiedProviderReplayCapability minting = forbidden
```

## Existing payment identity is an audit identity, not an idempotency key

The exact `send_payment` implementation generates:

```text
payment:{from_did}:{now_micros}
```

That has a useful uniqueness role for an ordinary payment record, but it is not stable
across a transport retry.

The provider identity required by F0C instead needs:

```text
same semantic constitutional effect
    -> same provider operation key
```

independent of worker, wall clock, retry nonce or transport attempt.

### Do not overload the historical payment ID

A successor should not simply rename or concatenate the old payment ID into an
`execution_id` string. Keep identities separate:

```text
F0 execution_id              semantic constitutional effect identity
provider_operation_key       deterministic provider idempotency/query identity
payment.id                    provider payment/audit record identity
provider receipt ID           evidence identity
attempt ID                    operational retry identity
```

The preferred successor is a dedicated provider-operation record/index keyed from the
exact F0 execution identity. Existing historical payment IDs can remain intact.

## Existing lookup surface is useful

The frozen provider already has:

- `PaymentIdToPayment` link indexing;
- `get_payment(payment_id)`;
- `get_receipt(payment_id)`.

These are good building blocks for a future provider query, but a lookup endpoint is not
by itself a reconciliation theorem.

The future provider query must use the deterministic provider operation key and return a
machine-classifiable result:

```text
KnownSuccess
KnownNoEffect
UnknownOutcome
```

A missing payment record cannot automatically mean `KnownNoEffect` unless the provider's
qualified lifecycle proves absence is authoritative for the exact operation key.

## Existing receipts are useful but not yet provider authority

`send_payment` creates a receipt and signs data covering the payment ID, parties, amount,
currency and timestamp with the sender agent key.

The integrity source also makes Receipt entries immutable.

Those are positive controls worth preserving.

However the exact integrity source explicitly states that full cryptographic receipt
verification requires the sender public key and is deferred. The receipt also does not
bind F0 `execution_id` or F0 `request_commitment`.

Therefore:

```text
signed receipt exists
!= peer-reproducible verified provider receipt
!= exact F0 effect receipt
```

A successor provider receipt must bind at least:

```text
provider_operation_key
execution_id
request_commitment
provider_outcome
provider_receipt_identity
provider_receipt_commitment
```

and its verification boundary must be explicit and qualified.

## Payment status is not an uncertainty protocol

New payments are created with `TransferStatus::Completed` under the current simplified
immediate-completion path.

F0C requires a provider query capable of distinguishing:

```text
KnownSuccess
KnownNoEffect
UnknownOutcome
```

especially after transport loss.

The successor must not convert:

```text
RPC timeout
record not immediately observed
network error
```

into `KnownNoEffect`.

## Current governance routing does not use payments

The exact Governance finance bridge currently targets:

```text
finance / treasury / execute_governance_transfer
```

not `finance / payments / send_payment`.

Therefore this tranche does not claim the payments zome is already the constitutional
Treasury effect provider. Selecting it is a future F1 design/integration act and must be
qualified independently.

## Value authority remains upstream

The historical payments surface consumes:

```text
amount: u64
currency: String
```

F0P0 does not reinterpret those fields as qualified SAP semantics.

A constitutional Treasury successor must consume the exact qualified SAP value subject
from the Finance value lane (or an exact qualified successor), not infer authority from
`currency == "SAP"` plus a raw integer.

## Preferred repair decomposition

A narrow successor can be built in stages:

### F1A — inert provider-operation semantics

Define a pure provider operation record:

```text
PaymentProviderIntentV1 {
    provider_operation_key,
    execution_id,
    request_commitment,
    exact qualified nested value/authority commitments,
}
```

and deterministic duplicate/conflict rules without changing live payment dispatch.

### F1B — provider query + receipt semantics

Add a source-bound result model:

```text
PaymentProviderResultV1 {
    provider_operation_key,
    execution_id,
    request_commitment,
    outcome: KnownSuccess | KnownNoEffect | UnknownOutcome,
    provider_receipt_identity?,
    provider_receipt_commitment?,
}
```

with peer-reproducible or explicitly verifier-owned receipt validation.

### F1C — qualified persistence/idempotency adapter

Only after value/authority/capacity prerequisites are qualified, persist the provider
operation before the declared dispatch boundary and prove same-key duplicate semantics,
concurrency behavior, lost-ack reconciliation and immutable evidence.

### F1D — capability minting and governance routing

Only a qualified provider verifier may mint an F0C
`QualifiedProviderReplayCapability`. Then, and only then, may the live constitutional
execution route be redirected to the provider.

## Positive controls to preserve

A successor should retain rather than erase:

- payment author binding to `from_did`;
- indexed payment lookup;
- indexed receipt lookup;
- immutable receipt entries;
- sender-agent receipt signing where it remains part of the qualified receipt protocol;
- existing value conservation/authorization controls not explicitly superseded;
- exact separation between local Holochain transaction semantics and external/provider
  replay semantics.

## Non-claims

This profile does not establish:

- that a duplicate live payment has occurred;
- exploitability;
- an overall payments-system safety verdict;
- provider qualification or provider truth;
- physical exactly-once effects;
- external settlement finality;
- public-fund authorization;
- live governance use of the payments provider;
- deployment currentness.
