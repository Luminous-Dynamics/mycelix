# FIN-MKT-002A Canonical Market Order Observations V1

## Scope

FIN-MKT-002A defines immutable provider-neutral observation identity for market-order lifecycle events, fills, and fill correction/bust relations.

```text
provider observation
!= provider truth beyond supplied evidence
!= current order state
!= position ownership
!= settlement
```

This crate is intentionally **observation-only**. FIN-MKT-002B owns deterministic projection over an exact observation frontier.

## Dependencies

V1 consumes FIN-MKT-001 r3 types for:

- exact market intent commitment;
- account subject;
- instrument reference;
- provider/profile references;
- exact market quantity and price values;
- canonical lowercase `Digest32`.

FIN-MKT-002A does not grant FIN-MKT-001 qualification. Its current source parent may itself remain unqualified.

## Subject binding

Every observation binds:

```text
exact FIN-MKT intent commitment
exact account subject
exact instrument ref
exact provider profile
optional provider order ref
```

The helper `validate_subject_against_intent_v1` checks the first three coordinates against one exact positive `CanonicalMarketOrderIntentV1`.

```text
provider order ID
!= global order identity
```

Provider order IDs remain scoped beneath the exact subject/provider coordinates.

## Observation identity versus semantics

Every canonical observation carries two commitments.

### Identity commitment

The identity commitment answers:

```text
which provider-scoped observation identity is this?
```

It binds the exact observation subject plus an exact provider observation reference:

```text
observation profile
provider observation ID
```

### Observation commitment

The observation commitment answers:

```text
what semantic content was observed under that identity?
```

This split freezes the conflict theorem:

```text
same identity + same observation commitment
-> idempotent duplicate

same identity + different observation commitment
-> conflicting reuse

different identity
-> distinct observations
```

Arrival order cannot resolve conflicting reuse.

## Event observations

Closed V1 event vocabulary:

```text
SubmitAttemptObserved
ProviderAcceptedObserved
PendingNewObserved
WorkingObserved
CancelRequestedObserved
CancelAcceptedObserved
CancelRejectedObserved
ReplaceRequestedObserved
ReplaceAcceptedObserved
ReplaceRejectedObserved
DoneForDayObserved
ExpiredObserved
SuspendedOrHaltedObserved
ProviderRejectedObserved
OpaqueProviderStatus(exact provider value)
SubmissionOutcomeUnknownObserved
```

Important non-equivalences:

```text
CancelRequestedObserved != CancelAcceptedObserved
CancelRejectedObserved != ProviderRejectedObserved
SubmissionOutcomeUnknownObserved != ProviderRejectedObserved
```

Provider values that cannot be mapped losslessly remain `OpaqueProviderStatus`.

## Fill observations

A fill observation binds:

```text
exact order observation subject
exact provider observation ref
provider execution ref
exact executed quantity
exact execution price
optional venue ref
chronology evidence
source evidence commitment
```

Zero execution quantity and zero execution price fail closed.

```text
fill observation
!= settled position
!= qualified settlement
```

Duplicate fill observation identities count only once only after a later projector applies the explicit duplicate theorem. 002A itself does not aggregate fills.

## Fill correction / bust observations

Historical fills are never mutated or deleted.

V1 adjustment vocabulary:

```text
Correction {
    prior_fill_commitment,
    replacement_fill_commitment
}

Bust {
    prior_fill_commitment
}
```

A correction cannot name the exact same fill as prior and replacement.

002A establishes only the append-only relation. FIN-MKT-002B or later position/settlement profiles determine the effective projection.

## Chronology boundary

Chronology is optional but typed.

```text
ProviderChronologyV1 {
    chronology_profile,
    sequence,
    provider_time
}
```

Chronology profile and chronology evidence must be present together.

Provider timestamps are exact provider evidence only:

```text
provider timestamp
!= trusted current time
!= universal total ordering
```

A provider-specific sequencing profile owns ordering semantics.

## Canonical encoding

All integers are unsigned big-endian. Text is exact UTF-8:

```text
u32 byte_length || exact UTF-8 bytes
```

All digests are raw 32 bytes.

Commitment profile revision:

```text
u32 = 1
```

### Common observation subject

```text
[32] intent commitment
account subject ref
instrument ref
provider profile ref
optional provider order ref
```

### Provider observation ref

```text
profile ref
external observation ID
```

### Optional external ID

```text
0
1 || text
```

### Event identity

```text
"MYCELIX_FIN_MKT_EVENT_ID_V1\0"
revision
subject
observation_ref
```

### Event observation

```text
"MYCELIX_FIN_MKT_EVENT_OBS_V1\0"
revision
[32] event identity commitment
event kind
chronology
[32] source evidence commitment
```

### Fill identity

```text
"MYCELIX_FIN_MKT_FILL_ID_V1\0"
revision
subject
observation_ref
```

### Fill observation

```text
"MYCELIX_FIN_MKT_FILL_OBS_V1\0"
revision
[32] fill identity commitment
text provider_execution_ref
market quantity
market price
optional venue ref
chronology
[32] source evidence commitment
```

### Adjustment identity

```text
"MYCELIX_FIN_MKT_ADJUST_ID_V1\0"
revision
subject
observation_ref
```

### Adjustment observation

```text
"MYCELIX_FIN_MKT_ADJUST_OBS_V1\0"
revision
[32] adjustment identity commitment
adjustment kind
chronology
[32] source evidence commitment
```

Adjustment tags:

```text
0 Correction || [32] prior || [32] replacement
1 Bust       || [32] prior
```

## Frozen independent vectors

Synthetic provider fixture:

`test-vectors/observations-v1.json`

Independent stdlib Python oracle:

`tests/reference/order_observation_v1_reference.py`

Frozen values:

```text
event identity
bd0a7c994a58859f7fd99455433def8e9bfda2781149e90a23192042d60a8b19

event observation
d772952075c747a3ca6d076218b093308aa2d8869a93479e0beeef098da17ae2

fill identity
b6fb55af9d6f0ed1e4b2d5f696f9d8e8bf659efcd20c2ee7d807f492cd8dd134

fill observation
d9cb8f1c8e6f4ea2dacb7cce3e0aeca060cb5f0df37e9fa6ba61d1c761202292

adjustment identity
f33bcce43385ce6d1f310439360baed8e3dbcc656890ce0e9ba286abbe2ba4d1

adjustment observation
1b6cec074f88d1767c6551a417d6b8b337ea0bf12db7aeedf5fc956869f5ce8c
```

Frozen preimage sizes:

```text
event identity       419
event observation    203
fill identity        422
fill observation     441
adjustment identity  429
adjustment observation 268
```

## Schema boundary

Every caller-supplied 002A struct denies unknown fields.

Imported FIN-MKT-001 quantity/price objects retain r3's schema-closed amount reconstruction.

Positive canonical observation types are serializable but not caller-deserializable.

```text
portable JSON
!= positive observation theorem
```

## Provider mapping firewall

FIN-MKT-002A contains no Robinhood, Alpaca, FIX, MCP, HTTP, WebSocket, database, Holochain, OAuth, credential, or wall-clock dependency.

Adapters map provider evidence into these closed observations.

```text
provider status string
!= canonical event by direct equality
```

Mapping profiles must remain explicit and loss-aware.

## Nonclaims

A future PASS for this exact product would establish only deterministic bounded observation identity under the frozen V1 profile.

It would **not** establish:

- current broker truth;
- account ownership;
- instrument truth;
- provider authenticity beyond supplied evidence;
- financial authority or buying power;
- current credential scope;
- current order state;
- fill aggregation;
- position ownership;
- execution quality or best execution;
- settlement/finality;
- investment suitability;
- legal/regulatory compliance;
- tax/accounting correctness;
- autonomous Symthaea financial authority.
