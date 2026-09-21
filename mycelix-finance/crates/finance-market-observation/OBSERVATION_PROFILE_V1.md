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

FIN-MKT-002A is observation-only. FIN-MKT-002B owns deterministic projection over an exact observation frontier.

## Three-layer commitment model

V1 deliberately separates three different questions.

### 1. Identity commitment

```text
which provider-scoped observation identity is this?
```

Identity binds:

- exact FIN-MKT intent commitment;
- exact account subject;
- exact instrument reference;
- exact provider profile;
- optional provider-order ref;
- exact observation profile;
- exact provider observation ID.

Provider IDs are never treated as global identifiers.

### 2. Semantic commitment

```text
what normalized FIN-MKT semantics were observed?
```

For events this includes event class + chronology.
For fills this includes execution ref + exact quantity + exact price + venue + chronology.
For corrections/busts this includes exact adjustment relation + chronology.

Raw provider evidence commitment is **not** part of semantic equivalence.

### 3. Evidence-binding commitment

```text
which exact source evidence supports this identity + semantic claim?
```

Evidence binding commits:

```text
exact identity commitment
exact semantic commitment
exact source-evidence commitment
```

This preserves provenance without conflating evidence aliases with semantic conflict.

## Duplicate / conflict theorem

```text
same identity
+ same semantics
+ same evidence binding
-> ExactReplay

same identity
+ same semantics
+ different evidence binding
-> SameSemanticsDifferentEvidence

same identity
+ different semantics
-> ConflictingReuse

different identity
-> DifferentIdentity
```

This distinction matters when a provider repeats one logical event through multiple independently captured payloads, streams, or evidence envelopes.

```text
new evidence
!= new semantics
```

Arrival order cannot resolve a semantic conflict.

## Subject binding

Every observation binds the exact FIN-MKT order lineage it concerns.

`validate_subject_against_intent_v1` verifies the observation subject against one exact positive `CanonicalMarketOrderIntentV1` for:

- intent commitment;
- account subject;
- instrument ref.

Provider profile/order coordinates remain adapter/provider scope.

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

Provider states that cannot be mapped losslessly remain `OpaqueProviderStatus`.

## Fill observations

A fill semantic commitment binds:

```text
provider execution ref
exact executed quantity
exact execution price
optional venue
typed chronology
```

The evidence-binding layer separately commits the raw/source evidence commitment.

Zero quantity and zero execution price fail closed.

```text
fill observation
!= position ownership
!= qualified settlement
```

002A does not aggregate fills.

## Fill corrections / busts

Historical fills remain append-only.

V1 adjustment semantics:

```text
Correction {
    prior_fill_commitment,
    replacement_fill_commitment
}

Bust {
    prior_fill_commitment
}
```

These references name **fill semantic commitments**, not evidence-envelope commitments. Therefore additional evidence for the same fill does not change correction lineage.

A correction cannot use the same semantic fill as both prior and replacement.

002A establishes only the append-only relation; 002B determines effective projection.

## Chronology boundary

`ProviderChronologyV1` binds:

```text
optional chronology profile
optional provider sequence
optional provider time
```

Chronology evidence and chronology profile must be present together.

```text
provider timestamp
!= trusted current time
!= universal total ordering
```

Ordering semantics belong to the exact provider/profile theorem.

## Canonical encoding

All integers are unsigned big-endian. Text is exact UTF-8:

```text
u32 byte_length || exact bytes
```

All digests are raw 32-byte values in canonical transcripts.

Profile revision:

```text
u32 = 1
```

### Common identity transcript

Per observation type:

```text
TYPE_ID_DOMAIN
revision
observation subject
provider observation ref
```

Domains:

```text
MYCELIX_FIN_MKT_EVENT_ID_V1\0
MYCELIX_FIN_MKT_FILL_ID_V1\0
MYCELIX_FIN_MKT_ADJUST_ID_V1\0
```

### Event semantic transcript

```text
MYCELIX_FIN_MKT_EVENT_SEM_V1\0
revision
[32] identity
event kind
chronology
```

### Fill semantic transcript

```text
MYCELIX_FIN_MKT_FILL_SEM_V1\0
revision
[32] identity
provider execution ref
exact quantity
exact price
optional venue
chronology
```

### Adjustment semantic transcript

```text
MYCELIX_FIN_MKT_ADJUST_SEM_V1\0
revision
[32] identity
adjustment relation
chronology
```

### Evidence-binding transcript

Per observation type:

```text
TYPE_EVIDENCE_DOMAIN
revision
[32] identity
[32] semantic commitment
[32] source evidence commitment
```

Domains:

```text
MYCELIX_FIN_MKT_EVENT_EVIDENCE_V1\0
MYCELIX_FIN_MKT_FILL_EVIDENCE_V1\0
MYCELIX_FIN_MKT_ADJUST_EVIDENCE_V1\0
```

## Frozen independent vectors

Fixture:

`test-vectors/observations-v1.json`

Independent stdlib-Python oracle:

`tests/reference/order_observation_v1_reference.py`

```text
event identity
bd0a7c994a58859f7fd99455433def8e9bfda2781149e90a23192042d60a8b19

event semantic
e1cb0ee74f32f092ce23eae260196d7edff1f4db6f65619a65ad3df5bd740d57

event evidence
dc2a1ad70de2b9219da30e5cd9af8cd78473008ab7eca83204ec9b6d3363345c

fill identity
b6fb55af9d6f0ed1e4b2d5f696f9d8e8bf659efcd20c2ee7d807f492cd8dd134

fill semantic
23121d3de29ccc6f5dce122a7b7475c6ebd86670a063e5ea54ff9901e82ee196

fill evidence
19816667fa931cee5b9ecd9a8579d8d39facc3927c05719dbb736c4f3cad71d0

adjustment identity
f33bcce43385ce6d1f310439360baed8e3dbcc656890ce0e9ba286abbe2ba4d1

adjustment semantic
62e2abcd01078dba27daed66d69e0a1fa9d9a43b6333749f051e5f433d792e73

adjustment evidence
fcec2b73c14fbd78eaf4e18273fdeb2ada4655fe7bed5f337cb163e3b976c476
```

Frozen sizes:

```text
event identity    419
event semantic    171
event evidence    134

fill identity     422
fill semantic     409
fill evidence     133

adjust identity   429
adjust semantic   236
adjust evidence   135
```

## Schema / positive-type discipline

Caller-supplied 002A structs deny unknown fields.

Imported FIN-MKT-001 r3 quantity/price amount wires remain schema closed.

Positive canonical observation values are serializable but not caller-deserializable.

```text
portable input
!= positive canonical observation theorem
```

## Provider mapping firewall

The crate contains no:

- Robinhood/Alpaca SDK;
- FIX engine;
- MCP;
- HTTP/WebSocket;
- Holochain;
- database;
- wall clock;
- OAuth/token/credential;
- Symthaea runtime dependency.

Provider adapters remain separate loss-aware mapping theorems.

```text
provider status text
!= canonical FIN-MKT event by string equality
```

## Nonclaims

A future PASS establishes only deterministic bounded observation identity, semantic normalization, and evidence binding under this exact profile.

It does not establish:

- provider truth beyond supplied evidence;
- account ownership;
- instrument truth;
- currentness;
- financial authority or buying power;
- credential validity/currentness;
- current projected order state;
- fill aggregation;
- position ownership;
- execution quality / best execution;
- settlement/finality;
- suitability;
- legal/regulatory compliance;
- tax/accounting correctness;
- autonomous Symthaea authority.
