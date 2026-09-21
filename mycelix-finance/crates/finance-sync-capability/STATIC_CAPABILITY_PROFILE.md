# FIN-SYNC-003A static rail-capability profile v1

Status: source specification for the first pure FIN-SYNC-003A candidate.

## Claim boundary

A `RailCapabilityProfileV1` describes the static semantics that one exact adapter/build/provider/rail/network/operation profile is designed to support.

It is not live capability evidence and it is not authority:

```text
static profile exists
!= provider capability current
!= adapter instance current
!= credential valid
!= capacity reserved
!= execution plan admitted
!= operation authorized
!= operation dispatched
!= settlement established
```

FIN-SYNC-003B owns currentness/provenance evidence. FIN-SYNC-003C owns execution-plan compatibility.

## Capability dimensions are not an ordinal score

V1 deliberately has no `capability_level`, strength score, or enum ordering theorem.

The profile carries separate closed dimensions for:

- capacity locking;
- transaction preparation;
- commit;
- cancellation/abort;
- query;
- structured evidence production;
- reversal exposure;
- multi-leg atomicity scope;
- idempotency;
- producible finality profiles;
- supported external synchronization profiles;
- disclosure;
- timing;
- resources.

These dimensions can be incomparable.

```text
strong query + best-effort cancel
!= universally stronger/weaker than
prepared commit + weak query
```

A later planner must test exact requirements, not compare enum ordinals.

## Capacity lock != transaction prepare

V1 separates capacity/resource locking from operation preparation.

Capacity-lock primitives include concepts such as provider hold/reserve, funds lock, asset lock, and transaction-scoped lock.

Preparation primitives include concepts such as local pre-validation, prepared-operation handles, conditional instructions, and atomic transaction prepare.

```text
funds reserved
!= transaction prepared

transaction prepared
!= funds reserved
```

This separation is intentional for systems where balance/resource locking and transaction orchestration are distinct steps.

## External synchronization

An external synchronization claim is admitted only when all three facts are present together:

```text
ExternallySynchronizedCommit
+ ExternalSynchronizationProtocol atomicity scope
+ at least one exact synchronization profile reference
```

Any missing leg is an invalid static profile.

A synchronization profile reference still does not prove that the provider or adapter currently supports it.

## Prepared-state consistency

`CommitAgainstPreparedState`, `CancelableWhilePrepared`, and `GuaranteedAbortOfPreparedState` require an actual prepared-state primitive:

- `PreparedOperationHandle`; or
- `AtomicTransactionPrepare`.

`LocalPreValidationOnly` does not create prepared state.

## Finality boundary

`FinalityEvidenceProduction` and `producible_finality_profiles` must either both be absent or both be present.

```text
profile can produce evidence for finality profile F
!= this leg currently satisfies F
```

FIN-ECO owns the qualified finality theorem.

## Idempotency is structured

V1 never stores `idempotent: true`.

The profile binds:

```text
mechanism
key scope
semantic subject scope
retention horizon
retry-after-unknown rule
collision/substitution behavior
```

Important rules include:

```text
NoGuarantee
-> no key/semantic scope
-> no guaranteed retention
-> NeverBlindRetry

ProviderKeyDedupWindow
-> exact non-empty scopes
-> explicit non-zero bounded retention
-> explicit collision semantics

NativeSemanticOperationIdentity
-> native-operation key + semantic scope
-> guaranteed retention profile
-> identity cannot be repurposed
```

An ambiguous external result can therefore never be made safely replayable by the existence of an arbitrary client string.

## Negative markers

Each closed dimension contains an explicit negative marker such as `NoCapacityLock`, `NoPrepare`, `NoCommitPrimitive`, or `NotCancelable`.

A negative marker cannot coexist with positive members of the same dimension.

Duplicates are rejected rather than silently normalized.

## Canonical identity

Profile identity is:

```text
SHA256(
  "MYCELIX_FIN_SYNC_RAIL_CAPABILITY_V1\\0"
  || u16(commitment profile revision = 1)
  || exact canonical fields...
)
```

Rules:

- integers are unsigned big-endian;
- text is `u32(byte_length) || UTF-8`;
- profile references are exact `(id, revision, 32-byte digest)` values;
- capability sets are sorted by explicit canonical tags;
- profile-reference sets are sorted by their language-neutral canonical bytes;
- collection lengths are `u32`;
- optionals use `0x00` or `0x01 || value`;
- serde/JSON field order never enters commitment bytes.

The following are identity-significant:

- adapter profile and exact adapter-build profile;
- provider profile;
- rail/network/operation profile;
- every declared primitive;
- idempotency semantics and retention;
- finality/synchronization profile references;
- disclosure profile;
- timing/resource limits;
- profile revision.

## Secret boundary

The static profile contains no:

- bearer token;
- API credential;
- private signing key;
- provider session secret;
- prepared-operation bearer handle.

Those belong to later live runtime/capability boundaries.

## Operational bounds

V1 binds static bounds for request bytes, batch items, inflight operations per subject, provider deadline, optional prepare lifetime, and optional query polling limits.

These are semantic inputs to later safe planning, not performance hints.

## External architectural grounding

Project Meridian FX demonstrates a technology-neutral synchronization operator coordinating heterogeneous settlement infrastructures. Project Agorá demonstrates an atomic wholesale path in which validation and locking/reservation precede atomic settlement.

FIN-SYNC uses those projects as architectural references only. They do not establish correctness, endorsement, legal status, or production readiness of Mycelix.

## Claim ceiling

A future executable PASS for this exact subject may establish only deterministic construction and canonical identity of the frozen static capability semantics.

It does not establish current capability, provider truth, adapter runtime correctness, credential validity, execution-plan admission, dispatch, settlement, legal finality, regulatory compliance, or commercial satisfaction.
