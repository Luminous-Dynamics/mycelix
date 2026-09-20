# SUP-CIV-000D3B — Protected-Read Admission Reference Model v1

Status: synthetic decision/reference model only. Exact parent: SUP-CIV-000D3A `4ebb8389c1f3a11492ec417a0f1f49ec7a6f8d75`.

This tranche executes no cryptography, opens no protected store, reads no protected record, authorizes no real actor, commits no real AccessReceipt and performs no public release. It freezes only deterministic composition decisions for a closed synthetic corpus.

## Why this layer exists

D3A defines the join theorem, but prose alone does not prove that an implementation will fail closed consistently when multiple dimensions are stale or missing.

D3B therefore implements a tiny independent state machine with the exact D3A dispositions and an explicit diagnostic precedence.

```text
ReferenceModelAdmit != RuntimeAuthorization
ReferenceModelAdmit != DependencyQualification
ReferenceModelAdmit != ProtectedDataReleased
```

## Closed dispositions

The oracle returns exactly one of:

```text
AdmittedProtectedReadUnderProfile
Refused
AuthorizationUnproven
PurposeExpiredOrOutOfScope
StorageUnavailable
ObjectAbsent
ObjectStale
ObjectConflict
EnvelopeInvalid
RecipientKeyStateInvalidOrStale
AccountabilityCommitFailed
RequiredAttestationMissing
OfflineLeaseExpiredOrUnproven
BreakGlassAuthorityUnproven
ReleaseProjectionRequired
```

There is no `bool can_read` output.

## Decision order

For diagnostic determinism the oracle evaluates:

```text
classification/profile
-> storage availability
-> object/currentness
-> envelope validity when required
-> recipient-key state when required
-> current authorization
-> purpose/scope
-> required attestations
-> offline/break-glass gate
-> accountability commit
-> final pre-disclosure linearization
-> public-release boundary
-> protected-read admission
```

The precedence only determines the reported disposition. An early failure never proves later dimensions would have passed.

```text
FailureReasonPrecedence != ProofLaterChecksPassed
```

## Authorization remains independent

The synthetic input includes a `receipt_outcome_hint` specifically to prove it has no authority effect.

A case with:

```text
receipt_outcome_hint = Allowed
authorization_state = unproven
```

must return:

```text
AuthorizationUnproven
```

not admission.

```text
ReceiptOutcomeAllowed != AuthorizationSource
```

## Offline mode

`offline_cache_present` is deliberately separate from `offline_lease_state`.

A present cache plus expired/unproven lease returns:

```text
OfflineLeaseExpiredOrUnproven
```

```text
OfflineCachePresent != OfflineLeaseValid
```

A valid offline case is only `AdmittedProtectedReadUnderProfile` under the frozen synthetic profile; it does not claim current-online authorization.

## Break-glass mode

`emergency_need` does not grant authority.

A break-glass case requires an admitted `break_glass_authority_state` and all ordinary downstream accountability/linearization requirements.

```text
EmergencyNeed != BreakGlassAuthority
```

A separate post-break-glass ordinary-access case proves that previous emergency success does not persist privilege.

## Linearization

The final revalidation model can independently surface:

```text
authorization_changed -> AuthorizationUnproven
object_changed        -> ObjectStale
recipient_key_changed -> RecipientKeyStateInvalidOrStale
policy_changed        -> AuthorizationUnproven
purpose_expired       -> PurposeExpiredOrOutOfScope
unavailable           -> AuthorizationUnproven
```

No initial success survives one of these changes automatically.

## Public release

A fully valid protected read aimed at `public_release` returns:

```text
ReleaseProjectionRequired
```

```text
ProtectedReadAdmitted != PublicReleaseAdmitted
```

A storage outage on the same public destination still reports `StorageUnavailable`; the model never falls back to public plaintext merely to satisfy the destination.

## Corpus

The checked-in manifest freezes 36 unique cases:

- 4 admitted profile-relative protected reads;
- 32 explicit non-admitted dispositions.

It covers classification, topology fit, storage outage, object absence/staleness/conflict, envelope state, recipient-key state, authorization, receipt-hint laundering, purpose/scope, attestations, accountability outage, offline lease behavior, break-glass authority, privilege non-persistence, final TOCTOU changes and protected-read/public-release separation.

## Qualification boundary

A green D3B run can establish only that the exact reference program maps the exact synthetic corpus to the exact frozen dispositions.

It cannot qualify D3A or any of D3A's referenced dependencies. It cannot authorize a real protected read.

## Continuation

```text
000D3A composition contract
000D3B synthetic admission oracle            <- this tranche
000D3C runtime adapter after exact dependencies qualify
000E   accountability persistence/notice/release adapters as required
005    narrow Support -> Civic Resilience adapter
```

## Claim ceiling

No cryptographic correctness, storage correctness, accountability durability, current real-world authorization, public-release safety, legal compliance, municipal legitimacy, endpoint security, Johannesburg readiness or deployment readiness is established.
