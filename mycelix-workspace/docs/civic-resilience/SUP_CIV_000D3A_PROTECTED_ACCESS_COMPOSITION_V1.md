# SUP-CIV-000D3A — Protected Access Composition Contract v1

Status: join/composition contract only. No protected data is read, decrypted, written, released, or persisted by this tranche.

Exact structural parent: SUP-CIV-000D0 `f4f48ef45bbaff7ae9e7668fc1820fdfadd1375f`.

The parent relationship is only repository topology. D3A depends semantically on multiple sibling/reference subjects and **inherits qualification from none of them**.

## Purpose

Freeze the admission theorem that joins protected-data classification, storage topology, envelope/crypto validity, exact object currentness, current authorization, purpose/scope, reciprocal accountability and special offline/emergency modes.

D3A creates no new source of civic authority and no second privacy/accountability system.

## Dependency registry rule

Every dependency is one of:

```text
ReferencePendingQualification
NoExactSubjectYet
ExternallyOwned
```

D3A itself cannot promote one of these to `Qualified`.

A runtime successor must replace every load-bearing `ReferencePendingQualification` / `NoExactSubjectYet` dependency it consumes with an exact independently qualified subject.

```text
DependencyReferenced != DependencyQualified
JoinContractPass != ParentCapabilityPass
```

The current public-release semantic reference is repaired CIV-RES-001B r2 / draft #2468 at exact head `306aee19526c4e88130372152d64365e509f79eb`.

SUP-CIV-000D1C2 still has **no exact recipient-wrap protocol/vector subject**. SUP-CIV-000D1C2A / draft #2481 at `805f68b6c9151bc390896b77bf108c00e33bfc93` is only an external OpenSSL ML-KEM/CMS toolchain preflight and cannot satisfy the C2 capability dependency.

```text
C2AToolchainPreflight != C2RecipientWrapQualified
```

## Protected-read theorem

```text
ProtectedDatumClassified
+ StorageProfileAdmitted
+ ExactObjectVersionCurrent
+ CryptographicEnvelopeValid          [when required]
+ RecipientKeyStateAdmitted           [when required]
+ CurrentAccessAuthorization
+ PurposeAndScopeCurrent
+ RequiredAttestationsReady
+ AccountabilityCommitReady
+ OfflineOrBreakGlassProfileSatisfied [when applicable]
= AdmittedProtectedReadUnderProfile
```

Any missing, unknown, stale, conflicting, expired or unavailable load-bearing dimension fails closed into an explicit non-admitted disposition.

## Closed admission dispositions

D3A freezes the following result vocabulary rather than `bool can_read`:

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

These dispositions are evidence states, not interchangeable labels. `StorageUnavailable`, for example, does not mean the policy denied the actor.

## Authorization and accountability

The reciprocal-accountability owner is PR #28, exact review head `165997366ec47fe9b9e5863649766f9b913dcc2c`.

It already defines requester/authenticated-source identity, authority assertion, purpose/scope+expiry, policy version, lookup outcome, disclosure summary, notice, subject rights, query-budget charge and required attestation roles.

D3A preserves its commit-before-disclose invariant but does not reinterpret receipt fields as authority:

```text
AccessReceipt != PermissionGrant
LookupOutcome::Allowed != AuthorizationSource
ReceiptValidation != AuthorizationDecision
AuthenticatedRequester != AuthorizedRequesterForDatum
```

The authorization evidence exists before the receipt records the attempted/admitted lookup.

## Required ordering

For person-linked protected reads:

```text
1. classify datum / resolve protected plane
2. resolve exact object/version/currentness
3. resolve current authorization + purpose/scope
4. validate required storage/crypto/recipient state
5. construct accountability receipt
6. bind required attestations/evidence
7. durably commit receipt or qualified escrow commitment
8. revalidate load-bearing release tokens under the selected linearization profile
9. disclose only the admitted minimum output
```

If accountability persistence/escrow fails, no protected output is released.

## Linearization / TOCTOU

A successful check at lookup start is not enough.

Final disclosure binds at least:

```text
object/currentness token
storage profile/version
crypto envelope/key epoch where applicable
recipient-key state where applicable
authorization/lease version
access-policy version
purpose expiry
accountability commitment
```

Any material change before release requires revalidation or refusal.

```text
AuthorizedAtLookupStart != AuthorizedAtDisclosure
```

## Offline access

Offline access is an explicit bounded mode, not inherited online authority.

A future `OfflineAccessLease` must bind subject/case scope, data class, actor/device, policy and authorization versions, issue/expiry time, maximum offline duration, cache/envelope version ceiling, accountability behavior and reconnect reconciliation.

```text
CachedAuthorization != CurrentOnlineAuthorization
OfflineCacheAvailable != OfflineAccessAuthorized
LeaseValidAtCacheTime != LeaseValidNow
Reconnect != SilentAuthorityRenewal
```

Where online currentness is unavailable, the disposition must remain truthfully profile-relative rather than claiming current-online authorization.

## Emergency / break-glass

Emergency access requires its own authority lineage:

```text
explicit emergency authority
+ narrow purpose/scope
+ short expiry
+ actor/device binding
+ reason/justification commitment
+ required approvals/witnesses
+ commit-before-disclose accountability
+ mandatory post-use review
+ applicable notice/delayed-notice rule
```

```text
EmergencyNeed != EmergencyAuthority
BreakGlassUsed != PermanentPrivilege
EmergencyAccess != PublicReleaseAuthority
```

## Public release boundary

Protected read admission is not publication authority.

CIV-RES-001B r2 / draft #2468 at exact head `306aee19526c4e88130372152d64365e509f79eb` remains the current repaired release semantic reference. It is still pending its own exact qualification and D3A inherits none of its authority.

```text
AdmittedProtectedReadUnderProfile != PublicReleaseAdmitted
```

A dashboard, map, community aggregate or research output separately consumes release-history/budget/disclosure-review evidence where required.

## Read/write separation

Protected write/update authority is independent from read authority and must bind exact parent/current version, mutation class, storage profile, envelope/key transition where required, and accountability/evidence obligations.

```text
ReadAuthorized != WriteAuthorized
Decryptable != Mutable
```

## No weaker fallback

```text
protected store unavailable
crypto profile unavailable
accountability store unavailable
current authorization unavailable
= no protected disclosure
```

Never silently fall back to public plaintext, a weaker crypto profile, stale authorization, an unlogged read, or a cached object outside its admitted lease.

## Required non-equivalences

```text
StorageLookupSucceeded != AccessAuthorized
DecryptionSucceeded != AccessLegitimate
AccessReceipt != PermissionGrant
LookupOutcomeAllowed != AuthorizationSource
ReceiptCommitted != PublicReleaseAuthorized
AuthenticatedRequester != AuthorizedRequesterForDatum
QualifiedEnvelope != QualifiedStorage
QualifiedStorage != CurrentAuthorization
AuthorizedAtLookupStart != AuthorizedAtDisclosure
CachedAuthorization != CurrentOnlineAuthorization
OfflineCacheAvailable != OfflineAccessAuthorized
EmergencyNeed != EmergencyAuthority
BreakGlassUsed != PermanentPrivilege
ReadAuthorized != WriteAuthorized
ProtectedReadAdmitted != PublicReleaseAdmitted
DeleteOrRetire != RetroactiveKnowledgeErasure
DependencyReferenced != DependencyQualified
JoinContractPass != ParentCapabilityPass
```

## Runtime gate

D3A may qualify while dependencies remain pending because its only theorem is composition structure and refusal behavior.

`SUP-CIV-000D3C` may **not** run a protected-data runtime profile until every load-bearing dependency for that profile has an exact qualifying receipt. In particular:

```text
AES-GCM vector PASS
+ ML-KEM recipient-wrap UNQUALIFIED
!= encrypted distributed protected-storage profile qualified
```

and:

```text
C2A toolchain preflight PASS
+ C2 recipient-wrap protocol/vector UNQUALIFIED
!= encrypted distributed protected-storage profile qualified
```

## Continuation

```text
000D3A composition/dependency contract          <- this tranche
000D3B independent admission state-machine oracle
000D3C runtime adapter after exact dependency qualification
000E   accountable persistence/notice/release adapters where still required
005    narrow Support -> Civic Resilience adapter
```

## Claim ceiling

A PASS establishes only this exact join vocabulary, dependency-status discipline, ordering, fail-closed composition laws and non-equivalences. It does not qualify any referenced dependency, authorize any real person, decrypt/read/write protected data, establish public-release safety, legal compliance, municipal legitimacy, Johannesburg deployment readiness, endpoint security, global erasure or retroactive knowledge revocation.
