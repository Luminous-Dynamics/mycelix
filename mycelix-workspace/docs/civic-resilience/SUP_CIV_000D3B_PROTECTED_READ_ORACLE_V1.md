# SUP-CIV-000D3B r2 — Protected-Read Admission Reference Model v1

Status: synthetic decision/reference model only.  
Execution/qualification parent: SUP-CIV-000D3A r2 `2ccb30692fb56ec555528860fcffb9421053d547`.  
Byte-identical corpus/oracle semantic origin: SUP-CIV-000D3B r1 / old D3A `4ebb8389c1f3a11492ec417a0f1f49ec7a6f8d75`.

This tranche executes no cryptography, opens no protected store, reads no protected record, authorizes no real actor, commits no real AccessReceipt and performs no public release. It freezes only deterministic composition decisions for a closed synthetic corpus.

## Repair theorem

D3A r2 changed only dependency-registry truthfulness: the public-release semantic reference now points to repaired CIV-RES-001B r2, while C2 remains explicitly unqualified. The D3A admission dispositions, ordering, linearization rules, offline/break-glass boundaries and authorization/accountability semantics did not change.

Therefore D3B r2 deliberately reuses the original D3B corpus and oracle **byte-for-byte**:

```text
cases blob  = 81579a1cfa6bd843c7430733492524df186f86db
oracle blob = 4837e14b0e0d15beedfd7e7764e69b0f28221d20
```

The embedded old `parent_exact_head` inside those immutable semantic fixtures is retained as their semantic-origin provenance. A separate closed lineage binding records the new executable parent.

```text
CorpusSemanticOrigin != CurrentGitParent
ByteIdenticalCorpus + ByteIdenticalOracle + RepairedLineageBinding
!= SemanticBehaviorChange
```

## Why this layer exists

D3A defines the join theorem, but prose alone does not prove that an implementation will fail closed consistently when multiple dimensions are stale or missing.

D3B executes a tiny independent state machine with the exact D3A dispositions and an explicit diagnostic precedence.

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

```text
receipt_outcome_hint = Allowed
authorization_state = unproven
-> AuthorizationUnproven
```

```text
ReceiptOutcomeAllowed != AuthorizationSource
```

## Offline and break-glass boundaries

`offline_cache_present` remains separate from `offline_lease_state`:

```text
cache present + expired/unproven lease
-> OfflineLeaseExpiredOrUnproven
```

and:

```text
OfflineCachePresent != OfflineLeaseValid
```

Likewise:

```text
EmergencyNeed != BreakGlassAuthority
```

and a successful synthetic break-glass case does not persist ordinary privilege.

## Linearization

The final revalidation model independently surfaces:

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

A storage outage on the same public destination still reports `StorageUnavailable`; the model never falls back to public plaintext.

## Corpus

The byte-identical checked-in corpus freezes 36 unique cases:

- 4 admitted profile-relative protected reads;
- 32 explicit non-admitted dispositions.

It covers classification, topology fit, storage outage, object absence/staleness/conflict, envelope state, recipient-key state, authorization, receipt-hint laundering, purpose/scope, attestations, accountability outage, offline lease behavior, break-glass authority, privilege non-persistence, final TOCTOU changes and protected-read/public-release separation.

## Qualification boundary

A green D3B r2 run can establish only that the **same exact r1 reference program and same exact r1 synthetic corpus** still produce the same frozen dispositions when bound to repaired D3A r2 by the explicit lineage manifest.

It cannot qualify D3A r2 or any D3A dependency. It cannot authorize a real protected read.

## Continuation

```text
000D3A r2 composition contract
000D3B r2 byte-identical synthetic oracle      <- this tranche
000D3C runtime adapter after exact dependencies qualify
000E   accountability persistence/notice/release adapters as required
005    narrow Support -> Civic Resilience adapter
```

## Claim ceiling

No cryptographic correctness, storage correctness, accountability durability, current real-world authorization, public-release safety, legal compliance, municipal legitimacy, endpoint security, Johannesburg readiness or deployment readiness is established.
