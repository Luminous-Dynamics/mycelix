# SUP-CIV-000D3C0 — Protected-read runtime promotion gate v1

Status: blocked promotion contract only. No protected-data runtime implementation is admitted by this tranche.

Exact parent: SUP-CIV-000D3B r2 `e356226ed2d0b3b2eeea4f8aeb854656d5d2788c` / draft #2503.

Tracking issue: #2505.

## Purpose

Freeze the exact evidence gate that must be satisfied before the first SUP-CIV-000D3C protected-read runtime profile can be implemented.

This exists because a review subject, architectural reference, queued workflow, failed qualifier, or toolchain preflight is not the same thing as a qualified capability.

```text
ReviewSubjectExists != DependencySatisfied
QueuedQualification != QualifiedEvidence
HarnessFailure != SemanticFailure
FormatGateFailure != SemanticFailure
ToolchainPreflight != ProtocolCapabilityQualified
D3APass + D3BPass != RuntimeDependenciesSatisfied
AuthorizationEvidenceShape != CurrentAuthorizationSource
AccessReceiptSchema != DurableAccountabilityCommit
```

## Selected profile

The first gated profile is:

```text
EncryptedMultiRecipientProtectedReadV1
```

It deliberately excludes optional high-risk modes:

```text
offline_mode_enabled = false
break_glass_enabled = false
public_publication_enabled = false
```

A successful future runtime for this profile may release only to an admitted protected-consumer boundary. Public/community/research publication remains a separate CIV-RES projection/release path.

## Frozen evidence rule

D3C0 is an immutable evidence cut.

It does **not** continuously query GitHub Actions and it does not become unblocked because a workflow later changes state.

```text
later external PASS
!= this frozen manifest changed
```

A future promotion must be a new exact subject that binds the exact qualifying receipt/run/artifact for every required role.

## Required roles

The encrypted multi-recipient profile requires all of the following:

1. protected/public classification boundary;
2. protected-storage reuse boundary;
3. protected-envelope lifecycle;
4. canonical envelope transcript;
5. payload AEAD profile;
6. multi-recipient PQ recipient-wrap capability;
7. storage topology/profile;
8. D3A composition theorem;
9. D3B admission oracle;
10. exact current authorization source;
11. exact purpose/scope currentness source;
12. reciprocal accountability semantics;
13. durable receipt/escrow commit persistence;
14. protected/public release-separation owner.

Every role is required and every role is unsatisfied in this frozen cut.

## Current evidence posture

### Protected/public boundary

SUP-CIV-000C r2 / #2504 / `5abbbb19c644924738b1de45d28758bd45209533` is an exact repaired subject whose qualifier was queued at freeze time.

```text
QueuedAtFreeze != QualifiedEvidence
```

### Storage/envelope/crypto line

The current review subjects are:

```text
SUP-CIV-000D0   f4f48ef45bbaff7ae9e7668fc1820fdfadd1375f   queued
SUP-CIV-000D1A  3e2d22b711311d91b3fbde8844817f932d5f8226   queued
SUP-CIV-000D1B  1152251e1b8d09bf69cc2497164a0fc1d58da320   queued; duplicate attempts exist
SUP-CIV-000D1C1 afdad857f39e2b989b2218d2a0982cc13cb05948   queued
SUP-CIV-000D2   25ed69f863bbbfc0dfe3b3f49387964d1dcbc1e0   queued
```

None supplies a qualification receipt to this D3C0 cut.

### Recipient wrapping

SUP-CIV-000D1C2 still has no exact recipient-wrap capability/vector subject.

SUP-CIV-000D1C2A / #2481 / `805f68b6c9151bc390896b77bf108c00e33bfc93` is only the pinned OpenSSL 3.6.4 ML-KEM/CMS toolchain preflight.

```text
C2AToolchainPreflight != C2RecipientWrapQualified
```

Therefore C2 remains unsatisfied regardless of whether C2A later passes.

### Composition/oracle

```text
D3A r2 #2488  2ccb30692fb56ec555528860fcffb9421053d547   queued
D3B r2 #2503  e356226ed2d0b3b2eeea4f8aeb854656d5d2788c   queued
```

Even future D3A/D3B PASS cannot substitute for the independent capability dependencies they compose.

### Authorization/currentness

No exact source-of-authorization binding is frozen for this runtime profile.

No exact purpose/scope-currentness source is frozen either.

This is intentionally stronger than accepting an `authority_ref` shape or an `AccessReceipt` field:

```text
AuthorizationEvidenceReference != CurrentAuthorizationDecision
ReceiptAuthorityAssertion != SourceAuthorization
```

### Accountability

The current accountability review head is PR #28 / `165997366ec47fe9b9e5863649766f9b913dcc2c`.

Its accountability-core and verifier workflows failed at rustfmt before tests/clippy; the Civic re-export job passed. D3C0 records this as an unsatisfied format-gate failure, not as a semantic failure and not as a PASS.

More importantly, the runtime profile also needs an exact durable receipt-or-qualified-escrow commit mechanism. No exact qualified persistence adapter is bound in this cut.

```text
AccessReceiptSchema != DurableAccountabilityCommit
ReceiptConstructed != ReceiptDurablyCommitted
```

### Public-release separation

CIV-RES-001B r2 / #2468 / `306aee19526c4e88130372152d64365e509f79eb` was queued at freeze time.

D3C's protected-consumer API will have no public-publication flag. This dependency owns the separate publication/projection boundary; it does not grant protected-read authority.

## Promotion result

The only valid result for this exact subject is:

```text
all_required_roles_satisfied = false
runtime_promotion = Refused
```

A validator failure is required if the manifest claims otherwise.

## Future promotion cut

A future subject may promote only after it binds, for every required role:

```text
exact subject identity
+ exact qualification evidence identity
+ exact selected-profile applicability
+ no stronger claim than the dependency established
```

A new cut must also re-evaluate whether the dependency itself was superseded, repaired, invalidated, or made stale by environment/toolchain/profile changes.

## Optional modes

Offline and break-glass are not part of this selected profile.

They must not be silently enabled merely because the base protected-read runtime later exists.

```text
BaseProtectedReadQualified != OfflineLeaseQualified
BaseProtectedReadQualified != BreakGlassAuthorityQualified
```

## Claim ceiling

A PASS for D3C0 establishes only that this exact repository subject truthfully freezes a blocked runtime-promotion state and the reasons for it.

It does not qualify any dependency, implement D3C, authorize a real actor, read/decrypt/write protected data, establish cryptographic or storage correctness, establish public-release safety, legal compliance, endpoint security, municipal legitimacy, Johannesburg readiness, or deployment readiness.
