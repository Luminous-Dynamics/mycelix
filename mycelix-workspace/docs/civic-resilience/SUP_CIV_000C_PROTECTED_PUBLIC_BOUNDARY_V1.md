# SUP-CIV-000C — Protected support payload / public operational envelope contract v1

Status: semantic contract only  
Parent subject: `main@a85369699099d4c7524e502e531735eed4ab36f4`  
Tracking issue: #2153  
Hardening program: #2030  
Civic program: #2006

## Purpose

Freeze the privacy/data-minimization boundary required before Commons Support can be composed into Civic Resilience.

The existing Support entries are useful operational objects, but Civic deployment introduces data classes that must not be treated as ordinary public DHT payload merely because they are useful for routing or service delivery.

This tranche changes no Rust, Holochain entry definitions, storage mechanism, access policy, municipal semantics, or legal authority.

## Audited source fact

At the parent subject, `support-tickets` declares ordinary app entry variants for support tickets, comments, autonomous actions, alerts, escalations and surveys without an explicit private visibility annotation. Under HDK 0.6 `hdk_entry_types`, entry visibility defaults to public unless explicitly declared private.

The existing `SupportTicket` payload includes person-/case-linkable and potentially sensitive fields such as:

```text
description
requester
assignee
system_info
is_preemptive
prediction_confidence
```

This does not mean every existing Support use is inappropriate. It means Civic Resilience must introduce a stronger data-plane theorem before reuse.

## Core theorem

```text
PublicOperationalEnvelope
+ ProtectedSupportPayloadRef
+ AccessPolicyRef
+ PurposeScopeRef
+ RetentionPolicyRef
+ ReleasePolicyRef
= reviewable Civic support composition
```

not:

```text
ServiceRequest -> DetailsMayBePublic
OperationalRouting -> PublicDisclosureAuthority
PublicDhtReplication -> PrivacyProtection
```

## Closed data-plane vocabulary

The v1 contract distinguishes exactly:

```text
PublicOperationalEnvelope
ProtectedSupportPayload
ProtectedAccessReceiptRef
ReleaseProjectionRef
```

These names describe semantic roles, not mandatory storage technologies.

## PublicOperationalEnvelope

A `PublicOperationalEnvelope` contains only information explicitly admitted for the relevant shared-data profile.

Semantic refs may include:

```text
public_envelope_id
support_ticket_or_case_ref
public_safe_service_class_ref | None
public_safe_lifecycle_ref | None
coarsened_geography_ref | None
public_safe_time_bucket_ref | None
protected_payload_ref | None
classification_profile_ref
release_policy_ref
provenance_refs[]
limitations[]
```

No field is universally declared safe merely because it appears in this candidate vocabulary. A profile must admit its exact projection.

The envelope must not contain unrestricted copies of the protected narrative, precise private location, private contact details, protected attachments, security-sensitive diagnostics, credentials/tokens, or other data classified outside its release profile.

## ProtectedSupportPayload

A `ProtectedSupportPayload` contains operationally necessary case material that is not admitted to the public envelope.

Semantic refs may include:

```text
protected_payload_id
support_ticket_or_case_ref
detailed_narrative_ref | None
precise_location_ref | None
private_contact_or_subject_refs[]
sensitive_attachment_refs[]
restricted_system_or_device_info_ref | None
special_handling_labels[]
provenance_refs[]
purpose_ref
retention_policy_ref
access_policy_ref
```

This contract does not mandate private Holochain entries, encrypted DHT entries, an institutional protected store, or any one cryptographic/storage mechanism. A future runtime must qualify the exact mechanism it chooses.

## ReleaseProjectionRef

Public/community dashboards, research releases, transparency views and geographic summaries must be produced by a distinct disclosure/projection theorem.

```text
ProtectedPayloadAccess != PublicReleaseAuthority
PublicRelease != AccessLegitimacy
```

A public projection may be coarsened, delayed, thresholded, redacted, privacy-budgeted or omitted depending on its qualified release profile.

This contract composes with CIV-RES-001B rather than replacing that release/disclosure layer.

## Free-text boundary

Free text is not public-safe merely because it was submitted into a service workflow.

```text
FreeTextDescription != PublicSafeSummary
```

Detailed narratives can contain names, phone numbers, health information, allegations, criminal-behaviour information, domestic circumstances, security details or other protected material.

If a public-safe summary exists, it is a separate bounded projection with provenance and disclosure review.

## Geography boundary

Operationally necessary precise location and public geography are separate.

```text
NeedForRouting != PermissionForPublicPreciseLocation
```

Exact homes, vulnerable-person locations, responder positions, incident coordinates and similar location facts belong in the protected plane unless an exact release profile establishes otherwise.

## Requester / identity boundary

A Holochain `AgentPubKey` is an identifier, not an anonymity theorem or public-disclosure consent artifact.

```text
RequesterAgentPubKey != AnonymousIdentity
RequesterAgentPubKey != PublicIdentityConsent
PseudonymousIdentifier != AnonymousData
```

Repeated identifier + time + place + category observations may create linkage/re-identification risk.

Civic profiles may therefore use protected mappings, pairwise/purpose-specific identifiers, omission or another qualified identity projection. This contract does not choose one universal technique.

## System-information boundary

The existing generic Support `system_info` field must not become a Civic public-data escape hatch.

```text
OperationalDiagnosticNeed != PublicSystemDisclosure
```

Tokens, credentials, IP/addressing information, internal topology, device identifiers and other security-sensitive diagnostics are forbidden from the public envelope unless an exact classifier/release policy establishes a narrowly safe projection.

## Linked-material boundary

Ticket/public-envelope visibility does not transitively determine the visibility of comments, evidence, attachments, operator notes, surveys or action records.

```text
PublicTicketEnvelope != AllLinkedMaterialPublic
```

Every linked artifact must have its own sensitivity/projection semantics or inherit them through an explicitly qualified policy.

## Access accountability

Protected person-/case-linked access should compose existing reciprocal accountability semantics rather than mint a generic `allowed=true` boolean.

A future protected access receipt should be capable of binding at least:

```text
actor_ref
purpose_ref
scope_ref
subject_or_case_ref
policy_or_capability_ref
access_time_ref
action_ref
notice_review_appeal_refs[]
```

The contract does not define one universal lawful basis, access role or disclosure rule.

## Retention / erasure boundary

Public DHT publication and protected retention are separate design choices.

```text
ApplicationDeleteAction != GuaranteedGlobalErasure
```

If a deployment requires stronger deletion/erasure behavior for sensitive material, that is a reason not to place the sensitive material in ordinary public DHT entry content initially.

The protected plane must bind explicit purpose/retention semantics appropriate to its storage mechanism and deployment profile.

## Legacy payload classification

Existing valid Support records do not automatically become Civic-public-safe records.

The closed v1 classification states are:

```text
LegacyPayloadNotClassifiedForCivicDisclosure
PublicEnvelopeQualifiedUnderProfile
ProtectedPayloadQualifiedUnderProfile
ReleaseProjectionQualifiedUnderProfile
```

A migration may retain legacy evidence while refusing public Civic projection of its raw content.

## Automated/preemptive boundary

Prediction or preemptive routing does not relax disclosure requirements.

```text
PredictionConfidence != PublicDisclosureAuthority
PreemptiveAlert != PublicIncidentTruth
```

Sensitive predictions can themselves create harm or linkage risk and require the same protected/release separation.

## Required runtime refusals

A future executable tranche must cover at minimum:

- phone/name in resident free text cannot enter raw public envelope;
- exact home address used for routing can remain protected while public geography is omitted/coarsened;
- a sensitive comment does not inherit public status from its ticket envelope;
- credential/token/IP/internal-topology material in system diagnostics is refused from public projection;
- public identifier + precise place/time can be suppressed or pairwise-mapped under profile;
- protected access without required purpose/scope evidence is refused;
- public dashboard cannot directly dereference protected payload absent a qualified release projection;
- stale/missing release-history or mosaic policy refuses a release when that profile requires it;
- delete action cannot be advertised as guaranteed global erasure;
- legacy public Support payload is not silently upgraded to Civic-safe publication status.

## Required non-equivalences

The closed v1 registry is:

```text
ServiceRequest != PublicDisclosureConsent
OperationalRouting != PublicDisclosureAuthority
PublicDhtReplication != PrivacyProtection
RequesterAgentPubKey != AnonymousIdentity
RequesterAgentPubKey != PublicIdentityConsent
PseudonymousIdentifier != AnonymousData
FreeTextDescription != PublicSafeSummary
NeedForRouting != PermissionForPublicPreciseLocation
OperationalDiagnosticNeed != PublicSystemDisclosure
PublicTicketEnvelope != AllLinkedMaterialPublic
ProtectedPayloadAccess != PublicReleaseAuthority
PublicRelease != AccessLegitimacy
ApplicationDeleteAction != GuaranteedGlobalErasure
LegacySupportPayload != CivicPublicSafePayload
PredictionConfidence != PublicDisclosureAuthority
PreemptiveAlert != PublicIncidentTruth
```

## Deferred runtime decisions

Explicitly deferred:

- private-entry vs encrypted-shared vs protected institutional storage;
- key/capability distribution;
- exact identity projection technique;
- exact public-safe field classifier;
- precise-location operational storage;
- data-retention engine;
- deletion/erasure implementation;
- CIV-RES-001B release-policy implementation;
- Johannesburg-specific POPIA/legal profile binding.

## Continuation

```text
SUP-CIV-000C  protected/public data-plane contract        <- this tranche
SUP-CIV-000D  executable envelope/payload types
SUP-CIV-000E  protected access + release adapters
SUP-CIV-005   narrow Civic Resilience Support adapter
```

## Qualification claim ceiling

A PASS may establish only that this exact semantic subject preserves a protected/public/release separation, freezes the required non-equivalences and refusal obligations, and does not hard-code one storage or legal policy.

It does not establish anonymity, privacy compliance, correct classification of arbitrary real-world data, irreversible deletion, runtime confidentiality, municipal authority, Johannesburg readiness, or deployment readiness.
