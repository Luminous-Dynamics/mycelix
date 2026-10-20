# SUP-CIV-000B — Support-ticket intake attribution / delegated intake contract v1

Status: semantic contract only  
Parent subject: `main@a85369699099d4c7524e502e531735eed4ab36f4`  
Tracking issue: #2152  
Hardening program: #2030  
Civic program: #2006

## Purpose

Freeze the attribution theorem required before a Support ticket's `requester` field can be treated as authenticated requester evidence by Civic Resilience or by the future stable-identity runtime.

The current coordinator accepts a caller-constructed `SupportTicket`, including its requester field. The inspected integrity validation establishes ticket shape but does not establish requester == create-action author. SUP-CIV-000B therefore separates payload attribution from payload validity.

This tranche changes no runtime code, Holochain schema, Civic authority or delegation mechanism.

## Core theorem

```text
RequesterField
+ SubmitterEvidence
+ IntakeMode
+ exact attribution/delegation policy
= RequesterAttributionDisposition
```

not:

```text
RequesterField = authenticated requester
Civic gate passed = requester impersonation authority
AgentToTicket link = requester authorization
```

## Closed intake-mode vocabulary

The v1 intake modes are:

```text
SelfIntake
DelegatedIntake
AutomatedOrPreemptiveIntake
LegacyIntake
```

## Closed attribution-disposition vocabulary

The v1 dispositions are:

```text
SelfAuthoredVerified
DelegatedVerified
AutomatedSourceVerified
LegacyAttributionUnverified
AttributionConflict
AttributionInvalid
InsufficientAttributionEvidence
```

These dispositions concern attribution of intake/requester semantics only. They do not establish legal identity, service entitlement, jurisdiction, need validity, case truth or public authority.

## SelfIntake

The default self-submitted Support theorem is:

```text
SupportTicket.requester == create_action.author
```

A runtime self-intake path must derive or validate requester against the actual create action author at the integrity boundary. A UI/coordinator check alone is insufficient if direct zome calls can bypass it.

Successful self-intake may yield `SelfAuthoredVerified`.

```text
SelfAuthoredVerified != LegalIdentityVerified
SelfAuthoredVerified != ServiceEntitlement
```

## DelegatedIntake

On-behalf intake is legitimate in many workflows, but requires explicit evidence instead of arbitrary requester substitution.

A delegated intake evidence object should be capable of binding:

```text
submitter_ref
represented_requester_ref
delegation_or_authority_ref
purpose_ref
scope_ref
validity_or_session_ref
provenance_ref
```

A `DelegatedVerified` disposition requires the exact delegated-intake policy to establish that the submitter may represent that requester for that purpose and scope at that time.

```text
Submitter != RepresentedRequester
DelegationExists != DelegationInScope
```

A delegation for requester A or purpose X cannot be reused for requester B or purpose Y unless its owning semantics explicitly permit it.

## AutomatedOrPreemptiveIntake

A system, diagnostic process, adapter or Symthaea-assisted predictor may originate an operational ticket without masquerading as human self-intake.

The evidence should distinguish:

```text
submitted_by_system_ref
represented_requester_ref | None
source_alert_or_observation_ref
intake_authority_ref
provenance_ref
```

Successful system-originated attribution may yield `AutomatedSourceVerified`.

```text
SymthaeaPredictedTicket != HumanRequestedTicket
AutomatedSourceVerified != HumanConsent
```

## LegacyIntake

Historical tickets created before this theorem may not contain enough evidence to establish self-authorship or valid delegation.

The safe default is:

```text
LegacyAttributionUnverified
```

unless stronger evidence actually establishes another disposition.

Do not rewrite the historical requester field merely to make migration easier.

```text
LegacyRequesterField != VerifiedRequesterAttribution
```

## AttributionConflict

Use `AttributionConflict` when available evidence materially disagrees, for example:

- requester payload says A while validated delegated evidence says B;
- two incompatible delegation records claim different represented requesters;
- migration sources disagree on the original requester attribution.

Conflict must remain explicit rather than selecting the convenient source silently.

## AttributionInvalid

Use `AttributionInvalid` when the evidence positively violates the applicable profile, such as self-intake requester != author with no delegated path, an expired/out-of-scope delegation, or a represented requester different from the delegation's subject.

## InsufficientAttributionEvidence

Use this disposition when the needed dependencies cannot be established safely. Absence of evidence is not equivalent to positive invalidity.

```text
InsufficientEvidence != AttributionInvalid
```

## Stable identity boundary

SUP-CIV-001A defines the canonical logical identity payload from genesis facts. That theorem may establish:

```text
canonical requester ref == requester encoded in genesis
```

but it does not establish:

```text
genesis requester was authorized/authenticated
```

Therefore:

```text
CanonicalIdentityMatchesGenesis != GenesisRequesterAuthorized
```

The future SUP-CIV-001B runtime must carry/bind attribution disposition separately if it exposes stronger requester claims.

## AgentToTicket index boundary

An `AgentToTicket` link is a discoverability projection after attribution; it is not proof of attribution.

```text
AgentToTicketLink != RequesterAuthorization
```

A later current-index theorem must derive requester indexes from admitted attribution/current-state semantics rather than use link existence as the source of truth.

## Civic-gate boundary

Passing a generic Civic/Sovereign participation requirement does not authorize an agent to impersonate arbitrary requester identities.

```text
CivicGatePassed != AuthorityToImpersonateRequester
```

Eligibility and representation/delegation authority are distinct theorems.

## Privacy boundary

Requester attribution and requester publication are separate.

```text
VerifiedRequesterAttribution != PublicRequesterDisclosureAuthority
```

SUP-CIV-000C owns the protected/public data-plane boundary. A requester may be correctly attributed while their identifier remains protected from public projection.

## Required runtime refusals

A future executable tranche must demonstrate at least:

- self-intake requester == create author -> admitted;
- self-intake requester != create author without delegation -> refused/invalid;
- direct integrity validation cannot bypass requester binding;
- coordinator/UI cannot silently substitute a different requester;
- exact in-scope delegation for represented requester -> admitted under profile;
- expired/revoked/out-of-scope delegation -> refused;
- delegation for requester A cannot name requester B;
- automated/preemptive intake is distinguishable from human self-intake;
- AgentToTicket link alone cannot upgrade attribution;
- legacy unknown attribution remains explicitly unverified;
- conflicting attribution evidence produces conflict rather than silent winner.

## Required non-equivalences

The closed v1 registry is:

```text
RequesterField != RequesterAuthorization
CanonicalIdentityMatchesGenesis != GenesisRequesterAuthorized
AgentToTicketLink != RequesterAuthorization
CivicGatePassed != AuthorityToImpersonateRequester
Submitter != RepresentedRequester
DelegationExists != DelegationInScope
SymthaeaPredictedTicket != HumanRequestedTicket
AutomatedSourceVerified != HumanConsent
LegacyRequesterField != VerifiedRequesterAttribution
SelfAuthoredVerified != LegalIdentityVerified
SelfAuthoredVerified != ServiceEntitlement
InsufficientEvidence != AttributionInvalid
VerifiedRequesterAttribution != PublicRequesterDisclosureAuthority
```

## Deferred runtime decisions

Explicitly deferred:

- exact delegation/capability representation;
- institutional on-behalf role semantics;
- caregiver/accessibility-assistant consent semantics;
- system/preemptive intake authority mechanism;
- legal identity binding;
- Civic service-entitlement semantics;
- requester public/private projection;
- stable-identity runtime integration.

## Continuation

```text
SUP-CIV-000B  intake attribution contract               <- this tranche
SUP-CIV-000B1 executable self/delegated intake boundary
SUP-CIV-001B  runtime stable ticket identity
```

SUP-CIV-001B may bind the genesis requester fact but must not advertise authenticated requester attribution unless the relevant SUP-CIV-000B runtime theorem is also satisfied.

## Qualification claim ceiling

A PASS may establish only that this exact semantic subject distinguishes self, delegated, automated and legacy intake attribution; freezes explicit conflict/uncertainty states; and prevents requester payload/index/generic civic eligibility from becoming representation authority by convention.

It does not establish human legal identity, service entitlement, delegation validity in any real institution, municipal jurisdiction, privacy compliance, Civic authority, Johannesburg readiness or deployment readiness.
