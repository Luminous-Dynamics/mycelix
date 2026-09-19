# CIV-RES-000B — Johannesburg deployment / threat / privacy profile v1

Status: deployment-profile template only  
Parent: CIV-RES-000A / `c6a96d40895e71642292b0e2c433b13cc58ecd4d`  
Program: CIV-RES-000 / issue #2006  
Tracking issue: #2011

## Purpose

Freeze the first Johannesburg-specific deployment profile for Civic Resilience without turning Johannesburg assumptions into universal CIV-RES semantics and without authorizing live sensitive-data processing.

This tranche is deliberately configuration/documentation only. It establishes a conservative deployment template, legal-review triggers, threat census, staged rollout boundary, service-channel interoperability posture, and explicit blockers that must remain unresolved before a real pilot.

```text
Johannesburg profile exists
!= Johannesburg deployment authorised
!= POPIA compliance established
!= City adoption
!= municipal authority
!= safety effectiveness
```

## Source posture

This profile uses public official sources only as **context/reference anchors**. Source presence does not grant authority, endorsement, legal compliance, or current operational integration.

### South African privacy-law anchor

Primary statutory source:

`https://www.justice.gov.za/legislation/acts/2013-004.pdf`

The profile exposes review hooks for at least:

- POPIA section 13 — collection for a specific, explicitly defined and lawful purpose;
- section 14 — retention/restriction and destruction/de-identification when continued retention is not authorised;
- section 15 — compatibility of further processing with the collection purpose;
- section 18 — data-subject notification/openess obligations and exceptions;
- sections 19–22 — integrity/confidentiality safeguards, operator safeguards, and compromise notification;
- sections 26–35 — special personal information and children's information;
- sections 57–59 — prior-authorisation regime for specified higher-risk processing;
- section 71 — solely automated decisions producing legal or substantial effects;
- section 72 — transfers of personal information outside the Republic.

These are **engineering review triggers**, not legal conclusions. A deployment must bind an independently reviewed legal/policy basis appropriate to the exact responsible party, purpose, data, operator, and processing flow.

### City-of-Johannesburg context anchors

Current official context sources include:

- City 2026/27 IDP/SDBIP document index: `https://joburg.org.za/documents_/Pages/Key%20Documents/SDBIP%20IDP%20and%20Budget/SDBIP-IDP-And-Budget.aspx`
- City 2026/27 IDP FAQ: `https://joburg.org.za/documents_/Documents/FAQ_IDP_26-27.pdf`
- Joburg Connect service channel: `https://joburg.org.za/services_/JoburgCares/Pages/call-centre.aspx`

The City materials describe service delivery/infrastructure, safety, economic opportunity, digital governance and public participation as municipal planning concerns. CIV-RES records those as context domains only; it does not inherit a political programme, policy preference, or municipal mandate from them.

## Existing-channel compatibility

Johannesburg already has resident/service channels. CIV-RES should prefer bridging/reference preservation over inventing a competing municipal CRM.

Reference-only initial channel vocabulary:

```text
JoburgConnect
CsdMobileApp
eJoburg
EServices
CityPower
JoburgWater
Pikitup
JohannesburgRoadsAgency
Metrobus
```

This profile claims no API availability or integration permission.

Where a City channel provides an external query/reference number, later CIV-RES service semantics should preserve that identifier as external provenance rather than silently replacing it with a Mycelix-owned truth source.

```text
ExternalServiceReference != municipal status truth
CIV-RES mirror != source system
local cache != current external status
```

## Deployment readiness state

The only allowed checked-in v1 state is:

```text
ProfileTemplateOnly
```

and:

```text
real_world_processing_authorized = false
```

The template must remain blocked until a real deployment binds, at minimum:

1. responsible party;
2. operator(s), if any;
3. information officer / accountable privacy role;
4. data steward(s);
5. exact purpose and legal/policy basis;
6. retention/restriction/deletion schedule;
7. disclosure/release policy;
8. cross-border transfer policy;
9. security safeguard/risk-review plan;
10. compromise/incident-response plan;
11. independent legal/privacy review;
12. pilot governance/oversight authority;
13. accessibility/language profile;
14. exit/rollback plan.

No placeholder, repository role, DID, token, model output, software configuration, or GitHub approval may substitute for those bindings.

## Data planes

The profile preserves four distinct planes.

### 1. PublicProjection

For public dashboards, open evidence, public research outputs and aggregate civic observability.

Requirements:

- no person-linked payload by default;
- geographic and temporal coarsening appropriate to release risk;
- disclosure/reconstruction review;
- exact release-policy reference;
- no assumption that aggregation automatically prevents re-identification;
- no live victim/witness/responder location.

### 2. ProtectedOperations

For legitimate case/service operation where person-linked information is actually necessary.

Requirements:

- purpose-bound capability;
- accountable access path compatible with `mycelix-accountability-core`;
- retention and disclosure policy;
- responsible-party/operator identity;
- current authority and role checks;
- minimum-sufficient disclosure;
- explicit currentness/staleness state.

### 3. ResearchEnclave

For exceptional, explicitly authorised analysis that cannot be responsibly performed on public projections alone.

Requirements:

- immutable input snapshot/evidence cut;
- explicit study/purpose contract;
- no raw-person-data egress by default;
- audited access;
- bounded output vocabulary;
- disclosure review;
- model/execution identity;
- no use as general conversational/model memory;
- no automatic mutation of adopted Mycelix state.

### 4. EmergencyEphemeral

For future narrowly scoped emergency functions where precise, time-sensitive data may be necessary.

Requirements:

- separate CIV-SAFE authority;
- explicit consequence/purpose scope;
- short-lived capability;
- bounded recipients;
- expiry and retention transition;
- no public projection of precise location;
- access receipts where person-linked lookup occurs;
- no ordinary reuse after emergency authority expires.

The current CIV-RES-000B profile authorizes no EmergencyEphemeral runtime.

## Sensitivity classes

The Johannesburg profile freezes four initial data-product sensitivity labels:

```text
PublicCivic
CommunitySensitive
Restricted
EmergencyEphemeral
```

These labels are metadata, not access authority. A later executable data-product theorem must combine sensitivity with purpose, responsible party, current capability, release policy and provenance.

## Precision classes

### Geographic

```text
City
Region
Ward
CoarseCell
ExactProtected
```

`ExactProtected` must never appear in a public projection.

### Temporal

```text
HistoricalAggregate
DelayedAggregate
OperationalCurrentProtected
EmergencyCurrentProtected
```

No specific grid size, k-anonymity threshold, delay duration, retention period, or disclosure budget is hard-coded here. Those values must come from a separately reviewed release/deployment policy appropriate to the actual data and threat model.

## Default-deny initial processing classes

Before later dedicated qualification, the Johannesburg profile refuses ordinary admission of:

- biometric processing;
- facial-recognition identification/search;
- criminal-behaviour personal-information processing;
- child personal-information processing;
- solely automated rights-affecting/substantially affecting decisions;
- law-enforcement target generation;
- public exact person/victim/witness/responder location;
- routine person-level cross-domain joins;
- cross-border personal-information transfer;
- permanent emergency-context retention.

A future specialized profile may only widen one of these boundaries through its own authority, legal/privacy, security, qualification and deployment evidence. CIV-RES-000B itself cannot waive the denial.

## Prior-authorisation review flags

CIV-RES does not decide whether POPIA prior authorisation is legally required. Instead, deployment tooling should raise an explicit external-review blocker when a proposed flow resembles any named statutory trigger, including at minimum:

- using unique identifiers for a different purpose and linking across responsible parties;
- processing criminal-behaviour/unlawful-conduct information on behalf of third parties;
- credit-reporting processing;
- certain transfers of special personal information or children's information to foreign recipients without adequate protection.

```text
review flag != legal determination
absence of flag != compliance
legal approval != technical security
```

## Automated-decision boundary

The profile treats person-affecting model outputs as advisory evidence only.

```text
ModelEstimate != AdministrativeDecision
ModelRecommendation != CivicAuthority
model confidence != legal basis
model agreement != human review
```

A future use involving a solely automated decision with legal/substantial effects must be handled as a separate high-risk profile with independent legal analysis and explicit safeguards. It is outside the initial Johannesburg pilot scope.

## Cross-border boundary

Cross-border personal-information transfer is **default denied** in the template.

A later deployment may only enable it after binding an explicit transfer policy/legal basis and recipient/operator safeguards appropriate to the exact flow.

Remote software hosting, observability, backups, crash reporting, support access, model APIs and analytics must all be included in the transfer-data-flow review rather than treating only the primary database as a transfer.

## Security and breach boundary

The profile requires a deployment-specific threat/risk assessment and security plan; it does not claim that cryptography alone establishes adequate safeguards.

The plan must cover at least:

- device theft/loss;
- credential theft;
- insider misuse;
- compromised operator/responder endpoints;
- key/token rotation;
- offline/replay semantics;
- backup/log/telemetry exposure;
- disclosure/mosaic risks;
- software supply chain;
- incident detection and evidence retention;
- compromise-notification workflow;
- recovery and post-incident currentness.

## Johannesburg adversary/threat census

Passing this census means only that threats remain visible. It does not mean they are mitigated.

### JHB-T-001 — insider person-linked query abuse
An authorised or semi-authorised operator attempts lookups outside legitimate purpose.

### JHB-T-002 — lost/stolen resident or responder device
Device compromise exposes credentials, cached records or live operational state.

### JHB-T-003 — abusive household member seeks protected location
A legitimate relationship is abused to locate or monitor another person.

### JHB-T-004 — reporter/witness coercion or intimidation
A person is pressured to reveal, withdraw or alter a report.

### JHB-T-005 — Sybil/brigading/manipulated demand signal
Many identities or coordinated users attempt to manufacture apparent civic priority.

### JHB-T-006 — false report / poisoned operational input
Incorrect or malicious observations enter the evidence surface.

### JHB-T-007 — public aggregate mosaic reconstruction
Repeated/coordinated public queries or releases reveal sensitive small-group/person information.

### JHB-T-008 — unauthorised cross-domain linkage / re-identification
Different domains are joined to construct a person dossier beyond the admitted purpose.

### JHB-T-009 — offline duplicate/replay/reordering
Queued messages are duplicated, replayed or delivered out of causal order after reconnection.

### JHB-T-010 — power/network partition
The system operates for extended periods without reliable connectivity or immediate currentness checks.

### JHB-T-011 — emergency data outlives emergency authority
Precise location/context remains accessible after the emergency scope expires.

### JHB-T-012 — fabricated completion evidence
A service actor claims work is complete without sufficient evidence or independent verification.

### JHB-T-013 — procurement/conflict signal promoted to accusation
Analytical or graph signals are mislabeled as wrongdoing, guilt or sanction authority.

### JHB-T-014 — captured local governance/oversight group
A committee, steward group or local operator attempts to monopolise or redirect system authority.

### JHB-T-015 — unequal digital uptake
Low-connectivity, low-literacy, low-income or device-constrained residents are systematically underrepresented.

### JHB-T-016 — language/accessibility exclusion
Interfaces or processes prevent meaningful participation by users with different language or accessibility needs.

### JHB-T-017 — confidently wrong model / out-of-distribution inference
Symthaea or another analytical component produces persuasive but invalid inference.

### JHB-T-018 — stale model/input snapshot
An analysis is correct only for an obsolete evidence cut but is presented as current.

### JHB-T-019 — geographic displacement
An intervention improves one measured area while moving harm/service pressure elsewhere.

### JHB-T-020 — reporting-rate/measurement shift
Observed incident/service counts change because reporting access or behaviour changed, not because the underlying phenomenon changed.

### JHB-T-021 — unintended cross-border processing path
Cloud, logs, support, analytics or model infrastructure moves personal information outside the Republic without the intended review path.

### JHB-T-022 — compromise-response failure
A breach is detected late, currentness is unclear, affected subjects cannot be identified appropriately, or required notification/recovery steps fail.

## Offline-first boundary

Offline capability is a resilience requirement, not a permission to weaken currentness.

Later operational queues should require:

```text
stable local message id
+ sender/device provenance
+ monotonic local sequence
+ idempotency key
+ bounded queue size
+ explicit created-at evidence
+ authority/currentness state at submission
+ authority/currentness revalidation where consequence requires it
+ duplicate/replay handling
+ reconciliation receipt
```

Disconnection may narrow authority; it must never increase authority.

## Accessibility / participation boundary

A Johannesburg pilot must not equate smartphone/web usage with resident representation.

The deployment profile requires a later explicit accessibility/language/channel plan covering at least:

- low-bandwidth paths;
- assisted/in-person or trusted-intermediary paths where legitimate;
- screen-reader/keyboard accessibility;
- plain-language surfaces;
- multilingual strategy;
- device-sharing privacy;
- non-smartphone participation;
- availability/failure fallbacks that do not silently lose submissions.

No particular language list is frozen by this tranche; the real deployment must bind the community/institution-specific profile.

## Pilot stages

### J0 — Synthetic / profile-only

Allowed now. No real resident data, no live institution effects.

### J1 — Public/reference interoperability

Future: public City information, service-channel discovery, external reference preservation, public aggregate/open-data experiments. No protected person processing required.

### J2 — Protected service operations

Future: service-request/follow-up workflows after CIV-RES-001A/B/C and exact responsible-party/privacy/security/governance bindings.

### J3 — Research enclave / shadow evaluation

Future: immutable study evidence cuts and prospective shadow-mode analysis with no decision authority.

### J4 — Sensitive safety operations

Blocked until separately qualified CIV-SAFE architecture, authority, privacy, security, incident-response and deployment evidence exists.

```text
J0 PASS != J1 authority
J1 PASS != J2 authority
J2 PASS != J3 research permission
J3 evidence != J4 safety authority
```

## Initial pilot bias

The first real-world pilot should prefer lower-risk, high-verifiability functions such as:

- preserving/following service references;
- verified commitments and completion evidence;
- local opportunity/resource discovery;
- public/non-sensitive community capability discovery;
- outcome measurement that does not require person dossiers.

It should not begin with criminal-behaviour profiling, facial recognition, police targeting, public incident maps with precise victim locations, or autonomous risk scoring.

## Qualification scope

CIV-RES-000B qualification may establish only that this exact profile preserves:

- the blocked deployment-readiness state;
- source/reference status;
- review-trigger census;
- data-plane and sensitivity vocabulary;
- default denials;
- threat census;
- pilot-stage monotonicity;
- nonclaims.

It does not establish the truth/currentness of external City systems, POPIA compliance, adequacy of any safeguard, completed legal review, municipal approval, deployment readiness, or effectiveness.
