# CIV-RES-002A0 — Current-main service-request owner and adapter census v1

**Status:** research / architecture decision record and qualification contract only  
**Snapshot:** Luminous-Dynamics/mycelix main at f16c41b5264c632ca3efa32c78b37bfc21bcfe68  
**Tracking issue:** #2028  
**Runtime changes:** none  
**Live municipal data:** not accessed or authorized

## Purpose

Determine which existing Mycelix domain, if any, should own the Mycelix-side operational state for a Johannesburg service-resolution pilot. This is a convergence gate, not a new ticket system. The census distinguishes the current main branch from open candidate PRs and keeps municipal source authority outside Mycelix unless an actual authorised interface and mapping are established.

The scope is a focused source census of the current-main service-ticket, civic evidence/case, water, work-order, bridge and presentation surfaces. A missing result means no suitable owner was identified in this scope; it is not a proof that no related code exists anywhere in the repository or in another branch.

## Findings

1. Commons Support is the closest existing internal operational ticket substrate. Its ticket record has a generic category, priority, status, requester, optional assignee and timestamps. The coordinator creates and updates those entries and can close a ticket. It does not identify a City/utility request, establish the external provider's current status, encode an adopted Johannesburg SLA clock or prove that a physical repair happened.
2. The current support schema has no separate stable logical municipal issue identity. Action hashes identify Holochain records/actions and update revisions; do not silently treat them as a stable municipal case identity. SUP-CIV-001A proposes a separate stable ticket identity, but that PR was open/unmerged at the snapshot and is not a main-branch guarantee.
3. Mycelix Civic has general justice/media evidence and formal dispute-case types. Those concepts may inform evidence custody, but a routine service request is not a justice case, and storing evidence does not verify its predicate.
4. The community water page is presentation for Mycelix resource data. The Mycelix Water capture/stewardship models cover resources and harvesting; manufacturing work orders encode product/BOM/routing semantics. None should be repurposed into a municipal utility CRM solely because the field names look similar.
5. Commons Bridge supports internal cross-domain dispatch/query/event patterns. That is not an external Johannesburg Water or City of Johannesburg adapter, and does not prove provider authentication, acceptance, currentness, retry semantics or API availability.
6. No deployed municipal provider adapter, municipal service-level clock/registry, or independent physical-repair verification path was identified in the inspected current-main surfaces. Rights-affecting administration also needs its actual procedure and authority owner identified before any automation is designed.

## Owner/adaptor matrix

| Concern | Scoped current-main classification | Decision |
|---|---|---|
| Stable service issue identity | Missing | Block implementation until the stable-identity contract and build-root authority are resolved. |
| Intake creation and local lineage | OperationalOwner (Commons Support, limited scope) | Reuse only as a candidate internal record layer; external source lineage is not yet bound. |
| Status and currentness | OperationalOwner (local support state only); external currentness Missing | Local ticket status must never be represented as the current City/utility status. |
| Assignment and dispatch | OperationalOwner (optional internal assignee only) | No municipal team, fleet, spares or field-dispatch contract identified; Symthaea stays advisory. |
| Resident-facing water view | PresentationOnly | Not evidence of a resident intake, accessibility or service-resolution workflow. |
| Evidence and custody | EvidenceOwner (general Civic evidence only) | Define protected references and a repair-specific verifier; an evidence record is not its own proof. |
| Administrative procedure affecting rights | Missing in scoped current-main census | Identify the lawful procedure/authority owner before benefits, billing disputes, disconnections, licensing or entitlement decisions. |
| External City reference and currentness | Missing | Preserve provider reference as an external typed reference, never mint it as local identity. |
| Completion claim and independent verification | Missing for municipal repair | Separate commitment, claim, evidence, predicate verification and outcome. |
| SLA source and clock | Missing | Pin the adopted, effective target and exact clock semantics before reporting compliance. |
| Offline/retry/idempotency | Missing for a City provider | No live retry integration until duplicate-submission and timeout reconciliation semantics are qualified. |
| Water resource model | NotApplicable to municipal repair operations | Do not treat capture, storage or stewardship as fault repair. |
| Manufacturing work order model | NotApplicable to municipal service requests | Do not repurpose product work orders as municipal case state. |
| Formal justice-case model | NotApplicable to routine service intake | Preserve the boundary between service operations and formal dispute adjudication. |
| Internal Commons bridge | NotApplicable to external provider transport | Internal dispatch is not a municipal connector. |
| Symthaea action authority | Missing / not granted | Recommendations may inform an authorised human; the model cannot grant itself execution or civic authority. |

The machine-readable manifest records exact paths/blob SHAs for inspected current-main candidates and includes fail-closed synthetic probes for common semantic collapses.

## Existing proposals are not inherited guarantees

At the snapshot, these relevant PRs were open/unmerged:

- #2009 — CIV-RES-000A composition boundary: architectural proposal, not active runtime authority.
- #2014 — CIV-RES-000B Johannesburg privacy/threat profile: template only; no live processing or compliance claim.
- #2026 — CIV-RES-001C completion evidence/verification separation: proposed contract, not merged qualification.
- #2045 — SUP-CIV-001A stable support-ticket identity: proposed contract, not on main.
- #2146 — SUP-CIV-002A typed ticket mutation: stacked on #2045, with no inherited qualification.
- #1082 — AC-015 municipal adversarial fixture: open test/docs proposal, not a service-issue owner.

A referenced branch, successful local check, queued workflow or synthetic fixture does not establish that a contract is on main, qualified at its exact head, connected to a provider or authorised for government use.

## Johannesburg source and SLA posture

The official City document index lists 2026/27 planning documents, including the SDBIP and final IDP, while the service-level standards charter listed there is for 2025/26. That older charter can help construct comparison fixtures, but it is not automatically the adopted 2026/27 service target. This census does not bind a current water SLA value. Before defining the clock, confirm the presently approved source, effective period, clock start/stop, dispatch/arrival/repair/restoration events, pauses/exclusions, denominator, reopening treatment and responsible operational owner.

Official source anchors:
- City SDBIP / IDP / service standards index: https://joburg.org.za/documents_/Pages/Key%20Documents/SDBIP%20IDP%20and%20Budget/SDBIP-IDP-And-Budget.aspx
- Joburg Connect channel information: https://joburg.org.za/services_/JoburgCares/Pages/call-centre.aspx
- POPIA Act 4 of 2013: https://www.justice.gov.za/legislation/acts/2013-004.pdf

These sources provide context only; none grants Mycelix API access, procurement approval, institutional authority or a lawful basis for a specific processing operation. The legal/privacy fields are review triggers, not legal conclusions.

## Decision

**Do not create a new generic service-issue owner and do not add runtime behavior in this tranche.** Continue toward a narrow external-source adapter only after authority, identity, source-currentness, SLA, privacy and idempotency gates are explicitly resolved. Commons Support may be evaluated as an internal coordination candidate; it is not designated the City's canonical source of truth.

The next implementation tranche must remain blocked until:
1. the owner/adaptor matrix is reviewed and the canonical repository/build root is confirmed;
2. stable internal identity and typed mutation contracts are qualified at exact heads;
3. the relevant City/utility system, authorised access route and provider semantics are evidenced;
4. a current approved service standard and reproducible clock contract are pinned;
5. privacy/security review and protected-data/retention rules are bound;
6. retries/offline behavior preserve idempotency, source truth and every original report's SLA clock;
7. completion claims are independently checked against an explicit predicate; and
8. synthetic/replay and shadow-mode evaluation demonstrate acceptable outcomes before any live operational trial.

## Safety invariants

- City reference number is not a Mycelix issue ID.
- Local mirror is not authoritative external currentness.
- Local submission or provider acceptance is not physical repair.
- Closing a ticket is not verification.
- Merging duplicate reports must not erase originals or reset clocks.
- Public/distributed receipts must not contain raw resident identifiers or precise protected locations.
- Symthaea is advisory only; no AI-only denial, disconnection, benefit, licensing or other rights-affecting decision.
- An attestation proves a bounded statement about provenance; it is not a truth oracle.
- Existing municipal channels remain available; no new app is required for the census to be useful.

## Evidence ceiling

This work is a scoped code/document census plus schema and synthetic negative-control validation. It does not establish that every repository path was semantically inspected, that a City API exists or is available, that official data was accessed, that current service levels were adopted, that Mycelix is POPIA-compliant, that the City endorses the project, or that the proposed pilot is deployment-ready.
