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
7. The inspected current-main Commons Support update path is author-bound, but its integrity update validation reruns field-shape checks rather than proving immutable-field restrictions or a Civic-specific status transition algebra.
8. Support creates status/category links at ticket creation; the ordinary ticket-update and close paths inspected do not visibly reindex those links. Status-query currentness therefore remains unqualified.
9. Support's `AutonomyLevel` and `AutonomousAction.approved/executed/success` fields are Support-domain state, not municipal or Civic approval/effect authority.

## Civic Support adoption blockers

Commons Support is an operational-owner candidate, **not currently a Civic-safe write target**. This source review is pinned to the exact current-main blob SHAs in the manifest.

| Blocker | Current-main observation | Required evidence before Civic writes |
|---|---|---|
| Stable logical identity | `SupportTicket` has no separate stable logical municipal issue ID; SUP-CIV-001A remains an open/unmerged proposal. | Exact-head stable identity contract qualified; migration, duplicate genesis, legacy linkage and revision tests pass. |
| Whole-entry mutation | `update_ticket` accepts a complete replacement `SupportTicket`. Integrity update validation reruns field-shape checks; the separate author-binding rule does not prove immutable fields. | Adversarial old/new comparisons prove requester/genesis identity and other protected fields cannot change without authorization. |
| Status/category index currentness | Creation adds status/category links. The inspected update/close paths do not visibly reindex them, while `list_tickets_by_status` queries those links. | Exact multi-revision tests for stale links, reindexing, close/reopen, concurrent updates and delete behavior. |
| Typed lifecycle | The inspected whole-entry update path does not establish a Civic-specific status-transition algebra. | Explicit allowed transition table, invalid-transition rejection corpus and exact-head qualification. |
| Autonomy/effect boundary | Support autonomy levels and action booleans are Support-domain state, not Civic or municipal authority. | Negative controls prove those fields cannot authorize civic dispatch, provider mutation, a rights-affecting decision or completion verification. |

Until these blockers are qualified, the Civic adapter must not mutate Support tickets, rely on its status index as current, treat `Closed/Resolved` as verified service restoration, or map Support autonomy fields into Civic authority. Author binding is useful, but is not a substitute for stable identity, immutable-field rules, transition validation or index correctness.

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
| Support-ticket field mutability | OperationalOwner (Commons Support; Civic safety gate open) | Whole-ticket replacement plus shape validation do not prove immutable-field restrictions; no Civic writes until qualified. |
| Support status-index currentness | OperationalOwner (Commons Support; currentness unqualified) | Status/category links are not visibly maintained on ordinary updates; do not assume the index is current. |
| Support transition algebra | OperationalOwner (generic Support lifecycle; Civic contract missing) | Require typed transitions and exact-head adversarial qualification. |
| Support autonomy authority | OperationalOwner (Support-domain semantics only) | Support autonomy/approval/execution fields must not become civic or municipal execution authority. |

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

The official index lists the separately named Institutional Service Level Standards Charter for 2025/26 and also lists a 2026/27 SDBIP PDF. Opening the City's **final Medium-Term Budget 2026/27–2028/29** PDF in the document viewer exposed a text layer for section 2.3.4 (printed page 125), including a Joburg Water service-standards table. The screenshot request for that page timed out, and the linked SDBIP PDF also timed out when opened in this pass. The current-year budget text is therefore directly observed, but the table has not been visually reviewed or reconciled against the SDBIP/charter.

The extracted text layer of the opened official budget-book PDF lists these Joburg Water service standards for 2026/27:
| Metric | Target shown in indexed text | Clock / scope shown |
|---|---:|---|
| Planned water supply interruptions concluded within prescribed timeline of notification | 50% | Duration of the prescribed timeline is not stated in the extracted row; do not convert to 12 hours |
| Fire hydrants repaired | 80% | Within 48 hours of notification |
| Stolen meters replaced | 90% | Within 24 hours of notification |
| Defective meters repaired | 80% | Within 3 days of notification |
| Leaking valves repaired | 92% | Within 48 hours of notification |
| Missing manhole covers replaced | 95% | Within 24 hours of notification |
| Water pipe bursts repaired | 85% | Within 48 hours of notification |
| Sewer blockages cleared | 92% | Within 24 hours of notification |

The 2025/26 charter separately states that 90% of water-supply interruptions should be concluded within 12 hours. That is **not the same metric text** as the 2026/27 planned-interruption measure. Preserve both with their source year and exact definitions; do not silently reinterpret one as the other.

**Evidence limitation:** the official budget-book PDF opened and its extracted text layer exposed section 2.3.4 and the Joburg Water rows on printed page 125. The screenshot/visual-review operation timed out, so no visual-table inspection is claimed. The current-year rows remain provisional for operational use pending visual review, reconciliation against the separately listed 2026/27 SDBIP, confirmation of the currently adopted target, and exact clock semantics. The table is not live data for any individual service issue. Before calculating compliance, bind the source/version, metric definition, effective dates, clock start/stop/pause rules, denominator, exclusions and reopen treatment. For the planned-interruption metric, the exact prescribed duration remains unresolved.

Official source anchors:
- City SDBIP / IDP / service standards index: https://joburg.org.za/documents_/Pages/Key%20Documents/SDBIP%20IDP%20and%20Budget/SDBIP-IDP-And-Budget.aspx
- Final Medium-Term Budget 2026/27–2028/29, section 2.3.4 text layer (printed page 125; screenshot review timed out): https://joburg.org.za/documents_/Documents/2026-27-Final-Medium-Budget-Reports/2026-27-FINAL-BUDGET-BOOK.pdf
- Official-index-listed 2026/27 SDBIP PDF (opening timed out; not reconciled in this pass): https://joburg.org.za/documents_/Documents/2026-27_SDBIP.pdf
- Johannesburg Water customer-facing channel page (water incident email and daily system-status update): https://www.johannesburgwater.co.za/
- 2025/26 Institutional Service Level Standards Charter: https://joburg.org.za/documents_/Documents/2025-2026-Institutional-Service-Level-Standards-Charter/Institutional-Service-Standards-Charter-2025-26.pdf
- Joburg Connect channel information: https://joburg.org.za/services_/JoburgCares/Pages/call-centre.aspx
- City eServices "Report a city problem": https://edit.eservices.joburg.org.za/Pages/Reportacityproblem.aspx
- Service Level Standards Unit mandate: https://joburg.org.za/departments_/Pages/City%20directorates%20including%20departmental%20sub-directorates/Service-Level-Standards-Unit-.aspx
- Joburg Connect FAQ: https://joburg.org.za/services_/Documents/Customer%20Interface/Joburg%20Connect%20FAQ.pdf
- POPIA Act 4 of 2013: https://www.justice.gov.za/legislation/acts/2013-004.pdf

These sources provide context only; none grants Mycelix API access, procurement approval, institutional authority or a lawful basis for a specific processing operation. The legal/privacy fields are review triggers, not legal conclusions.

## Joburg Water channels are not a unified provider API

The official [Johannesburg Water website](https://www.johannesburgwater.co.za/) lists a customer fault-logging email, `fault@jwater.co.za`, expressly for water-related incidents only; it also lists a call centre and a “Daily water supply system status update”. This is useful evidence of published resident-facing channels. It is **not** an API contract, proof that Mycelix may submit automatically, per-case status readback, or evidence that any repair occurred.

Treat these as different source identities:

| Issuer | Channel | What it can safely mean in the companion |
|---|---|---|
| City of Johannesburg | Joburg Connect / City eServices | A City query/reference and its follow-up history |
| Johannesburg Water SOC | Water-incident email or utility call centre | A separate water-provider interaction; record a reference only if the provider actually returns one |
| Johannesburg Water SOC | Daily water-supply status bulletin | A time-stamped, source-tagged system-wide observation, not an individual ticket status |

The external reference key must include `issuer_id + channel_id + provider_reference`. Equal reference strings from different issuers/channels must not be auto-merged. If an email interaction returns no provider reference, do not invent one; retain an explicitly unreferenced interaction and require manual matching for a later follow-up. A system-wide bulletin may inform context only; it cannot close a resident's case, prove a specific property's supply was restored, or satisfy completion verification without case linkage and an independent predicate.

The companion may explain published channels and let residents initiate contact themselves. This census authorizes no automated email, API call, or City/utility write. Provider currentness, per-case acceptance, and physical repair remain unqualified.

## Channel-first pilot shape (new finding)

The City documents an established intake/follow-up path. Joburg Connect issues a reference number for logged queries, and the City's eServices problem page instructs residents to use that reference on follow-up to prevent duplicate queries. It also documents resident-facing online tracking; this is not a grant of API access or evidence of a machine-to-machine contract.

The safe first user journey is a **resident-controlled companion**, not an autonomous submission bot:

1. Explain official intake and follow-up channels and preserve the existing City intake route.
2. Let a resident privately record an issuer-namespaced, user-reported reference (for example: City of Johannesburg / Joburg Connect / phone). Initial integrity state is `UserReportedUnverified`, not "City accepted" or "verified".
3. Record each follow-up as an append-only local timeline event. Reusing the same issuer + source reference follows up the existing external issue; it must not silently submit a new issue. A new source reference is a separate source record; possible relations to other reports are explicit links, not automatic merges.
4. Keep the raw reference and resident notes in private local/protected storage, never a public DHT or public receipt. Do not collect national ID, account numbers, exact addresses or photos by default.
5. Label status supplied by a resident as resident-reported, with an observation time. Without authorised source readback it is not authoritative currentness.
6. Do not calculate an SLA breach until the effective target, exact metric definition and start/stop/pause rules are pinned. The 2026/27 budget-book PDF text layer is observed, but visual review, reconciliation against the listed SDBIP/charter and clock semantics remain pending; its planned-interruption metric must not be relabelled as a 12-hour general-interruption target.

This creates useful value without assuming a City API, writing to City systems, or duplicating official intake. Because the City also documents offline/manual references, external reference identity must include issuer and channel; a bare reference string is not a safe global identifier.

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
- A utility-wide water-status bulletin is not per-case status readback or physical-repair evidence.
- Existing municipal channels remain available; no new app is required for the census to be useful.

## Evidence ceiling

This work is a scoped code/document census plus schema and synthetic negative-control validation. It does not establish that every repository path was semantically inspected, that a City API exists or is available, that official data was accessed, that current service levels were adopted, that Mycelix is POPIA-compliant, that the City endorses the project, or that the proposed pilot is deployment-ready.
