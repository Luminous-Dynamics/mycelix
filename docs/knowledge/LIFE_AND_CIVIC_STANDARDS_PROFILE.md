# Life and Civic Capability Map: Standards, Rules, Evidence, and Service Outcomes

**Status:** Research / architecture proposal  
**Version:** 0.1  
**Scope:** Cross-domain reference model for Mycelix; not a legal opinion, compliance certification, or production-readiness claim.

## Purpose

Mycelix already has domain clusters for essential resources and services, including housing, care, mutual aid, water, food, transport, justice, emergencies, governance, identity, health, and knowledge. This proposal defines the shared semantic and evidence contract those domains need to cooperate without collapsing distinct kinds of authority into one generic trust score.

The goal is to help a person move from a need or civic problem to a safe, understandable, verifiable next step: discover possible services; understand eligibility and duties; assemble the right evidence; contact the responsible provider; track the response; and challenge or correct an error.

This is a cross-domain composition map, not another replacement registry. Existing LEX-NET work remains the authoritative engineering track for standards conformance, instrument applicability, registry/resolver boundaries, and service discovery: issues #1055, #1069, #1111, #1147, and #1370.

## Governing distinctions

The system must preserve these distinctions:

- **Published standard ≠ applicable law.** A technical standard may be voluntary, incorporated by reference, contractually required, or legally mandatory only in a particular jurisdiction and context.
- **Recognized ≠ trusted ≠ compliant ≠ authorized.** Recognition is a scoped local policy decision; trust is evidence about a source or actor; conformance concerns a defined profile; authorization is a decision by the applicable authority for a specific action.
- **Registry membership ≠ endorsement.** Inclusion in a directory does not prove accreditation, government approval, current competence, or permission to act.
- **Consensus ≠ truth.** Attestations and reputation may be evidence about who supports a claim, but they do not replace source verification, domain expertise, adversarial testing, or independent observation.
- **A successful test ≠ legal compliance or operational safety.** The claim ceiling of every test or receipt must be explicit.
- **No result ≠ a negative result.** Unavailable, stale, ambiguous, conflicting, and unknown must not be silently converted to false, invalid, ineligible, or unauthorized.
- **Current policy ≠ historical policy.** Preserve the evidence and policy version used at the time of a decision without granting old evidence current actionability.

These rules align with the non-authority boundaries already described by Mycelix LEX-NET issues #1055 and #1069, and the registry distinction in #1111.

## 1. A common life-and-civic service lifecycle

All domains should expose a comparable high-level service journey while retaining domain-specific safety and legal rules.

1. **Need or issue:** A person or community reports a need, service gap, hazard, question, or proposed improvement.
2. **Context:** Establish only the necessary context: location or jurisdiction, timing, urgency, access needs, relevant circumstances, and the person's stated goal.
3. **Options:** Identify services, self-help resources, providers, responsible bodies, and alternatives. Discovery must not imply endorsement.
4. **Rules and eligibility:** Retrieve applicable requirements from the correct authority and version. Distinguish law, policy, contractual conditions, technical standards, and voluntary guidance.
5. **Evidence and consent:** Explain required documents or observations, why they are needed, who may see them, how long they are retained, and what can be omitted.
6. **Referral or request:** Create a purpose-bound referral, application, report, appointment, or coordination request only with the needed consent and authority.
7. **Acknowledgement and status:** Record the recipient, receipt, expected response window, next steps, and unresolved dependencies.
8. **Action and outcome:** Distinguish a promise, scheduled action, completed action, and independently observed outcome.
9. **Review and remedy:** Offer correction, complaint, escalation, appeal, mediation, or human review when available.
10. **Closure and learning:** Close only against an explicit outcome or a justified disposition; preserve a privacy-minimized evidence trail and lessons for service improvement.

No common lifecycle may override a domain's clinical, emergency, judicial, safeguarding, or other specialist controls.

## 2. Priority domains

| Domain | Capabilities to represent | Candidate Mycelix composition |
|---|---|---|
| Water and sanitation | Supply interruptions, safe-water advisories, sewer failures, leaks, access, repair status, contamination uncertainty | Commons water + civic emergency coordination |
| Food and essential goods | Immediate need, availability, dietary/safety constraints, collection or delivery, expiry, provenance | Commons food + mutual aid + supply chain |
| Housing and shelter | Housing search, tenancy and repair pathways, shelter availability, accessibility, maintenance, displacement risk | Commons housing/property + emergency shelters |
| Energy and communications | Outages, basic-service dependencies, restoration estimates, backup needs, verified provider updates | Commons resources + emergency communications |
| Health and care | Care navigation, consent, accessibility, medication safety, referral, continuity, emergency escalation | Health + personal vault + care/care circles |
| Identity and entitlements | Documents, credential provenance, application steps, renewal, correction, recovery, benefit navigation | Identity + personal wallet + civic referral |
| Education and capability | Learning resources, credential provenance, practical skill pathways, accommodations, recognition of prior learning | Knowledge + education/practice apps |
| Employment and enterprise | Job pathways, labor and business obligations, licensing steps, cooperative participation, procurement | Finance + knowledge + civic/enterprise apps |
| Transport and accessibility | Routes, service alerts, accessible alternatives, service hours, disruption, safe onward travel | Commons transport + geospatial adapters |
| Justice and civic rights | Rights information, evidence handling, complaint routes, due process, mediation, public participation | Civic justice/media + governance |
| Local services and public works | Report intake, responsible agency, budget/procurement links, work orders, completion evidence | Civic + Commons + governance |
| Community resilience | Volunteer skills, supplies, care networks, local risks, recovery commitments | Mutual aid + care + emergency coordination |

The first implementation slice should be intentionally narrow: water/sanitation, electricity or other critical-service outages, emergency shelter/referrals, and a general civic service-request pathway. These provide concrete end-to-end cases without claiming the platform is a universal government portal.

## 3. Which standards bodies and authorities should be recognized?

Recognize organizations and artifacts by **role**, not by a single global rank. The registry should identify at least the following classes.

### A. Primary law and public-authority sources

Examples for South Africa include the Constitution and Bill of Rights, official legislation and gazettes, the responsible national or provincial department, municipal by-laws and notices, and the Information Regulator's official POPIA/PAIA materials. The South African Constitution establishes rights and state duties; the specific legal rule and its application still require jurisdiction- and fact-specific analysis. SABS is South Africa's national standards body and develops, promotes, and maintains South African National Standards (SANS).

Examples for the United States include official federal statutes and regulations, agency rules and guidance within the agency's remit, state statutes and administrative codes, municipal ordinances, and court decisions where relevant. NIST publications are important technical and risk-management resources, but an NIST document must not automatically be described as binding law.

**Required treatment:** record the exact issuing institution, jurisdiction, instrument class, canonical source, publication and effective dates, amendment/supersession status, and any local applicability analysis. Never infer universal legal force from an institution's prominence.

### B. Formal standards-development organizations

| Organization / family | Recognition use | Important limit |
|---|---|---|
| SABS / SANS | South African product, service, safety, and technical standards metadata | The specific SANS document, edition, adoption, and legal incorporation must be checked; do not republish protected full text without rights |
| ISO and IEC | Quality, safety, accessibility, environmental, energy, security, privacy, and management-system standards | A standard is not automatically legislation, certification, or evidence that an implementer conforms |
| IETF | Internet protocols and RFCs relevant to secure, interoperable communication | RFC status and intended category matter; informational text is not equivalent to a mandatory protocol requirement |
| W3C / WAI | Web interoperability and accessibility, especially WCAG | A guideline's conformance level and version must be explicit; a UI self-check is not an independent audit |
| IEEE | Engineering, electrical, communications, and related technical standards | Applicability depends on the exact standard and domain |
| OGC | Geospatial data and API interoperability, including features, maps, and environmental data | Spatial data still needs provenance, accuracy, license, coordinate-reference, and currentness metadata |
| HL7 / FHIR | Healthcare information exchange and structured health data | FHIR interoperability does not establish clinical validity, consent, or legal permission to disclose a record |
| OASIS | Structured information-exchange and security specifications where used by a selected integration | Version and profile-specific conformance are required |
| ITU-T | Telecommunications and digital infrastructure recommendations | The selected recommendation's status and local/regulatory adoption must be checked |

### C. Open interoperability specifications and sector data standards

These may be stewarded by a standards body, a public-interest organization, an industry consortium, or an open community. Their governance should be recorded accurately rather than all being mislabeled as formal standards bodies.

- **OpenAPI** for describing HTTP APIs.
- **OAuth 2.0 / OpenID Connect** for delegated authorization and identity-related interoperability, used with secure implementation profiles.
- **GTFS Schedule and GTFS Realtime** for public-transport data exchange.
- **Open Contracting Data Standard (OCDS)** for structured publication of procurement stages, contracts, amendments, payments, and implementation.
- **UN/CEFACT** code lists and trade-document interoperability where cross-border commerce is relevant.
- **Wikidata, schema.org, and OpenStreetMap** as useful knowledge, vocabulary, and geospatial sources subject to source-specific quality, licensing, completeness, and coverage limits.
- **NIST AI Risk Management Framework and relevant security guidance** as risk-management references, with explicit treatment of their voluntary or advisory status.
- **OWASP, CVE, CWE, and CVSS** where the software/security risk workflow consumes their distinct vulnerability, weakness, and severity data. Do not confuse a vulnerability identifier with a severity score or proof that a system is exploitable.

This list is a prioritized seed, not an endorsement of every document published by every organization. Selection is use-case-specific and should be expanded only when an adapter or profile needs it.

## 4. Registry record: minimum fields

A standards or authority registry entry should be an immutable, versioned record with at least:

- Stable registry identifier and artifact class: law, regulation, standard, protocol, profile, vocabulary, dataset, guidance, certification scheme, or public directory.
- Issuing organization and the organization's role for this artifact; do not infer authority from the organization name alone.
- Canonical URI, exact title, identifier, edition/version, language, published status, and any official change/history URI.
- Jurisdiction, territorial scope, sector/domain, subject matter, and stated purpose.
- Normative status: draft, published, withdrawn, superseded, in force, advisory, informational, voluntary, or unknown where applicable.
- Publication, effective, expiry, withdrawal, retrieval, and verification times as separate fields when available.
- Legal adoption/incorporation evidence where applicable; keep this distinct from publication by a standards body.
- Source authenticity and integrity evidence, content digest where lawful and technically possible, retrieval method, and verifier version.
- License/access constraints; whether only metadata, a public abstract, a licensed copy, or a full text may be retained.
- Local mapping/profile version, mapped concepts, known omissions, semantic loss, unresolved interpretations, and required human/legal review.
- Currentness horizon, supersession links, conflicting-source links, and explicit Unknown/Indeterminate handling.
- Permitted claim ceiling and prohibited inferences for the artifact.

A record may be technically well-formed while its legal applicability remains unknown. Schema validity must not be treated as legal interpretation.

## 5. Domain adapter contract

Every domain adapter should be able to return typed, inspectable results rather than one opaque confidence or trust scalar:

- **Discovery result:** services/providers found, source and retrieval provenance, supported service class, currentness, and unresolved ambiguity.
- **Rule/applicability result:** exact rule or standard version, jurisdiction and effective interval, evidence used, assumptions, conflicts, and status such as Applicable, NotApplicable, Unknown, or RequiresHumanReview.
- **Eligibility result:** requirements, missing evidence, known exceptions, source and date for each requirement, and an explicit non-decision state when evidence is insufficient.
- **Referral/request result:** consent and purpose scope, minimum necessary fields, recipient and authority evidence, receipt identity, and expected follow-up.
- **Outcome result:** reported, provider-attested, independently observed, or externally verified completion; measurement method and confidence bounds; failures and unresolved steps.
- **Appeal/remedy result:** available routes, deadlines only when sourced and current, responsible reviewer, submission evidence, and current state.

Each result should bind the source content or commitment, captured time, profile version, verifier version, and relevant jurisdiction/subject key. Cross-domain bridges must preserve original source evidence and disclose any translation loss.

## 6. Privacy, access, and dignity requirements

- Collect only data required for the person's explicit purpose and the relevant service rule.
- Do not create a universal, cross-domain profile of a person's health, finances, justice history, civic participation, relationships, or location.
- Prefer purpose-scoped and pairwise identifiers where compatible with the service.
- Keep sensitive evidence in the appropriate vault or controlled domain; share a minimum-necessary claim or proof where feasible instead of the full underlying document.
- Require an actual authority and consent basis for each consequential action; a knowledge claim, credential, or standard-conformance receipt is not sufficient.
- Provide understandable reasons, correction, human review, and an appeal path for adverse or ambiguous results.
- Keep emergency safety intervention separate from ordinary recommendation and referral behavior.
- Design for low bandwidth, intermittent service, multiple languages, screen readers, cognitive accessibility, and assisted or offline pathways.
- Record evidence of service outcomes without turning community contributors or people seeking help into public reputation scores.

## 7. Initial qualification corpus

Build a small, versioned reference corpus before adding operational effects. Include positive, negative, adversarial, stale, ambiguous, and unavailable-source fixtures for at least these cases:

1. A SANS reference is discovered, but no evidence proves that the cited edition applies to the current situation.
2. A voluntary technical standard is incorrectly presented as mandatory law.
3. A local law differs from a generalized international guidance document.
4. A municipal service directory is stale, and no result must remain Unknown rather than “service unavailable.”
5. A provider is discoverable, but identity, competence, or authorization has not been established.
6. A successful schema/TCK validation is incorrectly promoted to legal-compliance status.
7. Many low-quality attestations conflict with one authoritative and current source.
8. A previously applicable instrument has been superseded; the historical result remains reconstructible while current actionability is reevaluated.
9. A water outage report is community-reported, then provider-acknowledged, then independently observed; each stage remains distinct.
10. A service referral would disclose unnecessary health or identity data; the request must be minimized or blocked.
11. A person cannot complete the digital path and needs an offline or human-assisted alternative.
12. A deadline or eligibility answer cannot be verified from a current primary source; the system must not invent a definitive answer.

Every fixture should define expected outputs and explicit nonclaims. The independent verifier should be able to reject a claim that exceeds the evidence even when the underlying inputs look plausible.

## 8. Staged implementation

### P0 — Shared semantics and evidence boundaries
- Establish the registry record shape and the domain lifecycle above.
- Link it to LEX-NET-007 (#1055), applicability (#1069), registry/resolver boundaries (#1111), capability discovery (#1147), and the LEX-NET program graph (#1370).
- Create machine-readable positive and negative fixtures, with exact source versions and explicit claim ceilings.
- Keep external effects disabled.

### P1 — South African service-navigation reference
- Start with one bounded municipality and one service class, such as water interruptions or general municipal service requests.
- Use official municipal and public-authority sources; label community reports as reports, not verified fact.
- Demonstrate report intake, jurisdiction routing, acknowledgement, status, and evidence-backed closure.
- Use SANS only where the selected task needs it, and acquire full normative text lawfully where necessary.

### P2 — Cross-domain and cross-jurisdiction composition
- Add health, housing, identity/entitlement, public procurement, accessibility, and transport profiles one at a time.
- Add United States profiles separately; never assume South African and United States legal rules are interchangeable.
- Qualify translation loss, historical reconstruction, privacy, and failure recovery before any production integration.

### P3 — Public-interest measurement
- Publish privacy-preserving service metrics such as time-to-acknowledgement, time-to-resolution, unresolved share, recurrence, accessibility gaps, and the fraction of closures with independent outcome evidence.
- Treat these as service-quality indicators, not as simplistic rankings of people, workers, or communities.

## Machine-readable reference slice

The first proposed reference artifacts are deliberately non-authoritative and effects-disabled:

- `schemas/life-civic/standard-instrument.schema.json` defines instrument metadata, including exact issuer role, version, lifecycle state, source provenance, access/license limits, mapping gaps, and a claim ceiling.
- `schemas/life-civic/applicability-assessment.schema.json` defines a context-bound assessment. A result marked `applicable` requires evidence and a completed human review; schema validation alone never qualifies a legal conclusion.
- `tests/reference/life-civic-standards/manifest.json` and `cases.json` define the reference corpus identity and expected dispositions. The manifest pins the exact Git blob identities of both schemas and the case corpus; the Rust verifier recomputes canonical Git blob IDs over the checked-out bytes to catch accidental or unreviewed drift. Git SHA-1 here is a repository-object identity, not a signature or source-authenticity proof. These files are test fixtures, not authoritative legal records or benchmark results.
- `schemas/life-civic/service-journey-reference.schema.json` defines an effects-disabled service-case lifecycle and outcome model.
- The service-journey **schema** remains v0.3.0. The active evaluation corpus is now v0.6.0 at `tests/reference/life-civic-service/cases-v0.6.0.json`, alongside the separately pinned `independently-verified-local-restoration-v0.3.0.json` and `manifest.json`. Its ten effects-disabled cases preserve the existing report/provider/conflict/unavailable-source/verification controls and synthetic City-announcement examples for the Johannesburg Water depot close-out, Braamfontein traffic/accessibility upgrades and Malvern Clinic reopening. The v0.6.0 addition is a synthetic clock-semantics control: it keeps the Services page's response wording and the Daily Water Notices category turnaround values in separate, explicitly synthetic evidence records, with null retrieval/check times and no raw page digest. It does not declare a source conflict solely because the labels differ, equate response with restoration, or state that the source values are an established SLA. The current service-journey schema has only broad `general_municipal` coverage for the City project-information examples; it does not yet have dedicated transport-accessibility or healthcare-access classes. The cases test claim/evidence boundaries only, not specialist workflows. The separately pinned positive control still requires captured independent evidence, completed review, and a narrowly scoped outcome claim. The manifest pins the schema and both active fixture files by exact Git blob identity.
- `tools/life-civic-reference-checker/` is an isolated Rust verifier; `.github/workflows/life-civic-reference-contract.yml` runs schema meta-validation and adversarial corpus checks when these paths change. It has no network-dependent reference resolution enabled and does not call service or Holochain effect paths.

The instrument schema preserves an issuer's exact lifecycle label separately from its coarser categories. This matters for standards tracks and committee-stage drafts, where flattening every item to `published` or `standard` would lose consequential distinctions.

Robot safety standards also need exact scope matching.

### Labour-market statistics and decent-work measurement are different evidence classes

Statistics South Africa's QLFS Q2 2026 reported a national official unemployment rate of 33.6% and publishes definitions, metadata, and labour-market series. This is authoritative descriptive evidence for its target population and period, but it is not a causal crime coefficient, not a Johannesburg service-area baseline, and not a predictor of individual behaviour. See [Stats SA QLFS Q2 2026](https://www.statssa.gov.za/?PPN=P0211&SCH=74512&page_id=1856).

The ILO's decent-work indicator framework offers a multidimensional way to specify constructs such as earnings, work safety, stability/security, and social protection. It does not establish the prototype's numeric viability gates and does not prove crime-prevention efficacy. See [ILO Decent Work Indicators](https://www.ilo.org/publications/decent-work-indicators-concepts-and-definitions) and the [ILO measurement framework](https://www.ilo.org/resource/ilo-framework-measurement-decent-work).

The associated Symthaea viability fixture keeps those source roles explicit: contextual labour statistics and measurement frameworks are not fed into the crime-event simulator as effect sizes. Its current numeric gates and low/high values are synthetic tests until locally operationalized, reviewed, and validated.
 The official ISO catalogue lists ISO 10218-1:2025 as an industrial-robot standard and ISO 10218-2:2025 as covering industrial robot applications and robot cells. The latter's catalogue description lists public-access service robots and law-enforcement uses among its exclusions. Therefore, a Mycelix record for a published industrial standard must not be presented as proof that a public-facing service/security robot is covered, safe, authorized, or legally deployable. Record the exact edition, scope and exclusions, then route the actual robot category and use case to a qualified safety/legal review. See [ISO 10218-1:2025](https://www.iso.org/standard/73933.html) and [ISO 10218-2:2025](https://www.iso.org/standard/73934.html).


## Service-journey reference slice

The service-journey v0.3.0 contract deliberately starts with effects disabled and an opaque, purpose-scoped service-area reference. It distinguishes `observed_on` from `retrieved_at`, `checked_at`, and `retrieval_state`: captured official pages carry retrieval and check timestamps, synthetic `example.invalid` fixtures have `retrieval_state=synthetic_fixture` and both timestamps null, and a failed refresh records `retrieval_state=unavailable`, `checked_at`, and `retrieved_at=null`. This avoids representing synthetic records as real source retrievals. The `official_contact_source_found` route state requires evidence IDs that resolve to captured `official_notice` records at a URI listed in the route's source references; it still does not mean that the route is legally or operationally confirmed. It separates the state of the case from the evidential status of the claimed outcome:

- `received` means a report was recorded, not independently confirmed.
- `work_reported_complete` plus `provider_attested_complete` means the provider reported the work complete; it does not prove service restoration.
- `source_conflict` plus `conflicted` preserves contradictory source claims and requires review; `routing_conflict` is reserved for genuinely conflicting destinations or routing instructions.
- `verified_resolved` plus `independently_verified_complete` requires independent observation or measurement and completed human review.

The schema and Rust fixture check reject a deliberately invalid case that promotes a provider statement alone to independent verification. The reference workflow cannot submit a real request, authorize a provider action, or close a service case.

## Checked civic-source conflict (2026-10-14)

The official [Johannesburg Water URL labelled Emergencies](https://www.johannesburgwater.co.za/emergencies/) renders a “Daily water Notices” heading and lists a 12-hour water-outage turnaround, while the [Daily Water Notices URL](https://www.johannesburgwater.co.za/daily-water-notices/) lists 24 hours. A fresh browser inspection at `2026-10-14T19:54:08+02:00` captured both pages and the [Contact Us page](https://www.johannesburgwater.co.za/contact-us/). The latter lists `fault@jwater.co.za` for logging water-related incidents only and `+27 (0)10 061 4470` for logging calls/queries. The captured rendered page content was not cryptographically hashed, so these records claim retrieval time—not immutable source integrity.

The reference case therefore marks the **turnaround claim state** `conflicted`, keeps `route.status=candidate_found`, and requires review. The contact details are a candidate pathway grounded in the official Contact Us page; they do not mean any request was submitted or prove that every issue must use that channel. A live pilot should retain the exact source URL, claim, retrieval/check time, capture state, integrity status, and known disagreement, and must not silently choose a target.

## Checked catalogue observations (2026-10-14)

These examples demonstrate why lifecycle needs to be versioned at the artifact level:

- **ISO 37120:** ISO's catalogue lists `ISO/WD 37120`, Edition 3, as a working draft under development intended to replace ISO 37120:2018. Do not call Edition 3 a published International Standard or certify conformance to the draft. [ISO catalogue entry](https://www.iso.org/standard/90316.html).
- **ISO 37101:** the published `ISO 37101:2016` entry has a separate Amendment 1:2024, while ISO also lists `ISO/WD 37101` as a working draft intended to replace the 2016 edition. The published edition, amendment, and replacement draft are three distinct lifecycle records until an official change establishes otherwise. [Published edition](https://www.iso.org/standard/61885.html), [Amendment 1:2024](https://www.iso.org/standard/88420.html), [replacement working draft](https://committee.iso.org/standard/91450.html).
- **WCAG:** the W3C overview identifies WCAG 2.2 as a published W3C Recommendation and notes ISO/IEC 40500:2025. It says a 2026 ISO publication was expected later in 2026; that expectation must not be converted into a confirmed publication without checking the ISO catalogue again. A guideline's publication status also does not itself establish local legal applicability. [W3C WCAG overview](https://www.w3.org/WAI/standards-guidelines/wcag/).
- **IETF RFC 9943:** the IETF Datatracker identifies this June 2026 document as an Internet Standards Track Proposed Standard. A transparency receipt can establish inclusion of a signed statement in a transparency structure, but it does not establish the statement's truth or make a compromised issuer honest. [RFC 9943](https://datatracker.ietf.org/doc/rfc9943/).
- **OCDS:** the official documentation identifies the published Open Contracting Data Standard as v1.1.5. A validator's pass establishes a defined structural conformance result, not the truth, completeness, legality, or integrity of the underlying procurement process. [OCDS v1.1.5](https://standard.open-contracting.org/latest/en/).

These are point-in-time source observations, not permanent truth labels. Any operational use needs retrieval timestamps, a currentness policy, retained historical versions, and an explicit unknown/stale result when the latest status cannot be established.

## References

- SABS, Standards Division and South African National Standards: https://www.sabs.co.za/standards
- South African Constitution, Chapter 2 — Bill of Rights: https://www.gov.za/ts/documents/constitution/chapter-2-bill-rights
- South African Government, PAIA / Information Regulator mandate background: https://www.gov.za/news/media-statements/information-regulator-takes-over-paia-functions-south-african-human-rights
- ISO management-system standards: https://committee.iso.org/management-system-standards.html
- W3C Web Content Accessibility Guidelines overview: https://www.w3.org/WAI/standards-guidelines/wcag/
- HL7 FHIR overview: https://fhir.hl7.org/fhir/overview.html
- Open Geospatial Consortium standards: https://www.ogc.org/standards/
- GTFS overview: https://gtfs.org/documentation/overview/
- Open Contracting Data Standard: https://standard.open-contracting.org/latest/en/
- NIST AI Risk Management Framework: https://airc.nist.gov/airmf-resources/airmf/
- OpenAPI Specification: https://spec.openapis.org/oas/
- South African Government, joint statement on power trips and water-supply disruptions (10 September 2026): https://www.gov.za/news/media-statements/government-measures-mitigate-water-supply-disruptions-resulting-power-trips

This proposal describes architecture and a candidate test plan only. It does not implement adapters, establish legal applicability, certify conformance, or demonstrate production readiness.
