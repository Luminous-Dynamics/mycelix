# GCPN–SADC–AMHEWAS Interoperability Alignment v0.1

**Status:** public-source architecture mapping and synthetic conformance proposal.  
**Parent:** issue #5606.  
**Boundary:** GCPN is an optional evidence/provenance and conformance layer. It is not an alerting authority, a regional situation room, a forecasting centre, or a replacement for CAP, SADC SHOC or AMHEWAS.

## 1. Strategic decision

Do not build a second Africa-wide warning centre.

The African Union Commission's AMHEWAS programme (2022–2030) already connects continental, regional and national situation-room capacity. UNDRR describes a free/open-source platform for analysis and integration of forecasting data with standardized bulletins, and includes the SADC Humanitarian and Emergency Operations Centre (SHOC) in its supported regional expansion.

SADC's Regional Climate Resilience Project is currently seeking consulting services to strengthen national MHEWS capacity and regional coordination through SHOC, including interoperability with AMHEWAS, CAP and other open standards, people-centred impact-based warnings, responsible AI/ML, cross-border coordination and anticipatory action. Its notice was issued 28 October 2026 and lists a closing date of 17 November 2026.

This is a strong signal that the actual programme gap is not "another alert platform". It is durable interoperation, institutional capacity, data quality, last-mile reach, common procedures and evidence of whether warnings result in early action.

## 2. Existing ecosystem and roles

| Actor/system | Existing role (publicly documented) | GCPN relationship |
|---|---|---|
| National meteorological and hydrological services | Monitoring, forecasting and hazard information | Do not replace forecasts; preserve origin, issue time, uncertainty and source provenance |
| National disaster management / competent alerting authorities | National warning authority, public instruction and coordination under national law | GCPN records the authority assertion and CAP lineage; it never authorizes a warning |
| National MHEWS and dissemination channels | National multi-hazard warning chain and local channel operation | Preserve local independence and receiver evidence; future adapters only with permission |
| SADC SHOC | Regional coordination, harmonized warning practice, monitoring and information exchange | Optional interoperability and conformance interface; no command over national authorities |
| AMHEWAS / African Union Commission / regional economic communities | Continental and subregional risk information, interoperable situation rooms, standardized bulletins and early-action coordination | Preserve references and provenance of public bulletins; never fork or replace regional situation-room outputs |
| IFRC / National Societies / communities | Preparedness, response and community-level early action | Measure accessibility and action only through voluntary, ethically governed trials |
| GCPN | Proposed provenance sidecar, conformance oracle, independent audit and synthetic resilience evaluation | Additive, optional and non-authoritative |

An alerting-authority signature, evidence record, SHOC coordination record and AMHEWAS bulletin are different objects with different issuers and legal status. A GCPN receipt must not make them interchangeable.

## 3. Standards and programme fit

### Common Alerting Protocol

CAP 1.2 remains the alert-message interchange baseline. The GCPN profile binds a provenance sidecar to an exact CAP message identity and digest. It does not change CAP semantics, replace national CAP profiles, create a transport protocol, or require a CAP consumer to understand GCPN.

### Early Warnings for All

The design maps to all four pillars:
1. risk knowledge and governance;
2. observation, monitoring, analysis and forecasting;
3. warning dissemination and communication;
4. preparedness and response.

GCPN's strongest distinctive contribution is in cross-pillar evidence lineage and conformance: being able to trace a public warning from the authorized CAP message back to its source references, through dissemination and receiver receipts, to an accountable after-action record.

### AMHEWAS and SADC

Use the publicly documented structures and exchange artefacts as the integration boundary:
- Continental Watch bulletins;
- Africa Drought Watch;
- disaster situation reports;
- standardized regional bulletins;
- publicly documented SHOC and regional coordination procedures;
- national CAP 1.2 profiles where publicly available.

The first adapter should be read-only and synthetic/public-data-only. Do not connect to live situation rooms or private feeds without a formal agreement, security review, legal basis and operational owner.

## 4. GCPN's non-duplicative technical contribution

### A. Provenance sidecar

An optional and separately versioned sidecar can carry:
- exact CAP message binding;
- source-record and source-family references;
- observation / receipt / verification timestamps;
- uncertainty and contradiction status;
- authority assertion ID and validation-profile reference;
- custody predecessor references;
- purpose / retention / disclosure rules;
- public receiver-receipt references.

It must not include sensitive facility details, private-person data, raw restricted sensor locations or protected source identities in public CAP content.

### B. Independent conformance

The GCPN evaluator tests:
- exact CAP message identity/digest binding;
- preservation of CAP multi-language, area, severity, urgency, certainty and update/cancel semantics;
- explicit distinction between authenticity and observation validity;
- source-family dependence rather than report-count inflation;
- authority-bound issuance and uncertainty disclosure;
- no sensitive provenance leakage into a public alert;
- source-to-alert and alert-to-receiver traceability;
- append-only history and dispute preservation.

### C. Local-first continuity

Local national and subnational authorities must retain their locally authorized procedures during WAN or regional federation partitions. The model tests message-path reachability only; local legal authority and emergency procedures remain external and unchanged.

### D. Receiver and early-action evidence

Measure separately:
- published CAP message;
- channel dispatch report;
- receiver-side delivery receipt;
- accessible-format availability;
- voluntary acknowledgement;
- independently defined preparedness/early action evidence;
- post-event outcome and loss assessment.

Do not claim human comprehension or protective action from a transport receipt. These require separate evaluation and, where people participate, consent and ethics review.

## 5. Synthetic SADC-style test families

Use only invented regions, receiver cohorts and hazard-event identifiers. Public hazard types may include:
- cyclone and coastal storm;
- riverine and flash flooding;
- drought and water scarcity;
- extreme heat and urban heat;
- wildfire and smoke;
- compound infrastructure disruption.

Cross-border scenarios may test whether two or more fictional member authorities can exchange a CAP-compatible warning reference and a standardized situation update while preserving distinct issuers, authority, timestamps, corrections and uncertainty.

Do not use live operational forecasts, exact critical infrastructure topology, restricted situation-room data, private feeds, or actual emergency channels in this benchmark.

## 6. Evaluation matrix

| Dimension | Primary measure | Hard failure |
|---|---|---|
| CAP interoperability | Round-trip semantic preservation within declared field scope | Silent loss or mutation of declared CAP semantics |
| Provenance | Exact source/authority/message binding | Unbound or mismatched sidecar accepted |
| Evidence epistemics | Authenticity, measurement validity, uncertainty and source-family dependence separated | Authentic record silently promoted to verified truth |
| Sovereignty | Authority references and member write boundaries | Unauthorized issuance or cross-member mutation |
| Partition resilience | Local continuation when supported local path and authority remain available | WAN partition suppresses all local service despite declared local fallback |
| Last mile | Receiver-side receipts by channel/cohort | Server dispatch counted as receiver delivery |
| Accessibility | Accessible-format availability and delivery by cohort | Aggregate hides excluded language or accessibility cohort |
| Early action | Auditable, authorized early-action receipt and outcome linkage | Delivery receipt represented as proof of action |
| Privacy | Unauthorized disclosure and correlation exposure | Sensitive provenance copied into public payload |
| Operational sustainability | Cost, staffing, maintenance, training and lifecycle burden | One-off prototype presented as sustainable service |

No composite score is allowed to offset a hard sovereignty, safety, privacy or accessibility failure.

## 7. Industry comparison protocol

Do not compare GCPN to "Africa's warning system" as if that were one product. Compare compatible layers and defined tasks:

1. CAP message conformance: GCPN mapping versus the relevant public national/regional CAP profiles.
2. Interoperability: CAP-only baseline versus CAP plus optional GCPN sidecar.
3. Regional topology: hub, regional federation and local-first reference topologies under identical synthetic fault rows.
4. Last mile: cell broadcast, radio/TV, mobile data/SMS, satellite and community relays only when a public/permissioned test adapter exists.
5. Human outcomes: voluntary, consented receiver and community studies, not synthetic delivery counts.

ITU's cell-broadcast paper notes that mobile-network coverage is high globally but adoption of cell-broadcast warning systems remains much more limited; its report identifies funding, institutional capacity, infrastructure and policy barriers. Thus cell broadcast should be tested as one valuable channel, not assumed to be universal protection.

A GCPN better-than-baseline claim requires paired scenarios, common receiver denominators, matched budgets, independent evaluator reproduction, confidence intervals, held-out cases, and transparent subgroup outcomes. No named SADC, AMHEWAS or national system should be ranked without an authorized, compatible and independently evaluated adapter.

## 8. Governance and legal boundary

- National alerting authorities retain authority to issue, update and cancel national public warnings.
- SADC SHOC and AMHEWAS retain their own coordination roles and operating mandates.
- GCPN cannot issue, amend, suppress or cancel public warnings.
- GCPN cannot allocate anticipatory finance or direct early-action resources.
- GCPN cannot independently determine physical truth, legal authority or treaty compliance.
- Participation and data exchange are subject to formal agreements, national law, data protection, retention, public-record and cybersecurity requirements.
- Local authorities and affected communities must participate in design and evaluation; they are not merely downstream endpoints.

## 9. Current opportunity signal

The SADC Regional Climate Resilience Project consultancy notice (reference SADC/3/5/2/474) is an open, firm-shortlisting request, issued 28 October 2026 with a published closing date of 17 November 2026. Its scope seeks technical and institutional capacity-building services, not a standalone software vendor. This is evidence of active demand in the ecosystem, not evidence that GCPN is selected, endorsed, qualified or a substitute for an eligible consulting consortium.

Public notice: https://www.sadc.int/procurement-opportunities/consultancy-undertake-capacity-building-arising-identified-mhews-gaps-and

## 10. Non-claims

This alignment proposal is not endorsed by SADC, SHOC, the African Union Commission, AMHEWAS, UNDRR, WMO, ITU or IFRC. It does not establish a partnership, integration, data access or operational capability. It does not operate a warning network, create an authority, or provide military defence. It is a research plan for optional interoperability, provenance and independent conformance work.
