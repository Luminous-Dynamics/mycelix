# GCPN Civil Multi-Domain Situational Awareness — v0.1

**Status:** research architecture / contract proposal; not a deployment authorization  
**Scope:** civilian UAS, maritime and civil-aviation evidence awareness  
**Implementation posture:** offline-first, read-only, provenance-preserving, human-authority-preserving

## Executive decision

Extend the existing UAS awareness research into a multi-domain civil situational-awareness and evidence-assurance platform. Share the evidence contract, source-trust model, event reconciliation, uncertainty reporting, dataset governance, statistical qualification and lifecycle-cost accounting. Do **not** share a universal detector, a single safety case, an unqualified coordinate model or a cross-domain leaderboard.

The platform's differentiator is not another command-and-control screen. It is an **evidence interoperability layer** that can explain, reproduce and challenge every material observation, association, conflict, abstention and downstream review request.

### Domain profiles

| Profile | Primary civil settings | Candidate data families | Authority that must remain external |
|---|---|---|---|
| UAS / low-altitude | Public safety, site safety, airport/port awareness | Authorized passive sensor event exports, official airspace/context data | Civil aviation authority, site/operator, emergency authority |
| Maritime | Ports, waterways, offshore safety, SAR, environmental response | AIS, authorized radar/EO events, hydrographic/weather/current products, port notices | Vessel master, VTS authority, port authority, SAR coordinator |
| Civil aviation | Airport/airspace context, airport safety, weather/NOTAM reconciliation, SAR handoff | Authorized ADS-B/radar/MLAT events, airport surface status, weather and aeronautical information | Pilot in command, ATC/ANSP, airport operator, aviation regulator |

## 1. Paradigm shifts

### 1.1 From "track" to an evidence graph

A track is a hypothesis assembled from observations, not a ground-truth object. Preserve each source observation and every association/fusion step. Never overwrite source-level assertions with a fused estimate. Keep independently adjudicated outcomes as a separate, governed record.

### 1.2 From confidence scores to calibrated uncertainty and abstention

Every value must identify its semantics, units, uncertainty representation, calibration/currentness state and evidence base. Unknown, stale, missing, contradicted, withheld and out-of-domain are explicit states—not zero, false or a low-confidence class. An abstention is a valid system output.

### 1.3 From sensor diversity to common-mode independence

Three vendor feeds may share the same upstream receiver, clock, network, power, model, data broker or training set. Record dependency groups and common-cause cuts. Count evidence sources only after examining their dependence. A fused estimate must not imply independent corroboration when its inputs are correlated.

### 1.3.1 From sensor count to independence-qualified corroboration

Use a source-dependency DAG rather than counting vendor names, sensors or APIs as independent confirmations. Multiple feeds may share a receiver, antenna, clock, power supply, network, data broker, model, training corpus or upstream authority source. Store declared dependencies and common upstream ancestors; unknown dependencies remain unknown, never independent.

For each event, report raw observation count, declared source count, independent dependency-group count (only when source-defined or independently stewarded), unknown-dependency count, known common-mode ancestors, conflict state and abstention reason. Do not multiply likelihoods or create a single trust score from source count without a qualified dependence model.

Qualify **false corroboration rate** against known dependent-DGPs, then evaluate event decisions using risk–coverage curves, area under the risk–coverage curve (AURC), selective risk at matched coverage, and abstention correctness. The programme now tracks this as [#5995 — independent corroboration and evidence sufficiency](https://github.com/Luminous-Dynamics/mycelix/issues/5995). It reuses Mycelix's existing common-mode dependency graph #5904 and GCPN statistical methodology #5973; it does not create a second graph or bootstrap framework.

### 1.4 From leaderboard scores to challengeable evidence

Maintain three separate evaluation partitions:
1. public development fixtures for reproducible engineering;
2. locked confirmatory data, stewarded independently and frozen before submission;
3. rotating external replication data to test transportability and contamination resistance.

Record every attempted run, including errors, abstentions, timeouts and incomplete results. The candidate artifact, pre-processing, evaluator, configuration, split and source digests are frozen before confirmatory evaluation.

### 1.5 From a single safety score to non-compensatory assurance

Do not combine detection, calibration, privacy, rights, workload, availability and cost into a composite that can hide a critical defect. Privacy/rights/authority violations and unsafe control-path connections are hard blockers, not penalties that can be offset by accuracy.

### 1.6 From proprietary C2 lock-in to a vendor-neutral evidence plane

Vendor systems remain responsible for their own certified/approved functions. Luminous adapters ingest only explicitly authorized, documented exports; preserve vendor assertions as untrusted source claims; and provide reproducible normalization, provenance, correction and audit. Unknown schema revisions fail closed.

## 1.7 Luminous system responsibilities

Keep responsibility partitioned so no subsystem becomes its own source of authority:

| System | Proposed responsibility | Explicit non-responsibility |
|---|---|---|
| **Symthaea** | Semantic interpretation, evidence triage, contradiction discovery, calibrated hypotheses, uncertainty and abstention | Does not convert its own inference into ground truth, identity proof, operator intent or action authority |
| **Mycelix** | Source authority/rights registry, provenance graph, trust and dependency evidence, consent/access policy, signed receipts, correction/revocation and audit | Does not self-authorize source access, journal an unverified fact as truth or command physical systems |
| **Sol-Atlas** | Spatial context and versioned domain layers: approved airspace, port/waterway geography, IHO S-100 products, hazards and map provenance | Does not turn map overlays into verified sensor observations or infer personal location |
| **Symtropy** | Synthetic scenario generation, replay, domain simulation, counterfactual tests, uncertainty/fault injection and benchmark orchestration | Simulation is not physical sensor truth; simulator fidelity must be separately qualified |
| **Luminous product lifecycle layer** | Hardware/software/configuration identity, BOM, calibration and maintenance history, installation records, release digests and field-change traceability | Does not approve regulatory use or substitute for a professional safety case |

### Existing cross-repository contract reuse

Do not duplicate the EMSR dataset/split/run-receipt validators. Symthaea's [draft PR #10875](https://github.com/Luminous-Dynamics/symthaea/pull/10875) contains a reusable Rust crate for pinned dataset catalogues, rights/ingestion state, grouped split manifests, run receipts, SHA-256 helpers and independent-verification declarations. Its exact responsibilities are different from Mycelix #5904:
- Symthaea #10875: dataset/split/receipt contract validation;
- Mycelix #5904: common-mode dependency and failure-domain graph;
- Mycelix #5451: shared evidence-plane maturity and receipt semantics;
- Mycelix #5992: civil envelope structure, field/rights/digest gates;
- Mycelix #5995: evidence-independence, false-corroboration and selective-risk qualification.

The cross-stack composition should pass the same source/split/receipt identities through these owners. No parallel dataset split format, receipt format, dependency graph or trust root is warranted. PR #10875 remains draft/open and its EMSR Evidence Contract workflow success is not whole-PR qualification.

The evidence flow remains:

`source observation → authorized adapter → provenance/rights checks → spatial context → evidence triage → independent review/adjudication → replayable receipt`

Use typed evidence states across services. A Symthaea hypothesis remains a hypothesis; a Sol-Atlas map feature remains a sourced spatial fact; a Symtropy simulated outcome remains synthetic; a Mycelix receipt proves what was recorded and by whom, not that the underlying physical claim is true.

## 2. Common evidence envelope

The canonical envelope is domain-neutral and versioned. Domain-specific payloads are namespaced and versioned separately. See the companion JSON Schema:
- [civil-awareness-evidence-envelope-v0.1.schema.json](./civil-awareness-evidence-envelope-v0.1.schema.json)
- [civil-awareness-field-policy-v0.1.json](./civil-awareness-field-policy-v0.1.json) — shared and domain-specific field allowlists, prohibited field families and fail-closed preflight rules
- [civil-awareness-envelope-examples-v0.1.json](./civil-awareness-envelope-examples-v0.1.json) — three synthetic, non-operational examples

Required conceptual groups:

| Group | Required semantics |
|---|---|
| Identity | Schema version, pseudonymous envelope ID, domain, event-class vocabulary and revision |
| Source | Pseudonymous source reference, source class, product/configuration revision if known, publisher/owner reference, authentication/integrity state |
| Time | Event-time interval, receipt time, clock-quality and uncertainty; correction time is separate |
| Claim | Measurement, source assertion, fused estimate or independently adjudicated outcome; value, units and uncertainty |
| Quality | Freshness, source health, calibration/currentness, missingness, conflict and abstention states |
| Dependence | Explicit dependency/common-mode groups and upstream lineage; no inferred independence from vendor count |
| Spatial context | Purpose-limited location reference and precision class; domain-specific exact geometry is stored only under its own approved access policy |
| Governance | Purpose, rights/authorization status, allowed-use policy, retention/expiry and privacy review state |
| Provenance | Input/output digests, model/evaluator/config digests, correction/revocation links and replay identity |
| Extension | Domain schema URI/revision and allowlisted domain payload |

A common envelope is not permission to centralize all raw telemetry. Store the minimum data needed for the declared purpose. Use separately governed references for high-resolution geometry or restricted evidence. The default Track A benchmark payload should be privacy-minimized and should not contain raw audio/video, personal identity, personal device data or precise remote-operator/controller location.

### 2.1 Claim-type rules

- `measurement`: a sensor or instrument output with units, calibration and uncertainty.
- `source_assertion`: a label/identity/status asserted by a vendor, operator, authority feed or witness; provenance remains attached.
- `fused_estimate`: a derived estimate with the full input/dependency lineage and method digest.
- `adjudicated_outcome`: a separately governed label tied to an adjudication method, evidence version and reviewer authority.

A source assertion must never be silently rewritten as an adjudicated outcome. A fused estimate must never acquire legal authority from a probability threshold.

### 2.2 Event-time and correction rules

Preserve at least:
- source event time or bounded event-time interval;
- source receipt/ingest time;
- clock quality / known offset / uncertainty;
- correction or revocation time;
- validity interval when the source provides one.

Late arrival, out-of-order messages, clock drift, duplicate deliveries and retrospective corrections must be reproducible. Corrections append a new version and link to the prior record; they do not erase history. Digests are not decorative identifiers: `field_allowlist_digest` is SHA-256 over the exact UTF-8 bytes of the pinned field-policy artifact; `payload_digest` is SHA-256 over the RFC 8785 canonical JSON bytes of the domain-extension payload; and `envelope_content_digest` is SHA-256 over the RFC 8785 canonical JSON bytes of the complete envelope after removing only the `lineage.envelope_content_digest` field. The last digest therefore binds source, timestamps, claims, quality, dependency, spatial and governance metadata, plus correction/supersession lineage, without self-reference. The canonicalization and digest formats are pinned in the field-policy artifact. In v0.1, `claim.value` is restricted by a `(domain, claim_type)` allowlist; objects must be flat, and nested objects/arrays are rejected until a domain-specific value schema is registered.

### 2.3 Domain-specific spatial precision

The common contract uses a purpose-limited spatial reference rather than assuming that exact latitude/longitude is appropriate in every domain. Domain adapters may use a higher-precision geometry only where:
- the declared civil safety purpose requires it;
- the source owner has authorized the use;
- a domain safety case and privacy assessment approve it;
- access, retention and export are separately controlled.

Do not ingest or infer operator/pilot/controller personal locations. Do not turn public availability into an unrestricted redistribution right.

## 3. Maritime profile

### 3.1 Domain facts and integration posture

IMO describes AIS as automatically exchanging ship identity, type, position, course, speed, navigation status and other information with equipped ships and shore authorities. SOLAS carriage requirements apply to specified vessel classes; they do not make AIS a complete census of every craft. Treat AIS as a cooperative source assertion, not truth and not proof of physical identity.

IMO's VTS description spans information services through traffic-management support. The vessel master retains responsibility for navigation and manoeuvring. The Luminous layer is therefore a read-only evidence and review adjunct; it must not issue navigational commands or VTS instructions.

IHO's S-100 framework is now operational: Phase 1 product specifications entered into force in January 2026. Pin the exact S-100 edition and the relevant product-specification edition/effective status in each evaluation; do not treat all S-100 specifications as simultaneously in force.

Primary references:
- [IMO — AIS transponders](https://www.imo.org/en/ourwork/safety/pages/ais.aspx)
- [IMO — Vessel Traffic Services](https://www.imo.org/en/ourwork/safety/pages/vesseltrafficservices.aspx)
- [IHO — S-100 framework operational, 10 March 2026](https://iho.int/en/the-s-100-framework-is-now-operational)
- [IHO — standards and specifications](https://iho.int/standards-and-specifications)
- [IALA — G1110, use of decision-support tools in VTS](https://www.iala.int/product/g1110/)

### 3.2 Civil use cases

- Port/waterway information integrity and human review.
- SAR incident timeline reconciliation and handoff completeness.
- Weather, tide, current, visibility and navigational-notice context.
- Port disruption, berth/terminal availability and essential-logistics continuity.
- Environmental incident evidence and recovery status.
- Consent- or authority-governed near-miss and safety-event review.

### 3.3 Benchmark measures

- event-level precision/recall by event class, with operating-hour and exposure denominators;
- AIS stale/missing/duplicated/malformed field handling and false association;
- time and position uncertainty only against a lawful independent reference;
- track continuity under source loss, delayed delivery and receiver/site outage;
- common-mode dependency detection and source-correlation-aware fusion;
- chart/hydrographic product edition, validity and semantic-mapping correctness;
- alert burden, reviewer minutes, handoff completeness and time to stable recovery;
- degraded-mode performance in port clutter, weather, multipath and communications loss;
- lifecycle cost per covered service-hour and verified civil outcome.

Never split message windows from one voyage across development and confirmatory partitions. Synthetic AIS corruption is not a substitute for real multipath, antenna shadowing, weather, port clutter, receiver gaps or actual missing-data regimes.

## 4. Civil aviation profile

### 4.1 Domain facts and integration posture

FAA Aeronautical Information Manual guidance states that ADS-B cockpit traffic display is not intended as a collision-avoidance system and does not relieve the pilot's see-and-avoid responsibility. ADS-B alone does not show unequipped or failed aircraft. This is a useful safety-boundary reference, not a claim that FAA rules govern South African operations.

Treat cooperative ADS-B reports as source assertions. Authorized radar/MLAT data, airport systems, weather and aeronautical information may supply additional observations, but identity, track association, separation assurance and certified collision-avoidance remain separate functions.

The Luminous system must not connect to flight controls, ATC/ANSP command interfaces, ACAS/TCAS, certified DAA or flight-management systems. No generated alert is a manoeuvre instruction or clearance.

Primary references:
- [FAA Aeronautical Information Manual — Surveillance Systems, ADS-B limitations](https://www.faa.gov/air_traffic/publications/aim_html/chap4_section_5.html)
- [South African Civil Aviation Act 13 of 2009](https://www.gov.za/documents/civil-aviation-act)
- [South African Civil Aviation Regulations — 36th Amendment, 2026](https://www.gov.za/documents/notices/civil-aviation-act-thirty-sixth-amendment-civil-aviation-regulations-2026-english)

### 4.2 Civil use cases

- Airport/airspace disruption and hazard information reconciliation.
- Weather, NOTAM and airport-status context for operational planning and emergency services.
- Evidence-quality monitoring of authorized cooperative surveillance feeds.
- Wildlife, obstacle, works-closure and runway-status information integrity.
- SAR support handoff and incident-timeline completeness.
- Authority-approved shared-airspace information for crewed and uncrewed civil aviation.

### 4.3 Benchmark measures

- event-level precision/recall and missed-event rates with traffic-exposure and operating-hour denominators;
- track continuity, timestamp correctness, clock uncertainty and source freshness;
- position/time uncertainty against an authorized independent reference;
- false association, duplicate identity and stale-state rates;
- uncertainty calibration and abstention under sparse/contradictory inputs;
- NOTAM/weather validity and semantic-mapping errors;
- service availability, outage recovery and degraded-mode performance;
- reviewer burden, handoff latency and disruption recovery;
- lifecycle cost, staffing, connectivity, maintenance and calibration.

Stratify by airspace/airport setting, traffic density, equipage and coverage, weather, day/night and source configuration. Keep flight/session, sensor/site, time block and weather-system grouping provenance. Adjacent slices from one flight or one radar capture cannot appear in both development and confirmatory sets.

## 5. Existing industry and standards landscape

| Domain | Existing ecosystem | Luminous position |
|---|---|---|
| UAS | Vendor-specific RF/radar/EO detection and C2; CWA 18150:2024 is a relevant DTI testing-methodology reference | Compare version-pinned, lawful awareness exports and evidence workflows; do not claim physical detection superiority without controlled Track B |
| Maritime | AIS, VTS, ECDIS/ENC and IHO S-100 product ecosystem | Preserve source assertion vs truth, chart/product edition and dependency lineage; augment audit/review, never replace master/VTS authority |
| Civil aviation | ADS-B, radar/MLAT, ANSP/ATC, ACAS/TCAS and airport systems | Improve authorized evidence reconciliation and disruption context; never substitute for certified surveillance/separation/collision-avoidance |

### Public incumbent capability snapshots (vendor descriptions, not independent test results)

| Incumbent | Publicly described capability | What this means for our comparison |
|---|---|---|
| [Wärtsilä Navi-Harbour VTMS](https://www.wartsila.com/marine/products/port-optimisation/vtms) | Vendor describes integration of AIS, radar, marine radio, CCTV, RDF, GMDSS and weather-station data, plus traffic-management, decision-support, open-interface and incident-recording functions. | We must compare against an integrated maritime workflow, not an AIS-only dashboard. Any test must pin the exact deployment/configuration and exclude communications/control functions outside our scope. |
| [Kongsberg Traffic Management](https://www.kongsberg.com/what-we-do/ocean-space/surveillance-monitoring/foresight/traffic-management/) | Vendor describes present/future vessel-traffic views, voyage/clearance workflows, mandatory reporting and incident-response checklists/reporting. | The differentiator cannot be a map alone. Compare evidence provenance, dependency transparency, rights, replay and quality of handoff/incident records. |
| [Saab multilateration and ADS-B](https://www.saab.com/products/multilateration-and-ads-b-solutions) | Vendor describes cooperative surveillance combining multilateration and ADS-B and use within wider ATM systems. | We are not competing with certified surveillance infrastructure; focus on read-only evidence quality and interoperability where authorized. |
| [Saab A-SMGCS](https://www.saab.com/products/a-smgcs) | Vendor describes cooperative MLAT/ADS-B/vehicle sensors plus non-cooperative surface-movement radar, airport safety support, routing and guidance. | Do not build a parallel runway safety/ground-control function. Any integration is read-only and authority-approved. |
| [Indra air-traffic surveillance](https://airtraffic.indragroup.com/en/what-we-do/surveillance) | Vendor describes primary/secondary radar and ADS-B systems for ATM surveillance. | Treat these as incumbent surveillance sources; our independent contribution is provenance, uncertainty, common-mode analysis and auditability across feeds. |
| [EUROCONTROL A-SMGCS](https://www.eurocontrol.int/service/advanced-surface-movement-guidance-and-control-system) | EUROCONTROL describes surveillance as the base service, with optional airport safety support, routing and guidance services. | This illustrates the safety-critical depth of the incumbent domain. Luminous should remain an assurance/evidence adjunct, not replicate its control functions. |

The features above are source-published product descriptions. They do not establish comparative accuracy, safety certification for a particular installation or independent performance. The matched study must identify the precise product and release and use equivalent lawful event frames.

A public vendor feature page is not an independent performance result. Every comparison must identify product, module, hardware/firmware/software/configuration, source publication/retrieval date, claim maturity, evaluation population and denominator. Unknown remains unknown.

## 6. Benchmark plan and target-setting discipline

### 6.1 Shared integrity gates

- 100% of records have a valid schema/domain, source reference, event time and receipt time.
- 100% of confirmatory artifacts bind exact code/model/config/preprocessing/evaluator/input/split-policy digests.
- 0 prohibited source-group overlap in confirmatory splits.
- 100% of initiated runs, including failed/abstained/incomplete, appear in the run census.
- 0 silent defaults for unknown probability, source health, calibration or rights status.
- 0 unauthorized fields, actions or control-path connections in negative controls.
- A 95% interval coverage target is judged against per-DGP-cell Monte Carlo uncertainty; a proposed 93–97% observed-coverage band is a research acceptance hypothesis, not an existing industry standard.

### 6.2.1 Evidence-sufficiency and selective-prediction metrics

Add these to every domain-specific scorecard:
- **False corroboration rate:** fraction of events reported as independently corroborated when their oracle dependency graph contains shared causes or duplicated feeds.
- **Dependency-edge completeness:** source-declared dependency relationships recovered against the independently stewarded fixture graph; unknown edges stay unknown.
- **Selective risk / coverage:** error or false-association risk as the system abstains less or more; publish the full risk–coverage curve and AURC, not just one chosen threshold.
- **Matched-coverage comparison:** baseline and candidate must be compared at the same coverage and event frame before efficiency or accuracy claims.
- **Abstention correctness:** whether the system appropriately leaves unresolved/conflicted/underspecified evidence unresolved, with reason-code accuracy and human-review cost.
- **Common-mode fault detection:** correctly identify when multiple input paths fail together or share a compromised upstream dependency.

Do not set a universal minimum number of sources, a universal coverage point or a composite reliability score. Pre-register thresholds by hazard/use case; zero false independence claims is a deterministic negative-control gate, not a target for real-world prevalence.

### 6.2 Domain metrics (never collapsed into one number)

| Metric family | Reporting requirement |
|---|---|
| Detection/association | Event-level recall, precision, PR curve, false alarms per operating hour, miss rates, class prevalence |
| Spatial/time quality | Error distribution, uncertainty, clock error, freshness and validity; only with approved independent truth |
| Calibration | Reliability curve, Brier/log score where probabilistic semantics are valid, calibration drift and abstention |
| Continuity | Track/event continuity, duplication, correction, source-loss and recovery |
| Dependency | Common-mode failure discovery, correlated-source overcount, upstream lineage completeness |
| Human factors | Review seconds, alerts/operator-hour, acknowledgement/handoff, interruption and override workload |
| Resilience | Availability, degraded-mode service floor, recovery time, data loss and offline operation |
| Economics | Full lifecycle cost, staffing, energy, communications, maintenance, calibration, downtime and decommissioning |
| Rights/safety | Privacy, source rights, authority and safety-case gates; violations block the result |

Do not publish a universal recall or latency threshold before the operational hazard analysis defines the event class, exposure, safety consequence and acceptable false-alert burden. Pre-register thresholds per use case, then test them on locked independent data.

### 6.3 SOTA comparison rules

Compare against best-of-breed systems **within each domain**, on the same event frame and operating conditions. Report sensor detection, downstream evidence workflow, operator workload and lifecycle cost separately. Where the incumbent is certified or safety-critical, Luminous is an adjunct and must not be presented as a replacement. If the baseline cannot export an equivalent lawful event envelope, record NOT_COMPARABLE rather than invent an equivalence.

### 6.4 Statistical qualification

Reuse GCPN's existing paired event-cluster and known-DGP methodology rather than create another generic bootstrap. For the new Track A workload estimand, qualify false-superiority control under nested site/session blocks, informative block sizes, skew/zero inflation, time drift, missing outputs and heterogeneous effects. No row-level pseudo-replication. No superiority claim from a synthetic arithmetic fixture.

## 6.5 Candidate public data: current rights posture

Candidate availability is not a data license. Keep these statuses explicit:
- **HAXR v1.1.0 / Zenodo 19824555 — RIGHTS_PENDING.** DLR's official page explicitly lists **CC BY-NC-ND 3.0** for the 13-station X-band radar/AIS dataset. This is not a commercial-use or derivatives clearance for Luminous. Obtain written DLR permission or a separate commercial agreement before commercial research, feature extraction, derived benchmark creation or ingestion. The aligned AIS data is described by DLR as a pseudo-reference where applicable, not universally adjudicated ground truth. The hourly chunking, UTC time-of-day/cycle groupings and overlapping radar stations require capture/station/hour dependency grouping.
- **MMDEC v1 / Zenodo 17491518 — RIGHTS_PENDING.** The Zenodo license field is blank; OpenAIRE indexes the record as CC BY-NC-SA and the article as CC BY-NC. That is not a commercial-use grant to Luminous. Obtain written confirmation of dataset and component-source rights before admission.
- **OpenSky historical data / API — RIGHTS_PENDING for Luminous.** OpenSky states that for-profit/commercial use needs a written license and operational REST API integration needs a written agreement. Evaluate any curated dataset and supplemental terms separately.
- **NASA ASRS — distinct evidence type.** Its de-identified voluntary safety reports may help build event taxonomies and human-review fixtures, but they are not sensor tracks or ground-truth aircraft positions. Confirm download/reuse/derived-data conditions before use.

Until rights are cleared, use synthetic fixtures and independently authored data only. See the admission registers #5993 and #5994.



## 6.6 Maritime spatial-reference standard snapshot (9 November 2026)

Pin IHO editions and their applicability by publication/effective status, not a generic "S-100 compatible" boolean. The IHO Standards-in-Force page (updated 29 October 2026) lists:
- S-100 Edition 5.2.1 (in force 1 January 2026);
- S-101 Edition 2.0.0, S-102 Edition 3.0.0, S-104 Edition 2.0.0, S-111 Edition 2.0.0;
- S-124 Edition 2.0.0, S-128 Edition 2.0.0 and S-129 Edition 2.0.0 (in force 1 January 2026);
- S-122, S-123, S-127 and S-131 Edition 2.0.0 (listed with entry into force 10 October 2026).

The broader IHO publications page lists S-97 Edition 2.0.0 (November 2026) and S-98 Edition 3.0.0 (November 2026), while the standards-in-force page notes Edition 2.0.0 as the initial operational S-98 version. Record release status, applicability and transition notes per source; do not silently treat publication as mandatory entry into force. These are spatial/data-product interoperability references, not permission for Luminous to issue navigational instructions or replace type-approved ECDIS.

Primary sources: [IHO Standards in Force](https://iho.int/fr/standards-in-force-0) and [IHO Standards and Specifications](https://iho.int/standards-and-specifications).


## 6.7 Civil-aviation surveillance standard snapshot (9 November 2026)

Pin data-format and surveillance-interface editions before comparing any adapter:
- ICAO Annex 10, Volume IV — Surveillance Radar and Collision Avoidance Systems: Sixth Edition, August 2026. It covers SSR Mode A/C/S and ACAS technical characteristics; it is not a licence for a research product to implement operational separation or collision avoidance.
- EUROCAE ED-129C — 1090 MHz Extended Squitter ADS-B Surveillance System: current published edition shown as 18 October 2023; includes partial ADS-B Version 3 and ASTERIX CAT021 support. ED-129C Change 1 is still shown as draft with a target publication date of 31 December 2026, so do not treat that change as published as of this snapshot.
- EUROCONTROL ASTERIX: current surveillance-data exchange library; page now announces Edition 1.6 of Category 025. Part I general surveillance data-exchange specification is listed as Edition 3.1 dated 23 November 2021. Pin the actual category and edition used for each interface rather than claiming generic ASTERIX support.

Research adapters should preserve the original category/edition, SAC/SIC or source identity fields where lawfully available, time-of-day semantics, update and reception time, quality indicators, track status and source lineage. The Luminous layer remains read-only and advisory; it does not provide ATC instructions, separation minima, ACAS/TCAS, flight-control or certified detect-and-avoid functions.

Primary references: [ICAO Annex 10 Vol IV, Sixth Edition](https://store.icao.int/en/annex-10-aeronautical-telecommunications-volume-iv-surveillance-radar-and-collision-avoidance-systems), [EUROCAE ED-129C](https://www.eurocae.net/product/ed-129c-technical-specification-for-a-1090-mhz-extended-squitter-ads-b-surveillance-system/), [EUROCAE WG-51 status](https://www.eurocae.net/working-group/wg-51-sg-4/), [EUROCONTROL ASTERIX](https://www.eurocontrol.int/asterix).

## 6.8 VTS / port workflow reference snapshot

IMO Resolution A.1158(32) (adopted 15 December 2021) is the current published VTS guideline reference in this review. It describes information services, traffic monitoring/management and response to developing unsafe situations. Those are VTS provider functions governed by competent authorities and trained operators; our system should supply evidence, not silently assume VTS authority. IALA's current VTS publication index lists G1141 operational procedures (28 June 2024), G1130 technical information exchange (16 December 2022), and G1156 VTS personnel recruitment/training/certification (12 December 2025).

Benchmark the *handoff* to the existing VTS/port workflow: event evidence completeness, source disagreement, freshness, time-alignment uncertainty, provenance, operator review time, duplicate alerts and correction propagation. Do not benchmark an unapproved Luminous route-advice or traffic-clearance function against certified VTS.

Primary references: [IMO VTS guidance](https://www.imo.org/en/ourwork/safety/pages/vesseltrafficservices.aspx), [IMO Resolution A.1158(32)](https://wwwcdn.imo.org/localresources/en/KnowledgeCentre/IndexofIMOResolutions/AssemblyDocuments/A.1158%2832%29.pdf), [IALA VTS publications](https://www.iala.int/product-category/publications/guidelines/vts/).

## 6.9 Benchmark design: compare at the workflow boundary

Do not compare unlike systems by headline detection percentages. Use the same authorized event frame and output semantics, then report distinct measures:
- **Surveillance/source layer:** detection probability, false plots/tracks per operating hour, track continuity/fragmentation, false association, update interval, latency, coverage and clock uncertainty, all stratified by source class and environment.
- **Evidence-reconciliation layer:** independent-dependency-group correctness, false corroboration rate, unresolved/conflict preservation, provenance completeness, source health, correction handling and replay reproducibility.
- **Human-workflow layer:** time to find the relevant evidence, time to adjudicate, false-review volume, missed escalation, handoff completeness and workload. Measure only where a valid workflow truth/annotation exists.
- **Lifecycle layer:** calibration/maintenance workload, integration cost, data-rights cost, storage/connectivity, downtime, version migration and vendor substitution cost.

An assurance layer can outperform an incumbent workflow without exceeding its radar/ADS-B detection performance, but only if it improves a declared outcome at matched event coverage and does not degrade false-association risk, evidence integrity, rights or human authority. Publish separate Pareto dimensions; never hide a safety-critical regression inside a composite score.

## 7. South African legal and regulatory gate

The current ICASA public guidance describes spectrum-licensing requirements under ECA section 31 and equipment type approval under section 35, with statutory exceptions and regulatory determinations. Do not infer that passive reception is automatically licence-free, that an imported unit is type-approved, or that a foreign vendor's wording settles South African legality.

Before any field activity, obtain a documented determination covering:
- equipment class, transmit/receive functions, frequency bands, installation and type approval;
- spectrum and ship-station licensing where applicable;
- SACAA, ANSP, airport, SAMSA, port/VTS and vessel/operator approvals;
- data-protection, data-owner rights, publication and cross-border transfer;
- professional safety case, human factors, incident reporting and emergency-service command.

Primary references:
- [ICASA — spectrum licensing](https://www.icasa.org.za/pages/spectrum-licensing)
- [ICASA — type approval](https://www.icasa.org.za/pages/type-approval)
- [South African Civil Aviation Act](https://www.gov.za/documents/civil-aviation-act)

This is an engineering issue register, not legal advice. Obtain legal and regulator confirmation for the exact hardware, data and activity.

## 8. Explicit exclusions and fail-closed rules

Prohibited scope includes weapons, interceptors, countermeasures, jamming/spoofing, takeover, mitigation/effectors, targeting, hostile-intent classification, tactical cueing, third-party control, pilot/remote-operator/controller geolocation, private communications, biometrics and covert personal surveillance.

No adapters to vessel navigation control, aircraft flight controls, ATC/VTS command or certified collision-avoidance systems. Any unknown schema revision, unrecognized field, missing rights basis, failed authentication, invalid clock state or unverifiable configuration must be rejected, quarantined or explicitly marked unresolved—not silently normalized.

## 9. Execution sequence

1. **Contract:** freeze the common envelope and domain extension registry; build negative fixtures for prohibited fields and authority boundaries.
2. **Evidence:** reuse #5451 receipt/digest verification and #5899 provenance; add explicit domain/product/schema revision binding.
3. **Statistics:** implement #5973 known-DGP qualification through the existing GCPN methodology; do not duplicate bootstrap code.
4. **Integrity:** implement #5979 source-group/partition, run-census and artifact-freeze controls.
5. **Replay:** create synthetic UAS, vessel-voyage and flight-session fixtures with analytically known truths and controlled common-mode errors.
6. **Public-data pilot:** admit only rights-cleared data through #5960; keep public development data separate from locked confirmatory evidence.
7. **Comparators:** pin exact product revisions and export semantics via #5975; report public claims as vendor claims only.
8. **Physical evaluation:** separate future gates for each domain, with regulator/site authorization, independent truth, human-factors protocol and professional safety review.

## 10. Implementation work

The offline Rust validator and negative-control suite are tracked in [#5992 — fail-closed envelope validation](https://github.com/Luminous-Dynamics/mycelix/issues/5992). It validates structure and digests but deliberately stops short of accepting evidence until an independent source-attestation and authority-receipt verifier is connected to the shared evidence plane.

## 11. Current claim ceiling

The architecture and schema define an implementable research contract. They do not establish actual vessel/aircraft detection performance, navigational or aviation safety, certified system status, vendor superiority, legal compliance, or field readiness. Each of those requires a separate independently governed qualification.

## Issue links

- Multi-domain architecture: https://github.com/Luminous-Dynamics/mycelix/issues/5986
- Maritime awareness: https://github.com/Luminous-Dynamics/mycelix/issues/5987
- Civil aviation awareness: https://github.com/Luminous-Dynamics/mycelix/issues/5988
- Statistical DGP qualification: https://github.com/Luminous-Dynamics/mycelix/issues/5973
- Benchmark integrity: https://github.com/Luminous-Dynamics/mycelix/issues/5979
