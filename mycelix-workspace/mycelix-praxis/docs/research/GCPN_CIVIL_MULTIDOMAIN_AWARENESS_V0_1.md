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

Late arrival, out-of-order messages, clock drift, duplicate deliveries and retrospective corrections must be reproducible. Corrections append a new version and link to the prior record; they do not erase history. Digests are not decorative identifiers: `field_allowlist_digest` is SHA-256 over the exact UTF-8 bytes of the pinned field-policy artifact; `payload_digest` is SHA-256 over the RFC 8785 canonical JSON bytes of the domain-extension payload. The canonicalization and digest formats are pinned in the field-policy artifact.

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

## 10. Current claim ceiling

The architecture and schema define an implementable research contract. They do not establish actual vessel/aircraft detection performance, navigational or aviation safety, certified system status, vendor superiority, legal compliance, or field readiness. Each of those requires a separate independently governed qualification.

## Issue links

- Multi-domain architecture: https://github.com/Luminous-Dynamics/mycelix/issues/5986
- Maritime awareness: https://github.com/Luminous-Dynamics/mycelix/issues/5987
- Civil aviation awareness: https://github.com/Luminous-Dynamics/mycelix/issues/5988
- Statistical DGP qualification: https://github.com/Luminous-Dynamics/mycelix/issues/5973
- Benchmark integrity: https://github.com/Luminous-Dynamics/mycelix/issues/5979
