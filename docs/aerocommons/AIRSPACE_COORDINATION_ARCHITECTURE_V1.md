# Airspace Coordination Architecture V1

**Status:** Research/architecture contract; not an operational service  
**Authority ceiling:** planning, simulation, interoperability, and evidence organization only  
**Research checked:** 2026-10-14  
**Tracking issue:** [AIRSPACE-COMMONS-001](https://github.com/Luminous-Dynamics/mycelix/issues/5023)  
**Related work:** [AeroCommons #3638](https://github.com/Luminous-Dynamics/mycelix/issues/3638), [Mobility Commons #3708](https://github.com/Luminous-Dynamics/mycelix/issues/3708)

## 1. Executive thesis

If aircraft become a much larger share of passenger and freight mobility, the bottleneck will not be aircraft design alone. It will be the coordinated use of shared airspace amid changing weather, heterogeneous vehicle capabilities, existing crewed aviation, emergency flights, infrastructure capacity, local noise limits, and people on the ground.

The proposed direction is a **federated, layered airspace-coordination profile**. It should exchange qualified flight intent and constraints, plan trajectories against time-varying capacity, identify conflicts early, monitor conformance, and preserve explicit degraded modes. It must not turn a general AI, a ledger, an optimizer, or a traffic reservation into a new source of legal or flight-safety authority.

This document defines a design boundary and qualification path. It does not claim to be a complete ATM/UTM specification, certified safety case, approved separation method, or flight authorization.

## 2. What current systems teach us

Existing aviation work already supplies important building blocks:

- **FAA UTM** describes a cooperative information-sharing ecosystem for low-altitude unmanned aircraft. Flight intent, airspace constraints, and conflict-management services are exchanged through distributed services, complementary to traditional air traffic services.
- **EU U-space** requires flight authorization, geo-awareness, network identification, and traffic information in designated U-space; weather and conformance services may also apply.
- **Authorisation lifecycle exchange:** EU U-space rules/guidance describe multi-provider exchange of authorisation changes including `Accepted`, `Activated`, `Withdrawn`, and `Ended`. The prototype uses these state names as a vocabulary reference, while adding its own `Proposed` pre-authorisation planning state; this does not claim protocol or regulatory conformance. Article 10 of Commission Implementing Regulation (EU) 2021/664 requires service providers to establish arrangements for conflicting requests received through different providers; the current consolidated EUR-Lex text was checked for this research update. The prototype only checks a supplied synthetic snapshot and deliberately does not implement the regulatory priority rules or provider-exchange protocol.
- **ICAO's UTM framework** emphasizes interoperability with existing ATM, safety of people on the ground, detect-and-avoid, security/environmental risk, and a harmonized but adaptable path toward integration.
- **NASA's 2025 strategic path-planning flight evaluation** found that tightly constrained flow corridors can leave fewer degrees of freedom for resolving conflicts. This is a warning against treating fixed lanes as the entire solution.
- **NASA's UAM service-provider simulation** explored demand/capacity balancing, sequencing, scheduling, strategic conflict management, and data exchange.

The research direction should therefore compose with these efforts, not replace them. A Mycelix implementation should map to applicable national rules, existing ATM/UTM services, approved data standards, and local risk assessments.

## 3. Non-negotiable separation of responsibilities

The architecture needs at least five distinct authority domains:

1. **Regulatory and air-navigation authority** — legally binding airspace restrictions, authorizations, procedures, separation standards, certification and oversight.
2. **Strategic coordination services** — schedule and route candidates, demand/capacity balancing, reservation exchange, and early conflict detection.
3. **Tactical air traffic and aircraft safety** — certified separation services where provided, pilot responsibility where applicable, and validated onboard detect-and-avoid/flight-control behavior.
4. **Operational infrastructure** — landing sites, vertiports, charging/fueling, weather observations, emergency landing options, rescue access and surface-mode transfers.
5. **Evidence and assurance** — provenance, source freshness, reproducible evaluation, independent review, and explicit records of unresolved cases or failures.

A successful check in one domain cannot silently confer authority in another. Examples:

- A valid Mycelix record does not imply a valid flight authorization.
- A valid reservation does not prove an aircraft can physically follow it.
- A conflict-free planned trajectory does not prove tactical separation will remain conflict-free.
- A passing simulation is not evidence that a physical system has been certified.
- A consensus or reputation score must never substitute for measured evidence or legally competent decision-making.

## 4. Proposed operational lifecycle

```text
mission request
  -> aircraft / operator capability qualification
  -> qualified flight intent
  -> airspace, weather, obstacle, ground-risk and noise constraints
  -> strategic conflict assessment
  -> capacity/sequence proposal
  -> competent-service authorization / reservation
  -> preflight freshness and conformance checks
  -> flight with tactical/onboard safety
  -> contingency response if needed
  -> post-flight observation and evidence reconciliation
```

The coordination service proposes and tracks; the applicable authority/service authorizes. Before launch, intent and restrictions are rechecked against the current operational picture. In flight, onboard or approved tactical systems must remain able to respond to hazards that were not in the plan.

## 5. Airspace should be modeled as qualified space-time, not just lines on a map

A flight intent should not be represented as a 2D polyline or a bare sequence of GPS tuples. The semantic model must bind:

- spatial reference system and relevant vertical datum;
- qualified clock/time basis and time interval;
- intended path and operating envelope;
- speed, climb/descent, turn and performance limits;
- uncertainty bounds and their validity horizon;
- aircraft capability and failure/degraded-mode profile;
- provenance, source freshness, schema/profile version and revocation/supersession state;
- applicable authority, airspace constraints and operating conditions;
- alternate/contingency options and required clearances.

Use a **trajectory tube**: a time-indexed intended trajectory plus a qualified uncertainty envelope and capability bounds. The first schema must define the meaning of every bound, its units, frame and validity—not invent universal separation minima. Safety thresholds and decision rules must reference the applicable authority-, operation- and aircraft-specific source.

Altitude bands and corridors can be useful capacity tools, but must not be treated as universal. Reserve room for maneuvering and contingency paths; evaluate cross-flow, climb/descent and degraded-navigation cases. As the NASA flight evaluation highlights, overly tight corridors can remove options precisely when a conflict needs to be resolved.

## 6. Layered, federated architecture

### A. Authoritative constraint and common-information layer

Publish versioned, source-qualified airspace restrictions, temporary closures, weather, obstacle/terrain information, landing-site status, reference data and service health. Keep source authority, issue time, validity interval and quality flags attached. Competing data sources should remain distinguishable; a projection must not erase disagreement.

### B. Federated strategic-coordination layer

Operators and approved service providers exchange intent and relevant constraints using documented interoperable contracts. A federation can scale discovery and data sharing across providers, but it must have explicit rules for reservation ownership, update ordering, conflict resolution, expiry and partition recovery. Conflicting reservations must be detected; two providers must not silently report mutually incompatible reservations as authoritative.

The first system should use transparent, deterministic conflict logic and explicit policy profiles. Optimization can improve utilization only after hard constraints are satisfied. A scalar score must never trade away a hard safety gate.

### C. Capacity and infrastructure layer

Balance demand against airport/vertiport throughput, weather, charging/fueling capacity, noise limits, emergency access, protected areas, and ground exposure. Consider local and regional routes, not only high-revenue urban corridors. Model passenger transfers to ground transport so each trip is optimized end-to-end rather than forcing air travel as the default.

### D. Aircraft-local safety layer

The aircraft remains responsible for the onboard functions assigned by the applicable operational concept: navigation/position integrity, detect-and-avoid, control laws, lost-link behavior, and declared contingency procedures. If the coordination network is unavailable, the aircraft must transition according to its validated vehicle/operation profile. There is no universal safe fallback; loiter, divert, hold, land, or return may each be unsafe in some contexts.

### E. Evidence/assurance layer

Store decisions as scoped claims with evidence and authority references. Preserve the difference between proposal, reservation, authorization, observation, prediction, simulation result, and post-flight analysis. Keep failed tests and dissent available. A reviewer must be able to reproduce a result from exact inputs, software/profile versions and deterministic seeds.

## 7. Data concepts for a first schema

| Concept | Minimum semantics |
|---|---|
| `AircraftCapabilityProfile` | Declared vehicle/operator capabilities, limitations, equipment, approved operation/profile references and failure modes |
| `FlightIntent` | Mission, trajectory reference, time window, operating envelope, capability-profile reference and source lineage |
| `AirspaceConstraint` | Geometry/volume, validity interval, source authority, status, affected operations and supersession |
| `TrajectoryTube` | Time-indexed trajectory, uncertainty envelope, coordinate/time qualification, validity horizon and model/profile identity |
| `Reservation` | Reserved resource/time-space window, issuer, accepted intent digest, version, expiry and conditions; explicitly not itself an authorization unless an authority says so |
| `ConflictAssessment` | Input digests, rule/profile version, result, limiting assumptions, uncertainty and unresolved dependencies |
| `AuthorityDecisionReference` | Verifiable reference to the competent decision process; no locally minted substitute authority |
| `ConformanceObservation` | Observed state, source, time quality, freshness, uncertainty and comparison with the currently valid intent |
| `ContingencyDeclaration` | Trigger, issuer, applicable vehicle/profile, urgency, affected volume and operational follow-up |
| `SeparationEvent` | Classified event with source and uncertainty; preserve near misses and negative evidence |
| `CoordinationEvidenceReceipt` | Provenance and integrity for the above; must not amplify its authority ceiling |

These concepts are candidates, not frozen wire formats. Schema implementation should wait until required fields, versioning, state transitions and negative controls are reviewed.

## 8. Fault and adversarial model

At minimum, tests must include:

- stale or expired flight intent;
- duplicate, replayed or conflicting reservation messages;
- mismatched spatial reference frames or vertical datums;
- unqualified time source, excessive clock skew or reordered updates;
- GNSS degradation/spoofing, sensor disagreement and growing navigation uncertainty;
- communications loss, service partition, delayed replication and recovery;
- corrupt or unavailable weather/obstacle data;
- newly closed airspace or rapidly changing weather;
- emergency flight inserted into a high-demand schedule;
- non-cooperative or untracked traffic;
- aircraft unable to meet its declared trajectory or performance envelope;
- infeasible corridor constraints that remove all valid escape options;
- service-provider compromise, identity abuse, denial of service and privacy leakage;
- a technically valid record whose underlying operational claim is stale, disputed or outside its scope.

Every case needs an explicit expected outcome such as reject, conflict, degraded, unresolved or requires-authority-review. **Missing information must not be converted into a clear-to-fly result.** At the same time, missing network data alone is not proof that the physical world is unsafe; it is a qualification loss that must be handled by the declared operational policy.

## 9. Security, privacy and governance

- Authenticate operators, providers and data origins; protect messages from tampering and replay.
- Define data minimization and role-based visibility for passenger identity, commercial routes and operational telemetry.
- Record who can see or change operational data and under what authority, without requiring public disclosure of every payload.
- Keep network coordination distinct from the certified onboard control boundary.
- Support independent provider implementation and exit; avoid a single proprietary global scheduler.
- Define responsibility for inaccurate data, contested reservations, revocation, outages and recovery before live use.
- Make emergency priority explicit without assuming other aircraft can obey physically impossible instructions.
- Assess equity, community noise, energy/lifecycle burden, wildlife and ground risk alongside utilization, delays and commercial throughput.

Mycelix may provide audit and provenance receipts, but a distributed trust substrate is not a deterministic real-time control network. Reputation or consensus must never resolve a physical conflict by vote.

## 10. Symthaea / Mycelix / Symtropy work split

### Symthaea — bounded planning and research

- Generate route/schedule candidates and counterfactual scenarios.
- Forecast demand/capacity and identify bottleneck sensitivity.
- Surface uncertainty, contradictory data and missing dependencies.
- Rank experiments by value of information.
- Orchestrate established simulation tools and preserve exact input/toolchain digests.
- Propose; do not grant permission to fly or directly control aircraft in the initial development stages.

### Mycelix — federation and evidence

- Reuse existing identity, provenance and evidence primitives.
- Define interoperable, versioned coordination envelopes and authority references.
- Preserve lifecycle transitions, revocation, conflicts, disputes and negative evidence.
- Produce reproducible audit receipts that state their exact authority ceiling.
- Do not put Holochain consensus/reputation in a safety-critical, low-latency separation loop.

### Symtropy — infrastructure and public value

- Compare vertiport/landing-site siting and distributed charging.
- Optimize maintainability, local supply chains, energy and lifecycle burden.
- Evaluate noise exposure, access for rural/remote communities and emergency-service reach.
- Optimize the whole trip across air, ground, rail and other available modes; flying is not automatically best for every origin/destination or payload.

## 11. Jurisdiction-first rollout: South Africa and regional interoperation

A South African pilot should integrate with existing national aviation institutions and published procedures rather than assume airspace coordination starts from zero. The South African AIP's ENR 1.9 section published for 15 October 2022 describes the Central Airspace Management Unit (CAMU) managing air-traffic flow/capacity, flexible use of airspace, weather-related rerouting, and strategic, pre-tactical and tactical flow-management phases. That document is historical reference material, not sufficient proof of current procedures: implementation must check the latest AIP/NOTAMs, ATNS procedures, SACAA requirements and competent-authority approvals at design and deployment time.

SACAA publishes RPAS/UAS guidance, and South African aviation regulations have continued to be amended in 2026. Therefore, jurisdictional requirements must be represented as **versioned, effective-dated policy profiles with authoritative source links**, not hard-coded global rules or facts inferred from old pages. A policy adapter should make the source version, effective interval, jurisdiction, operation class, approving authority and unresolved legal questions inspectable.

For other African states, reuse the protocol semantics while keeping each country's airspace, certification, privacy, security and operator-authorization policies separately governed. Cross-border handoff needs explicit agreement on flight intent, identifiers, data disclosure, time/reference systems, reservation authority, emergency handling and responsibility at FIR/jurisdiction boundaries. Regions with sparse communications or surveillance need stronger onboard contingency requirements and locally appropriate infrastructure assumptions, not a degraded copy of a high-connectivity city model.

The initial target should be **interoperability and decision support alongside existing institutions**, not autonomous control of the national airspace.

## 12. Qualification plan

### Gate 0 — architecture

Review object definitions, authority boundaries, state machines, threat model and links to local regulation and international reference material. No aircraft control integration.

### Gate 1 — deterministic simulation

**Initial research harness is now authored** in `tools/airspace-sim`, as a standalone Rust crate rather than another production zome. It replays versioned JSON scenarios with integer millimetre units, a fixed time horizon, explicit intent states, frame/time qualification, uncertainty expansion, a bounded pair-sample budget, and expected-vs-actual classifications.

The eleven traffic fixtures cover separated parallel paths, nominal crossing, vertical separation, uncertainty expansion, a between-sample encounter, reference-frame mismatch, unqualified time, mismatched time epochs, expired intent, withdrawn intent, and emergency review. Thirteen lifecycle-transition cases exercise valid state progression and rejection of sequence gaps/duplicates, time reversal, issuer/digest/time-basis changes, and terminal-state misuse. Fourteen reservation-snapshot cases cover cross-provider overlap, half-open adjacency, separate resources, expiry, withdrawal, stale/partitioned views, duplicate reservation versions, time-basis mismatch, emergency review/no-preemption, invalid intervals, future-issued claims, and recovery to a complete fresh snapshot. Sixteen event-stream cases exercise out-of-order delivery, sequence gaps, same-sequence forks, broken predecessor links, illegal transitions, exact retransmission, cross-provider overlap, withdrawal, partition degradation, expiry, future-issued events, digest/payload disagreement, mutation of reservation fields, time reversal, missing history prefixes, and malformed intervals. The evaluator does not choose a reservation winner: any overlap is reported as a conflict, duplicate reservation identities remain unresolved, and stale/partitioned snapshots cannot claim no conflict. The replay command reports expected-vs-actual outcomes for all three fixture families. A dedicated GitHub Actions workflow runs Rust unit tests and replays the committed fixture set with `--verify`.

This is deliberately a **small semantics prototype**, not a realistic air traffic or flight-dynamics simulator. It includes snapshot-level reservation conflict screening and bounded deterministic replay of provider-local event streams. The event reducer checks sequence continuity, predecessor linkage, issue-time monotonicity, stable reservation semantics, legal lifecycle transitions, exact retransmission and cross-provider overlaps. It still omits networked provider federation, authenticated peer exchange, cryptographic digest/signature verification, cross-provider causal ordering, replicated state merge and supersession, capacity-aware slot allocation, dynamic weather/airspace changes, non-cooperative traffic, validated aircraft dynamics, and full one-to-one executable coverage of the requirement corpus. Conservative sample margins may produce false positives, and the model has no authority to clear a real flight. Local Rust tooling is unavailable in the editing environment, so the updated crate must be compiled and its full replay gate confirmed on the new exact PR head before describing the new event-stream extension as passing.

### Gate 2 — adversarial replay and independent review

Replay the negative controls in `AIRSPACE_COORDINATION_REQUIREMENTS_V1.json`. Verify that stale, mismatched or unavailable state cannot become a successful authorization. Review both failures and successes; report limits and model-validity ranges.

### Gate 3 — hardware/software-in-the-loop

Only after independent review, assess real interfaces and failure timing in a protected test environment. No live-airspace integration without competent-authority approval and a separate safety case.

### Gate 4 — controlled interoperability trials

Begin in a controlled, low-risk environment with approved participants, independent monitoring, explicit stop conditions and a published incident/near-miss process. Expand only on measured evidence, not simulator throughput alone.

## 13. Evaluation metrics and decision rule

Report a vector, not one headline score:

- hard safety constraints and violations;
- unresolved/conflicting intent and data-quality rates;
- conformance, contingency and near-miss distributions;
- capacity, delay, cancellations and recovery time;
- noise and ground-population exposure;
- energy/lifecycle cost and infrastructure utilization;
- emergency-response performance;
- access and allocation across regions and user classes;
- cyber-resilience, partition behavior and degraded-mode coverage.

Safety gates are non-compensatory: better throughput or lower cost cannot offset a safety-gate failure. Simulation outcomes describe the tested scenario and model only; they do not certify an aircraft, operational concept or deployed service.

## 14. External references

1. FAA, [Unmanned Aircraft System Traffic Management](https://www.faa.gov/uas/advanced_operations/traffic_management).
2. FAA, [Aeronautical Information Manual: UTM operations](https://www.faa.gov/air_traffic/publications/ATpubs/AIM_html/chap11_section_4.html).
3. ICAO, [Unmanned Aircraft System Traffic Management Framework, Edition 4](https://www.icao.int/safety/UA/Documents/UTM%20Framework%20Edition%204.pdf).
4. EASA, [U-space](https://www.easa.europa.eu/en/domains/air-traffic-management/u-space).
5. European Commission, [Implementing Regulation (EU) 2021/664 on a regulatory framework for U-space, current consolidated text](https://eur-lex.europa.eu/legal-content/ENG/TXT/?uri=CELEX%3A32021R0664).
6. NASA, [Flight Evaluation of In-Flight Strategic Path Planning Automation for Future High-Density Operations (2025)](https://ntrs.nasa.gov/citations/20250003243).
7. NASA, [Provider of Services for Urban Air Mobility Prototype Simulation X5](https://ntrs.nasa.gov/citations/20250001367).
8. ITU-T, [X.1385: Security requirements and guidelines for telecommunications in an urban air mobility environment](https://www.itu.int/dms_pubrec/itu-t/rec/x/T-REC-X.1385-202504-I%21%21SUM-HTM-E.htm).
9. SACAA, [Unmanned Aircraft Systems](https://www.caa.co.za/industry-information/uas/).
10. South African Government, [Civil Aviation Regulations: 36th Amendment (2026)](https://www.gov.za/documents/notices/civil-aviation-act-thirty-sixth-amendment-civil-aviation-regulations-2026-english).
11. ATNS, [South African AIP ENR 1.9: Air Traffic Flow Management Procedures (edition valid 15 October 2022; historical reference)](https://cad.atns.co.za/SmartAIM/EAipPackages/15-OCT-22/2022-10-15-000000/html/eAIP/FA-ENR-1.9-en-US.html).

## Change discipline

This document is a research contract. Future implementation must use versioned schema changes, tests for both positive and negative cases, and exact-head evidence. No test is considered passed merely because it is queued or was run against another commit. A passing contract is not a claim of aviation safety or operational authorization.
