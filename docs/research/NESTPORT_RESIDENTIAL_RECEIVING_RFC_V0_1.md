# NestPort — Residential Aerial Delivery, Receiving, and Storage RFC v0.1

**Status:** Research RFC / architecture proposal. Documentation only; no operational, regulatory, safety, or production-readiness claim.  
**Date:** 2026-11-15  
**Working name:** NestPort  
**Related programme:** SkyGarden / DreamHome / AeroWeft

> **Primary design rule:** The aircraft is not the delivery endpoint. A residence needs a secure, accessible, environmentally appropriate, interoperable receiving system.

## 1. Executive proposal

NestPort is a vendor-neutral residential receiving and local-logistics layer for parcels arriving by conventional courier, autonomous ground robot, remotely piloted aircraft, tethered aerial delivery, or a future qualified aircraft docking interface.

It joins four capabilities usually designed separately:

1. **Transfer:** move a package from a courier, aircraft, tether, or ground robot into a controlled receiving boundary.
2. **Custody:** establish which operator and package were accepted, what happened during transfer, and whether custody is still known.
3. **Storage:** protect goods from weather, theft, tampering, temperature excursions, pests, and accidental access.
4. **Resident experience:** allow residents to receive, defer, reject, collect, return, or delegate delivery without being present at the moment of arrival.

NestPort is not a drone, flight controller, UTM service, aircraft certifier, or autonomous-dispatch authority. It must never command an aircraft or infer that a flight is safe. Aircraft operators and aviation authorities retain those responsibilities. The receiving station publishes capabilities and a bounded acceptance state; the carrier decides whether and how to fly under its own approvals.

The opportunity is not to beat Wing, Zipline, or Amazon by copying their aircraft. Those operators already demonstrate capable aircraft and increasingly mature delivery networks. NestPort targets the less-standardized boundary between a qualified logistics operator and a private home: interoperable handoff, verified storage, resident control, privacy, returns, and resilient household integration.

## 2. Industry baseline and the gap

The following are published operator/manufacturer descriptions, not independently normalized comparative test results:

| Reference | Published capability | Relevant design lesson |
|---|---|---|
| Wing | Up to 5 lb payload, about 60 mph, up to 12 miles; Wing reports over one million residential deliveries | Lightweight VTOL/cruise architecture and scaled residential operations are already commercial baselines |
| Zipline Platform 2 | Up to 8 lb; 10-mile service radius; up to 24 miles one-way in dock-to-dock operation; tether delivery from up to 300 ft | Aerial delivery can avoid landing at the home; tethered payload is a distinct handoff subsystem |
| Amazon MK30 | Up to 7.5 miles from delivery centre, 60+ mph; descends to 13 ft for delivery and can return when it detects a delivery-site hazard | A delivery site must be able to reject a handoff when the local zone is unsafe |

Links: [Wing technology](https://wing.com/technology), [Wing residential delivery scale](https://wing.com/about), [Zipline fact sheet](https://www.zipline.com/about/zipline-fact-sheet), [Amazon MK30 technology](https://www.primeair.amazon/technology), [Amazon operating profile](https://www.primeair.amazon/operations).

Do not compare payload, range, or speed without matching payload definition, reserve policy, route geometry, weather, delivery method, and whether range is one-way, round-trip, or service radius. These figures are design reference points, not a leaderboard.

### 2.1 The gap NestPort should own

A carrier may provide a controlled delivery mechanism, but the home still needs a secure, weatherproof receiving boundary; separate storage for ambient, chilled, frozen, and restricted-access packages; a machine-readable capability interface; a verifiable handoff receipt; package returns and reusable-container circulation; resident-defined permissions and quiet hours; accessibility; and local operation during internet or grid outages.

NestPort should make the receiving endpoint reusable even if the household changes aircraft providers.

## 3. Product architecture

### 3.1 Station capability tiers

| Tier | Name | Capability | Initial position |
|---|---|---|---|
| N0 | Secure Parcel Cabinet | Passive lockable, weather-resistant courier cabinet | First build; no aircraft dependency |
| N1 | Verified Receiving Cabinet | Compartment sensors, access logging, signed receipt, offline-safe lock control | First software/hardware prototype |
| N2 | Environmental Pantry | Separate ambient, chilled, frozen and monitored compartments | Research and appliance qualification |
| N3 | Aerial Transfer Receiver | Carrier-approved tether/drop interface feeding an enclosed handoff chamber; no default touchdown | First aviation-facing target |
| N4 | Residential Drone Dock | Aircraft-specific landing, servicing and charging interface | Future, only with aircraft/operator approval |
| N5 | Shared Community Hub | Multi-carrier receiving, bulk storage, sorting and optional ground-robot dispatch | Community deployment path |

N0–N2 should deliver value without waiting for aircraft certification. N3 is the preferred aerial interface to investigate because it can keep rotorcraft away from residents and the property itself; it is not automatically safer and must be tested against tether, wind, obstacle, and package-swing hazards. N4 is not the default.

### 3.2 Physical zones

Separate these zones even when they share one enclosure:

1. **Arrival / transfer zone:** landing surface, winch target, chute, or courier interface. It is not a storage compartment.
2. **Transfer vestibule:** an interlocked boundary that prevents access to moving aircraft or transfer mechanisms and prevents a package being pulled back out.
3. **Storage compartments:** isolated ambient, chilled, frozen, and restricted-access storage.
4. **Resident collection side:** accessible doors, clear signage, safe manual recovery, and optional collection scheduling.
5. **Service and energy zone:** isolated electrical equipment, backup power, maintenance access, and fire separation appropriate to the selected hardware.

A home should not require an aircraft to fly over a person, animal, play area, road, neighbour's property, or open window to use NestPort. The site-specific safe transfer envelope must be established before aerial service is enabled.

### 3.3 Storage classes

Each compartment advertises a capability profile; a carrier must not infer that all parcels are acceptable.

- **Ambient:** sealed dry storage for ordinary parcels.
- **Chilled:** temperature-controlled storage with calibrated sensing, excursion alarms, and an explicit permitted temperature/time profile.
- **Frozen:** a separate freezer-grade compartment, qualified independently for thermal recovery and outage behavior.
- **Restricted:** individually authorized, access-logged storage for eligible high-value items.
- **Returns:** separate intake so returned goods cannot be mistaken for incoming goods.

Medicines, biological material, food requiring a validated cold chain, dangerous goods, and other regulated items are not accepted merely because a compartment has a temperature sensor. Their handling requires appropriate product, operator, packaging, chain-of-custody, and regulatory qualification. NestPort v0.1 does not authorize such carriage.

Do not co-locate aircraft battery charging, household energy storage, and parcel storage in one unsegregated enclosure. Thermal-runaway and fire scenarios require specialist design and review.

## 4. Delivery lifecycle and safety semantics

The proposed logical lifecycle is:

Requested → CapabilityMatched → ResidentAuthorized → CarrierAccepted → ManifestBound → SiteReady → OperatorFlightReleased → ArrivalObserved → TransferAuthorized → TransferInProgress → PackageReceived → CompartmentSealed → CustodyReceiptVerified → ResidentNotified → Collected / Returned / Quarantined.

The carrier's flight release is external to NestPort. A station must not turn a software booking into permission to fly.

### 4.1 Fail-closed conditions

The station must reject or hold a transfer when any of these applies:

- invalid, expired, replayed, unsigned, or mismatched authorization;
- unknown aircraft, operator, parcel, compartment, or requested handling class;
- site not in its declared safe/available state;
- people, animals, vehicles, debris, standing water, or other hazards in the transfer area;
- wind, rain, temperature, or other conditions outside the qualified envelope;
- transfer mechanism, latch, hatch, sensor, power, or communication state is ambiguous;
- package mass, geometry, orientation, or temperature class violates declared capability;
- evidence is stale, incomplete, conflicting, or cannot be reconciled.

An ambiguous transfer must not be guessed to have succeeded. Freeze further state-changing actions for that delivery, preserve observations, and reconcile with the operator and station before accepting another transfer into the same occupied path. The physical station must remain locally safe when every cloud service is unavailable.

### 4.2 Safe failure and recovery paths

Every mission profile must define what happens when delivery is impossible: refuse transfer and divert/return; retry only after a new positive site-readiness assessment; route to a staffed/shared hub; request ordinary ground delivery; quarantine a package after suspected tampering or environmental excursion; or use an accessible human-assisted recovery path for a jammed hatch or power failure.

A successful abort is a safe operational outcome, not a failed safety outcome. Report mission completion and safe-abort quality separately.

## 5. Proposed data contracts

The first implementation should use versioned Rust-first domain types and canonical serialization. These are conceptual fields, not existing Mycelix schemas.

### 5.1 Station capability

- stable station ID and hardware/configuration revision;
- supported handoff modes: courier, ground robot, tether, controlled drop, aircraft dock;
- max package mass, dimensions, centre-of-mass envelope, and transfer tolerances;
- allowed storage classes and temperature/time profiles;
- local power reserve, communications status, and safe fallback capability;
- accessibility and resident collection constraints;
- site permission reference and qualification status;
- evidence references for calibration, maintenance, tests, and approval.

### 5.2 Parcel manifest

- delivery ID and carrier-issued package reference;
- cryptographic digest of the carrier manifest (not public personal details);
- dimensions, mass, packaging class, handling class, and declared temperature profile;
- intended station and compartment capability;
- delivery window and expiry;
- sender/operator authorization reference;
- resident consent/receiving policy reference;
- prohibited/restricted-goods classification result where applicable.

### 5.3 Handoff receipt

- delivery ID, station ID, compartment ID, and carrier/operator identity;
- authenticated aircraft/robot identity where relevant;
- authorization nonce and expiry;
- transfer start/end timestamps from a qualified time source;
- sensor observations: door/latch state, parcel presence, mass delta, compartment state, and calibration versions;
- environmental observations for temperature-controlled compartments;
- status: accepted, rejected, aborted, uncertain, quarantined;
- signature and digest over the exact receipt payload;
- evidence class: synthetic, simulated, bench-measured, field-measured, independently reproduced, or externally approved.

A sensor observation is not proof by itself. Qualification needs independent fixtures and fault injection to show that stuck, spoofed, stale, missing, or mutually inconsistent sensors do not produce false acceptance.

### 5.4 Privacy boundary

Do not put names, residential addresses, access codes, raw video, or high-frequency location traces into public DHT records. Store personally identifying data in an appropriately protected system and expose only minimum necessary references and verifiable commitments to Mycelix. Residents choose delivery quiet hours, allowed carriers, allowed storage classes, notification preferences, and whether aerial delivery is disabled entirely.

## 6. Stack responsibilities

| System | Role | Explicit boundary |
|---|---|---|
| Symthaea | Propose delivery modes, identify trade-offs, plan demand and resource schedules, explain alternatives | Advisory only; cannot authorize a flight or override station interlocks |
| Symtropy | Simulate transfer geometry, vehicle interactions, tether/payload motion, station mechanisms, queues and fault scenarios | A simulation is not structural, aircraft, or operational certification |
| Mycelix | Resident consent, station/parcel capability records, cooperative ownership, service agreements, custody receipts, maintenance and dispute evidence | Not the aircraft command channel, airspace authority, or sole truth source for live safety |
| Sol-Atlas | Site context: terrain, buildings, vegetation, land rights, protected/heritage areas, access and environmental constraints | A map layer is not a current legal permission or flight clearance |
| AeroWeft | Aircraft-specific mission envelope, payload, energy, kinematics, reachability and risk evidence | No generic aircraft capability may be inferred from another platform |
| Physical station controller | Local sensors, interlocks, compartment locks, safe stop, manual recovery and local event journal | Must remain safe offline; remote software cannot bypass physical interlocks |

### 6.1 Existing Mycelix implementation boundary

The public Mycelix README describes Commons as pre-alpha and without a deployed real housing UI. The checked-in transport audit explicitly marks transport-routes/sharing/impact as dormant, unpacked, never deployed, and not verified as WebAssembly builds. These are source artifacts, not deployable dependencies.

The current transport model includes a generic Drone vehicle type; robotics dispatch includes generic asset, dispatch-order, and telemetry concepts. These do not yet provide the parcel manifest, storage capability, authenticated transfer, custody receipt, or environmental storage lifecycle described here.

Therefore this RFC does not claim NestPort is already implemented. It defines the next contract without pretending dormant transport code is live. Any future code change must start with an exact-head workspace/build and validation audit.

References:
- [Mycelix maturity matrix](https://github.com/Luminous-Dynamics/mycelix/blob/main/README.md)
- [Dormant transport audit](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-commons/zomes/DORMANT_TRANSPORT.md)
- [Transport route integrity types](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-commons/zomes/transport-routes/integrity/src/lib.rs)
- [Robotics dispatch integrity types](https://github.com/Luminous-Dynamics/mycelix/blob/main/mycelix-civic/zomes/robotics-dispatch/integrity/src/lib.rs)

## 7. Energy and charging

Charging is an optional station capability, not a prerequisite for receiving parcels.

- N0–N2 operate without any drone charging interface.
- For a small qualified drone dock, conductive charging or hot-swappable batteries are initial reference baselines.
- Wireless charging is a research track. A 4 November 2026 study reports 70.10 ± 0.55% transmitter-rail-to-battery efficiency and tight alignment tolerance on one indoor small-UAV setup; the authors state that outdoor conditions, electromagnetic compatibility, avionics interference, security, and a full fly-land-recharge loop were not tested. This cannot be extrapolated to a high-power eVTOL or universal home charging pad.
- Passenger/cargo eVTOL charging is a separate high-power engineering system with aircraft-specific connectors, grid interconnection, thermal management, fire separation and operator procedures. It must not share assumptions with a small delivery-drone dock.

Reference: [Konar et al., 2026, wireless fully automated UAV charging station](https://doi.org/10.1108/AEAT-05-2026-0201).

## 8. Benchmark programme

Do not use a single weighted score that allows better delivery time to compensate for an unsafe release or broken chain of custody. Safety and data-validity gates are non-compensatory.

### Track A — authorization and protocol correctness

- 100% rejection in the frozen test corpus for expired, malformed, replayed, unsigned, mismatched, and unsupported manifests.
- 100% fail-closed behavior for missing, stale, contradictory, or invalidated station evidence.
- Zero unauthorized compartment openings in the defined software, hardware-in-loop, and red-team campaigns; report test count and confidence bounds.
- Exact chain-of-custody continuity for every accepted, rejected, aborted, and quarantined transaction in the test corpus.

### Track B — handoff and parcel integrity

- Transfer success rate conditional on an explicitly declared aircraft, parcel, weather, and site envelope.
- Misdelivery, drop, snag, collision, jam, parcel damage, and false-receipt rates.
- Handoff time distributions (median, p95, p99).
- Tether excursion, parcel swing, load, and contact envelope where tether transfer is used.
- Safe-abort rate and recovery time, reported separately from successful handoffs.

### Track C — storage quality

- Temperature/humidity envelope compliance and excursion detection latency.
- Time to detect power loss, open door, sensor failure, tampering, or compartment fault.
- Thermal recovery and holdover time under declared ambient conditions.
- Parcel retrieval accessibility and manual recovery success under power/network failure.
- Fire separation, battery placement, ingress protection, and environmental qualification for exact hardware.

### Track D — service and logistics

- Delivery success within the declared service window;
- on-time p50/p95/p99;
- successful accepted deliveries per station-hour;
- station occupancy and blocked-transfer duration;
- ground-courier substitution and safe-diversion rates;
- cost per successful delivery, including charging, maintenance, storage depreciation, staffing and failed attempts.

### Track E — energy and environmental impact

- Wh per delivered parcel and Wh per kg-km, including ground station, charging loss, return flight and failed/rejected attempts;
- lifecycle emissions per successful delivery using the same electricity/carbon assumptions for drone and ground baselines;
- measured acoustic exposure at receiving property and nearest affected boundary (LAeq and sound exposure level), not just aircraft claims;
- packaging reuse, waste, and return-collection rate.

### Track F — security, privacy and inclusion

- replay, spoofed identity, compromised operator key, sensor spoof, network outage and malicious-parcel simulations;
- least-privilege authorization and key-rotation tests;
- raw imagery collection and retention audit;
- resident opt-out, carrier allowlist, guest access, quiet-hour and delegated-collection correctness;
- accessibility of station approach, controls, indicators and manual recovery.

**Research targets, not achieved results:** establish a reproducible baseline for every track first. A later controlled pilot may set a ≥99% secure-handoff target inside a tightly declared operating envelope, but no percentage target replaces zero-tolerance hard gates for unauthorized access, false acceptance, or a safety-critical validation failure.

Comparisons must match route, parcel mass/dimensions, weather, service window, reserve policy, and ground alternative. Publish per-scenario results and Pareto frontiers; do not publish only a blended score.

## 9. South African regulatory and site constraints

South Africa is the first regulatory context to assess, but we must not assume that a commercial drone-delivery concept is already authorized.

SACAA's current public UAS page lists commercial-operation approvals including ASL, UASOC, RLA, RPL and Certificate of Registration, and separately lists autonomous unmanned aircraft among the exclusions from Part 101. The same page states that the operator must meet Part 101 requirements for applicable RPAS activity. Rule 101.05.11 describes B-VLOS operation as requiring Director approval in the UASOC operations manual and compliance with SA-CATS 101. These sources create a material classification question for an autonomous delivery service: **the exact aircraft/control concept and approval route must be determined in writing with SACAA before any flight operation is planned.**

This RFC is not legal advice. It requires a formal regulatory applicability determination rather than treating remotely piloted, automated, and autonomous aircraft as interchangeable.

Site qualification must also account for airspace, nearby aerodromes, people, roads, power lines, neighbouring property, land-use permissions, privacy, noise, environmental restrictions, and current aeronautical information. Sol-Atlas may screen and preserve evidence; it cannot grant permission.

References:
- [SACAA Unmanned Aircraft Systems](https://www.caa.co.za/industry-information/uas/)
- [SACAA Flight Operations publications](https://www.caa.co.za/industry-information/flight-operations/)
- [Part 101 regulation 101.05.11 — B-VLOS](https://beta.acts.co.za/civil-aviation-act-2009/nr425_101_05_11_beyond_visual-.php)
- [EASA U-space services](https://www.easa.europa.eu/en/faq/143018) (comparative architecture only; not South African law)

## 10. Symtropy / AeroWeft research reuse

The public Symtropy research branches already contain relevant work on Nano-quadrotor system-identification data intake and multi-horizon metric receipts; intervention-conditioned recoverability and failure-aware rollout; and AeroWeft kinematic reachability and an aviation benchmark suite.

NestPort should reuse those as aircraft-dynamics, uncertainty, reachability, and safe-recovery foundations rather than creating a competing flight-model benchmark. The missing research increment is the coupled residential delivery problem:

**approach volume × payload/tether dynamics × home geometry × transfer apparatus × occupancy hazards × storage state × abort/diversion.**

The existing research artifacts are on separate research branches and are not thereby claims of default-branch integration or physical-flight qualification:
- [AeroWeft benchmark suite](https://github.com/Luminous-Dynamics/symtropy/blob/feat/aeroweFt-kinematic-reachability-001/docs/research/aviation/AEROWEFT_BENCHMARK_SUITE_V1.md)
- [AeroWeft state of practice and benchmarks](https://github.com/Luminous-Dynamics/symtropy/blob/feat/aeroweFt-kinematic-reachability-001/docs/research/aviation/AVIATION_STATE_OF_PRACTICE_AND_BENCHMARKS_2026.md)
- [Nanodrone recoverability frontier](https://github.com/Luminous-Dynamics/symtropy/blob/research/nanodrone-recoverability-frontier-v0-1/docs/research/SYMTROPY_NANODRONE_RECOVERABILITY_FRONTIER_V0_1.md)
- [Nanodrone system-identification adapter](https://github.com/Luminous-Dynamics/symtropy/blob/research/nanodrone-metrics-parity-v0-1/docs/research/SYMTROPY_NANODRONE_SYSTEM_IDENTIFICATION_ADAPTER_V0_1.md)

Symtropy is currently documented as a rigid-body engine; it must not be treated as a validated flexible-tether, aerodynamic CFD, structural FEA, or building-physics solver. Use qualified specialist models and independent analytic fixtures where those phenomena are needed.

## 11. Implementation sequence

1. **Contract first:** freeze station capability, parcel manifest, delivery intent, handoff receipt, storage record and failure semantics as versioned Rust-first types.
2. **N0/N1 prototype:** build a secure parcel cabinet with local-first lock control, parcel presence/weight sensing, signed local journal, manual recovery, and offline behavior. No flight integration.
3. **N2 storage prototype:** qualify ambient and temperature-controlled compartments with calibration, power-loss tests and excursion alarms.
4. **Digital-twin fixtures:** create synthetic geometry, parcel envelopes, transfer modes, weather/hazard scenarios and fault-injection cases; connect to existing Symtropy/AeroWeft research without claiming flight validation.
5. **Mycelix governance and evidence:** add resident consent, carrier allowlists, service rights, maintenance records and verifiable custody receipts only after housing/commons maturity gates are met.
6. **N3 supervised handoff test:** use a qualified partner/operator and approved test site; begin with constrained, supervised trials and no BVLOS/autonomous assumption.
7. **Community hub:** test multi-carrier shared storage, ground-robot dispatch, accessible pickup, and equitable service allocation.
8. **N4 aircraft dock:** only after a particular aircraft, station, charging interface, operator, and site have their own qualification and approval path.

## 12. Decision requested

Adopt NestPort as a research programme and accept the following constraints:

- receiving and storage are a product in their own right, independent of a specific aircraft;
- no default residential drone landing; tether, controlled drop, or ground transfer must be compared first;
- local physical interlocks are authoritative for compartment safety;
- Mycelix records consent, rights, maintenance and evidence but never controls flight;
- every accepted handoff has a signed, auditable custody receipt;
- uncertain outcomes trigger hold and reconciliation, not a second mutation;
- the first prototype is a secure cabinet, not a flying vehicle;
- field operation remains gated on written regulatory determination, an approved operator, and independently reviewed safety evidence.

This path creates immediate work on residential infrastructure while preserving a route toward genuinely interoperable aerial logistics.
