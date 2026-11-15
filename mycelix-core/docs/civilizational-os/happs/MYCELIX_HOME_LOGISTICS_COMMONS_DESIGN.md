# Mycelix Commons: Mobile Logistics, Home Receiving, and Shared Robotics

**Status:** Research/design contract v0.1; not deployed functionality  
**Date:** 2026-11-15

## 1. Decision

Extend the existing Mycelix Transit and Commons direction into a shared logistics commons. Do not create a parallel mobile-commons ledger or a second asset registry. Reuse and repair the existing transport-routes, transport-sharing, transport-impact, housing, food/provision, and civic robotics-dispatch boundaries.

The system coordinates people, goods, vehicles, aircraft, mobile robots, community hubs, homes, and storage. It does not own real-time motor control, flight control, safety interlocks, or local environmental regulation.

Working product names:
- **Mycelix Transit / Mobile Commons:** shared vehicles, routes, freight, missions, and community logistics.
- **NestPort:** residential and community receiving, storage, and handoff infrastructure.
- **HomeOps:** local receiving, moving, sorting, containerization, pantry/storage and inventory workflows.

## 2. Current repository truth — important maturity boundary

The public repository contains a substantial Transit design at mycelix-core/docs/civilizational-os/happs/MYCELIX_TRANSIT_DESIGN.md, a Commons transport implementation, a transport SDK integration, and a separate civic robotics-dispatch zome.

However, mycelix-commons/zomes/DORMANT_TRANSPORT.md explicitly records that transport-routes, transport-sharing, and transport-impact are dormant, unpacked, never deployed, and not confirmed to build for wasm32-unknown-unknown. It also documents that the transport UI uses hardcoded figures and that ignored Sweettests do not qualify a deployed multi-agent service.

This document therefore defines a revival and extension plan. It does not claim that Mycelix Transit, its UI, its transport zomes, or a delivery network is operational.

The existing dormant-transport audit identifies material defects that must be addressed before deployment:
- Create-path ownership is not bound for exactly the entities named by the audit: vehicle owner, ride-offer driver, ride-request requester and ride-review reviewer.
- Ride matches are not validated sufficiently; seat counts, state transitions, duplicate bookings and bilateral confirmation are incomplete.
- Ride offers lack discoverable origin/destination coordinates; nearby discovery is consequently incomplete.
- Route geometry does not yet provide a verified route solver or ETA.
- There is no demonstrated live commons telemetry path.
- Emissions values are uncited hardcoded values; some mode assumptions and carpool arithmetic are wrong.
- No transport-linked payment settlement is present.
- Current test inventory is not equivalent to deployed Holochain multi-agent correctness.

Keep the audit's correct fail-closed authorization behavior and author-bound update/delete checks. Do not regress those while fixing the system.

## 3. Existing components and intended responsibilities

| Existing component | Intended role | Required treatment |
|---|---|---|
| Commons transport-routes | Vehicle registry, routes, stops, maintenance | Repair and qualify; extend capability descriptions |
| Commons transport-sharing | Ride sharing, cargo offers, matching | Repair authorization, geo-discovery and state machine |
| Commons transport-impact | Trip records and impact accounting | Replace uncited factors with versioned, sourced, lifecycle-aware factors |
| Civic robotics-dispatch | Robotic asset identity, mission authority, telemetry, recall | Reuse as registry/mission record boundary; not real-time control |
| Commons Housing / Property | Home, cooperative, land and maintenance relationships | Link receiving devices and shared facilities to authorized properties/tenure |
| Provision / Food domains | Food source, distribution, preservation, community hubs | Link groceries, food batches and hub-to-home deliveries |
| SupplyChain SDK / provenance concepts | Origin and custody checkpoints | Reconcile actual app maturity before operational reliance |
| Finance / Treasury | Payments and shared service settlement | Use explicit service settlement; no implicit ledger-to-bank authority |

The Commons Transit design's freight-consolidation and multi-modal vision should be retained. The implementation must be repaired before those design statements are advertised as available capabilities.

## 4. Core architecture

Four separate planes are required:

1. **Commons coordination:** people, organizations, property, assets, service offers, booking, agreements, community funding and disputes.
2. **Mission planning:** deterministic routing and fleet allocation, with typed physical constraints. Planning can run outside a Holochain zome.
3. **Local robot autonomy and safety:** each truck, aircraft, ground robot and home machine executes only within its local validated capability and independent safety boundary.
4. **Evidence and custody:** minimal, privacy-aware receipts record authorized dispatch, physical handoffs, storage acceptance, exceptions and maintenance.

Holochain records should not become high-rate flight or motor telemetry streams. Publish signed, bounded summaries and evidence references at operationally meaningful intervals. High-rate sensor data remains local or in a controlled evidence store, with digests and authorized disclosure.

## 5. Canonical logistics concepts

The following are proposed v0.1 data contracts, not existing deployed zome types.

### 5.1 Asset capability profile

A shared asset identity should refer to a versioned capability document:
- stable asset ID and asset type;
- owner, operator and maintenance authority;
- vehicle, aircraft, manipulator, AMR, quadruped, fixed station or home receiver class;
- mass, dimensions, payload, volume, range and duty-cycle limits with units;
- attachment, dock and load-carrier interfaces;
- battery chemistry, energy, power, charging and thermal envelope;
- operating region and permission references;
- software/hardware configuration digests;
- availability, maintenance and fault state;
- assurance scope and evidence freshness.

The robot's Mycelix governance tier is not a physical safety permission. Mission eligibility must be checked by the platform-specific safety supervisor and applicable operating authority.

### 5.2 Package and container manifest

Represent cargo as a hierarchy: shipment, package, reusable container/tote, and item or batch when legitimately known.

Each manifest carries only necessary fields:
- stable opaque item ID;
- mass, external dimensions and centre-of-gravity range;
- handling class, fragility and orientation limits;
- allowable modes (truck, ground robot, aircraft class, handoff);
- temperature and humidity requirements, where applicable;
- contamination or food-separation class;
- custody and delivery window;
- restricted-handling indicator;
- container compatibility;
- provenance and evidence references.

Never infer aircraft eligibility from weight alone. Dimensions, drag, restraint, centre of gravity, reserve energy, weather, route, landing method and approvals matter.

### 5.3 Mission, segment and handoff

A mission is a directed sequence of segments, not one vehicle ID:
- carrier road segment;
- aircraft segment;
- ground-robot segment;
- fixed facility transfer;
- receiving-station acceptance;
- storage placement or human collection.

Each segment binds a responsible asset, capability snapshot, time window, route/area constraints, payload manifest, reserve policy, fallback and evidence requirements.

A handoff is a two-sided event: releasing custodian, receiving custodian, item/container IDs, location and time evidence, seal state, environmental state, and acceptance/rejection reason. An unacknowledged handoff remains unresolved; it must not be marked delivered by an upstream agent.

## 6. NestPort / HomeOps architecture

A residence becomes a logistics endpoint with a local receiving appliance and optional internal automation. It is not a public warehouse and should not expose household item inventories to a community DHT.

### 6.1 Home receiving boundary

A physically separated external service vestibule accepts authorized delivery:
- weather-resistant receiving drawer or secure locker;
- optional aircraft winch landing plate or drone-compatible handoff interface;
- no requirement for a drone to enter the house or land inside a living area;
- parcel identity, manifest, seal, weight and outer-condition verification;
- local acceptance/rejection logic and accessible manual fallback;
- separate fire, drainage, ventilation and emergency access design review.

The receiving boundary must be reachable by conventional courier and residents even when aerial delivery is unavailable.

### 6.2 Automated internal logistics spine

The home can be fitted with a service wall/service room that connects receiving to storage. Prefer fixed, guarded mechanisms for predictable high-frequency transfers:
- drawer or tote elevator;
- low-speed enclosed conveyor or transfer shelf;
- modular tote buffer;
- vertical storage lift or carousel;
- fixed manipulator within a protected workspace;
- optional wheeled mobile manipulator for objects outside the fixed cell;
- accessible handoff drawers at resident-friendly heights.

Do not make the entire house a robot's unrestricted workspace by default. Use explicit zones, speed/force limits, presence detection, reachable manual stops, and clear human/animal right-of-way.

### 6.3 Sorting and destination classes

Initial routing classes:
1. Ambient parcel storage.
2. Refrigerated grocery compartment.
3. Frozen grocery compartment.
4. Pantry / household consumables.
5. Tools, repair parts and workshop supplies.
6. Returns and reusable packaging.
7. Recycling / packaging recovery.
8. Human review / unknown / restricted.

A package may be accepted into ambient quarantine while its contents remain unknown. Do not autonomously open an unknown parcel in v0.1.

### 6.4 Containerization

Use standard reusable totes as the interchange unit across carrier, community hub, home receiver, internal lift, storage rack and ground robot.

A v0.1 containerization operation means:
- confirm package/tote identity;
- transfer the sealed item into the correct compatible reusable tote;
- record mass, dimensions, seal state and container ID;
- route the tote to a known storage location;
- update the local inventory and custody receipt.

Automated depackaging, cutting, unpacking, food handling and individual-item sorting are later capabilities. They require dedicated guarded equipment, a known item model, contamination controls and a safe human exception route. Never introduce autonomous blade cutting as a first demonstrator.

### 6.5 Environmental storage

Ambient, chilled and frozen spaces have independent temperature sensors, alarms, door-open logging, fault state and backup-energy assessment. Acceptance must be item- and commodity-specific. An out-of-range reading creates quarantine/escalation rather than a false "stored successfully" event.

Medical or regulated products require a separate qualified handling process; a generic home refrigerator is not a medicine-management system.

## 7. Privacy, autonomy and community ownership

- Household item-level inventory stays local by default.
- Public/community records expose only what is necessary for a shared service: a service slot, an opaque delivery reference, a shared asset booking, or a minimized receipt.
- The resident controls which devices, helpers, couriers and community operators can access the receiving station.
- A homeowner can disable aerial delivery without losing road or human delivery.
- Cooperative facilities can be community-owned, with transparent booking, maintenance reserves and dispute processes.
- Cameras should be aimed at the receiving boundary, not continuously at residents or neighbours.
- Telemetry, images and provenance data require explicit retention and deletion policies compatible with applicable law.
- Mycelix governance cannot override physical safety, resident consent, a building's emergency access rules or aviation permissions.

## 8. Cross-system integration contracts

### Symthaea
Proposes modes, manifests, schedules, sorting plans and exception explanations; uses capability-bounded task proposals; must not self-authorize hazardous acts; active-inference ranking remains experimental until action dependence, purity and matched-baseline preconditions pass.

### Symtropy
Represents carrier, docks, robots, transfer stations, packages, totes, people and obstacles in the physical digital twin; coordinates replayable scenarios; consumes specialist structural, thermal, electrical and robotic models rather than claiming one rigid-body solver certifies everything.

### Sol-Atlas
Provides geographic context and site-screening evidence for land, climate, access, ecology, infrastructure and future mobility nodes; records dataset source, coordinate reference, resolution, freshness and uncertainty; is not treated as cadastral registry, certified terrain model or aviation authorization database unless that data and authority are explicitly integrated.

### AeroWeft
Supplies aircraft and carrier configuration envelopes, payload compatibility, charging/dock requirements, mission energy and environmental operating limits; aircraft eligibility is evaluated against the actual configuration and evidence, not a generic product class.

## 9. Revival plan for dormant transport

Before extending product functionality:

**P0 — establish build and authority truth**
1. Re-read DORMANT_TRANSPORT.md against current main.
2. Run cargo build --workspace --target wasm32-unknown-unknown --release.
3. Pack each Commons DNA and the hApp using the currently pinned Holochain toolchain.
4. Record exact commit, command, toolchain, output and artifact digests.
5. Fix the four create-path author bindings and validate RideMatch creation without weakening correct update/delete author checks.
6. Add tests that invoke the production validator rather than test-local replicas.

**P1 — repair the transport lifecycle**
- location-bearing ride offers and versioned route geometry;
- state transitions and two-party booking without violating Holochain author ownership;
- seat/cargo capacity reservation with duplicate/concurrency controls;
- cancellation, expiry, failed handoff, no-show and reconciliation states;
- route/ETA adapters with stated source and uncertainty;
- live telemetry snapshots through authenticated, privacy-bounded channels;
- maintenance status and asset configuration freshness.

**P2 — evidence-qualified logistics**
- cargo package and reusable tote manifest;
- multimodal segment plan and handoff receipts;
- robotic asset capability bridge to civic robotics-dispatch;
- home receiving and storage events;
- Provision food-batch and cold-chain integration;
- versioned lifecycle energy/emissions factors, with no self-awarded credits from unverified trip declarations.

**P3 — real deployment qualification**
- build to WASM;
- pack and install the hApp;
- multi-agent Sweettests using real conductors;
- validator negative controls, duplicate bookings, replay, partition and stale-event tests;
- threat model and privacy review;
- deploy only after all prerequisites are evidenced.

Unknown outcome means no new mutation until reconciliation.

## 10. Benchmarks

Report the service as a complete network, not just a drone or robot:
- successful delivery and receiving acceptance;
- P50/P95/P99 delivery and internal put-away time;
- correct item/container/storage assignment;
- duplicate, lost, damaged, rejected and unresolved deliveries;
- package custody completeness;
- temperature excursions against validated limits;
- robot interventions per 100 transfers;
- minutes of human labour per 100 accepted items;
- kWh per delivery and per internal move;
- tote reuse cycles and packaging waste;
- shared-facility utilization and availability;
- service accessibility and resident opt-out success;
- safety events, near misses and human/animal exposure.

Proposed programme targets (not achieved results):
- 100% custody records for accepted packages;
- zero accepted hard-constraint violations in the frozen qualification suite;
- 99.5% correct storage-class routing in the HIL suite;
- at least 90% end-to-end completion on a preregistered routine intake/put-away task set;
- fewer than five routine human interventions per 100 deliveries after a declared stabilization phase;
- all unknown, damaged, unsafe or low-confidence parcels are safely quarantined or returned.

No single aggregate score can hide misdelivery, safety failure or temperature excursions.

## 11. Benchmark and standards alignment

- Use HomeRobot OVMM for open-vocabulary mobile pick-and-place and its sim/real baseline.
- Use BEHAVIOR-1K / the active 2026 BEHAVIOR challenge for long-horizon household tasks, with exact evaluation split recorded.
- Use RoboCasa365 for kitchen manipulation, seen/unseen tasks and multi-task policy comparison.
- Use VDA 5050 v3.0 as an interoperability reference for mixed-fleet mobile robots. Mycelix should coordinate assets and service contracts, not replace low-level fleet-control interfaces.
- Use aircraft- and region-specific aviation requirements for aerial activity. An interoperability standard does not confer permission to fly.

## 12. Scope and non-claims

This document is a research and implementation contract. It does not claim deployed Mycelix Transit; certified delivery drones, eVTOLs, trucks or mobile robots; an operational NestPort appliance; permission for BVLOS, autonomous commercial flight or operation over people; automatic medicine or hazardous-goods handling; production supplychain settlement or inventory correctness; or validated whole-home manipulation performance.

The first demonstrator should use a stationary receiving station, sealed packages, a supervised ground robot or fixed transfer mechanism, and no autonomous depackaging. This creates a testable path from dormant Commons transport to a real human-controlled logistics endpoint.
