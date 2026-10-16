# Traffic Diversity and Future Airspace Profile V1

**Status:** Research and architecture profile; not an operational ATM/UTM specification  
**Research checked:** 2026-10-15  
**Authority ceiling:** Synthetic research metadata and test planning only; no flight authorization, separation service, or aircraft-control authority  
**Related work:** [Airspace Coordination Architecture V1](AIRSPACE_COORDINATION_ARCHITECTURE_V1.md), [Controller Assistance Benchmark V1](CONTROLLER_ASSISTANCE_BENCHMARK_V1.md), [tracking issue #5023](https://github.com/Luminous-Dynamics/mycelix/issues/5023)  
**Machine-readable catalogue:** [traffic_diversity.json](../../tools/airspace-sim/fixtures/traffic_diversity.json)

## 1. Why a single traffic model is inadequate

Airspace is not a collection of interchangeable points moving at constant velocity. Traffic classes differ in speed range, turn/acceleration capability, climb/descent profiles, endurance, weather sensitivity, sensor/communication links, cooperation, emergency behavior, and environmental footprint. These differences become especially pronounced when conventional aircraft share interfaces with low-altitude UAS/AAM and higher-airspace operations.

This is not just a speculative future problem:

- The FAA's current [UTM concept](https://www.faa.gov/uas/advanced_operations/traffic_management) describes a cooperative, highly automated information-sharing ecosystem for multiple beyond-visual-line-of-sight drone operations where traditional positive air traffic services are not provided. It also says operators need a way to avoid crewed aircraft.
- The FAA's [Higher Airspace Traffic Management (HATM)](https://www.faa.gov/uas/advanced_operations/hatm) page describes a diverse future fleet including free balloons, long-endurance high-altitude platforms, remotely piloted aircraft, supersonic transports and hypersonic aircraft. Its cooperative-area model allocates responsibilities under regulatory authority; it is not a blanket transfer of authority to an arbitrary distributed AI.
- EUROCONTROL's [ECHO 2 project](https://www.eurocontrol.int/project/european-concept-higher-altitude-operations) covers space-launch monitoring, re-entry, High-Altitude Platform Systems (HAPS), supersonic/hypersonic, and suborbital vehicles.
- NASA's [Advanced Air Mobility research](https://www.nasa.gov/aeronautics/envisioning-a-future-airspace-with-advanced-air-mobility/) emphasizes the challenge of integrating new passenger/cargo aircraft and conventional traffic in shared airspace.

These programmes motivate an architecture, not a claim that Mycelix implements or conforms to their concepts.

## 2. Separate dimensions instead of one enormous vehicle enum

Vehicle name alone is insufficient. Model the following as orthogonal, versioned attributes once a real system schema is designed:

1. **Vehicle / kinematic family:** fixed-wing, rotorcraft, multirotor, tiltrotor/tiltwing, glider, balloon, airship, high-altitude platform, high-speed vehicle, launch/re-entry body, or a bounded aggregate of cooperative vehicles.
2. **Operating regime:** airport/terminal, en-route, low-altitude urban/rural, offshore, high-altitude, launch/re-entry corridor, or transition between regimes.
3. **Control and cooperation:** onboard human pilot, remotely piloted, automated with supervised operator, or other explicitly declared mode; cooperation may be authenticated, limited, degraded, unknown, or absent.
4. **Trajectory and dynamics model:** model identifier, version, validity domain, bounded acceleration/turn rates where supported, wind dependence, uncertainty representation, and declared unsupported conditions.
5. **Communications / navigation / surveillance quality:** source provenance, timestamp and latency, integrity, continuity, update rate, coverage gaps, and behavior when links or sensors fail.
6. **Operational mission:** passenger, cargo, emergency/medical, search-and-rescue, fire response, inspection, science, telecommunications, agriculture, defense/state, or test flight. Mission urgency is not the same thing as a legal priority rule.
7. **Environmental and infrastructure effects:** wake/rotor wash, noise, downwash, ground exposure, landing/vertiport/charging needs, energy reserve, contingency landing options, and ground-service dependencies where relevant.

Keep regulatory authorization, separation minima, right-of-way/priority rules, and operator privileges in jurisdiction-specific policy modules supplied by competent authorities. They must not be inferred from a descriptive class, mission, provider reputation, or a Mycelix consensus result.

## 3. Vehicle / operation coverage catalogue

The companion JSON catalogue deliberately separates **known categories** from **implemented models**. Its entries are coverage requirements, not executable traffic dynamics.

| Traffic family | Examples to cover | Distinguishing aspects the model must represent |
|---|---|---|
| Conventional fixed-wing | Airline, regional, business and general-aviation aircraft | Performance envelope, terminal/en-route procedures, climb/descent, wake category from authoritative data, diversion and runway dependencies |
| Rotorcraft | Helicopters and other rotorcraft | Hover/low-speed regimes, constrained landing options, rotor wash, mission-specific routes and contingency landing |
| Gliders / unpowered flight | Sailplanes and similar craft | Energy-state dependence, thermals/wind, limited climb capability and launch/recovery areas |
| Low-altitude UAS | Small multirotors, fixed-wing drones, inspection and delivery fleets | High density, BVLOS operations, link/surveillance quality, low-altitude obstacles and ground exposure |
| Larger uncrewed / cargo | Larger cargo UAS and remotely piloted fixed-wing aircraft | Mixed traffic, performance envelope, operational containment and detect-and-avoid capability |
| eVTOL / AAM | Passenger air taxis, electric lift-plus-cruise and tilt designs | Transition phases, vertiport throughput, charging/energy reserves, noise, urban obstacles and diversion capacity |
| Public-service / specialized | Firefighting, medical, SAR, agriculture, law-enforcement or disaster-response flights | Dynamic mission needs, protected activity areas, rapidly changing constraints and human-authorized precedence rules |
| Balloons / airships | Free balloons, tethered balloons, powered or unpowered airships | Wind-field dependence, drift, low acceleration authority, long endurance, volume/altitude uncertainty and recovery planning |
| HAPS / HALE | Solar aircraft, stratospheric platforms and high-altitude long-endurance RPAS | Persistent loiter/orbits, high-altitude environmental conditions, long-duration state estimates, cross-border continuity |
| Supersonic / hypersonic | High-speed civil, research or state vehicles | Rapid space-time evolution, thermal/propulsive phases, long-distance prediction sensitivity, sonic-boom/environmental constraints and special contingency areas |
| Launch / ascent / re-entry | Suborbital vehicles, launch stages, spaceplanes and returning bodies | Phase transitions, time-varying hazard/footprint volumes, high-energy failure contingencies, launch windows and cross-domain coordination |
| Unknown / non-cooperative | Unexpected, uncorrelated, lost-link or data-poor traffic | Uncertain identity/intent, track quality, conservative degraded states and escalation to an appropriate existing service |
| Cooperative aggregate / swarm | A fleet of coordinated UAS or multiple provider-managed vehicles | Membership and identity changes, aggregate-vs-individual resolution, inter-vehicle separation, communications partitions and bounded computational load |

**Current implementation status:** The airspace-sim kernel remains a synthetic local-coordinate, constant-velocity research evaluator. It does **not** currently model the physical dynamics, regulatory separation standards, performance envelopes, detect-and-avoid behavior, wake/downwash, wind fields, transitions, launches/re-entry, cooperative swarms, or high-altitude atmospheric effects of any of the real-world families above. The table is a roadmap; it is not a claim of operational coverage.

## 4. Interaction matrix matters more than isolated vehicle demos

An airspace-management system must consider interactions across families and regime boundaries. The JSON catalogue starts a named backlog including:

- Conventional aircraft with a low-altitude drone and with a helicopter near a constrained route.
- eVTOL arrival flow competing for vertiport slots while a runway/approach disruption changes conventional traffic.
- A glider or balloon whose motion depends on environmental forces, rather than a fixed-velocity extrapolation.
- Fire response or medevac mixed with scheduled flow without hard-coding a universal priority order.
- HAPS/high-altitude long-endurance traffic near a high-speed trajectory.
- Launch/re-entry hazard volumes appearing, moving, or expiring across transiting traffic.
- Cooperative swarms with membership changes, duplicate identifiers, provider partitions, or lost communications.
- Non-cooperative/unknown traffic in the presence of stale or incomplete surveillance data.
- A model-unsupported class or cross-regime transition that must return **unresolved**, not inherit an unrelated aircraft's model by default.

These are **planned scenario descriptors**, not yet executable interaction simulations. Each descriptor must be turned into model-specific fixtures only after a justified dynamics approximation and an independent oracle are documented.

## 5. Recommended layered system

### A. Shared qualified situation picture

Exchange time-qualified intent, track state, airspace constraints, weather, infrastructure state, and source freshness. Preserve provenance and uncertainty. Distinguish measured, predicted, planned, and operator-declared information. Cross-domain translations must retain units, reference frame, vertical datum, temporal semantics, and transformation uncertainty.

### B. Domain models behind a stable interface

Do not force every aircraft into a single point/constant-velocity class. Define explicit model contracts that state supported operating regimes, state variables, input requirements, bounds, uncertainty representation, and invalidation conditions. A model should answer whether it can evaluate this state; it must never silently extrapolate outside its validated domain.

### C. Domain-specific planners and cross-domain coordination

Specialize where necessary (e.g. terminal arrival sequencing, high-density low-altitude cooperative flow, HAPS persistent operation, time-varying launch/re-entry hazard regions). Coordinate through shared intent and constraint contracts. Treat a crossing between regimes as an explicit interface and handoff, not an accidental effect of matching numeric coordinates.

### D. Independent constraint/invariant evaluation

Every candidate is evaluated by a checker independent from the proposal generator. Hard constraints cannot be traded against throughput, fuel, delay, equity, or noise in one weighted score. If the checker or domain model cannot establish its declared property, report unresolved/no-feasible-candidate.

### E. Existing service and human fallback

Keep applicable air-navigation authority and approved operational services in charge of authorization and regulated responsibilities. Design human interfaces for mixed traffic, conflicting data, degraded coverage and handovers. A nominal override button is insufficient unless the fallback has time, information, authority and training to be effective.

### F. Evidence / data governance

Every evaluation binds source snapshots, model and checker versions, configuration, corpus revision, random seed, and results. Use real traffic data only under proper access, safety, privacy, and licensing arrangements. Record unknowns rather than manufacture complete tracks.

## 6. Roadmap by evidence, not by futuristic appeal

1. **Now — taxonomy and negative controls:** version the class/mission/regime catalogue; test that IDs, interaction descriptors, support status and non-claims are explicit and unique. Keep new real-world classes outside the existing evaluator until modeled.
2. **Next — model interfaces and generic capability qualification:** give every dynamics adapter a declared domain and uncertainty contract; add capability compatibility tests. Unsupported combinations must fail closed.
3. **Then — separate synthetic profiles:** validate fixed-wing/terminal-flow abstractions, low-altitude cooperative UAS, eVTOL/vertiport sequencing, and wind-dependent balloon/HAPS profiles independently. These are different projects, not one switch on FlightIntent.
4. **Cross-domain interaction:** only after the constituent models have independent, domain-appropriate tests, evaluate UAS/crewed-aircraft interactions, terminal/urban transitions, and high-altitude/launch integration.
5. **Shadow research and approved trials:** compare with authorized existing services and qualified human reviewers under suitable approvals. Do not infer a live operating capability from synthetic tests.

## 7. Fail-closed rules

- Unknown class, unsupported dynamics, invalid operating-regime transition, incompatible datum/time/units, excessive uncertainty, stale surveillance, or absent wind input where required is **unresolved/degraded**, not “clear.”
- Never substitute the conventional fixed-wing model for a rotorcraft, balloon, high-speed vehicle, or launch/re-entry body without explicit model approval and evidence.
- Vehicle class or mission does not confer authorization, priority or permission to enter airspace.
- A valid route reservation does not show that the vehicle can safely fly it.
- Missing or uncorrelated traffic cannot be interpreted as empty airspace.
- A passing corpus means only conformance to its encoded assumptions; it is not a probability of safety or regulatory approval.

## 8. Sources reviewed

- FAA, [Unmanned Aircraft System Traffic Management](https://www.faa.gov/uas/advanced_operations/traffic_management) — collaborative, automated low-altitude UAS operations, complementary to existing air traffic services.
- FAA, [Higher Airspace Traffic Management](https://www.faa.gov/uas/advanced_operations/hatm) — cooperative-area concept, higher-airspace vehicle diversity and differentiated operational responsibilities; HATM ConOps v2.0 is listed as December 2025 and the page was updated February 4, 2026.
- EUROCONTROL, [ECHO 2: European Concept for Higher Altitude Operations](https://www.eurocontrol.int/project/european-concept-higher-altitude-operations) — launch/re-entry monitoring, HAPS integration and advanced high-speed/suborbital vehicle operations.
- EUROCONTROL, [Space and higher airspace operations](https://www.eurocontrol.int/space-and-higher-airspace-operations) — integration research for space vehicles, HAPS, supersonic/hypersonic traffic and cross-border operations.
- NASA, [Envisioning a Future Airspace with Advanced Air Mobility](https://www.nasa.gov/aeronautics/envisioning-a-future-airspace-with-advanced-air-mobility/) — integration of new passenger/cargo operations with existing aircraft and shared airspace.

Sources checked 2026-10-15. Research summaries are not regulatory interpretations. Applicable rules and competent authority requirements must be rechecked for any real operational proposal.
