# Robotics Lifecycle Evidence Profile v0.1

**Status:** documentation / owner-crosswalk proposal. This file does not add a new schema, ledger, zome, controller, maintenance scheduler, or qualification authority.

**Tracking issue:** [ROBOT-LIFECYCLE-EVID-001 #5985](https://github.com/Luminous-Dynamics/mycelix/issues/5985)

**Scope:** compose existing Mycelix manufacturing, product evidence, process-run, temporal evidence and circularity owners for serialized robotic assemblies.

## 1. Design objective

A robot's behavior and qualification depend on its exact as-built configuration: base, joints, actuators, transmissions, brakes, sensors, controller, firmware, power system, tool, payload, calibration and safety configuration. The asset record must therefore identify an exact configuration generation and preserve its change history.

The governing distinctions are:

```
product definition != design revision != BOM revision
component lot != serialized component instance
planned BOM != as-built assembly
as-built assembly != commissioned embodiment
commissioned embodiment != qualified task capability
work order completed != physical function restored
signed claim != independently verified physical truth
```

This profile is an integration map over existing Mycelix and Symthaea owners. Before implementation, each field must be assigned to its canonical schema owner. Do not copy owner structs into a new robotics crate.

## 2. Canonical owners to compose

| Concern | Existing owner | Robotics-profile use |
|---|---|---|
| Design revision, manufactured unit and qualification receipt | [MFG-EVID-001 #3084](https://github.com/Luminous-Dynamics/mycelix/issues/3084) | Distinguish immutable design, exact physical unit, BOM, component lots, calibration, protocol and qualification evidence |
| Typed process definitions and resource routing | [MFG-PROC-001 #3085](https://github.com/Luminous-Dynamics/mycelix/issues/3085) | Reference process definitions, capability profiles, tooling, fixtures, routes and machine offers |
| Process execution evidence | [Process-run receipt PR #5228](https://github.com/Luminous-Dynamics/mycelix/pull/5228) | Bind a run to exact process, machine, inputs, outputs, model/configuration and evidence references |
| Bitemporal physical-state evidence | [PR #5436](https://github.com/Luminous-Dynamics/mycelix/pull/5436) | Preserve occurred-time vs recorded-time, clock/timebase and source mapping semantics |
| Lifecycle and circularity | [CIRC-MFG-001 #3095](https://github.com/Luminous-Dynamics/mycelix/issues/3095) | Compose Manufacture, Install, Use, Service, Repair, Reuse, Refurbish, Remanufacture, HarvestParts, Recycle, RecoverMaterial and Dispose events |
| Closed-loop manufacture/repair/recovery pilot | [CIRC-MFG-PILOT-001 #3101](https://github.com/Luminous-Dynamics/mycelix/issues/3101) | Synthetic end-to-end lifecycle fixture and negative controls |
| Exact Symthaea lifecycle references | [XREF-LIFE-001 #3112](https://github.com/Luminous-Dynamics/mycelix/issues/3112) | Preserve the four-field portable reference tuple: namespace, subject_id, semantic_version, content_blake3 |
| Manufacturing reference cell | [MFG-003 #4689](https://github.com/Luminous-Dynamics/mycelix/issues/4689) | Establish metrology, CNC, polymer additive, electronics/assembly and inspection context |
| Robot observation provenance precedent | [PR #5139](https://github.com/Luminous-Dynamics/mycelix/pull/5139) | Reuse patterns for source IDs, mission receipts, map snapshots, sensor artifact and calibration references |
| Robotics lifecycle profile | [ROBOT-LIFECYCLE-EVID-001 #5985](https://github.com/Luminous-Dynamics/mycelix/issues/5985) | This crosswalk; it does not replace any owner above |

The separate `mycelix-supplychain` mirror is archived. New implementation work belongs in the active unified `Luminous-Dynamics/mycelix` repository and must preserve the maturity limits stated by its README.

## 3. Identity model — conceptual roles, not new canonical structs

Every integration fixture must keep the following identity roles distinct. The concrete type and serialization owner must be identified during implementation.

1. **Product definition:** commercial or internal product-family identity.
2. **Design revision:** immutable, versioned engineering intent and exact artifact digests.
3. **BOM revision:** exact declared part/quantity/substitution set for a design revision.
4. **Component lot:** production cohort / material batch and its source lineage.
5. **Serialized component instance:** one physical actuator, gearbox, sensor, controller, tool, battery or other replaceable item.
6. **Assembly generation:** one exact parent-child configuration of serialized parts.
7. **Commissioned embodiment:** exact physical article bound to its device/controller topology and calibration state, when the Symthaea owner has established that identity.
8. **Qualification receipt:** a claim-specific test result bound to the exact article, configuration, protocol, instruments, calibration and evidence.
9. **Lifecycle event:** one immutable manufacture, installation, use, service, repair, return or disposition assertion.

Do not use a friendly robot name, USB path, fieldbus address, machine type, DID, lot ID or display label as a substitute for an exact physical serial/configuration identity.

### External Symthaea design subjects

Symthaea's robot-design work is the canonical owner for as-designed robot intent; it is not a fabricated unit or qualification. Cross-repository consumers should use qualified exact-subject exports when available. The planned portable tuple is:

```text
namespace
subject_id
semantic_version
content_blake3
```

A Mycelix-local wrapper digest may differ; cross-repository equality applies to the four source fields, not to local wrapper hashes. See Symthaea [ROB-DESIGN-001A PR #4806](https://github.com/Luminous-Dynamics/symthaea/pull/4806) and Mycelix [XREF-LIFE-001 #3112](https://github.com/Luminous-Dynamics/mycelix/issues/3112). The design PR is still an open draft at the time of this profile; do not freeze golden vectors from an unqualified or superseded source head.

## 4. Configuration generation and change impact

A robot configuration is a reference set, not a mutable bag of latest values. At minimum it should resolve:

- product/design revision and BOM revision;
- serialized component instances and approved substitution references;
- parent-child installation relations;
- tool/end-effector and payload configuration where applicable;
- firmware/software build identity and controller configuration;
- safety configuration identity;
- commissioned embodiment and topology reference where available;
- calibration references and validity/currentness;
- task-specific qualification receipts and their claim/ODD scope.

Replacing a component or changing software creates a new configuration generation. The previous generation and its evidence remain immutable and queryable.

Change impact must be **claim-specific**, not a blanket invalidation rule. Illustrative cases:
- encoder replacement may require recalibration and dependent position/velocity tests;
- gearbox replacement may require backlash, torque, thermal and endurance retesting;
- gripper replacement may affect tool-center-point calibration, grasping, payload and manipulation qualification;
- controller/firmware change may require timing, dispatch, regression and safety review;
- battery replacement may affect power, thermal, endurance and charging tests.

The responsible engineering/qualification owner determines the applicable affected-claim set. Mycelix records the decision and its provenance; it does not invent the engineering rule.

## 5. Lifecycle sequence for a serialized robotic assembly

The following is a conceptual trace through existing owners, not a request to create a second lifecycle state machine:

```text
design revision released
  -> BOM revision frozen
  -> material/component lots received
  -> component instances manufactured / serialized
  -> inspection and nonconformance dispositions
  -> assembly generation recorded
  -> commissioning / topology / calibration references attached
  -> experiment or qualification protocol frozen
  -> process/test run and outcomes recorded
  -> task-specific qualification reviewed
  -> use / service observations
  -> fault or damage observed
  -> repair / replacement / recalibration
  -> new assembly/configuration generation
  -> post-repair functional verification
  -> affected claims reviewed or requalified
  -> reuse / refurbishment / remanufacture / harvest / recycle / dispose
```

A lifecycle event may be planned, asserted, observed, verified, reviewed, qualified, disputed, rejected, superseded or unknown under the owning schema. These words must not be collapsed into one generic `completed` flag.

### Evidence origin and trust boundaries

A credential signature can establish that an issuer signed a specific assertion and that its bytes have not changed. It does not establish that:
- the physical event occurred;
- a sensor measured the claimed quantity accurately;
- calibration was valid and correctly performed;
- an inspection was independent;
- a product met a specification;
- a machine is safe or qualified for a particular ODD;
- a task was successfully completed;
- an actor had statutory or operational authority.

Keep these checks separate:
1. byte integrity;
2. signature validity;
3. issuer identity and role authorization;
4. source/event authenticity;
5. instrument identity and calibration currentness;
6. measurement validity and uncertainty;
7. engineering acceptance / qualification;
8. statutory approval and operational authority.

Mycelix credentials and DKG claims do not grant actuator or machine authority.

## 6. First integration fixture: Symthaea JointLab

Use the bounded single-joint physical laboratory from Symthaea [ROB-JOINTLAB-001 #4814](https://github.com/Luminous-Dynamics/symthaea/issues/4814) as the first cross-repository example.

The fixture should represent:

1. robot/joint design subject and exact revision;
2. BOM and component lot references;
3. serialized actuator, transmission, bearing, encoder, controller and sensing instances;
4. assembly/topology generation;
5. instrumentation identities and calibration references;
6. commissioned article identity where available;
7. preregistered experiment protocol and exact run ID;
8. measurements/results as evidence references, not copied values promoted to truth;
9. a maintenance/replacement event;
10. new configuration generation and required recalibration/retest references;
11. post-repair result and comparison against the old generation.

Start with a synthetic fixture. A synthetic event sequence establishes schema composition only; physical events must not appear until real instrumented observations with source/calibration lineage exist.

### Required negative controls

- design revision asserted as a manufactured unit;
- planned BOM part asserted as consumed or installed;
- lot ID substituted for serialized-instance identity;
- serial number reused under a conflicting issuer/namespace;
- component installed into the wrong parent/configuration generation;
- missing or stale calibration used for a measurement claim;
- wrong experiment protocol or wrong physical article attached to a result;
- duplicate event ID replayed with different content;
- stale, reordered or missing predecessor in a configuration/lifecycle chain;
- signed assertion treated as independently verified measurement;
- simulated outcome promoted to physical observation;
- work order marked completed treated as functional restoration;
- replaced component silently retaining old qualification references;
- one task's qualification generalized to a different tool, payload or ODD;
- historical failure, correction or dispute erased by a later successful event;
- private supplier, operator or site data disclosed in a public projection.

## 7. Privacy and interoperability

- Keep proprietary process recipes, commercial terms, supplier relationships, exact sensitive facility location and personal operator information private or selectively disclosed.
- Prefer digest/reference links for high-rate telemetry; do not replicate raw high-frequency motor or sensor streams into a DHT by default.
- Preserve external protocol identity/version for OPC UA Robotics, VDA 5050 or other integration messages. A protocol message is not a physical qualification receipt.
- Use versioned projections/adapters at system boundaries. Do not treat Digital Product Passport, GS1 EPCIS, or a vendor equipment API as a new canonical internal asset ontology.
- Correction, revocation, dispute, supersession, retention and source-rights semantics must remain available; immutability must not freeze incorrect assertions as truth.

## 8. Acceptance criteria for implementation

1. A crosswalk identifies the canonical owner for each identity/event/evidence field.
2. Product definition, design revision, BOM, lot, serialized instance, assembly generation, commissioning and qualification are distinguishable.
3. A synthetic JointLab trace connects design through component/assembly, calibration, experiment, maintenance and new configuration generation.
4. Exact external Symthaea subject refs preserve all four semantic fields without relabeling or local-hash substitution.
5. Duplicate/replayed events are idempotent where appropriate; conflicting duplicate identities are rejected or quarantined.
6. Event order, source time, record time and timebase semantics are explicit.
7. Stale/missing calibration and configuration mismatches block dependent claims.
8. Repair/replacement preserves historical evidence and produces a new configuration generation.
9. Signature validity, issuer authority, source authenticity, measurement validity and engineering acceptance are separate outcomes.
10. Selective disclosure preserves allowed verification without revealing restricted information.
11. Existing manufacturing/circularity schemas and fixtures remain backward-compatible.
12. No product safety, physical qualification, fleet availability, statutory compliance or operational authority is claimed by the lifecycle record itself.

## 9. Non-goals

- No new robotics identity ontology, asset passport ledger, credential format or DKG event graph.
- No new maintenance scheduler, work-order engine, digital twin, CAD/BOM engine or manufacturing controller.
- No actuation permission or autonomous safety approval.
- No raw telemetry ingestion design in this profile.
- No claim that this document or a synthetic fixture means the Mycelix manufacturing cluster is production-ready.
