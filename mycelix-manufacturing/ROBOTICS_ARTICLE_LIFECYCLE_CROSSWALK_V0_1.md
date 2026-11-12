# Robotics article lifecycle crosswalk v0.1

**Status:** source audit and ownership crosswalk only. This document does not introduce a new Mycelix schema, product passport, configuration database, serial authority, lifecycle graph, or qualification engine.

**Audited source head:** `4b9d7aa7f1a69fa19b1bbd19a903ff3f4e7c63a5` (Mycelix `main`, 2026-11-10).

**Primary owner:** [ROBOT-LIFECYCLE-EVID-001 #5985](https://github.com/Luminous-Dynamics/mycelix/issues/5985).

## 1. Finding

The manufacturing cluster already represents design/BOM revisions, manufacturing routes, work orders and production workstations. These are useful inputs to a serialized robotic-assembly lifecycle, but none of the inspected integrity entry definitions is itself a complete as-built robotic article/configuration record.

The next engineering step is therefore a precise crosswalk between existing records and the missing robotics-specific relationships. Do not create a second lifecycle event graph or duplicate generic product identity / qualification semantics already assigned to [MFG-EVID-001 #3084](https://github.com/Luminous-Dynamics/mycelix/issues/3084), [CIRC-MFG-001 #3095](https://github.com/Luminous-Dynamics/mycelix/issues/3095), or [XREF-LIFE-001 #3112](https://github.com/Luminous-Dynamics/mycelix/issues/3112).

## 2. Source files inspected

The observations below refer to the exact source head above:

- `mycelix-manufacturing/zomes/bom/integrity/src/lib.rs`
- `mycelix-manufacturing/zomes/workorders/integrity/src/lib.rs`
- `mycelix-manufacturing/zomes/operations/integrity/src/lib.rs`
- `mycelix-manufacturing/zomes/machines/integrity/src/lib.rs`
- `mycelix-manufacturing/zomes/planning/integrity/src/lib.rs`
- `mycelix-manufacturing/zomes/bridge/integrity/src/lib.rs`
- `mycelix-manufacturing/crates/manufacturing_common/src/lib.rs`

This is a bounded audit of those manufacturing-cluster entry definitions, not a claim that every repository module or historical prototype has been exhaustively reviewed.

## 3. Existing-to-required crosswalk

| Engineering concept | Existing source / owner | What is currently represented | What still needs to be bound for robotics |
|---|---|---|---|
| Product design and BOM revision | BOM integrity `BomEntry`; #3084 | `design_id`, `revision`, item list and subassembly BOM references. BOM records are immutable in the inspected validator. | Bind the exact product definition and design-artifact digests that the robotic article was built against. A BOM line is not a serialized physical component. |
| BOM item / part quantity | `BomItemEntry` | `part_id`, `quantity_per`, `unit`, optional subassembly BOM hash and notes. | Bind actual selected component instances, lots and approved substitutions to each as-built article generation. Do not treat quantity and part number as serial identity. |
| Manufacturing work order | Workorders integrity `WorkOrderEntry` | Product ID, quantity, due date, status, priority, notes, optional BOM and routing hashes, creation/update timestamps. | A completed quantity is not a set of individually identified robot articles. Each physical output needs an exact manufactured-unit subject under #3084's ownership. |
| Production workstation | Machines integrity `MachineEntry` | Name, machine type, capabilities, location, throughput, status and current work order. | This is a manufacturing machine/workstation record, not the identity/configuration of the exoskeleton, hand, or JointLab assembly being produced. |
| Manufacturing routing | Operations integrity `RoutingEntry` | Design ID/revision and ordered operation steps. | Preserve the exact routing revision and execution evidence for an article without treating the plan itself as proof that the operations occurred. |
| MRP planning | Planning integrity entries | Planning run, work-order references, material demand and shortage quantities. | Planning output is not procurement, installation, commissioning or qualification evidence. |
| Bridge notification | Bridge integrity `BridgeEventEntry` | String event type, source/target cluster, string payload, optional reference hash and created time. | Do not use an opaque bridge payload as the canonical article/configuration or lifecycle model. Bind exact source subjects and evidence under the existing cross-reference owner. |
| Physical manufactured unit / serial | #3084; specialized by #5985 | Generic issue owner defines the distinction between design revision, manufactured unit and qualification receipt. No robotics-specific serialized-assembly entry was found in the inspected manufacturing-cluster definitions. | Preserve one stable physical article identity and its exact as-built configuration generation. Confirm the canonical #3084 implementation owner/schema before adding code. |
| As-built assembly generation | #5985, composed with #3084 and #3095 | #5985 requires configuration generation over exact serialized instances, parent/child relations, firmware/controller, calibration and qualification references. | Bind the exact selected component instances and installation/removal relationships for each generation. A changed configuration must not rewrite its predecessor. |
| Sensor identity and calibration | Sensor provenance #5899 | Existing sensor-provenance owner covers authenticated observation/source/clock/calibration lineage and revocation semantics. | Reference its exact sensor/calibration evidence; do not create another sensor-authentication or calibration store. A signature is not measurement truth. |
| Qualification receipt | #3084; #5985 robotics specialization | #3084 requires a receipt bound to exact unit, design/BOM revision, protocol/profile and evidence digests. | Scope each result to exact claims, task/profile, environmental/operating conditions, configuration generation and physical or simulated evidence class. |
| Repair / replacement / reuse | #3095; #5985 | #3095 owns the common manufacturing/circularity event graph and event types across manufacture, install, use, service, repair, reuse, recovery and disposal. | Use those lifecycle events to create a new article generation or successor configuration and record claim-specific revalidation obligations. |
| Cross-repository engineering subject | #3112 | Exact portable tuple: `namespace`, `subject_id`, `semantic_version`, `content_blake3`. | Consume qualified Symthaea source refs byte-for-byte when #3112's gate is met. Do not invent a competing cross-repository hash or interpret reference agreement as proof of physical execution. |
| Geometry and human-device interfaces | Sol-Atlas #708 | Spatial owner defines frame graph, geometry, registration, tolerances and uncertainty for cybernetic interfaces. | Bind exact geometry/frame revisions and applicability to the physical article generation; geometry does not itself establish fit, pressure safety or clinical validity. |

## 4. Minimum article-generation relationship

The first JointLab example should be representable as references to existing owner-controlled subjects, not as a new monolithic robot record.

For a given physical article, the reader must be able to resolve:

1. Exact product definition and design revision.
2. Exact design/BOM revision and permitted substitutions.
3. Exact manufactured-unit / serial identity.
4. As-built component instances, component serials/lots and parent-child installation relationships.
5. Firmware, controller, safety-configuration and executable/build identities where applicable.
6. Sensor identities and calibration evidence under #5899.
7. Geometry/frame and fixture revisions under Sol-Atlas #708.
8. Commissioning, experiment protocol, raw observation and qualification evidence references.
9. Predecessor configuration generation and the event that created the current generation.
10. Change-impact and revalidation dispositions for the claims affected by the change.

The stable article identity, configuration-generation identity, BOM/design identity, lifecycle-event identity and evidence-receipt identity are different things. Do not substitute Holochain `ActionHash` or `EntryHash` for engineering identity unless an explicitly qualified owner contract defines that relationship.

### Configuration change semantics

A component replacement or software/calibration change should append a new generation or successor relation. It must not mutate the historical as-built record.

Impact is claim-specific rather than blanket invalidation:

| Change | Candidate affected claims / evidence (must be decided by responsible engineering owner) |
|---|---|
| Encoder or load-cell replacement | Sensor identity, calibration, timing, force/position measurement and dependent task evidence |
| Gearbox, transmission or actuator replacement | Joint torque/speed, backlash, thermal, work/energy, load and endurance evidence |
| Hand, fingertip or end-effector replacement | Geometry/contact, grasp-force, POMDAR task subset, sensor placement and physical durability |
| Firmware/controller change | Exact build identity, control behavior, saturation/stop, regression and applicable safety evidence |
| Interface pad/liner/cuff change | Geometry, clearance, interface migration and any pressure/shear observations |
| Repair with no change to a qualified interface | Repair evidence plus a documented impact decision; do not automatically declare every claim invalid or valid |

If the impact is unknown, preserve `review_required` / `not_evaluated` rather than guessing that the prior qualification remains applicable.

## 5. First bounded demonstration: JointLab

Use Symthaea [ROB-JOINTLAB-001 #4814](https://github.com/Luminous-Dynamics/symthaea/issues/4814) and [ROB-EXP-001 #4800](https://github.com/Luminous-Dynamics/symthaea/issues/4800) as the first article, not a complete wearable exoskeleton.

A synthetic fixture should demonstrate this chain:

```
design + BOM + routing revisions
  -> work order / manufacturing process references
  -> manufactured JointLab article identity
  -> serialized actuator, transmission, bearings and instruments
  -> as-built assembly/configuration generation
  -> instrument/calibration and geometry references
  -> frozen experiment protocol and run
  -> raw observations + evidence digests + result disposition
  -> maintenance/replacement event
  -> successor configuration generation
  -> impact assessment and required recalibration/retest
  -> second experiment and generation-to-generation comparison
```

The fixture must state which links are actual existing Mycelix entries and which are unresolved external references. Do not synthesize a physical serial, calibration, measurement or qualification result and label it real.

## 6. Adversarial and negative-control matrix

The crosswalk should eventually drive conformance fixtures. Each scenario needs a positive control so a reject-all implementation cannot pass.

| Scenario | Required disposition |
|---|---|
| Valid article, BOM and exact component instances | Admit the reference chain; do not infer physical functionality |
| Wrong article serial or a component already bound to a conflicting active assembly | Reject or leave indeterminate under the owner policy |
| Planned BOM differs from as-built component list | Preserve discrepancy; do not call the planned BOM as-built truth |
| Work order status is completed but no physical article or inspection evidence exists | Do not infer assembly completion, commissioning or qualification |
| Missing predecessor for a claimed replacement/configuration generation | Reject or unresolved; never invent history |
| Duplicate/conflicting event identity | Preserve conflict; do not silently choose a winner |
| Calibration expired, revoked or for a different sensor serial | Do not admit dependent measurements as current qualified evidence |
| Firmware/controller digest differs from tested configuration | Mark affected claims review-required/not-evaluated until the owner resolves impact |
| Qualification receipt copied to a successor generation without impact review | Reject the unsupported inheritance |
| Signed assertion with no independent measurement/inspection | Preserve issuer/authenticity status separately from physical truth |
| Repair work order closed without post-repair functional evidence | Do not infer function restored or capability requalified |
| Private scan, supplier, operator or process payload in public projection | Enforce disclosure/consent policy; publish only permitted reference/digest/derived evidence |

## 7. Ownership and implementation sequence

1. **#3084 owns generic manufactured-unit and qualification semantics.** Confirm the canonical field/type owner and avoid adding a second generic physical-product model.
2. **#5985 owns the robotics specialization.** Define the smallest robotics-specific configuration-generation crosswalk using #3084 subjects.
3. **#3095 owns lifecycle events and circularity relations.** Repair/replacement/remanufacture should reference that graph, not a parallel event store.
4. **#3112 owns exact portable cross-repository subject references.** Wait for its stated Symthaea #5748 source qualification gate; do not freeze unqualified external refs.
5. **#5899 owns sensor provenance/calibration lineage.** Article configuration references these records; it does not duplicate them.
6. **Sol-Atlas #708 owns spatial geometry and uncertainty.** Preserve exact frame, artifact and registration revisions.
7. **Symthaea #4800/#4801/#4802/#4814 own experiment, physical article, system identification, simulation-to-real discrepancy and the JointLab test protocol.**

Implementation should begin only after the owners agree on the canonical identity/subject contracts and the synthetic crosswalk passes its negative controls.

## 8. Claim ceiling

This crosswalk is a source audit and engineering plan. It establishes no implemented serialized robot lifecycle schema, no physical JointLab article, no calibration validity, no manufacturing execution, no repair success, no functional capability, no safety qualification, no regulatory compliance and no production readiness.
