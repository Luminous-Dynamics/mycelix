# Integral COS Reference Node v1

**Status:** Design artifact / reference conformance model  
**Parent:** #3332  
**Formal companion:** #3333  
**External specification baseline:** Integral Development Guide v0.1 and Technical White Paper v0.1

## 1. Purpose

This document defines a deliberately small, executable-oriented reference model for mapping Integral's Cooperative Organization System (COS) semantics onto existing Mycelix production/evidence owners.

It is **not** a second manufacturing database and is **not** a claim that Mycelix implements the Integral system.

The reference node exists to make four things testable:

1. source-to-source semantic mapping;
2. evidence provenance and temporal validity;
3. separation between operational observations and downstream projections;
4. adversarial conformance against semantic collapse.

Integral currently describes COS as the operational layer that turns designs into coordinated production, including task decomposition, voluntary skill/availability matching, materials, workflow, capacity/throughput constraints, distribution, quality/safety, and inter-cooperative coordination. Its architecture also routes production information into ITC and FRS feedback loops. The Mycelix mapping therefore treats COS as an adapter/projection boundary over source-owned production evidence rather than as a duplicate source-of-truth store.

## 2. Reference graph

```text
AcceptedDesignRef
      |
      v
ProductionPlan / WorkBreakdown
      |
      +----> ResourceRequirement
      |
      +----> MaterialRequirement
      |
      v
AvailabilityObservation ----> WorkAssignment
                                      |
                                      v
                              ObservedWorkEvent
                                      |
                       +--------------+--------------+
                       |                             |
                       v                             v
             MaterialConsumption              ProcessObservation
                       |                             |
                       +--------------+--------------+
                                      |
                                      v
                                QualityEvidence
                                      |
                                      v
                              OutputDisposition
                                /           \
                               /             \
                         Outcome          Failure/Loss
                               |
                               v
                      Correction/Supersession
                               |
                 +-------------+-------------+
                 |                           |
                 v                           v
           ITC Projection             FRS Signal Projection
```

The arrows are evidence/derivation relations, not automatic authority grants.

## 3. Proposed reference records

### AcceptedDesignRef

Binds the production plan to a specific design identity and revision.

Minimum fields:

- design identity;
- revision/version;
- source/provenance;
- acceptance/reference event;
- validity/supersession state.

**Claim ceiling:** design reference only. It does not prove feasibility, safety, or current material availability.

### ProductionPlan

Represents intended work.

Minimum fields:

- plan identity;
- design reference;
- work breakdown;
- required resources;
- required skills;
- material requirements;
- intended schedule;
- dependencies;
- created/valid timestamps;
- revision lineage.

**Invariant:** plan != execution.

### ResourceRequirement

Represents a required resource without claiming that the resource is currently available.

Minimum fields:

- resource identity/type;
- required capability;
- quantity;
- time window;
- dependency references.

**Invariant:** requirement != availability.

### AvailabilityObservation

A time-bounded observation about whether a resource, worker, machine, or dependency is available.

Minimum fields:

- subject identity;
- observed state;
- observed_at;
- valid_from;
- valid_until/freshness profile;
- evidence source;
- calibration/maintenance references where relevant;
- conflict/correction state.

**Invariant:** capability != availability.

### WorkAssignment

Represents a proposed/accepted allocation of a person or resource to planned work.

It does not itself prove that work occurred.

### ObservedWorkEvent

Represents evidence that work was actually performed.

Minimum fields:

- actor/resource identity;
- plan/task reference;
- start/end or observation window;
- evidence source;
- observation provenance;
- correction/supersession lineage.

**Invariant:** declared work != observed work.

### MaterialConsumption

Represents observed material consumption, distinct from BOM/MRP requirements and procurement plans.

Minimum fields:

- material identity;
- lot/instance provenance where available;
- quantity and unit;
- consumed_at/window;
- work/process reference;
- source evidence;
- local/external origin;
- substitution reference.

**Invariants:**

- planned material != consumed material;
- imported material must remain externally originated;
- database identity != physical continuity.

### ProcessObservation

Captures measured or observed process facts.

It must retain measurement method, source, timestamp, validity and uncertainty where applicable.

**Claim ceiling:** process observation only.

### QualityEvidence

Represents a quality/test observation.

It must not silently become certification or safety acceptance.

**Invariant:** quality observation != certification acceptance.

### OutputDisposition

Represents the disposition of an output.

Possible states should remain explicit, for example:

- produced;
- rejected;
- quarantined;
- reworked;
- released under an applicable qualification;
- disposed.

**Invariant:** useful output != safe/qualified output.

### Failure/Loss

Failure is first-class historical evidence.

A later successful run must not remove or mutate the historical failure into nonexistence.

### OutcomeObservation

Captures downstream evidence about what happened after the output entered use/access/distribution.

Outcome observations may trigger FRS signals or design/process review.

They do not retroactively upgrade the original production evidence.

## 4. Projection boundaries

### COS -> ITC

The COS adapter may expose eligible source-owned labor evidence to an ITC adapter.

The following transition is intentionally prohibited:

```text
ObservedWorkEvent
      !=
ITCLedgerEntry
```

An explicit ITC policy/adapter transition is required.

Therefore:

```text
production observation
    -> eligible contribution evidence
    -> ITC policy evaluation
    -> ITC ledger record
```

The first arrow is data exposure; the second is a policy decision.

### COS -> FRS

COS may expose source observations to FRS.

FRS may derive:

- anomaly findings;
- diagnostics;
- forecasts;
- simulations;
- recommendations.

But:

```text
FRS recommendation != production observation
FRS finding != source observation
FRS recommendation != authorization
```

If a recommendation causes an operational change, the resulting action requires its own authorized transition and its own effect/observation evidence.

### COS -> CDS

Operational evidence can inform CDS deliberation and review.

It must not bypass the applicable decision/authorization boundary.

## 5. Federation

Foreign-node evidence keeps its origin.

A recognized external attestation can be used under an explicit recognition/trust contract:

```text
foreign evidence
    -> recognized foreign evidence
    -> local decision/input
```

It must never become:

```text
foreign evidence -> locally originated observation
```

Recognition is not provenance rewriting.

## 6. Current Mycelix implementation anchors

The current manufacturing common layer already provides useful source owners:

- `WorkOrder` and lifecycle transitions;
- `BillOfMaterials` / `BomItem`;
- `RoutingSequence` / `RoutingStep`;
- `Machine` / `MachineStatus`;
- `MrpResult`;
- `PlannedOrder`;
- `ScheduledOperation`;
- `CapacityWarning`;
- `MaterialShortage`.

These are strong planning/production primitives, but the reference node must not infer that they already satisfy all COS evidence requirements.

In particular, the current work-order status lifecycle alone does not prove observed work, consumed material, quality qualification, or downstream outcome.

The reference node therefore treats these types as **source-owner candidates**, not as a completed COS implementation.

## 7. Adversarial conformance corpus

The negative corpus must execute before positive conformance.

| ID | Invalid transition | Expected result |
|---|---|---|
| COS-N-001 | Completed WorkOrder -> observed work | reject |
| COS-N-002 | Planned BOM quantity -> consumed material | reject |
| COS-N-003 | Skill credential -> available worker now | reject |
| COS-N-004 | Machine capability -> available machine now | reject |
| COS-N-005 | Expired calibration -> current QA evidence | reject |
| COS-N-006 | Useful output -> safe/qualified output | reject |
| COS-N-007 | One successful run -> general capability | reject |
| COS-N-008 | Later success -> erase historical failure | reject |
| COS-N-009 | Imported component -> local dependency | reject |
| COS-N-010 | FRS prediction -> production observation | reject |
| COS-N-011 | Production event -> automatic ITC mint | reject |
| COS-N-012 | ITC entry -> physical work proof | reject |
| COS-N-013 | Foreign attestation -> local-origin evidence | reject |
| COS-N-014 | Recommendation -> operational effect without authority | reject |
| COS-N-015 | Distribution complete -> quality certification | reject |
| COS-N-016 | Unknown denominator -> throughput/share claim | reject |

Positive fixtures should mirror each negative case and demonstrate the missing explicit binding or authorization.

## 8. Formal refinement map

The reference node is intended to supply concrete subjects for #3333:

| Formal property | Reference subject | Refinement target |
|---|---|---|
| COS-FV-001 | production -> ITC boundary | adapter transition |
| COS-FV-002 | WorkOrder/Plan vs ObservedWorkEvent | evidence binding |
| COS-FV-003 | Machine/worker capability vs AvailabilityObservation | temporal state |
| COS-FV-004 | OutputDisposition vs QualityEvidence | qualification boundary |
| COS-FV-005 | Availability/QA evidence validity | freshness evaluator |
| COS-FV-006 | Failure/Loss + later Outcome | append/supersession history |
| COS-FV-007 | ProductionPlan vs execution | plan/receipt separation |
| COS-FV-008 | FRS finding/recommendation | projection boundary |
| COS-FV-009 | MaterialConsumption origin | dependency classification |
| COS-FV-010 | foreign attestation | provenance origin |

A theorem without a production refinement remains **AbstractModelOnly**.

## 9. Qualification states

The reference node must expose maturity explicitly:

```text
Conceptual
  -> Designed
  -> Implemented
  -> LocallyTested
  -> RepositoryQualified
  -> PilotObserved
  -> ExternallyValidated
```

No single scalar replaces this vector.

A passing semantic test suite does not establish physical productivity, safety, ecological performance, economic outcomes, or Integral validation.

## 10. Cross-domain reuse

The same reference semantics should eventually be instantiated against:

- H2 hydroponic lettuce;
- repair/restoration;
- fabrication/manufacturing;
- water treatment;
- energy service.

The purpose is to test whether the semantic invariants survive domain substitution.

If the same invariant can be exercised against multiple productive domains without weakening its meaning, the evidence becomes stronger for the **neutral substrate semantics**, while still not proving the real-world performance of those domains.

## 11. Nonclaims

This artifact does not claim:

- Integral endorsement or ratification;
- that Mycelix is Integral's preferred implementation;
- that current Mycelix manufacturing code implements the complete COS;
- physical production performance;
- safety certification;
- economic or ecological outcomes;
- successful operation of a real Integral node.

## 12. Source baseline

Integral's Development Guide v0.1 describes COS as the operational system for translating designs into tasks/workflows, matching skills and availability, managing materials, balancing capacity/throughput, distribution, quality/safety, and cooperative coordination.

The White Paper v0.1's greenhouse example similarly routes CDS -> OAD -> COS -> ITC -> FRS as a continuous feedback loop.

These documents are treated as external requirements/specification inputs, not as evidence that the described system has already been implemented or validated.
