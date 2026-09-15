# REGEN-011 — Exact Material, Allocation, and Admission Refinement v1

Status: normative preregistration refinement only. No resource-extraction, property, custody, procurement, processing, agronomic, climate, market, or physical-action authority.

This document sharpens `REGEN-011_BIOMASS_FEEDSTOCK_PROVENANCE_PREREG_V1.md` against the current Mycelix repository boundaries. It does not replace the original preregistration.

## 1. Refinement theorem

REGEN-011 needs to distinguish not only *what evidence exists*, but also *which material-state proposition that evidence is allowed to support*.

The refined ladder is:

```text
resource occurrence
-> physically accessible quantity
-> ecologically allocable quantity
-> rights-cleared quantity
-> formed physical lot
-> controlled/custodied lot state
-> composition-resolved lot state
-> process-admissible quantity
-> unreserved process-admissible quantity
-> process reservation
-> consumed process input
```

No arrow is automatic.

In particular:

```text
physically accessible
!= ecologically allocable

 ecologically allocable
!= rights-cleared

 rights-cleared
!= controlled/custodied

 controlled/custodied
!= composition-resolved

 composition-resolved
!= process-admissible

 process-admissible
!= unreserved

 reservation
!= ownership
!= custody
!= execution

 process execution
!= qualified output product
```

Unknown or unresolved state at any required rung MUST remain explicit rather than being promoted to the next rung.

## 2. Repository ownership audit

The current repository already contains useful adjacent concepts, but none should be widened into biomass provenance authority.

### 2.1 Manufacturing BOM

`mycelix-manufacturing` models `BillOfMaterials` / `BomItem` and its Holochain BOM entries as **design requirements**: part identity, required quantity, unit, and optional sub-assembly relationships.

Therefore:

```text
BomItem(part_id, quantity_per, unit)
!= physical lot
!= current inventory
!= custody
!= provenance
!= process qualification
```

A BOM may later request biomass-compatible inputs, but it cannot manufacture evidence that those inputs physically exist.

### 2.2 Manufacturing MRP

Manufacturing also exposes planning outputs including `MrpResult`, `PlannedOrder`, `MaterialShortage`, and `quantity_available`.

These are planning facts within the manufacturing model.

```text
MRP quantity_available
!= identified physical biomass lot
!= ecologically allocable quantity
!= rights-cleared quantity
!= measured quantity
!= safe feedstock
```

A later adapter MAY project a qualified REGEN quantity into Manufacturing planning. The reverse direction MUST NOT strengthen evidence.

### 2.3 Commons batch utilities

`commons-types::batch` is a Holochain record batch-fetch utility. It is not a material-batch ontology.

REGEN MUST NOT reuse `BatchGetResult` or similar fetch abstractions as physical batch identity merely because they use the word `batch`.

### 2.4 Marketplace

A marketplace listing or offer is a social/economic artifact.

```text
listed quantity
!= measured quantity
!= seller title
!= custody
!= process admission
!= immediately available quantity
```

Marketplace MAY reference a REGEN lot and its qualification evidence, but publication does not create that evidence.

## 3. Canonical conceptual subjects

The first executable REGEN-011 implementation SHOULD keep the following subjects distinct even if v1 initially implements only a subset.

```text
ResourceOccurrence
LotFormationEvent
BiomassLotProfile
MaterialStateSnapshot
EcologicalAllocationAssessment
RightsAndCustodyReferenceSet
FeedstockAssessment
ProcessInputReservation
LotTransformationEvent
```

The purpose is to prevent one overloaded `BiomassLot` struct from becoming an implicit inventory, property, scientific-evidence, and process-authority database.

## 4. Resource occurrence

A `ResourceOccurrence` describes an observed/reported/inferred/forecast resource context before a bounded physical lot necessarily exists.

Examples:

- standing forest residues;
- crop residues still in a field;
- predicted harvest residues;
- municipal trimming forecasts;
- processing residues expected from a future work order.

Occurrence quantity SHOULD be referenced through PEF evidence where practical.

```text
Forecast resource occurrence
!= present inventory
```

and:

```text
Inferred resource occurrence
!= physically inspected material
```

## 5. Lot formation event

A `BiomassLotId` begins to identify a physical aggregation only after a declared lot-forming event such as:

- harvest;
- collection;
- segregation;
- receipt;
- generation as an output/by-product of a known process;
- controlled consolidation of existing child/parent lots.

Conceptually:

```text
LotFormationEvent {
    event_id,
    resulting_lot_id,
    event_kind,
    source_occurrence_refs[],
    parent_lot_refs[],
    event_evidence_refs[],
    occurred_at?,
    location_ref?
}
```

The event record does not itself establish rights, measurement truth, contamination state, or process admission.

## 6. Lot identity versus changing state

A physical lot can change state without every change necessarily creating a new lot identity.

Examples include:

- drying;
- cooling;
- ordinary storage aging;
- a new moisture measurement;
- movement between custody locations.

These SHOULD normally update evidence/state snapshots rather than silently minting unrelated lots.

By contrast, operations that materially alter aggregation identity SHOULD create explicit lot lineage, including at least:

- split;
- merge;
- blending;
- segregation;
- removal of a sub-lot;
- addition of external material;
- transformation into a different material/product class.

The exact v1 identity rule may remain conservative, but the protocol MUST NOT allow hidden merge/split/blend operations under an unchanged semantic history.

## 7. Material state snapshot

Measurements are time-dependent properties of a lot, not timeless attributes of the lot ID.

A conceptual `MaterialStateSnapshot` may bind:

```text
lot_id
snapshot_ref
observation_refs[]
state_context_refs[]
```

PEF continues to own scalar measurements, units, uncertainty, temporal/spatial support, evidence class, and provenance.

This permits:

```text
same lot_id
+ moisture observation at T1
+ moisture observation at T2
```

without pretending the earlier moisture value remained current.

## 8. Quantity basis is mandatory for conservation claims

No REGEN allocation or conservation theorem should use an unqualified generic `mass` quantity when different physical bases may coexist.

At minimum the protocol SHOULD distinguish bases such as:

```text
AsReceivedMass
DryMatterEquivalent
```

and MAY later add other explicitly defined compatible bases.

The critical rule is:

```text
100 kg as-received
!= 100 kg dry matter
```

A moisture-based conversion from one basis to another is a **Derived** evidence product and should carry PEF-2 lineage.

No conservation check may add/subtract quantities across incompatible bases without an explicit conversion lineage.

## 9. Quantity-claim semantics

A quantity claim SHOULD bind at least:

```text
lot_or_occurrence_ref
quantity_observation_ref
quantity_basis
claim_role
```

Possible `claim_role` semantics include:

```text
GrossOccurrence
PhysicallyAccessible
EcologicallyAllocable
RightsCleared
ProcessAdmissible
Reserved
Consumed
Residual
LossOrUnobserved
```

These roles are interpretations of evidence in context; they do not replace PEF measurement semantics.

## 10. Ecological retention is a hard eligibility constraint

Ecological retention MUST NOT be treated as merely another optimizer weight.

A future optimizer may compare alternatives only **after** the adopted ecological floor has been applied.

```text
optimizer preference
cannot override
adopted ecological retention constraint
```

If a required ecological-retention proposition is unresolved, the system may retain a candidate/unknown state, but it MUST NOT promote the quantity to strongly qualified `EcologicallyAllocable` merely by assuming the unknown retention requirement is zero.

## 11. Ecological allocation is context-bound

An `EcologicalAllocationAssessment` SHOULD bind:

```text
resource_context_ref
applicable_profile_or_policy_refs[]
source_quantity_refs[]
retention_claim_refs[]
competing_essential_use_refs[]
candidate_allocable_quantity_ref?
result
limitations[]
```

Suggested result state:

```text
Unresolved
Allocable
AllocableWithConstraints
NotAllocable
```

`Unresolved` is not `Allocable`.

## 12. Rights are plural, not one title boolean

REGEN core MUST NOT infer legitimate process authority from one generic `owner_ref`.

Relevant authority may come from several independent contexts, including where applicable:

- ownership/title;
- lease/use right;
- harvest/collection permit;
- commons/community allocation;
- indigenous/customary right;
- waste-handler/transport authorization;
- processing permission;
- transfer/sale authority;
- landholder consent;
- jurisdictional restrictions.

REGEN-011 SHOULD therefore preserve bounded external references and required-role semantics while leaving actual rights adjudication to the owning legal/governance systems.

```text
one title reference
!= all required rights resolved
```

## 13. Custody is a history, not a boolean

A current custodian reference can be useful but is weaker than a full custody history where chain-of-custody matters.

Future adapters may expose custody events such as:

```text
received
transferred
stored
released
split
merged
processed
rejected
```

REGEN core SHOULD reference these events rather than becoming a second supply-chain ledger.

Loss of custody continuity may reduce the admissible evidence state without deleting historical evidence.

## 14. Source-history hazard flags versus contaminant measurements

Source/treatment history and analytical contaminant evidence MUST remain separate.

Example:

```text
history: painted demolition timber
```

may trigger an adopted profile to require particular testing or reject a class outright, but it is not itself a quantitative contaminant measurement.

Conversely:

```text
one laboratory contaminant panel
```

does not prove that all relevant source-history hazards are absent.

The system therefore needs both:

```text
HistoryRiskEvidence
AnalyticalMeasurementEvidence
```

without collapsing one into the other.

## 15. Unknown provenance is a first-class state

Absence MUST NOT become a favorable claim.

```text
no treatment-history evidence
!= untreated

no contaminant evidence
!= uncontaminated

no custody history
!= uninterrupted custody

no competing-use evidence
!= no competing use
```

Fields that are unknown should be absent/explicitly unresolved according to their schema, not populated with optimistic defaults.

## 16. Process admission is three propositions, not one

REGEN-011 should freeze:

```text
FeedstockAssessment
!= ProcessInputReservation
!= ProcessExecution
```

A feedstock may be scientifically/profile-admissible but not reserved.

A reserved input may never be executed because the process is cancelled.

A process may execute only through a separate authority/safety boundary.

No `FeedstockAssessment` type should contain an actuator command.

## 17. Process-specific feedstock assessment

The preregistered assessment should bind an exact lot, process family/profile, evidence set, and result.

Recommended conceptual shape:

```text
FeedstockAssessment {
    assessment_id,
    lot_id,
    process_key,
    quality_profile_ref?,
    material_snapshot_refs[],
    rights_requirement_refs[],
    ecological_allocation_ref?,
    input_evidence_refs[],
    result,
    limitations[],
    assessment_evidence_ref
}
```

Result remains:

```text
Unresolved
Admissible
AdmissibleWithConstraints
NotAdmissible
```

An assessment is valid only relative to its exact evidence/profile context.

## 18. Reservation prevents double allocation but creates no property right

A later `ProcessInputReservation` exists to prevent one physical lot from being allocated at full quantity to incompatible concurrent uses.

Conceptually:

```text
ProcessInputReservation {
    reservation_id,
    lot_id,
    process_ref,
    quantity_ref,
    quantity_basis,
    reservation_state,
    authority_ref
}
```

This is an accounting/coordination primitive, not title.

```text
reservation
!= ownership
!= custody
!= environmental permission
!= execution permission
```

## 19. Allocation anti-double-spend theorem

For mutually exclusive uses under one compatible quantity basis:

```text
sum(active_reserved_quantities)
<= qualified_unreserved_capacity_before_reservation
```

or an explicit over-allocation failure MUST be produced.

A future system MUST NOT allow the same 1 tonne lot to simultaneously satisfy:

- 1 tonne compost input;
- 1 tonne pyrolysis input;
- 1 tonne animal-bedding allocation;

unless the allocations refer to distinct physical sub-lots or otherwise compatible/nonexclusive claims.

## 20. Split/merge conservation

A lot transformation event SHOULD permit explicit accounted residual/loss rather than assuming perfect conservation.

For a compatible basis:

```text
sum(input quantity)
= sum(output quantity)
+ explicit measured/estimated loss
+ explicit unresolved residual
```

where each non-observed term is assigned the appropriate evidence class/lineage.

The protocol MUST NOT force:

```text
loss = 0
```

merely to close an equation.

## 21. Merge/blend quality inheritance is fail-closed

A merged or blended child lot does not automatically inherit the strongest parent qualification.

A safe default theorem is:

```text
qualified A
+ unknown B
!= qualified blend
```

and:

```text
admissible A
+ not-admissible B
!= admissible blend
```

unless an exact downstream profile/assessment explicitly establishes the child state.

## 22. Alternative-use evidence stays plural

REGEN MUST NOT embed a universal hierarchy such as:

```text
pyrolysis > compost > mulch
```

or reduce alternatives to one hidden utility score.

An `AlternativeUseAssessment` may later expose plural dimensions including:

- ecological retention;
- soil nutrient return;
- service value;
- food/feed value;
- material substitution;
- energy/heat service;
- transport burden;
- cost;
- resilience;
- emissions/pollution;
- uncertainty.

Symthaea may later produce Pareto comparisons, but a recommendation remains recommendation-only.

## 23. Manufacturing bridge is a projection, not evidence promotion

A future adapter may emit something like:

```text
QualifiedFeedstockCapacityProjection {
    biomass_lot_id,
    manufacturing_part_id,
    admissible_quantity_ref,
    quantity_basis,
    assessment_ref,
    projection_time_context
}
```

Manufacturing can then consume that projection for planning.

The adapter MUST remain one-way in evidence strength:

```text
REGEN qualification -> Manufacturing planning
```

not:

```text
Manufacturing planning -> REGEN qualification
```

## 24. Marketplace bridge is advertisement plus evidence references

A future listing may carry:

```text
biomass_lot_id
seller_offer_quantity
supporting_assessment_refs[]
```

but the listing contract MUST preserve:

```text
seller offer quantity
!= independently measured lot quantity
```

and:

```text
seller claims ownership
!= ownership externally established
```

Qualification/state changes should invalidate or stale the listing's supporting claims rather than rewrite the underlying historical evidence.

## 25. PEF class admission mirrors REGEN-010

Where REGEN-011 resolves PEF evidence, it should adopt the same anti-laundering boundary preregistered for REGEN-010:

- `Reported | Observed` may be accepted as validated raw `EnvironmentalObservation` when appropriate;
- `Derived | Inferred | Forecast | Scenario` require validated PEF-2 lineage / `LineagedObservation` where they are used as computed evidence products.

This particularly matters for:

- calculated dry mass;
- regional recoverability models;
- next-season residue forecasts;
- alternative-allocation scenarios.

A bare computed observation MUST NOT bypass its lineage merely because its scalar value looks plausible.

## 26. Exact phenomenon binding

Where a REGEN-011 role references a PEF observation, the binding SHOULD also commit to the expected exact PEF `phenomenon` string.

This prevents:

```text
correct observation ID reference
+ substituted phenomenon semantics
```

from being accepted accidentally.

No case folding or alias normalization should occur silently inside the dependency-light core.

## 27. Currentness is use-specific

A lot's moisture, contamination risk, custody, and quantity can change on different timescales.

There is no valid universal statement:

```text
biomass evidence is fresh for N days
```

The consuming process/adopted profile should define currentness requirements by evidence role.

A historically valid observation remains historical evidence even when it is no longer current enough for admission.

## 28. Availability is not one boolean

REGEN-011 MUST NOT expose a single canonical field such as:

```text
available = true
```

without a precisely bounded proposition.

Preferred named states/claims should answer *available for what?*:

```text
physically_present
physically_accessible
ecologically_allocable
rights_cleared
custody_controlled
process_admissible
unreserved
```

A UI may summarize these, but protocol authority remains in the separate underlying propositions.

## 29. No implicit resilience claim

Local biomass can strengthen resilience only when it actually improves a constrained system.

```text
local feedstock
!= resilient feedstock
```

A locally sourced stream that damages soil cover, habitat, watershed function, food/feed supply, or critical material uses may reduce resilience.

REGEN-011 therefore supplies evidence for later resilience analysis; it does not assign a resilience score itself.

## 30. Qualification and implementation ordering

Executable REGEN-011 should remain gated on the shared PEF/REGEN evidence waist needed to resolve its evidence references.

Recommended sequence:

```text
REGEN-009 convergence qualified
-> REGEN-010 evidence implementation qualified
-> common resolved-evidence conventions reusable
-> REGEN-011 executable biomass core
```

REGEN-011 remains a sibling domain conceptually; this ordering is an implementation-risk reduction, not a claim that soil semantics are logically required for biomass provenance.

The eventual Rust crate SHOULD follow REGEN-008 `ProductFrozen` dependency semantics where practical and record system-closure state separately.

## 31. First executable test theorem set

The initial code should prove at least:

1. occurrence is not constructible as a present lot without a lot-forming subject/event;
2. unknown source/history remains representable;
3. no-test does not become clean;
4. MRP `quantity_available` is not an accepted provenance input in the core contract;
5. Marketplace listing is not an accepted qualification input in the core contract;
6. wet/as-received and dry-matter bases cannot be conserved together without a Derived conversion lineage;
7. process admission does not create reservation;
8. reservation does not create execution authority;
9. active incompatible reservations cannot over-allocate one lot;
10. split/merge allows explicit loss/residual rather than assuming zero loss;
11. unknown/failed material in a blend prevents automatic child qualification;
12. computed PEF evidence without required lineage is rejected;
13. exact observation ID and expected phenomenon must both match;
14. stale evidence remains distinguishable from invalid historical evidence;
15. feedstock qualification cannot become output-product qualification.

## 32. Deliberate non-claims

This refinement establishes no:

- current biomass availability;
- sustainable harvest quantity;
- land/resource title;
- harvest/collection permission;
- indigenous/customary-right adjudication;
- chain-of-custody authenticity;
- feedstock cleanliness;
- process safety;
- pyrolysis/compost operating parameter;
- output-product quality;
- agronomic suitability;
- carbon removal or credit eligibility;
- economic optimality;
- resilience superiority;
- process execution authority;
- physical-actuation authority.

Its narrower theorem is:

> REGEN can represent the path from resource occurrence to process input without allowing ecological allocation, rights, physical-lot identity, measurement state, process admission, reservation, execution, or output qualification to impersonate one another.
