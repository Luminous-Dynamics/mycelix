# REGEN-011 — Biomass Feedstock Provenance Preregistration v1

Status: preregistered design only. No harvest, procurement, processing, agronomic, carbon-credit, or physical-action authority.

## 1. Purpose

REGEN-011 defines the evidence boundary between a biomass resource and a candidate process feedstock.

Its central theorem is:

```text
resource exists
!= sustainably removable
!= lawfully/legitimately removable
!= harvested/generated lot
!= controlled/custodied lot
!= composition known
!= uncontaminated
!= process-suitable
!= output-product-qualified
```

The protocol MUST preserve these as separate propositions.

REGEN-011 is not a forestry model, waste-management engine, inventory ledger, marketplace, procurement system, or pyrolysis controller. It provides a small evidence-bearing waist that downstream systems can resolve and assess.

## 2. Existing-system boundary

REGEN-002 already defines `BiomassLotId`; REGEN-011 reuses it rather than inventing another lot identity.

Existing Mycelix Manufacturing already models BOM demand, planned material availability/shortage, work orders, machines, and routing. Those planning quantities are useful downstream, but they do not currently establish biomass-lot provenance, ecological removability, contamination state, custody, or process-specific admissibility.

Therefore:

```text
REGEN qualified biomass lot
    -> optional manufacturing / marketplace / logistics adapters
```

not:

```text
manufacturing quantity_available
    -> automatically qualified biomass feedstock
```

REGEN-011 MAY carry bounded opaque references to rights, custody, logistics, procurement, marketplace, or governance artifacts. Presence of a reference is not validation of the referenced claim.

## 3. Five independent gates

A candidate biomass lot SHOULD be evaluated through five independent dimensions:

1. **Occurrence / generation** — what material physically exists or is claimed to exist?
2. **Ecological allocation** — what fraction may be removed without violating adopted ecological floors?
3. **Rights / custody** — who is entitled to remove, transfer, possess, or allocate it?
4. **Material evidence** — what is actually known about composition, treatment history, moisture, contamination, and source?
5. **Process admissibility** — is this exact lot admissible for this exact intended process under an adopted profile?

Passing one gate MUST NOT imply another.

## 4. Resource occurrence is not a lot

Standing biomass, crop residues still in a field, municipal tree trimmings not yet collected, and future processing residues are resource occurrences or forecasts, not automatically material lots.

A `BiomassLotId` SHOULD identify a bounded physical aggregation after a lot-forming event such as harvest, collection, segregation, receipt, or generation by a defined process.

```text
forest inventory estimate
!= BiomassLot

forecast crop residue
!= BiomassLot
```

A future occurrence/availability model may reference PEF observations and Symthaea projections, but REGEN-011 MUST not convert forecast quantities into present inventory.

## 5. Candidate lot profile

The first executable contract SHOULD remain close to:

```text
BiomassLotProfile {
    schema_version,
    lot_id: BiomassLotId,
    source_class,
    origin_ref?,
    lot_formed_at?,
    quantity_claims[],
    material_evidence_refs[],
    treatment_history_refs[],
    custody_refs[],
    rights_refs[],
    ecological_allocation_refs[],
    competing_use_refs[],
    qualification_refs[]
}
```

Most scientific measurements SHOULD remain canonical PEF observations referenced by ID rather than copied into this structure.

## 6. Source class is descriptive, not permission

A bounded source classification MAY distinguish broad origins such as:

```text
ForestryResidue
AgriculturalResidue
FoodProcessingResidue
UrbanTreeResidue
CleanWoodProcessingResidue
DedicatedCrop
ManureDerived
OtherBiogenic
Unknown
```

This classification is descriptive only.

```text
ForestryResidue
!= sustainably harvested

AgriculturalResidue
!= surplus to soil-cover needs

UrbanTreeResidue
!= chemically untreated

CleanWoodProcessingResidue
!= laboratory-proven clean

DedicatedCrop
!= ecologically desirable
```

`Unknown` MUST remain representable.

## 7. “Waste” is not an intrinsic material property

REGEN-011 SHOULD NOT have a universal boolean such as:

```text
is_waste = true
```

and then infer priority or safety from it.

Waste/residue status depends on holder, process, jurisdiction, current use, and alternatives.

A material called waste may still be:

- needed as soil cover;
- used as animal bedding/feed;
- habitat/deadwood;
- a source of nutrients returned directly to soil;
- contaminated;
- economically valuable elsewhere;
- legally regulated.

Thus:

```text
called waste
!= available for extraction
!= low-opportunity-cost
!= safe feedstock
```

## 8. Ecological allocation ledger

REGEN-011 SHOULD represent ecological allocation as explicit referenced claims rather than one sustainability boolean.

Conceptually:

```text
EcologicalAllocation {
    resource_context_ref,
    gross_available_claim,
    ecological_retention_claims[],
    soil_cover_retention_claims[],
    habitat_retention_claims[],
    erosion_constraints[],
    nutrient_return_constraints[],
    water/watershed_constraints[],
    legal_or_governance_reserves[],
    candidate_recoverable_claim?
}
```

The general accounting relation is:

```text
gross physically available
- required ecological retention
- required soil-cover / nutrient-return fraction
- habitat/deadwood obligations
- legal/governance reserves
- already committed essential uses
- inaccessible/unqualified material
= candidate recoverable quantity
```

Every term remains evidence-bearing and may be uncertain.

`candidate recoverable` is NOT process suitability.

## 9. Competing uses remain explicit

REGEN SHOULD not maximize pyrolysis feedstock by assuming every recoverable residue is best used as biochar feedstock.

A lot/resource may have competing uses such as:

```text
soil return
animal feed/bedding
mulch
compost
anaerobic digestion
material reuse
construction/fiber product
fuel/heat
pyrolysis
habitat retention
other local essential use
```

A future optimizer may compare these on a Pareto frontier, but REGEN-011 MUST retain the underlying alternatives and evidence.

## 10. Quantity semantics

One biomass quantity is not enough.

At minimum, downstream profiles must be able to distinguish claims such as:

```text
as-received wet mass
dry mass
moisture fraction
candidate recoverable mass
qualified process-admissible mass
```

REGEN-011 SHOULD avoid embedding a custom generic measurement type. Measurements such as mass and moisture SHOULD normally be PEF observations where evidence/class/provenance matter.

```text
1 tonne wet biomass
!= 1 tonne dry feedstock
```

No silent dry/wet conversion is allowed.

## 11. Lot formation and split/merge

Physical lots may split or merge.

REGEN-011 SHOULD preregister lineage semantics such that:

```text
parent lot(s)
-> physical split / merge / segregation / blending event
-> child lot(s)
```

is explicit.

A child lot MUST NOT inherit stronger quality/contamination claims merely because a parent possessed them; applicability must be established by the relevant process/profile.

Blending an unknown/failed lot into a qualified lot MUST NOT silently yield a qualified blend.

## 12. Custody is not quality

A custody reference can support claims about who handled or possessed a lot and when.

It does not prove:

```text
material identity
composition
absence of contamination
ecological sustainability
legal title
process suitability
```

unless the referenced system/evidence explicitly establishes those propositions.

REGEN-011 therefore uses `custody_refs` as references rather than inventing a universal custody authority in the dependency-light crate.

## 13. Rights are not custody

Possessing a biomass lot is distinct from having legitimate authority to harvest, sell, transfer, process, or apply it.

```text
physical custody
!= ownership
!= harvest right
!= transfer right
!= processing permission
```

`rights_refs` remain separate from `custody_refs`.

## 14. Material and treatment history

Important history may include:

- painted/coated wood;
- preservative treatment;
- glue/resin/engineered wood;
- demolition origin;
- sewage/sludge contact;
- pesticide exposure;
- saltwater exposure;
- industrial contamination;
- mixed-waste contact;
- prior thermal/chemical processing.

Unknown history MUST remain explicit.

No default `clean=true` is allowed.

## 15. Contamination firewall

REGEN-011 binds contamination evidence but does not define universal safe thresholds.

```text
contaminant measured
!= safe
!= unsafe
```

Decision thresholds belong to adopted quality/legal/process profiles, including REGEN-016 where soil-use contamination semantics are planned.

Likewise:

```text
no contaminant test supplied
!= contaminant absent
```

## 16. Process-specific admissibility

The same lot may be acceptable for one process and unacceptable for another.

Conceptual process keys may include:

```text
process:pyrolysis
process:compost
process:co-compost
process:anaerobic-digestion
process:mulch
process:direct-soil-return
process:combustion
process:material-reuse
```

A `FeedstockAssessment` SHOULD bind:

```text
lot_id
process_key
quality_profile_ref
input_evidence_refs[]
result
limitations[]
assessment_evidence_ref
```

with a result such as:

```text
Unresolved
Admissible
AdmissibleWithConstraints
NotAdmissible
```

`Unresolved` is first-class and MUST NOT be treated as `Admissible`.

## 17. Assessment is profile-relative

```text
lot admissible under profile A
!= lot admissible under profile B
```

An assessment MUST identify the exact adopted quality/process profile revision it used where such a profile exists.

This composes with REGEN-003 rather than hard-coding one biochar standard into the biomass schema.

## 18. Feedstock qualification is not product qualification

This boundary is non-negotiable:

```text
qualified pyrolysis feedstock
!= qualified biochar
```

The process may introduce, concentrate, transform, or fail to remove hazards.

A resulting `BiocharBatchId` requires its own process/batch/output evidence.

Similarly:

```text
qualified compost input
!= mature/safe compost output
```

## 19. Carbon firewall

Biogenic carbon or dry-matter evidence is not carbon-removal evidence.

```text
biomass carbon content
!= additionality
!= counterfactual fate
!= durable storage
!= leakage assessment
!= permanence
!= carbon credit
```

Climate authority remains downstream in REGEN-070+.

## 20. Locality firewall

A local lot may reduce transport dependency, but locality alone conveys no sustainability preference.

```text
local
!= ecologically benign
!= low carbon
!= resilient
!= cheaper
!= safer
```

A local extraction that depletes soil organic matter or habitat may reduce rather than increase resilience.

## 21. Evidence classes and provenance

Material measurements referenced by REGEN-011 SHOULD inherit PEF semantics.

Raw reported/observed facts remain distinct from computed/forecast/scenario quantities. Where REGEN-011 consumes computed PEF evidence, it SHOULD follow the same REGEN-010 rule requiring validated PEF-2 lineage rather than accepting an unlineaged computed observation.

Example:

```text
weighed lot mass             -> Observed
owner-reported source mass   -> Reported
calculated dry mass          -> Derived + lineage
regional residue estimate    -> Inferred + lineage
next-season residue forecast -> Forecast + lineage
allocation what-if           -> Scenario + lineage
```

## 22. Temporal/currentness semantics

Biomass state changes with time through drying, decomposition, mixing, contamination, movement, and processing.

REGEN-011 MUST NOT define one universal freshness window.

A consuming process profile may require fresh moisture/contamination/custody evidence.

```text
valid old observation
!= current lot state
```

## 23. Location semantics

Origin location, current custody location, and intended processing location are distinct.

REGEN-011 SHOULD NOT collapse them into one `location` field.

A future logistics adapter may bind movement events explicitly.

## 24. Manufacturing bridge

Existing Manufacturing `MrpResult`, `PlannedOrder`, and `MaterialShortage` reason about required and available quantities for `part_id` values.

A future narrow adapter MAY project a qualified/admissible biomass quantity into manufacturing planning.

The direction is one-way with respect to evidence strength:

```text
qualified REGEN feedstock assessment
-> manufacturing planning availability
```

Manufacturing's `quantity_available` MUST NOT back-propagate into:

```text
ecological allocation
material quality
contamination safety
feedstock admissibility
```

without independent evidence.

## 25. Marketplace bridge

A future marketplace listing may advertise or offer a `BiomassLotId`, but:

```text
listed
!= exists
!= owned by seller
!= qualified
!= available now
```

Marketplace semantics must consume explicit REGEN qualification/evidence references rather than minting them through listing publication.

## 26. Resolver boundary

The dependency-light REGEN contract MUST make no hidden network/DHT/database calls.

Validation should accept explicit resolved evidence or caller-provided resolver interfaces.

This mirrors REGEN-010's structural-vs-resolved validation rule.

## 27. Structural validation

The future executable profile SHOULD at minimum validate:

- supported schema version;
- canonical `BiomassLotId`;
- bounded source classification/custom keys;
- bounded reference counts and text lengths;
- canonical ordering where wire identity requires it;
- no duplicate evidence/reference identities where duplication could launder meaning;
- finite/nonnegative numeric protocol bounds if any protocol-local integers are used.

Structural validity proves no real-world truth.

## 28. Resolved validation

Resolved validation SHOULD additionally establish, as requested by a consuming profile:

- referenced observations resolve exactly;
- nested PEF/lineage objects validate;
- evidence class is acceptable;
- expected phenomenon matches exactly;
- required rights/custody/allocation/profile artifacts resolve;
- process assessment references the intended lot/profile/process;
- no reference substitution occurred.

## 29. Adversarial/mutation campaign

The first executable campaign SHOULD attempt at least:

1. unknown source class remains representable;
2. residue classification does not grant process admission;
3. candidate recoverable quantity does not grant feedstock qualification;
4. custody does not imply rights;
5. rights do not imply contamination safety;
6. no-test does not become clean;
7. pyrolysis admission does not imply compost admission;
8. input qualification does not imply output qualification;
9. wet mass cannot silently become dry mass;
10. bare computed PEF quantity without lineage is rejected where lineage is required;
11. stale evidence remains distinguishable from current state;
12. split/merge lineage cannot duplicate mass without an explicit reconciliation failure;
13. a failed/unknown parent lot cannot be blended into an automatically qualified child;
14. marketplace listing cannot create qualification;
15. manufacturing `quantity_available` cannot create ecological or safety evidence.

## 30. Conservation and anti-double-counting

Future split/merge accounting SHOULD support an explicit conservation check over compatible mass bases.

It MUST NOT compare wet and dry mass as though directly identical.

Likewise, one physical lot MUST NOT be simultaneously allocated at full quantity to several mutually exclusive uses without an explicit over-allocation failure.

This will later feed Symthaea's material-flow and settlement-metabolism models.

## 31. Symthaea boundary

Symthaea may later rank competing biomass allocations or predict process outcomes.

Its result remains recommendation/model evidence only.

```text
optimizer selects pyrolysis
!= ecological permission
!= lot qualification
!= process authority
```

## 32. Physical-action boundary

REGEN-011 creates no authority to:

- harvest biomass;
- fell trees;
- remove crop residues;
- transport controlled material;
- start a chipper/dryer/pyrolyzer;
- dose compost/biochar;
- dispose of rejected material.

Any later automated actuation belongs behind the separate consequential-physical-action safety/authority architecture.

## 33. Implementation gate

REGEN-011 is deliberately preregistration-only.

The first executable implementation SHOULD wait for the shared evidence waist needed by its PEF references to be qualified, and SHOULD adopt REGEN-008 `ProductFrozen` dependency semantics where practical.

It should not be made a dependency of REGEN-010 soil evidence; soil observations and biomass provenance are sibling evidence domains that can converge later in field-trial/process lineages.

## 34. Proposed first executable crate

Preferred later location:

```text
crates/mycelix-regenerative-biomass
```

Likely direct dependencies:

```text
mycelix-regenerative-core
mycelix-core-types
optional mycelix-regenerative-quality
optional serde
```

It SHOULD NOT initially depend directly on Holochain, Marketplace, Manufacturing, Symthaea, Climate, Finance, databases, network clients, or physical-control runtimes.

Adapters belong downstream.

## 35. Deliberate non-claims

REGEN-011 establishes no biomass availability, legal title, harvest right, ecological sustainability, feedstock cleanliness, process safety, product quality, agronomic benefit, carbon removal, climate credit, economic optimality, or physical-action authority.

Its theorem is narrower:

> a regenerative system must be able to trace a bounded biomass lot and keep ecological availability, rights/custody, material evidence, competing uses, and process-specific admissibility separate all the way to the process boundary.
