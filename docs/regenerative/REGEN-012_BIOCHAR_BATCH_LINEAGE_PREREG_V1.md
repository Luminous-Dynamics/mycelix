# REGEN-012 — Biochar Batch Lineage Preregistration v1

Status: preregistered design only. No pyrolysis-control, product-safety, agronomic, carbon-credit, market, regulatory, or physical-action authority.

## 1. Purpose

REGEN-012 defines the evidence boundary between one or more biomass input lots and a resulting biochar batch.

Its central theorem is:

```text
qualified biomass input
+ documented transformation
+ identified output batch
!= qualified biochar product
!= agronomic suitability
!= carbon removal
!= carbon credit
```

REGEN-012 exists to preserve transformation identity, mass/material accounting, process evidence, output identity, and provenance without turning Mycelix into a pyrolysis controller or hard-coding one production recipe.

## 2. Dependency boundary

REGEN-012 is downstream of several independent propositions:

```text
REGEN-002 subject identity
REGEN-010/PEF resolved-evidence conventions
REGEN-011 biomass lot/provenance semantics
REGEN-003 adopted quality-profile semantics where qualification is evaluated
```

These are not interchangeable prerequisites.

A future implementation may initially consume only the narrow contracts needed by its theorem, but it MUST NOT recreate their semantics locally.

## 3. Existing subject identity

REGEN-002 already defines `BiocharBatchId` and `BiomassLotId`.

REGEN-012 MUST reuse those identities rather than minting parallel batch identifiers.

A `BiocharBatchId` provides semantic identity only.

```text
BiocharBatchId exists
!= batch physically exists
!= batch quantity known
!= batch composition known
!= batch qualified
```

## 4. Process-run identity boundary

REGEN-002 v1 does not currently freeze a dedicated `PyrolysisRunId` in the subject-ID roster.

REGEN-012 MUST NOT silently widen that already-qualified identity grammar.

The first implementation SHOULD therefore use a bounded `process_run_ref` / external exact identity whose semantics are explicit, or introduce a future typed process-run identity through a separately reviewed identity-protocol revision.

```text
need process-run identity
!= permission to mutate REGEN-002 v1 grammar in place
```

## 5. Batch lineage shape

The first executable contract SHOULD remain conceptually close to:

```text
BiocharBatchLineage {
    schema_version,
    batch_id: BiocharBatchId,
    process_run_ref,
    facility_ref?,
    input_allocations[],
    process_evidence_refs[],
    output_quantity_refs[],
    co_product_refs[],
    loss_or_residual_refs[],
    custody_refs[],
    quality_assessment_refs[],
    created_at_context?
}
```

Most measured quantities and process observations SHOULD remain PEF observations rather than duplicated scalar/unit/uncertainty fields.

## 6. Input allocation is not merely an input reference

A process run must bind not only which biomass lot participated but how much of that lot was allocated/consumed on an explicit compatible quantity basis.

Conceptually:

```text
ProcessInputAllocation {
    biomass_lot_id,
    feedstock_assessment_ref?,
    reservation_ref?,
    consumed_quantity_ref,
    quantity_basis
}
```

This composes with REGEN-011's distinction:

```text
FeedstockAssessment
!= ProcessInputReservation
!= ProcessExecution
```

An input listed in a process record MUST NOT imply that the entire source lot was consumed.

## 7. Feedstock qualification does not flow through automatically

A biomass lot may have been admissible under a process-input profile.

That does not qualify the output.

```text
input admissible for pyrolysis
!= biochar output conforms to profile
```

The process may:

- concentrate contaminants;
- introduce contamination;
- incompletely transform material;
- create heterogeneous output;
- mix inputs with different evidence states;
- produce an output whose intended use requires different measurements.

Output qualification requires its own evidence and applicable profile.

## 8. No universal pyrolysis recipe

REGEN-012 MUST NOT freeze a universal temperature, residence time, heating rate, pressure, oxygen condition, moisture threshold, reactor type, or yield target as protocol truth.

Those may be represented as observed process evidence or requirements of an adopted exact process/product profile.

```text
record process condition
!= prescribe process condition
```

The protocol is evidence-bearing, not an operating manual.

## 9. Process evidence roles

A future process profile may reference observations for roles such as:

```text
process:temperature
process:duration
process:pressure
process:oxygen-context
process:feed-rate
process:input-moisture
process:energy-input
process:electrical-energy-input
process:thermal-energy-input
process:output-mass
process:char-yield
process:gas-output
process:liquid-output
process:useful-heat-output
process:emission-observation
```

These names are role identifiers only.

They do not define acceptable ranges or operating instructions.

## 10. PEF remains measurement authority

REGEN-012 SHOULD bind exact PEF observations rather than introduce a second process-measurement schema.

PEF remains authoritative for:

- value;
- unit;
- evidence class;
- uncertainty;
- temporal support;
- spatial support;
- external evidence;
- generic provenance.

REGEN adds transformation-specific interpretation/context only.

## 11. Raw versus computed evidence

REGEN-012 SHOULD inherit the anti-laundering boundary used by REGEN-010/011:

- `Reported | Observed` may be validated raw `EnvironmentalObservation` where appropriate;
- `Derived | Inferred | Forecast | Scenario` require validated PEF-2 lineage where used as computed products.

Examples:

```text
scale-recorded input mass       -> Observed
operator-reported batch note    -> Reported
calculated dry input mass       -> Derived + lineage
calculated char yield           -> Derived + lineage
estimated unmeasured gas output -> Inferred + lineage
future run prediction           -> Forecast + lineage
what-if process state           -> Scenario + lineage
```

## 12. Exact role/phenomenon binding

Each REGEN process-evidence binding SHOULD carry both:

- exact requested PEF observation ID;
- exact expected PEF `phenomenon` string.

This prevents right-ID/wrong-semantic substitution.

No silent aliasing, case folding, or phenomenon normalization belongs in the dependency-light core.

## 13. Mass-basis firewall

All mass-balance arithmetic must use compatible explicit bases.

At minimum REGEN-012 should understand the distinction between:

```text
AsReceivedMass
DryMatterEquivalent
```

A dry-matter conversion from measured wet mass + moisture is a `Derived` evidence product with lineage.

```text
wet input mass
cannot be directly compared to
dry output mass
```

without an explicit compatible conversion.

## 14. Material-balance theorem

For an explicitly chosen compatible basis, a process run SHOULD be able to state:

```text
sum(input material)
=
identified char output
+ identified non-char products
+ measured/estimated process loss
+ unresolved residual
```

The equation MUST NOT force unresolved material to zero merely to close accounting.

```text
unmeasured fraction
!= zero
```

An `UnresolvedResidual` is preferable to manufactured precision.

## 15. Co-products stay explicit

Pyrolysis may produce outputs other than char.

REGEN-012 MUST NOT model every non-char fraction as waste or pretend it did not exist.

Conceptual dispositions may include:

```text
gas stream
condensable/liquid stream
recoverable heat
captured material
emitted material
unknown/unresolved residual
```

Downstream use/disposition may be referenced but remains owned by the appropriate Energy, Climate, waste-management, or material-flow domain.

## 16. Useful heat is a separate service proposition

Observed/recovered useful heat may later feed the Energy/resource-quality architecture.

But:

```text
pyrolysis occurred
!= useful heat was recovered
```

and:

```text
heat generated
!= heat delivered to a compatible load
```

A future bridge must bind heat quantity/quality, time, delivery path, and compatible service separately.

## 17. Output batch formation

The resulting `BiocharBatchId` should be bound to an explicit output-batch formation event or process-run completion context.

The output batch may later split into sub-lots or merge/blend with other batches.

Those operations require explicit lineage rather than mutating history under an unchanged aggregate identity.

## 18. Batch state changes over time

A batch identity is not a timeless state snapshot.

Properties may change through:

- storage;
- moisture uptake/loss;
- handling;
- contamination;
- grinding/sizing;
- mixing;
- transport;
- aging;
- later activation/charging/co-composting.

New state evidence should be attached without rewriting historical process evidence.

## 19. Raw biochar and amended/charged material are distinct subjects

REGEN-012 specifically concerns a biochar output batch.

Later nutrient charging, composting, co-composting, inoculation, blending, or other amendment creates a new transformation proposition and may require a distinct subject/batch identity.

```text
raw biochar batch
!= co-composted amendment batch
```

REGEN-014 owns the combined-amendment boundary.

## 20. Quality assessment is profile-relative

A `BiocharQualityAssessment` SHOULD bind:

```text
batch_id
quality_profile_ref
required_evidence_refs[]
result
limitations[]
assessment_evidence_ref
```

Suggested result semantics:

```text
Unresolved
Conforms
ConformsWithConstraints
DoesNotConform
```

`Unresolved` is first-class.

A result proves conformance only to the exact referenced profile/evidence context.

## 21. Product quality is not agronomic suitability

This firewall is mandatory:

```text
biochar conforms to product profile
!= suitable for soil X
!= suitable for crop Y
!= beneficial at application rate Z
```

REGEN-017 owns later context-specific agronomic suitability semantics.

REGEN-012 MUST contain no universal application-rate recommendation.

## 22. Contamination firewall

Measured contaminants remain observations until interpreted under an applicable profile.

```text
contaminant measured
!= universally safe
!= universally unsafe
```

Likewise:

```text
no contaminant measurement
!= contaminant absent
```

A process profile may require tests based on feedstock history or intended product use.

## 23. Input-history propagation is evidence, not automatic verdict

REGEN-011 may flag feedstock history such as painted/coated wood, demolition origin, mixed-waste contact, pesticide exposure, or other risk context.

REGEN-012 must preserve those references through the process lineage where relevant.

But it should not itself assert a universal transformation/removal factor for those hazards.

```text
hazardous-source history
+ process run
!= hazard proven removed
```

## 24. Custody remains separate from transformation truth

Process-run evidence, batch identity, and custody are distinct.

A custody trail can show who handled a batch but does not prove the process conditions or batch composition.

A process record can show a declared transformation but does not prove current custody.

REGEN should reference custody artifacts rather than creating a parallel supply-chain ledger.

## 25. Facility reference is contextual, not certification

A `facility_ref` may bind where/which facility produced the batch.

Presence of that reference does not prove:

- facility ownership;
- operator competence;
- regulatory approval;
- emissions compliance;
- calibration;
- process safety;
- equipment condition.

Those require separate evidence/authority domains.

## 26. Software/model evidence does not become process truth

Symthaea may later predict yield, quality, energy recovery, or emissions.

Those products remain `Forecast`, `Inferred`, or `Scenario` evidence as appropriate.

```text
predicted process condition
!= observed process condition
```

and:

```text
model predicts qualifying batch
!= batch qualifies
```

## 27. Carbon firewall

A biochar batch lineage does not establish carbon removal.

```text
biomass carbon input
!= stable carbon output
!= counterfactual additionality
!= durability
!= leakage accounting
!= net life-cycle removal
!= carbon credit
```

REGEN-070+ / Climate remain the owners of carbon-project and credit authority.

No REGEN-012 result may auto-mint carbon units.

## 28. Yield firewall

A high char yield is not a universal goodness metric.

```text
higher char yield
!= better biochar
!= lower emissions
!= better soil outcome
!= better economics
!= better resilience
```

REGEN-012 records/derives yield when evidence supports it; it does not optimize yield as a universal objective.

## 29. Energy firewall

Energy use and recovered heat remain plural outcomes.

```text
net process energy
!= whole-system energy benefit
```

A future settlement-metabolism model may compare:

- process energy demand;
- recovered thermal service;
- avoided disposal burden;
- transport energy;
- alternative uses;
- other coupled services.

REGEN-012 does not collapse these into one score.

## 30. Emissions evidence boundary

Where emissions are measured or modeled, they SHOULD be referenced through canonical evidence semantics.

REGEN-012 does not define universal environmental-compliance limits.

```text
emission observation
!= regulatory compliance verdict
```

Applicable legal/quality profiles remain downstream.

## 31. Split/merge output lineage

When one output batch is split:

```text
parent BiocharBatchId
-> split event
-> child batch IDs
```

When batches are merged/blended:

```text
parent batch IDs
-> merge/blend event
-> new child batch ID
```

The child does not automatically inherit the strongest parent qualification.

```text
conforming A
+ unresolved B
!= conforming blend
```

## 32. Conservation after split/merge

On a compatible quantity basis:

```text
sum(parent quantity)
= sum(child quantity)
+ explicit loss/residual
```

Unknown handling loss stays explicit.

No zero-loss assumption is allowed merely to satisfy accounting.

## 33. Serialization and validation boundary

The future dependency-light crate SHOULD distinguish:

```text
well-formed lineage bytes
!= resolved/validated evidence lineage
!= quality-conforming batch
```

Structural validation may check IDs, bounded refs, ordering, duplicate prevention, and shape.

Resolved validation should require explicit caller-provided evidence/profile resolution and make no hidden network/DHT/database calls.

## 34. Proposed executable crate

Preferred later location:

```text
crates/mycelix-regenerative-biochar
```

Likely direct dependencies:

```text
mycelix-regenerative-core
mycelix-core-types
mycelix-regenerative-biomass
optional mycelix-regenerative-quality
optional serde
```

It SHOULD NOT initially depend directly on:

- Holochain runtime;
- Symthaea;
- Climate;
- Finance;
- Marketplace;
- Energy runtime;
- device/control libraries;
- databases/network clients.

Adapters belong downstream.

## 35. Implementation gate

Executable REGEN-012 SHOULD wait until the evidence conventions it depends upon are qualified enough to consume.

Recommended implementation order:

```text
REGEN-009 convergence qualification
-> REGEN-010 resolved-evidence waist
-> REGEN-011 biomass core
-> REGEN-012 biochar process/batch core
```

REGEN-003 quality semantics may converge before quality-assessment code is enabled.

Scientific/release qualification SHOULD use REGEN-008 `ProductFrozen` dependency semantics where practical and record system-closure state separately.

## 36. First executable test theorem set

The first implementation SHOULD prove at least:

1. exact input lot identity is preserved;
2. consumed input quantity is not silently the whole lot;
3. incompatible mass bases cannot be reconciled directly;
4. dry-matter conversion requires Derived lineage;
5. unmeasured residual cannot silently become zero;
6. co-product streams remain explicit;
7. process evidence cannot be relabeled across PEF evidence classes;
8. computed process evidence requires lineage;
9. expected observation phenomenon must match exactly;
10. input feedstock qualification does not qualify output;
11. product-profile conformance does not imply agronomic suitability;
12. batch lineage does not create carbon-removal authority;
13. raw biochar batch cannot silently become co-composted amendment;
14. merge with unresolved/nonconforming input does not inherit strongest-parent status;
15. facility reference does not become regulatory/process-safety authority;
16. no control/actuation type is reachable from the dependency-light contract.

## 37. Deliberate non-claims

REGEN-012 establishes no:

- pyrolysis operating instruction;
- reactor safety;
- emissions compliance;
- facility certification;
- operator competence;
- batch cleanliness;
- product-profile conformance unless separately assessed;
- agronomic suitability;
- application rate;
- crop response;
- soil-health outcome;
- carbon removal;
- carbon-credit eligibility;
- economic optimality;
- resilience superiority;
- process execution authority;
- physical actuation.

Its narrower theorem is:

> Mycelix can preserve the exact evidence lineage from bounded biomass input allocations through one declared transformation into an identified biochar output batch, while keeping material accounting, process evidence, product qualification, agronomy, carbon accounting, and physical authority separate.
