# MYC-INT-018E — Production-Environment Reference Profile

Status: architecture / adapter profile only. Tracks #3229. Child of MYC-INT-018D / PR #3228.

Observed design date: 2026-09-27.

## 1. Purpose

Define the smallest interoperability-level reference that can bind a food-production subject to its actual cultivation environment without forcing every production system through the current soil-first `food-production::Plot` model.

The profile exists because the current source model legitimately reflects its original soil/community-garden scope:

```text
Plot {
  soil_type: SoilType,
  plot_type: PlotType,
  ...
}

Crop {
  plot_hash: ActionHash,
  ...
}
```

but the productive-node showcase must also represent hydroponic and future cultivation systems truthfully.

The adapter rule is:

```text
schema mismatch
!= permission to invent source facts
```

## 2. Core theorem

A production-environment reference provides identity and typed relationship only.

```text
ProductionEnvironmentRef
!= environment observation
!= suitability assessment
!= certification
!= operating policy
!= actuation authority
```

Likewise:

```text
same crop species
!= same production environment

same physical greenhouse shell
!= same grow-system state

same runtime object hash
!= same cross-domain semantic identity
```

## 3. Why not extend `SoilType`

Do not add variants such as:

```text
SoilType::Hydroponic
SoilType::Aeroponic
SoilType::None
```

That would preserve the wrong abstraction by encoding non-soil production modes as soil classifications.

Similarly, do not treat:

```text
PlotType::Greenhouse
```

as sufficient to identify hydroponic production. A greenhouse is an enclosure/profile; its production substrate may be soil, hydroponic, aeroponic, aquaponic, mixed, experimental, or something else.

## 4. Candidate reference shape

Prefer existing namespace-safe `SchemaRef` / `SemanticRef` primitives from the interoperability line.

Conceptually:

```text
ProductionEnvironmentRefV1 {
    environment: SemanticRef,
    environment_profile: SchemaRef,
    source_owner: SchemaRef,
}
```

Where:

- `environment` identifies the exact production environment object;
- `environment_profile` identifies the semantics under which that object is interpreted;
- `source_owner` identifies the source-domain schema/owner responsible for the native object.

No generic `verified`, `safe`, `certified`, `active`, or `authorized` bit belongs in this identity type.

## 5. Production-subject binding

A separate relation binds a crop/batch/production subject to an environment:

```text
ProductionSubjectBindingV1 {
    production_subject: SemanticRef,
    environment: ProductionEnvironmentRefV1,
    relation_profile: SchemaRef,
    translation_receipt: Option<SemanticRef>,
}
```

The relation must not imply ownership transfer.

```text
binding to environment
!= environment owns crop semantics
!= crop owns environment state
```

## 6. Environment profiles

The reference deliberately uses a versioned schema/profile instead of a closed technology enum.

Examples of profile identities may include conceptually:

```text
mycelix.commons.food-production.plot/v1
mycelix.showcase.hydroponic-system/v1
mycelix.showcase.soil-reference-bed/v1
future.aquaponic-system/v1
future.aeroponic-system/v1
```

The exact namespace registry should come from the existing interoperability namespace owner rather than be invented here.

This permits future production systems without changing the core reference shape.

## 7. Legacy soil compatibility

For an existing native soil crop:

```text
food-production::Crop
  -> plot_hash
  -> food-production::Plot
```

an adapter may construct a semantic projection:

```text
ProductionEnvironmentRefV1 {
    environment = SemanticRef(native Plot identity),
    environment_profile = food-production plot schema/profile,
    source_owner = food-production source schema,
}
```

The adapter must preserve the fact that the original source used a Holochain `ActionHash` and must not pretend that the cross-domain semantic reference was present in the historical source record.

If a translation receipt is required by the active interoperability profile, it must be carried.

## 8. Hydroponic compatibility

The current food-production `Crop` type cannot represent a hydroponic crop truthfully without a `plot_hash`, and the current `Plot` requires `SoilType`.

Therefore the first productive-node showcase must **not** do this:

```text
hydroponic bay
-> fake Plot { soil_type = Mixed }
-> native Crop
```

Instead, until a source-domain V2 exists, hydroponic production subjects remain explicitly typed showcase/adapter records with their own source identity.

Conceptually:

```text
HydroponicProductionBatchFixtureV1 {
    id,
    species_or_crop_profile,
    environment: ProductionEnvironmentRefV1,
    started_at,
    lifecycle_state,
    source_profile,
}
```

This is a fixture/profile concept only, not a proposed universal core type.

The important invariant is truthful provenance:

```text
adapter-owned hydroponic fixture
!= native food-production::Crop
```

## 9. Source-domain V2 trigger

Do not modify `food-production::Crop` merely because the showcase needs hydroponics.

A source-domain V2 becomes justified only if repeated use cases show that food production itself needs an environment-neutral production subject.

A future candidate might separate:

```text
ProductionSite / Environment
ProductionBatch / CropCycle
Output / Harvest
Input / Treatment
```

but that belongs to an explicit source-domain migration tranche with compatibility rules for existing `Plot`, `Crop`, `YieldRecord`, and `ResourceInput` history.

## 10. Currentness

Reference resolution is not currentness.

```text
SemanticRef resolves
!= source object is current
!= environment is operational
!= environment is suitable for this crop
```

Currentness must come from the existing source/currentness architecture.

The production-environment profile may carry or reference currentness evidence, but this identity layer must not define a second latest-state mechanism.

## 11. Provenance

The profile must preserve:

- native source owner;
- native source object identity;
- source schema/profile;
- adapter/translation profile when transformed;
- translation receipt where required;
- generation/revision semantics where the source supports them.

A later consumer must be able to distinguish:

```text
native source field
from
adapter-added classification
from
derived analytical conclusion
```

## 12. Runtime independence

Holochain is an implementation/runtime profile, not the semantic identity layer.

Therefore:

```text
ActionHash
!= SemanticRef
```

An adapter may encode/reference a native Holochain identity inside the semantic object identity profile, but replacing the runtime must not change the intended production-environment semantics.

Exported productive-node history must preserve enough information to reconstruct the source identity after runtime replacement.

## 13. Environment identity versus physical topology

An environment reference should identify one semantic production environment, not every physical component attached to it.

For example:

```text
HydroponicSystem
  references/contains
    reservoir
    pump loop
    grow channels
    sensors
    dosing system
```

but:

```text
HydroponicSystem identity
!= reservoir identity
!= pump identity
!= sensor identity
```

Component topology should remain OAD/design or operational-domain data as appropriate.

## 14. Environment identity versus measurements

Measurements always bind both a subject and a measurement profile.

Examples:

```text
reservoir pH observation
-> subject = reservoir / solution context

ambient humidity observation
-> subject = greenhouse climate zone

soil moisture observation
-> subject = soil/reference bed
```

The mere existence of a `ProductionEnvironmentRefV1` does not make measurements interchangeable.

## 15. Soil / hydroponic comparison theorem

A shared comparison envelope may compare quantities only where the measurement semantics genuinely align.

Comparable candidates:

- water input over a declared interval;
- electrical energy over a declared interval;
- accepted output mass;
- rejected biomass;
- labor duration;
- crop-cycle duration;
- downtime.

Non-equivalent examples:

```text
soil moisture
!= reservoir volume

soil nutrient assay
!= hydroponic EC

soil pH sample
!= nutrient-solution pH
```

A higher-level analysis may compare their implications, but must not claim identity between the source measurements.

## 16. Relation profiles

The binding relation itself should be versioned.

Candidate relation semantics include:

```text
CultivatedIn
ProducedWithin
ReferenceComparisonFor
```

Do not use a free-form relation string if an existing relationship/profile registry can express the semantics.

A comparison relation does not imply common source ownership or shared authority.

## 17. Translation receipts

Whenever a native source object is projected into the production-environment interoperability profile, preserve whether the transformation was:

- identity-like wrapper;
- structural projection;
- lossy translation;
- derived interpretation;
- not losslessly representable.

Reuse the interoperability translation-receipt semantics rather than defining a local translation enum if that owner is available.

## 18. Unknown/future profiles

A consumer receiving an unknown environment profile must not silently guess its semantics.

Depending on the active interface policy it should:

- preserve the opaque reference;
- reject semantic use;
- mark the interpretation as unknown/unsupported;
- retain the original bytes/identity for later tooling.

Unknown must never become `Soil`, `Hydroponic`, or another familiar default.

## 19. Authority boundary

No production-environment reference may authorize effects.

```text
ProductionEnvironmentRefV1
+ current source object
+ suitable crop profile
```

still does not imply:

```text
permission to dose nutrients
permission to start pump
permission to alter climate control
permission to harvest
```

Those remain separate bounded authorization/effect paths.

## 20. Certification boundary

Likewise:

```text
environment_profile
!= certified production method
```

Any organic, food-safety, engineering, electrical, water-quality, or other certification remains its own evidence/authority domain.

A profile may reference certification evidence but cannot mint it.

## 21. ITC boundary

Environment identity must remain independent from Integral-specific contribution/accounting semantics.

```text
ProductionEnvironmentRefV1
!= ITC account
!= ITC valuation
!= labor contribution weight
```

The productive-node showcase must remain usable with ITC disabled.

## 22. Symthaea boundary

Symthaea may consume the reference as part of typed analysis input.

Example:

```text
AnalysisRequest {
  subject/environment refs,
  source observations,
  constraints,
  requested analysis profile,
}
```

But:

```text
Symthaea inferred environment classification
!= source environment identity
```

unless explicitly admitted through a separate source/translation process.

## 23. Required negative controls

At minimum:

1. hydroponic environment encoded as `Plot` with fabricated soil type -> reject;
2. soil plot assigned hydroponic profile -> reject;
3. source-owner mismatch -> reject;
4. schema/profile version mismatch -> reject;
5. copied object ID under another namespace -> remains distinct identity;
6. runtime hash equality used as cross-profile semantic equality -> reject;
7. unknown profile coerced to a known profile -> reject;
8. environment ref treated as source measurement -> reject;
9. environment ref treated as certification -> reject;
10. environment ref treated as actuation authority -> reject;
11. transformed legacy record lacks required translation receipt -> reject/partial according to policy;
12. source record removed while derived projection claims authoritative source ownership -> reject;
13. module removal makes historical environment refs unparsable -> fail export/replaceability gate;
14. same crop species across two environments collapsed into one production identity -> reject;
15. environment change rewrites the historical environment of an earlier crop cycle -> reject.

## 24. Positive controls

At minimum:

1. native soil `Crop` + `Plot` projects to a soil environment reference while preserving native provenance;
2. hydroponic showcase batch binds to a hydroponic environment reference without a fake native `Plot`;
3. two production batches of the same species in different environments remain distinct;
4. an unknown future environment profile remains opaque and round-trippable;
5. runtime-specific identity can be exported while the semantic namespace/profile remains stable.

## 25. Implementation gate

Architecture and fixture work may proceed now.

Executable Rust types should only be promoted after reconciling exact ownership with:

- `SchemaRef` / `SemanticRef`;
- translation receipts;
- EPI provenance;
- source/currentness primitives;
- any existing resource/production reference types found in the current qualified lineage.

Avoid creating a parallel identity stack.

## 26. Nonclaims

MYC-INT-018E does not establish:

- a new canonical food-production source model;
- agronomic equivalence across production methods;
- crop suitability;
- environmental superiority;
- food-safety compliance;
- certification;
- operating authority;
- runtime choice.

It defines a truthful interoperability reference so the productive-node showcase can represent multiple production environments without falsifying source semantics.
