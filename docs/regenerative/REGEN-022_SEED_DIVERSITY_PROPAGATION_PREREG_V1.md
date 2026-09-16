# REGEN-022 — Seed Diversity and Propagation Preregistration v1

Status: preregistration only. This document defines an evidence/accounting boundary for seed lots, propagation capacity, diversity, reserve state, and resilience planning. It does not prescribe crops, planting conditions, germination treatments, breeding programs, diets, or agricultural operating instructions.

## 1. Purpose

A resilient food system needs more than current harvest output. It also needs the ability to reproduce future planting material without silently consuming the reserve that enables recovery.

REGEN-022 therefore separates:

```text
seed lot exists
seed identity declared
seed identity verified
seed quantity known
seed viable
seed germination observed
seed produces healthy plants
seed reproduces true-to-type where relevant
seed legally/contractually propagable
seed ecologically/agronomically appropriate
seed locally adapted
seed reserve sufficient
```

No arrow is automatic.

Core theorem:

```text
identified seed lot
+ exact provenance/evidence
+ explicit viability/propagation observations
+ reserve accounting
+ diversity dimensions
= reviewable propagation-resilience evidence
```

not:

```text
seeds stored
= future crop security
= locally adapted seed
= genetic diversity
= propagation right
= nutritional resilience
```

## 2. Identity layers remain separate

The first seed core should distinguish at least:

```text
physical seed lot identity
claimed species/taxon identity
claimed cultivar/variety/landrace identity
genetic/assay identity where available
source/provenance identity
```

A label on a packet or inventory record does not itself prove genetic identity.

Likewise:

```text
same cultivar label
!= same seed lot
```

and:

```text
same seed lot
!= stable viability over time
```

## 3. No implicit REGEN-002 identity widening

REGEN-002 does not currently freeze a typed `SeedLotId` or `SeedAccessionId`.

The first implementation should therefore use bounded canonical opaque seed-lot/accession references rather than silently widening the qualified regenerative identity grammar.

A future typed identity belongs in its own reviewed REGEN identity revision.

## 4. Seed quantity is not viability

```text
1000 seeds in inventory
!= 1000 viable seeds
```

Physical count/mass and viability evidence remain separate.

A seed inventory record may preserve exact lot quantity while viability is unknown, stale, estimated, or separately measured.

## 5. Viability is time/context dependent

A historical viability observation does not remain current indefinitely merely because it was once valid.

REGEN-019B currentness semantics apply:

```text
valid historical viability evidence
!= current-enough viability evidence
```

No universal shelf-life or retest interval belongs in this core.

## 6. Germination is not establishment

A germination observation does not prove successful field establishment, mature plant survival, harvest performance, or future seed production.

```text
germinated
!= established
!= productive
!= reproductively successful
```

These stages remain distinct evidence propositions.

## 7. Propagation success is not true-to-type identity

For crops/propagules where identity stability matters:

```text
plant reproduced
!= offspring identity verified
```

Open-pollination, hybridization, segregation, contamination, mutation, labeling error, or other biological/process factors may alter expected traits.

REGEN-022 does not create a universal genetic-purity standard.

## 8. Genetic identity evidence remains evidence-classed

Morphological description, supplier declaration, community knowledge, pedigree record, assay result, or genomic evidence may each have different evidentiary status.

PEF/shared evidence semantics preserve the evidence class and provenance rather than reducing identity to `verified=true`.

The seed core should record what identity proposition was supported and by which evidence snapshot/profile.

## 9. Diversity is multidimensional

Do not collapse diversity into a raw count of seed packets, crop species, cultivars, or accessions.

Useful dimensions may include, where explicitly defined:

```text
species/taxon diversity
cultivar/landrace diversity
within-population diversity
functional diversity
seasonal/phenological diversity
stress-response diversity
pollination/reproductive-system diversity
storage-longevity diversity
source/geographic diversity
```

This list is illustrative, not a mandatory agricultural ontology.

## 10. Diversity is not automatically resilience

```text
high diversity
!= resilience
```

A diverse collection can still share one storage failure, one water dependency, one disease vulnerability, one legal restriction, or one unavailable propagation skill.

Likewise:

```text
lower diversity
!= automatically fragile
```

Resilience depends on capabilities, dependencies, redundancy, recovery pathways, and context—not diversity count alone.

## 11. No universal seed-diversity score

The default system should expose plural diversity/reserve/dependency dimensions.

It should not emit one canonical:

```text
seed resilience = 87/100
```

without a separately reviewed exact metric defining boundaries, weighting, population, time horizon, dependencies, and uncertainty.

## 12. Seed reserve is not edible inventory by default

REGEN-021 already separates seed retention from edible food availability.

REGEN-022 reinforces:

```text
propagation reserve
!= immediately available food stock
```

A system must not double-count the same physical material simultaneously as both required future propagation reserve and freely consumable food inventory unless an explicit dual-use policy models that tradeoff.

## 13. Reserve accounting should preserve purpose

Conceptually, one physical lot may have purpose-scoped allocations such as:

```text
propagation reserve
research/reference reserve
exchange/distribution reserve
consumption-eligible surplus
unresolved/unallocated stock
```

Allocating seed to one purpose does not automatically authorize another use.

## 14. Reservation does not consume seed

As with biomass:

```text
reserved for planting
!= planted
```

and:

```text
planned distribution
!= transferred
```

Actual physical transitions require their own evidence/accounting events.

## 15. Propagation rights remain external

The system should preserve separately relevant legal/contractual/customary/community rights or restrictions without deciding law inside the dependency-light seed core.

Possible distinctions include:

```text
possession
storage
exchange
sale
propagation
breeding/research use
cross-border movement
```

The presence of a seed lot does not manufacture any of those rights.

## 16. Rights reference does not prove validity

As in biomass:

```text
rights reference present
!= authentic
!= current
!= legally sufficient
```

The seed core should carry exact external/prior assessment references rather than invent a parallel legal registry.

## 17. Local origin is not local adaptation

```text
locally sourced seed
!= locally adapted seed
```

and:

```text
seed grown locally once
!= adaptation proven
```

Adaptation is a contextual biological/agronomic proposition requiring appropriate evidence over relevant conditions and time.

## 18. Imported seed is not inherently fragile

External seed sources may provide valuable genetics, diversity, disease resistance, nutritional traits, or recovery options.

```text
imported
!= bad

local
!= inherently resilient
```

The relevant resilience question is whether critical propagation capability has adequate alternatives and recovery paths.

## 19. Seed-source concentration is visible

A seed system should be able to expose dependencies such as:

```text
single supplier
single storage location
single propagator/skill holder
single parent line
single compatible pollinator
single climate window
single transport route
```

without declaring those dependencies unacceptable by default.

## 20. Common-mode storage failure matters

Ten accessions stored in one freezer, building, warehouse, or power-dependent facility do not provide ten independent failure domains.

REGEN resilience analysis should distinguish:

```text
logical diversity
!= failure-domain diversity
```

## 21. Storage conditions remain evidence, not universal recipes

Seed storage environment may materially affect viability, but REGEN-022 defines no universal temperature, humidity, packaging, treatment, or storage-duration prescription.

Storage conditions/observations remain evidence tied to the exact lot and time period.

## 22. Viability evidence preserves method identity

A viability/germination result should bind its exact method/protocol/profile reference, sample/lot relation, date/time evidence, sample size where relevant, and provenance.

```text
reported germination percentage
!= universal viability truth
```

Different methods/conditions may answer different questions.

## 23. Sample representativeness remains separate

REGEN-018 specimen/sample lineage and REGEN-019A sampling-frame semantics remain relevant.

```text
valid tested seed sample
!= entire seed lot represented
```

The seed core must not infer whole-lot viability or identity merely because one sample produced a valid observation.

## 24. Currentness remains purpose-specific

REGEN-019B applies separately to:

```text
lot quantity
viability
identity evidence
storage state
rights assessment
propagation capability
```

A current inventory count with stale viability evidence does not become current viable inventory.

## 25. Seed availability is not propagation capacity

A usable propagation service may depend on more than the seed lot itself:

```text
seed viability
water
soil/substrate
nutrient system
season/window
space
pollination/reproductive requirements
skills/labor
equipment
storage/nursery capacity
plant-health controls
```

The initial core should preserve those dependencies as external capability references rather than silently assuming them available.

## 26. Propagation capacity is not harvest capacity

Even if seedlings/plants can be produced:

```text
propagation capability
!= harvest service capability
```

REGEN-021 remains the crop/food-service layer.

## 27. Seed production is a material transformation/flow

When a crop produces seed reserved for future propagation, the output becomes a new seed-lot/accession state with explicit lineage rather than silently replenishing the original lot.

```text
parent seed lot
-> crop/propagation episode
-> newly identified seed lot
```

Identity/provenance does not collapse across generations.

## 28. Seed multiplication is not free inventory creation

A forecast or expected multiplication factor cannot mint inventory.

```text
expected seed yield
!= observed stored seed stock
```

Forecast/Scenario evidence remains forecast/scenario through REGEN-019.

## 29. Seed exchange preserves provenance

Transfer between communities/sites should preserve exact source lot/provenance, transfer/accounting evidence, receiving lot state, and any material legal/quality context.

The seed core need not own marketplace/payment mechanisms.

## 30. Genetic-resource and community provenance deserve explicit preservation

Where landrace, community, indigenous, breeder, institutional, or conservation provenance materially applies, systems should be able to preserve those references rather than stripping origin context during exchange.

This architecture itself does not adjudicate ownership, sovereignty, benefit-sharing, or legal status; those remain explicit external/governance propositions.

## 31. Biosecurity/phytosanitary status remains separate

```text
viable seed
!= safe/legal seed movement
```

Pest/pathogen/quarantine/import/export requirements remain separately assessed. REGEN-022 defines no phytosanitary treatment or movement instructions.

## 32. Seed quality is not crop suitability

```text
high-quality viable seed
!= suitable for this site/crop system
```

Site-specific agronomic/ecological suitability remains contextual and must not be inferred solely from lot quality.

## 33. Nutritional value does not control seed reserve

A seed/food material may be nutritionally valuable while still being critical propagation reserve.

REGEN-021 food-service optimization must not consume required seed reserves merely because doing so improves a near-term nutrition objective.

Hard reserve floors, where adopted, remain constraints.

## 34. Seed bank inventory is not resilience by itself

A seed bank may improve recovery options, but resilience also depends on:

```text
viability maintenance
refresh/regeneration capability
skills
land/water
access rights
distribution
redundant storage
documentation/provenance
propagation success
```

Symthaea should model the full capability graph rather than count accessions.

## 35. Seed regeneration can create drift

Regenerating stored accessions may introduce selection, contamination, bottleneck, crossing, or other changes.

REGEN-022 therefore separates:

```text
accession continuity claim
!= unchanged genetic identity
```

Any identity-continuity claim should preserve the evidence/method used.

## 36. No breeding optimization authority

Symthaea may later analyze seed/crop alternatives or experimental hypotheses, but REGEN-022 creates no autonomous breeding, release, planting, genetic modification, or biological-intervention authority.

Recommendation remains separate from action.

## 37. Symthaea resilience boundary

Symthaea may compare qualified seed-system alternatives across dimensions such as:

```text
failure-domain redundancy
diversity
viability/currentness
propagation dependencies
recovery time
water/nutrient dependence
skills
storage energy dependence
rights/access constraints
food-service contribution
```

but hard ecological, legal, biosafety, or adopted reserve constraints remain constraints rather than optimizer weights.

## 38. Initial executable direction

Potential crate:

```text
crates/mycelix-regenerative-seeds
```

Likely dependency-light foundation:

```text
mycelix-core-types
mycelix-regenerative-core
mycelix-regenerative-admission
```

with food/nutrient/spatial/specimen/storage/governance integrations as adapters rather than circular dependencies.

## 39. First executable campaign

A future implementation should test at least:

1. physical lot identity remains distinct from claimed cultivar/species identity;
2. inventory count does not imply viable count;
3. stale viability does not become current viability;
4. germination does not imply establishment;
5. establishment does not imply harvest;
6. propagation does not imply true-to-type identity;
7. seed-label identity does not manufacture genetic verification;
8. diversity count does not manufacture resilience;
9. seed reserve is not silently counted as edible inventory;
10. one physical quantity cannot be fully reserved simultaneously for incompatible purposes;
11. reservation does not imply planting/transfer;
12. rights refs do not manufacture propagation authority;
13. local origin does not manufacture adaptation;
14. imported origin does not manufacture fragility;
15. common storage failure domain remains visible;
16. tested sample does not automatically represent whole lot;
17. valid seed evidence can remain biosecurity/phytosanitary unresolved;
18. forecast multiplication cannot mint observed inventory;
19. newly produced seed gets new material/provenance state;
20. hard reserve floors cannot be overridden by food-value optimization;
21. valid seed quality does not manufacture site suitability;
22. no seed record carries planting/breeding/process-execution authority.

## 40. ProductFrozen qualification target

Executable seed code should follow REGEN-008/008A:

```text
authored source
-> explicit pinned-toolchain preparation
-> machine-preserved source + lock capsule
-> exact-byte ProductFrozen promotion
-> exact-head qualification
-> Q001 machine-readable receipt validation
```

## 41. Deliberate non-claims

REGEN-022 establishes no crop recommendation, planting procedure, germination treatment, seed-storage recipe, breeding instruction, genetic-purity certification, phytosanitary clearance, legal propagation right, local-adaptation claim, yield guarantee, food-security guarantee, governance authority, biological intervention authority, or physical actuation.

Its proposition is narrow:

> seed physical inventory, identity evidence, viability, propagation evidence, reserve purpose, diversity, provenance, and critical dependencies remain explicit and evidence-bearing rather than collapsing stored seed into future food-system resilience.
