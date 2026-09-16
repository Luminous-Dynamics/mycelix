# REGEN-021 — Crop and Nutritional Service Preregistration v1

Status: preregistration only. This document defines an evidence/accounting boundary for crop output and food/nutritional service. It does not prescribe diets, crop choices, treatment, rations, or public-health policy.

## 1. Purpose

A resilient food system cannot be represented faithfully by tonnes harvested alone.

REGEN-021 freezes the distinction between:

```text
crop biomass produced
harvested product
edible product
stored usable food
accessible food service
nutrient composition
actual consumption
health outcome
```

No arrow is automatic.

Core theorem:

```text
harvest evidence
+ explicit post-harvest transformations/losses
+ exact composition evidence
+ storage/access state
= reviewable food-service evidence
```

not:

```text
high yield
= food security
= nutritional adequacy
= equitable access
= healthy diet
```

## 2. Crop identity is contextual

The initial crop/food core should preserve exact crop/product/cultivar or source references where available without inventing a universal taxonomy inside REGEN.

Opaque bounded identifiers/references are acceptable until a separately reviewed canonical vocabulary exists.

```text
crop label
!= cultivar identity
!= harvested product form
!= edible form
!= processed food form
```

## 3. Yield is not edible output

A reported field yield may include stems, shells, moisture, damaged material, unmarketable fractions, or other components not directly available as food.

Therefore:

```text
gross harvest mass
!= edible mass
```

An edible fraction or conversion must be an explicit evidence/derivation proposition.

## 4. Wet/dry and mass-basis firewalls

Crop and food masses must preserve exact basis where material accounting matters.

```text
fresh mass
!= dry matter
```

No implicit moisture conversion is permitted.

PEF remains the owner of measurements and derived environmental/crop observations; REGEN should not duplicate measurement value/unit/uncertainty/provenance.

## 5. Post-harvest losses remain visible

The model should preserve explicit stages such as:

```text
harvested
-> cleaned/graded
-> processed
-> stored
-> distributed
-> available
```

with losses, diversion, spoilage, damage, export, seed retention, feed use, and unresolved residuals visible where materially relevant.

```text
not available for human food
!= waste
```

A diverted stream may have another legitimate use and should be routed explicitly rather than disappearing.

## 6. Storage is inventory state

Food placed into storage remains an inventory stock.

Storage loss, spoilage, quality degradation, power/refrigeration dependency, packaging dependency, and shelf-life/currentness are separate propositions.

No universal shelf-life table belongs in the dependency-light core.

## 7. Production is not access

A locality may produce substantial food while some residents cannot access it.

```text
food produced locally
!= food accessible locally
```

Access may depend on distribution, affordability, entitlement, logistics, storage, cultural use, preparation capacity, and other social conditions.

REGEN-021 should preserve these as separate evidence/coordination dimensions rather than declaring food security from production alone.

## 8. Nutrient composition is evidence, not a constant

Nutrient composition may vary with cultivar, soil, maturity, processing, storage, moisture basis, analytical method, fortification, and data source.

Therefore:

```text
food name
!= fixed nutrient vector
```

Composition values should be evidence-bearing and source/version specific.

## 9. Nutrient-service vector stays plural

The default model should expose plural nutritional-service dimensions rather than one opaque `nutrition_score`.

Potential dimensions may include, where appropriate and evidenced:

```text
food energy
protein
selected essential amino-acid context
fat/fatty-acid context
fiber
selected vitamins
selected minerals
```

This list is illustrative and not a diet prescription.

A deployment should use an explicit adopted nutritional vocabulary/profile rather than silently treating this list as universal.

## 10. No universal dietary adequacy theorem

REGEN-021 does not define one universal diet, minimum ration, demographic requirement, or medical nutrition target.

Population nutritional requirements vary with age, sex, pregnancy/lactation, activity, health, climate, and adopted public-health standards.

Any adequacy comparison must bind an exact adopted requirement/profile and population context.

```text
nutrient supply estimated
!= dietary adequacy proven
```

## 11. Population denominator is explicit

Any statement such as:

```text
this community can supply X days of food
```

requires explicit population/service assumptions and cannot be inferred from total kilograms alone.

The denominator, population revision, adopted nutritional profile, storage assumptions, and uncertainty must be reviewable.

## 12. Calories alone are insufficient

```text
calorie sufficiency
!= nutritional adequacy
```

Likewise:

```text
protein mass
!= protein-quality/essential-amino-acid sufficiency
```

REGEN should preserve relevant dimensions separately rather than allow one abundant macronutrient to hide other constraints.

## 13. Diversity is not automatically adequacy

Crop/food diversity can be useful resilience evidence, but:

```text
many crop species
!= adequate nutrition
```

and:

```text
low crop diversity
!= necessarily inadequate nutrition
```

Diversity belongs as a separate resilience/ecology/seed-system dimension, not a substitute nutritional theorem.

## 14. Food safety remains separate

Nutritional value cannot compensate for a hard food-safety or contamination failure.

```text
nutritious
+ unsafe
!= eligible food service
```

Contamination/pathogen/allergen/regulatory/handling evidence remains in the appropriate safety/domain layers.

REGEN-021 does not create food-safety thresholds.

## 15. Application/production causality remains separate

A soil amendment, nutrient input, irrigation method, or other intervention followed by higher crop output does not by itself prove the intervention caused the result.

REGEN-015 field-trial semantics remain the causal evidence boundary.

## 16. Forecasts remain forecasts

Projected harvest, modelled yield, or scenario production may be useful planning evidence but cannot be silently promoted to observed inventory.

```text
Forecast yield
!= harvested stock

Scenario diet service
!= actual food availability
```

REGEN-019 shared admission preserves the PEF evidence class.

## 17. Harvested food can leave the locality

Export/sale/transfer must remain explicit.

```text
locally produced
!= locally retained
```

Food leaving the locality may still create economic value, but it no longer counts as local physical food inventory unless an explicit return/substitution flow is modelled.

## 18. Imports are not failure

A resilient community need not produce every food locally.

External supply can improve nutrition, variety, efficiency, and ecological outcomes.

The relevant question is dependency structure and recoverability, not ideological autarky.

```text
imported
!= bad

local
!= inherently resilient
```

## 19. Critical dependency mapping

Useful food-resilience analysis should expose dependencies such as:

```text
seed
water
fertility/nutrients
energy
cold storage
packaging
processing
transport
repair/spares
labor/skills
pollination/ecology
market/distribution access
```

A high-yield crop that depends on one unavailable critical input may provide little disruption resilience.

## 20. Service floor is multidimensional

A later resilience model may define minimum food/nutrition service floors, but the default should remain a vector of constraints rather than a weighted average.

For example:

```text
energy-service floor
protein-service floor
selected micronutrient-service floors
safe-water/food-preparation dependency
storage/distribution floor
```

An adopted deployment profile defines which dimensions matter.

## 21. Hard minima are not compensable

Where an adopted profile defines a hard minimum:

```text
critical nutrient/service below floor
+ surplus elsewhere
!= floor satisfied
```

Do not hide severe shortage in one dimension behind a weighted aggregate score.

## 22. Edible-service accounting remains bounded by physical inventory

A nutritional-service calculation must not create more edible material than the exact inventory/flow chain permits.

Conceptually:

```text
harvested mass
= edible retained
+ seed retained
+ feed/other use
+ exported/transferred
+ post-harvest loss
+ unresolved residual
```

where categories and bases are appropriate to the adopted crop/product profile.

## 23. Nutrient composition does not create mass

Composition analysis transforms one physical food stock into a nutrient-service estimate; it does not create additional physical inventory.

Likewise, nutrient vectors should not be added across incompatible edible-state/basis revisions without explicit normalization.

## 24. Cooking/processing may change service

Processing can change edible mass, water content, nutrient composition, digestibility, safety, storage life, and energy demand.

No universal processing factor belongs in v1.

A processed product is a new state/product proposition with its own evidence/derivation identity.

## 25. Availability is not consumption

```text
food available
!= food consumed
```

and:

```text
food consumed
!= nutrient absorbed
!= health outcome
```

REGEN-021 deliberately stops before medical/individual-health inference.

## 26. No individual medical inference

The core must not use community food-service accounting to diagnose individuals, prescribe diets, or infer health status.

Personal/clinical nutrition belongs in appropriate health/clinical systems with separate consent/authority/safety rules.

## 27. Food waste/residual loops

Post-consumer and processing residuals may later feed compost, digestion, animal feed, biochar-compatible biomass, or other systems only after each destination's safety/quality/ecology rules are satisfied.

```text
food residual exists
!= safe compost input
!= safe animal feed
!= safe pyrolysis feedstock
```

Cross-domain adapters preserve the evidence rather than manufacturing eligibility.

## 28. Nutrient-flow bridge

REGEN-020 may account for nutrient movement through food/crop stocks.

REGEN-021 should reference those exact nutrient-flow propositions rather than create a second N/P/K accounting system.

Likewise, REGEN-020 nutrient stock does not prove edible nutritional service.

```text
agronomic nutrient stock
!= human nutritional service
```

## 29. Seed retention remains separate

Harvest retained for seed is not immediately edible inventory unless the deployment explicitly allows dual-use accounting.

Seed resilience should be handled by the planned REGEN-022 seed-diversity/propagation contract rather than silently consuming seed reserve into food availability.

## 30. Time/currentness

Food inventory and composition assessments bind exact evidence snapshots.

REGEN-019B remains the purpose-specific currentness layer.

Old harvest/storage data cannot silently remain current because the original observation was once valid.

## 31. Spatial scope

Farm/site/plot/community geometry and containment remain REGEN-019A responsibilities.

A regional crop forecast cannot automatically become one community's available inventory.

## 32. Symthaea optimization boundary

Symthaea may later compare qualified crop/food system alternatives across plural objectives such as:

```text
service floors
water
nutrient dependence
energy
storage
losses
labor
cost
biodiversity/ecology
external dependency
recovery time
```

but it must not trade away hard food safety, ecological, rights, or adopted nutritional-service minima for a better aggregate score.

## 33. No single food-resilience score by default

Default outputs should expose plural dimensions such as:

```text
harvest service
edible retained inventory
storage loss
external dependency
critical-input dependency
energy/protein/micronutrient service vectors
seed reserve separation
recovery options
```

A later aggregate metric requires its own exact definition and must not replace the underlying dimensions.

## 34. First executable direction

Potential crate:

```text
crates/mycelix-regenerative-food
```

Likely dependency-light foundation:

```text
mycelix-core-types
mycelix-regenerative-core
mycelix-regenerative-admission
```

with nutrients/seeds/water/market/storage integrations as adapters rather than circular dependencies.

## 35. First executable campaign

A future implementation should test at least:

1. gross harvest cannot equal edible output without explicit derivation;
2. wet/dry basis mismatch fails;
3. post-harvest losses remain explicit;
4. seed-retained mass is not silently counted as edible availability;
5. export is removed from local physical inventory;
6. storage stock remains stock, not consumption;
7. forecast/scenario yield cannot become observed inventory;
8. nutrient composition source/version remains bound;
9. crop name alone cannot manufacture a nutrient vector;
10. calories cannot satisfy unrelated nutrient-service minima;
11. weighted surplus cannot hide a hard service-floor failure;
12. availability cannot become consumption;
13. consumption cannot become health outcome;
14. unsafe/ineligible food cannot be rescued by nutritional value;
15. local origin cannot override hard safety/ecology gates;
16. import origin does not imply non-resilience;
17. critical external dependencies remain visible;
18. live evidence cannot rewrite historical snapshots;
19. food residual cannot automatically become eligible compost/feed/pyrolysis input;
20. no food-service record carries process-execution or physical-actuation authority.

## 36. ProductFrozen qualification target

Executable crop/food code should follow REGEN-008/008A:

```text
authored source
-> explicit pinned-toolchain preparation
-> machine-preserved source + lock
-> exact-byte ProductFrozen promotion
-> exact-head campaign
-> Q001 receipt validation
```

## 37. Deliberate non-claims

REGEN-021 establishes no dietary prescription, individual nutritional requirement, medical advice, food-safety certification, crop recommendation, yield guarantee, treatment efficacy, affordability guarantee, social entitlement, market value, governance authority, process authority, or physical actuation.

Its proposition is narrow:

> crop production, edible conversion, storage, loss, transfer, composition, and plural nutritional-service estimates remain explicit and evidence-bearing rather than collapsing yield into food security or health.
