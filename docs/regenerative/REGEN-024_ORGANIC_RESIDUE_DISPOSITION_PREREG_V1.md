# REGEN-024 — Organic Residue Disposition Preregistration v1

Status: preregistration only. This document defines an evidence/routing boundary for organic and potentially organic residual material. It creates no compost recipe, pyrolysis recipe, animal-feed recommendation, sanitation procedure, treatment condition, disposal instruction, or physical process authority.

## 1. Purpose

Regenerative systems often begin with a useful intuition:

> waste should increasingly become feedstock.

That intuition becomes dangerous if the protocol silently upgrades every residual stream into a safe input.

REGEN-024 therefore freezes:

```text
residual exists
!= waste
!= valueless
!= available
!= rights-cleared
!= clean
!= homogeneous
!= compost-eligible
!= biochar-feedstock-eligible
!= feed-eligible
!= soil-eligible
!= nutrient-recovery-eligible
!= safe to handle
```

Core theorem:

```text
identified residual stream/lot
+ source-history evidence
+ material-state evidence
+ rights/custody state
+ destination-specific assessment
= reviewable disposition candidate
```

not:

```text
organic label
= safe circular feedstock
```

## 2. Residual is a neutral term

REGEN should prefer `residual` or another neutral material-flow term over assuming `waste` means unwanted or valueless.

A residual may be:

```text
reusable product
animal feed candidate
soil amendment input candidate
compost input candidate
biochar/pyrolysis feedstock candidate
anaerobic-digestion candidate
industrial input candidate
energy-recovery candidate
regulated disposal material
hazardous/unresolved material
```

The categories are possible destinations, not automatic qualifications.

## 3. Source history remains explicit

A material's present appearance does not erase its source history.

Relevant source context may include, where applicable:

```text
crop/agricultural residue
food-processing residual
food-service/post-consumer residual
wood/forestry residual
animal-origin residual
manure
municipal organic fraction
landscape/green residual
industrial biogenic residual
sanitation/wastewater-derived residual
mixed/unknown residual
```

This list is illustrative, not a universal regulatory classification.

## 4. Source class is not safety

```text
agricultural origin
!= clean

food origin
!= compost-safe

wood origin
!= untreated wood

biogenic origin
!= non-hazardous
```

Source class narrows questions; it does not answer them.

## 5. Unknown history remains unknown

The system must never infer:

```text
no treatment history recorded
=> untreated
```

or:

```text
no contamination record
=> uncontaminated
```

Unknown history is a first-class unresolved state.

## 6. Mixed material is not the sum of its best-known components

```text
qualified A
+ unknown B
!= qualified mixture
```

A mixed residual requires its own material state/lineage and destination-specific qualification.

The strongest input label cannot silently transfer to the mixture.

## 7. Physical lot/state identity matters

Disposition should bind an exact residual material subject/lot/state rather than a generic category such as `food waste` or `wood waste`.

Where existing REGEN identities are appropriate, they should be reused. Where no typed identity exists, the first version may use bounded opaque material-lot references rather than widen REGEN-002 implicitly.

## 8. Residual quantity is evidence-bearing

Quantity may be measured, derived, reported, forecast, or unresolved.

```text
estimated generation rate
!= observed available stock
```

and:

```text
container capacity
!= contained material quantity
```

PEF/shared admission remains the evidence foundation where appropriate.

## 9. Availability is a ladder

For residual material, the protocol should preserve at least:

```text
residual generated/exists
-> physically accessible
-> rights/custody cleared
-> source/material state sufficiently resolved
-> destination-specific admissibility
-> unreserved quantity
-> destination reservation
-> actual transfer/consumption
```

No step is automatic.

## 10. Existing use/competing use remains visible

A material described as `waste` may already provide useful service.

Examples can include:

```text
soil cover
nutrient return
animal bedding/feed
habitat
mulch
existing industrial use
community reuse
fuel/heat use
```

The existence of a new circular process does not establish that diverting the material is beneficial.

## 11. Ecological retention remains a hard gate

REGEN-011B5 applies where biomass/ecological function matters.

```text
residue technically collectable
!= ecologically surplus
```

A pyrolysis/compost facility cannot create removal eligibility merely by having demand for the material.

## 12. Destination-specific qualification is mandatory

The core disposition model should represent candidate destinations, but each destination owns its qualification theorem.

```text
eligible compost input
!= eligible pyrolysis input
!= eligible animal feed
!= eligible direct soil amendment
!= eligible nutrient recovery input
```

One positive assessment cannot be reused as a universal material-quality bit.

## 13. Compost boundary

REGEN-013 owns compost lineage/material semantics.

REGEN-024 can determine that a residual is a **candidate** input under an exact compost-input profile.

It does not declare finished compost quality or agronomic suitability.

## 14. Biochar/pyrolysis boundary

REGEN-011/011C/012 own qualified biomass input, consumption accounting, and biochar transformation/output lineage.

REGEN-024 may route a residual toward a candidate feedstock assessment.

It does not create pyrolysis eligibility, process settings, output quality, carbon removal, or carbon-credit authority.

## 15. Animal-feed boundary

A material that is biologically edible or historically used as feed is not automatically safe/legal/appropriate animal feed.

REGEN-024 defines no feed formulation, species-specific recommendation, disease-control procedure, or legal feed eligibility.

Feed use requires its own authoritative profile/domain assessment.

## 16. Direct soil-use boundary

```text
organic material
!= safe direct soil amendment
```

Direct soil use requires separate contamination, pathogen where applicable, material-quality, site, crop/use, and agronomic-suitability evidence.

REGEN-017 remains the contextual suitability layer.

## 17. Sanitation and human-waste-derived streams are high-gate

Where residuals originate from human sanitation/wastewater streams, the protocol requires stronger pathogen/hazard/treatment/regulatory/use restrictions before any soil/food-system pathway can be considered.

REGEN-024 deliberately defines **no treatment recipe, pathogen-kill target, temperature/time condition, or application instruction**.

```text
recoverable organic/nutrient content
!= safe agricultural material
```

## 18. Animal-origin/manure streams also retain hazard context

Animal-origin material may carry biological, pharmaceutical, contaminant, regulatory, or disease-context considerations.

Source class alone does not establish destination eligibility.

The protocol records the unresolved requirement rather than inventing a generic `safe manure` bit.

## 19. Treated/painted/coated/composite materials do not become clean biomass by appearance

Wood-like or fiber-like material can contain treatment/coating/adhesive/paint/composite or other non-biogenic constituents.

```text
looks like wood
!= clean wood feedstock
```

Unknown treatment state is unresolved.

REGEN-024 should route such streams through source-history/material-quality evidence rather than visual/category inference.

## 20. Plastics and mixed packaging remain explicit contaminants/co-materials

Organic residual collection may contain packaging or other non-organic material.

A mixed stream must preserve this state rather than being relabeled as pure organic feedstock.

The protocol defines no sorting or processing instruction; it simply prevents hidden purity claims.

## 21. Quarantine/biosecurity context remains separate

Plant material affected by pest/disease/quarantine restrictions may not be freely transferable or suitable for ordinary circular pathways.

REGEN-024 defines no destruction/treatment procedure.

It preserves the need for authoritative biosecurity/regulatory assessment.

## 22. Contamination evidence remains REGEN-016 territory

REGEN-024 references exact contamination/profile evidence where destination rules require it.

It preserves:

```text
not detected
!= zero

tested panel
!= all hazards absent

source history
!= analytical testing
```

## 23. Specimen identity remains REGEN-018 territory

Where laboratory results rely on sampled material:

```text
valid lab result
!= tested specimen represents entire residual lot
```

Sample/lot chain and representativeness remain separate propositions.

## 24. Currentness remains REGEN-019B territory

Residual state can change through storage, decomposition, mixing, contamination, moisture change, custody, or other events.

A historical qualification is not automatically current.

Disposition assessments must bind exact state/evidence snapshots.

## 25. Spatial/source-area relation remains REGEN-019A territory

Regional generation estimates or nearby observations do not automatically establish one facility/site's residual inventory or source legitimacy.

## 26. Rights/custody remain separate

```text
material physically discarded
!= abandoned in the legal sense
!= available to take
```

REGEN-024 does not decide ownership, custody, salvage rights, municipal authority, waste-management rights, indigenous/customary rights, or contractual claims.

It preserves exact external/prior references where required.

## 27. Disposition is tri-state per destination

Avoid a universal boolean `recyclable` or `circular` field.

For each destination profile, prefer:

```rust
pub enum DispositionAssessment {
    Eligible(...),
    Ineligible(...),
    Unresolved(...),
}
```

The outcome is scoped to one exact material state + destination profile + evidence snapshot.

## 28. Positive eligibility does not create reservation or execution

```text
destination eligible
!= material reserved
!= transferred
!= consumed by process
```

Planning/accounting state remains separate from physical action.

## 29. Reservations prevent double allocation

One physical material quantity cannot simultaneously be fully promised to multiple incompatible destinations.

A future pure reservation evaluator may reuse the same fail-closed accounting pattern as REGEN-011B2.

Reservation creates no ownership or execution authority.

## 30. Actual transfer is distinct

```text
reserved for compost
!= delivered to compost facility
```

Actual transfer/custody transition requires its own evidence/event proposition.

## 31. Actual processing is distinct

```text
delivered
!= accepted as process input
!= consumed
!= transformed
```

REGEN-011C's reservation-vs-consumption discipline is a useful general pattern for destination adapters.

## 32. Output qualification never back-propagates automatically

A successful process output does not retroactively prove every input state was universally safe or suitable for another destination.

Likewise a failed output does not prove every input was invalid.

Input and output propositions retain separate lineages.

## 33. Environmental burdens remain visible

A circular pathway may create transport, energy, emissions, water, equipment, labor, or other burdens.

```text
material diverted from disposal
!= net ecological benefit proven
```

Symthaea may compare these tradeoffs only after eligibility gates.

## 34. Avoid circularity theater

REGEN should explicitly resist metrics where moving material through any local process automatically increases a `circularity` score.

A pathway that contaminates soil, increases ecological extraction, consumes excessive scarce resources, or produces unusable output is not made beneficial by being circular in shape.

## 35. No universal hierarchy hard-coded

The core should not universally assert one disposition hierarchy such as:

```text
reuse > compost > pyrolysis > energy > disposal
```

Context matters. Safety, ecology, material quality, infrastructure, transport, existing uses, and local conditions can change the preferable option.

Any adopted hierarchy/policy is an exact external profile.

## 36. Symthaea optimization boundary

Symthaea may later compare **eligible** disposition options across plural objectives such as:

```text
material recovery
soil service
energy service
nutrient recovery
transport
cost
emissions
water
labor
infrastructure dependence
resilience
```

but cannot convert `Unresolved` or `Ineligible` pathways into eligible ones because of a high modeled benefit.

## 37. Initial executable direction

Potential crate/module:

```text
crates/mycelix-regenerative-residues
```

Likely dependency-light foundation:

```text
mycelix-core-types
mycelix-regenerative-core
mycelix-regenerative-admission
```

with biomass/compost/biochar/nutrients/water and authoritative regulatory/rights systems integrated through adapters.

Avoid circular domain-crate dependencies.

## 38. First executable campaign

A future implementation should test at least:

1. `organic` label cannot create destination eligibility;
2. unknown source history remains unresolved;
3. no treatment-history record does not become `untreated`;
4. no contamination result does not become clean;
5. mixed qualified + unknown material is not qualified;
6. existing beneficial use remains visible;
7. ecologically non-surplus biomass cannot become feedstock because a facility demands it;
8. compost eligibility does not imply pyrolysis eligibility;
9. pyrolysis eligibility does not imply direct soil eligibility;
10. one destination's positive assessment cannot become universal quality;
11. rights/custody unresolved blocks pathways that require them;
12. destination eligibility does not create reservation;
13. reservation does not create transfer;
14. transfer does not create process consumption;
15. process consumption does not create output qualification;
16. duplicate/incompatible reservations cannot overbook one material quantity;
17. current material-state change invalidates silent reuse of an older assessment;
18. valid sample result does not manufacture whole-lot representativeness;
19. human-waste-derived nutrient value cannot override unresolved safety gates;
20. animal-origin nutrient value cannot override unresolved hazard/regulatory gates;
21. local pathway cannot override hard safety/ecology constraints;
22. no disposition record carries process-execution or physical-actuation authority.

## 39. ProductFrozen qualification target

Executable residue code should follow REGEN-008/008A:

```text
authored source
-> explicit pinned-toolchain preparation
-> machine-preserved source + lock capsule
-> exact-byte ProductFrozen promotion
-> exact-head qualification
-> Q001 machine-readable receipt validation
```

## 40. Deliberate non-claims

REGEN-024 establishes no compost recipe, pyrolysis recipe, feed recommendation, sanitation method, pathogen-control procedure, sorting instruction, hazardous-material handling instruction, treatment condition, direct-soil application recommendation, disposal instruction, regulatory approval, rights determination, process authority, or physical actuation.

Its proposition is narrow:

> residual material remains an identified, evidence-bearing material state whose candidate destinations are separately assessed, reserved, transferred, consumed, and qualified rather than being automatically promoted from `organic/waste` into `safe circular feedstock`.
