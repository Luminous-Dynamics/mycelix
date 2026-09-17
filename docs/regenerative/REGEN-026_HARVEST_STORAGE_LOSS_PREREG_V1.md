# REGEN-026 — Harvest, Storage, and Food-Loss Evidence Preregistration v1

Status: preregistration only. This document freezes how REGEN represents the journey from crop/food production to actually available nutritional service without collapsing physical production, edible yield, storage, access, consumption, or loss into one optimistic quantity.

## 1. Purpose

A regenerative system should not report food resilience from gross production alone.

Core theorem:

```text
produced
!= harvested
!= edible
!= accepted/graded
!= processed
!= stored
!= currently usable
!= locally accessible
!= nutritionally available
!= consumed
```

Likewise:

```text
not consumed
!= waste
!= spoilage
!= avoidable loss
```

## 2. Upstream composition

REGEN-026 composes with:

- REGEN-019 shared PEF admission;
- REGEN-019B evidence snapshots/currentness;
- REGEN-020 nutrient stock/flow;
- REGEN-021 crop/nutritional service;
- REGEN-022 seed/propagation evidence;
- REGEN-023 Water bridge;
- REGEN-024 residue disposition;
- REGEN-025 ecological obligations.

It does not replace crop, warehouse, market, water, nutrition, seed, waste, or property systems.

## 3. Stage-separated accounting

The first contract should preserve explicit stage quantities such as:

```text
ProducedQuantity
HarvestedQuantity
EdibleQuantity
AcceptedQuantity
ProcessedQuantity
StoredQuantity
UsableStoredQuantity
AccessibleQuantity
ConsumedQuantity
```

No stage transition is automatic.

A downstream stage must not exceed its upstream physical quantity except where an explicit transformation changes basis and carries its own conservation/evidence theorem.

## 4. Loss/disposition categories

Material leaving one food-service stage must remain visible under an explicit disposition rather than disappear from accounting.

Potential categories include:

```text
field_nonharvest
harvest_damage
grading_rejection
processing_loss
storage_spoilage
pest_damage
quality_downgrade
seed_reserve
animal_feed_diversion
industrial_use
compost_feedstock
other_material_recovery
export_or_transfer
human_consumption
unresolved_residual
```

The ontology should distinguish a physical disposition from a causal explanation.

```text
stored quantity decreased
!= spoilage proven

food left local inventory
!= waste

low-grade output
!= unsafe food
```

## 5. Explicit unresolved residual

Every resolved mass/quantity reconciliation should permit an explicit unresolved residual.

Conceptually:

```text
upstream quantity
=
downstream retained/service quantity
+ identified dispositions
+ explicit unresolved residual
```

`unmeasured != zero`.

A reconciliation that closes only by forcing the residual to zero must fail.

## 6. Quantity and basis discipline

The first implementation must preserve exact quantity basis.

Examples that must not be silently mixed include:

```text
wet crop mass
!= dry matter

whole harvested crop
!= edible fraction

fresh mass
!= processed dry product mass

physical mass
!= nutritional-energy equivalent
```

Any conversion is a separate Derived evidence proposition with exact method/profile identity.

## 7. Food-service quantity is not one scalar

REGEN-026 must not define a canonical `food_resilience_score` or reduce food service to tonnes alone.

Food availability may require plural dimensions such as:

- edible quantity;
- nutritional composition/service;
- storage life/currentness;
- safety/quality status;
- accessibility;
- population/use context;
- reserve purpose;
- substitutability/diversity.

These dimensions remain inspectable rather than hidden in one weighted score.

## 8. Harvest evidence

Evidence that a crop was grown or estimated does not establish harvest.

```text
standing crop
!= harvested inventory

forecast yield
!= observed harvest
```

Reported/Observed and computed PEF classes retain their original evidence class and provenance through REGEN-019 admission.

## 9. Grading and edible fraction

Grade/rejection status must remain relative to an exact adopted profile/use context.

```text
rejected from premium market
!= inedible
!= unsafe
!= valueless
```

Likewise:

```text
edible fraction estimate
!= measured edible quantity
```

Any calculated edible fraction requires its own method/evidence lineage.

## 10. Processing transformation

Processing may change mass, water content, concentration, composition, packaging, shelf life, and recoverable coproducts.

REGEN-026 should therefore reference explicit processing/transformation evidence rather than assume conservation on incompatible bases.

Process input, product output, coproducts, losses, emissions, water, and unresolved residuals remain distinct.

No food-processing recipe or operating instruction is defined here.

## 11. Storage state

Storage capacity is not stored inventory.

```text
storage capacity
!= inventory
!= usable inventory
```

A storage snapshot may need exact:

- facility/container reference;
- inventory subject/batch reference;
- quantity/basis;
- observation time;
- quality/safety status reference;
- reserve/use purpose;
- evidence snapshot.

A later observation creates a new state rather than silently rewriting historical inventory.

## 12. Currentness

Stored food can remain valid evidence while becoming too stale for a current service proposition.

REGEN-019B currentness applies:

```text
valid historical inventory observation
!= current usable inventory
```

Unknown observation time must remain unknown and cannot be replaced with ingestion time.

## 13. Spoilage and quality degradation

Spoilage is a proposition requiring evidence.

```text
inventory decrease
!= spoilage

visual defect
!= pathogen contamination

passed old quality test
!= current safety
```

REGEN-016 contamination semantics remain separate.

A material may be downgraded, redirected, or no longer suitable for one use while remaining eligible for another destination under REGEN-024.

## 14. Seed/feed/diversion firewall

Seed reserves and other future-production assets must not be double-counted as ordinary near-term human food inventory.

REGEN-022 applies:

```text
stored seed
!= immediately available edible food
```

Likewise animal-feed, processing, export, donation, emergency reserve, or other allocations remain visible rather than being simultaneously counted in multiple service pools.

## 15. Access and locality

Food physically present in a region does not automatically establish community access.

```text
local inventory
!= locally accessible inventory
```

Access may depend on ownership/custody, allocation, distribution, price, emergency reserve policy, transport, or other authoritative systems.

REGEN-026 does not adjudicate those rights; it references their resolved state when needed.

## 16. Consumption evidence

Inventory leaving storage/distribution is not automatically human consumption.

```text
dispatched
!= delivered
!= consumed
```

Consumption/service evidence should preserve its own observation or authoritative record lineage.

## 17. Causal claims remain separate

A loss classification is not causal proof.

```text
storage loss observed during outage
!= outage caused the loss
```

Causal attribution requires its own design/evidence and, where relevant, REGEN-015 trial/causal-analysis semantics.

## 18. No blame inference

The evidence core must not convert loss into responsibility, negligence, fraud, or governance failure.

```text
loss observed
!= actor fault
```

Responsibility/legal/accountability determinations live elsewhere.

## 19. Resilience use

Resilience analysis may compare service floors under shocks using stage-separated inventory and loss observations.

Examples include transport interruption, storage power loss, water constraint, crop failure, processing outage, pest event, or access disruption.

But:

```text
historical loss rate
!= universal future failure probability
```

unless an explicit model establishes that proposition.

## 20. Symthaea boundary

Symthaea may later infer, forecast, optimize, or recommend only with evidence classes preserved.

It may not convert:

```text
Forecast
-> Observed
```

or treat uncertain loss estimates as exact physical inventory.

It may optimize among qualified options, but cannot override food-safety, reserve, rights, ecological, or other hard gates.

## 21. Proposed executable shape

A later dependency-light core may define concepts approximately like:

```text
FoodFlowSubject
FoodStageSnapshot
FoodDispositionRecord
FoodReconciliation
LossObservationBinding
```

with external authoritative identity/adapters where existing domain systems already own the subject.

Do not create a parallel inventory/WMS/ERP system inside REGEN merely to represent evidence relationships.

## 22. First regression campaign

A first executable campaign should prove at least:

1. produced quantity cannot directly instantiate consumed quantity;
2. harvested cannot exceed produced on the same basis without transformation evidence;
3. edible cannot exceed harvested on the same basis;
4. stored cannot exceed accepted/processed input without transformation accounting;
5. unresolved residual remains explicit;
6. forced-zero residual fails;
7. mixed quantity bases fail;
8. forecast harvest remains Forecast;
9. storage capacity cannot instantiate stored inventory;
10. stale inventory cannot silently become current;
11. inventory decrease cannot automatically classify as spoilage;
12. grade rejection cannot automatically mean unsafe;
13. seed reserve cannot be double-counted as ordinary food service;
14. feed diversion cannot remain in human-food availability simultaneously;
15. export/transfer cannot remain in local available inventory simultaneously;
16. dispatch cannot become consumption without consumption evidence;
17. loss category cannot create causal attribution;
18. loss evidence cannot create actor blame;
19. valid PEF admission cannot manufacture accessibility;
20. high yield cannot manufacture nutritional adequacy;
21. local production cannot override food safety or rights;
22. no record carries execution/actuation authority.

## 23. Qualification discipline

A later executable core should follow REGEN-008/008A:

```text
authored source
-> pinned-toolchain preparation
-> machine-preserved formatted source + Cargo.lock
-> exact-byte ProductFrozen promotion
-> exact-head qualification
-> Q001 receipt validation
```

System closure remains a separate proposition.

## 24. Deliberate non-claims

REGEN-026 establishes no current crop yield, food inventory, food safety, nutritional adequacy, storage suitability, shelf life, causal source of loss, actor responsibility, property/access right, distribution priority, market price, rationing policy, food-processing procedure, treatment instruction, resilience superiority, governance authority, or physical actuation.

Its proposition is deliberately narrow: regenerative food accounting should preserve where material/service was lost, diverted, stored, transferred, consumed, or remains unresolved instead of equating gross production with usable food resilience.
