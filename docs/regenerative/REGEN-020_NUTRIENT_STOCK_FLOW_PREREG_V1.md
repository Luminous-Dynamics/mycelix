# REGEN-020 — Nutrient Stock and Flow Preregistration v1

Status: preregistration only. This document defines a nutrient-accounting/evidence boundary for regenerative systems. It does not prescribe fertilizer rates, sanitation processes, treatment conditions, crop recommendations, or physical control.

## 1. Purpose

Local resilience improves when nutrient loops become more observable, recoverable, and less dependent on fragile imports. But a single `N/P/K recycled` number would collapse chemically, agronomically, environmentally, and clinically different propositions.

REGEN-020 therefore freezes nutrient accounting as an explicit stock/flow/evidence model rather than a generic circularity score.

Core theorem:

```text
nutrient source
+ exact species/basis
+ explicit transformation/flow
+ exact destination/state
+ evidence lineage
= reviewable nutrient accounting
```

not:

```text
nutrient present
= plant available
= safe
= legally usable
= fertilizer equivalent
= agronomically suitable
```

## 2. Nutrient identity is not one scalar

The protocol MUST NOT treat all nitrogen, phosphorus, potassium, carbon, sulfur, calcium, magnesium, or micronutrients as interchangeable within an element family.

Examples of distinctions that may matter include:

```text
total nitrogen
ammonium-N
nitrate-N
organic-N

 total phosphorus
orthophosphate-P
operationally defined plant-available P

 total potassium
soluble/exchangeable K
```

This list is illustrative, not a universal agronomy ontology.

The key invariant is:

```text
same element
!= same chemical form
!= same mobility
!= same plant availability
!= same environmental risk
```

## 3. Nutrient-species keys

REGEN-002 does not freeze a typed nutrient-species identity. The first implementation should therefore use bounded canonical namespaced keys rather than widening the identity grammar implicitly.

Conceptually:

```rust
pub struct NutrientSpeciesKey(String);
```

Example namespaces may resemble:

```text
nutrient:n-total
nutrient:n-ammonium-as-n
nutrient:n-nitrate-as-n
nutrient:p-total-as-p
nutrient:k-total-as-k
```

Exact production vocabulary requires a separately reviewed registry/profile. Free-form human labels do not control accounting arithmetic.

## 4. Element/compound basis firewall

Common agricultural reporting can express the same underlying nutrient in different chemical-equivalent conventions.

For example:

```text
P
!= P2O5 equivalent

K
!= K2O equivalent
```

REGEN-020 MUST NOT silently add, subtract, or compare values across these bases.

Any conversion is a separately versioned Derived proposition with explicit stoichiometric basis and lineage.

## 5. Physical quantity basis remains explicit

Nutrient concentrations and nutrient stock quantities are different propositions.

```text
mg/kg concentration
!= total mg nutrient stock
```

Turning concentration into total stock requires a separately evidenced material mass/volume basis and explicit derivation.

Likewise:

```text
wet/as-received basis
!= dry-matter basis
```

No hidden moisture conversion is allowed.

## 6. PEF remains the measurement owner

Raw/derived environmental measurements remain PEF-owned.

REGEN-020 should reference qualified/shared-admitted PEF observations/products for:

- concentration measurements;
- stock measurements;
- flow measurements;
- environmental losses/discharges;
- relevant soil/water/process observations.

REGEN does not duplicate PEF `value`, `unit`, uncertainty, spatial, temporal, source, or lineage fields.

Computed nutrient products require the same REGEN-019 shared admission theorem as other regenerative domains.

## 7. Stock and flow are separate types

The first nutrient core should distinguish at least:

```rust
pub struct NutrientStock { ... }
pub struct NutrientFlow { ... }
```

A stock represents a nutrient quantity associated with one identified material/place/state snapshot.

A flow represents a bounded transfer/transformation over a declared event/interval between exact source/destination references.

```text
stock present
!= flow occurred
```

and:

```text
planned flow
!= observed flow
```

## 8. Suggested stock shape

Conceptually:

```rust
pub struct NutrientStock {
    pub stock_ref: String,
    pub material_subject_ref: String,
    pub state_snapshot_ref: String,
    pub species: NutrientSpeciesKey,
    pub quantity_evidence_ref: String,
    pub evidence_snapshot_ref: String,
}
```

The v1 core need not duplicate the scalar itself if doing so would create a second numerical truth beside PEF.

If later exact ledger quantities are introduced, they require an explicit normalization/derivation theorem analogous to REGEN-011B3.

## 9. Suggested flow shape

Conceptually:

```rust
pub struct NutrientFlow {
    pub flow_ref: String,
    pub source_stock_ref: String,
    pub destination_subject_ref: String,
    pub species: NutrientSpeciesKey,
    pub quantity_evidence_ref: String,
    pub flow_class: NutrientFlowClass,
    pub evidence_snapshot_ref: String,
}
```

Potential flow classes may include:

```rust
Recovered,
Transferred,
Applied,
Uptake,
ExportedInProduct,
ReturnedResidue,
Stored,
LostToWater,
LostToAtmosphere,
OtherLoss,
```

Exact vocabulary should remain bounded and reviewable.

## 10. Transformation does not conserve every named species

A process may transform one chemical form into another.

Therefore nutrient conservation cannot naively require:

```text
input nitrate-N = output nitrate-N
```

where chemistry legitimately changes species.

Element-level conservation may be evaluated across compatible element bases while species-level transformations remain explicit.

The model must never manufacture same-species conservation merely to close accounting.

## 11. Element balance keeps residuals visible

Where an element-level mass balance is claimed, it should have the shape:

```text
sum(element input)
=
sum(identified outputs)
+ identified environmental loss
+ stored/intermediate change
+ unresolved residual
```

with compatible bases and explicit uncertainty/derivation lineage.

```text
unmeasured nutrient
!= zero
```

## 12. Availability is not total content

Plant availability/bioavailability is a separate contextual proposition.

```text
total nutrient content
!= plant-available nutrient
```

and:

```text
plant available in context A
!= plant available in context B
```

Availability may depend on soil, pH, mineralogy, biology, moisture, temperature, timing, crop/species, amendment interactions, and analytical method.

The core must not derive one universal availability fraction.

## 13. Nutrient source class is explicit

The system should preserve source context without implying source quality.

Illustrative source classes may include:

```text
crop residue
animal manure
compost
biochar/co-compost
food-processing residual
municipal organic residual
mineral input
recovered wastewater nutrient
other recovered nutrient stream
```

A source class does not establish safety, legality, or agronomic suitability.

## 14. Safety and contamination remain independent

Nutrient concentration alone is never a sufficient material-safety theorem.

```text
high nutrient value
!= safe material
```

REGEN-016 contamination evidence/profile semantics remain separate.

Potential pathogen, contaminant, pharmaceutical, salinity, trace-element, or other hazards cannot be compensated by nutrient benefit.

## 15. Human-waste and wastewater-derived streams require stronger gates

Where nutrient recovery involves human sanitation/wastewater streams, the protocol must treat pathogen/hazard control, treatment evidence, regulation, handling, exposure, and intended-use restrictions as separately qualified prerequisites.

REGEN-020 deliberately provides **no home treatment recipe, sanitation procedure, temperature/time target, or application instruction**.

```text
nutrient recoverable
!= safe for food-soil use
```

## 16. Compost/biochar composition

REGEN-013/014 material lineage may carry nutrient evidence references, but:

```text
compost/biochar contains nutrient X
!= nutrient X is available
!= batch is suitable for crop/soil Y
```

REGEN-017 remains the contextual agronomic-suitability layer.

## 17. Crop uptake is not disappearance

A nutrient taken up by a crop changes stock location/form; it does not vanish from the system model.

A resilient settlement metabolism model should be able to follow nutrient movement through:

```text
soil/material
-> plant
-> harvested food/feed/biomass
-> local consumption/use
-> residues/by-products
-> recovery/return/export/loss
```

without pretending every loop closes locally.

## 18. Harvest exports stay visible

Locally grown food exported from the locality may also export nutrients.

```text
local production high
!= nutrient loop closed
```

A community can have excellent yields while mining local nutrient stocks.

REGEN resilience analysis must expose that distinction.

## 19. Atmospheric/water losses are first-class

Loss pathways must not disappear from a circularity calculation because they are inconvenient.

Nutrient loss/discharge evidence may include separately observed or modelled pathways to water/atmosphere/other sinks.

Reported/observed/derived/inferred/forecast/scenario classes remain distinguishable through PEF.

## 20. No universal nutrient circularity score

Do not collapse all species and flows into one number such as:

```text
nutrient circularity = 83%
```

unless a separate exact metric defines its species, boundaries, basis, exclusions, time period, losses, weighting, and uncertainty.

The default REGEN representation should expose plural stocks/flows and unresolved gaps.

## 21. Recovery efficiency is process/profile scoped

A recovery fraction is meaningful only under an exact process/material/species/basis/evidence context.

```text
recovery efficiency for N
!= recovery efficiency for P
```

and:

```text
process recovery
!= usable agronomic recovery
```

## 22. Storage is a state transition, not disappearance

Recovered nutrient stored for later use remains an inventory stock.

Losses during storage, if measured/modelled, remain explicit flows.

Inventory aging/currentness belongs to purpose-specific policies, not one universal TTL.

## 23. Application is not uptake

```text
nutrient applied
!= nutrient retained in soil
!= nutrient plant uptake
!= yield benefit
```

Each is a separate evidence proposition.

No REGEN core should infer downstream effect solely from material application.

## 24. Trial evidence remains separate

REGEN-015 field-trial semantics remain the route for empirical treatment/outcome evidence.

A nutrient-stock/flow model can describe what moved; it does not prove that the movement caused improved yield, soil health, or resilience.

## 25. Rights and regulation remain external

Availability of a nutrient-bearing material does not establish permission to collect, transport, process, sell, or apply it.

Relevant legal/rights/regulatory records remain external/prior assessments, referenced but not reinvented by the nutrient core.

## 26. Currentness and evidence snapshots

REGEN-019B remains the purpose-specific currentness layer.

Nutrient assessments should bind exact evidence snapshots so later laboratory results or changing inventories create new assessments rather than silently rewriting old ones.

## 27. Spatial boundary

Nutrient stock/flow claims may be spatially scoped, but geometry/containment remains REGEN-019A territory.

A regional concentration observation does not automatically establish the stock of one farm/plot/material batch.

## 28. Symthaea optimization boundary

Symthaea may later optimize nutrient-loop alternatives only after hard safety/ecological/legal/quality constraints are supplied.

It may compare, for example:

```text
import dependence
recovery options
storage losses
transport burden
energy demand
cost
soil outcomes
water-risk constraints
```

but must not transform unresolved or unsafe nutrient streams into eligible inputs merely because the model predicts high resilience benefit.

## 29. Resilience metrics

Useful resilience outputs should remain plural, for example:

```text
N import dependency
P import dependency
K import dependency
recoverable local stocks
qualified usable recovery
storage reserve duration
loss pathways
substitution options
critical process dependencies
```

not one weighted `nutrient resilience` score by default.

## 30. Locality is not a hard objective

```text
local nutrient source
!= preferable source
```

A local source that is unsafe, ecologically damaging, energy-intensive, rights-conflicted, or poorly suited can be worse than an external qualified source.

Locality is a preference/optimization dimension after hard gates.

## 31. First executable crate direction

Potential crate:

```text
crates/mycelix-regenerative-nutrients
```

Likely dependency-light boundary:

```text
mycelix-core-types
mycelix-regenerative-core
mycelix-regenerative-admission
```

with material-domain adapters to biomass/compost/biochar/water later rather than circular crate dependencies.

No Holochain, Symthaea, Marketplace, Finance, database, networking, or actuator runtime belongs in the initial accounting core.

## 32. Initial test campaign

A future executable implementation should test at least:

1. nutrient species/basis keys are bounded/canonical;
2. unlike species cannot be silently added;
3. P vs P2O5-equivalent basis cannot be silently combined;
4. K vs K2O-equivalent basis cannot be silently combined;
5. concentration evidence cannot masquerade as total stock;
6. wet/dry basis mismatch fails;
7. raw computed evidence without lineage fails through shared admission;
8. element-level balance keeps explicit residual;
9. unmeasured loss is not zero;
10. total nutrient cannot automatically become plant-available nutrient;
11. source class does not manufacture safety;
12. contaminant/pathogen unresolved state prevents positive use eligibility where required;
13. application does not imply uptake;
14. uptake does not imply yield effect;
15. harvested/exported nutrient remains visible;
16. storage remains a stock;
17. loss-to-water and loss-to-atmosphere remain distinct;
18. live/current evidence cannot silently rewrite a historical snapshot;
19. locality cannot override hard safety/ecological gates;
20. no nutrient record carries process-execution or physical-actuation authority.

## 33. ProductFrozen qualification target

Executable nutrient code should follow the established REGEN sequence:

```text
authored source
-> explicit pinned-toolchain preparation
-> machine-preserved source + lock capsule
-> exact-byte ProductFrozen promotion
-> exact-head qualification
-> Q001 machine-readable receipt
```

The qualification proposition remains accounting/provenance only.

## 34. Deliberate non-claims

REGEN-020 establishes no fertilizer recommendation, application rate, crop prescription, sanitation method, wastewater-treatment instruction, pathogen elimination, contamination safety, regulatory approval, soil suitability, treatment efficacy, carbon credit, market value, governance authority, process authority, or physical actuation.

Its proposition is narrow:

> nutrient species, stocks, flows, transformations, recovery, storage, exports, and losses remain explicit and evidence-bearing without collapsing presence into availability, safety, suitability, or benefit.
