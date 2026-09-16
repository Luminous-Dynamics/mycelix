# REGEN-023 — Regenerative Water Bridge Preregistration v1

Status: preregistration only. This document defines the evidence/coordination bridge between REGEN and the existing `mycelix-water` domain. It creates no irrigation controller, pump command, water-right adjudication, potable-water certification, treatment recipe, or physical actuation.

## 1. Purpose

Regenerative agriculture, soil restoration, compost/biochar systems, food production, seed propagation, and nutrient loops all depend on water. But REGEN should not create a second water system.

The current repository already contains `mycelix-workspace/mycelix-water` with dedicated `capture`, `flow`, `purity`, `steward`, `wisdom`, and `bridge` zomes.

REGEN-023 therefore freezes a one-way ownership rule:

```text
mycelix-water owns authoritative water-domain state/coordination
REGEN binds exact water evidence/state into regenerative propositions
```

not:

```text
REGEN invents parallel water quality/allocation/stewardship/control state
```

## 2. Water source identity is not water suitability

A source reference answers which water source/state record is being discussed.

It does not by itself establish:

```text
quantity available
quality suitable
right to withdraw
right to store
right to transfer
right to apply
currentness
sustainability
```

Each remains a separate proposition.

## 3. Quantity, quality, timing, rights, and ecology are distinct

REGEN must not collapse water into one `available=true` field.

At minimum, consequential regenerative use may depend on separately resolved:

```text
source identity
physical quantity/flow
quality/use suitability
temporal availability/storage
withdrawal/allocation right
delivery capability
ecological/depletion constraint
currentness/evidence snapshot
```

A positive result in one dimension does not manufacture another.

## 4. Capture is not usable supply

Water captured or collected is not automatically a usable regenerative input.

```text
captured
!= stored
!= available
!= suitable quality
!= delivered
!= applied
```

REGEN should preserve the exact water-domain references supporting each transition rather than infer downstream service from a capture event alone.

## 5. Flow is not delivery outcome

A measured or scheduled flow does not necessarily establish that the intended destination received the intended usable volume.

```text
source flow
!= destination delivery
!= crop/root-zone availability
```

If later systems model delivery efficiency or losses, those are separately evidenced/derived propositions.

## 6. Purity/quality is purpose-specific

A water-quality observation or conformance decision is meaningful only for an exact intended-use/profile context.

```text
quality acceptable for use A
!= quality acceptable for use B
```

REGEN-023 defines no universal irrigation-water, potable-water, composting-water, livestock-water, processing-water, or ecological-water threshold.

Quality profiles remain exact adopted references rather than hard-coded global limits.

## 7. Potable suitability is not irrigation suitability and vice versa

The bridge explicitly prevents cross-purpose promotion:

```text
potable profile PASS
!= agronomic suitability automatically

irrigation-use profile PASS
!= potable safety
```

Different use cases can care about different chemical, biological, salinity, nutrient, contaminant, infrastructure, or regulatory dimensions.

## 8. Water quality evidence remains PEF-owned where appropriate

Environmental measurements remain PEF-owned and may enter REGEN through qualified shared admission.

The bridge should reference exact observations/products rather than copy:

```text
value
unit
uncertainty
spatial support
temporal support
source/provenance
```

into a second REGEN water-measurement schema.

## 9. Existing Water domain remains authoritative

Where `mycelix-water` already owns capture, flow, purity, stewardship, wisdom, or bridge records, REGEN should consume exact external references/adapters.

REGEN does not mutate those authoritative records into its own canonical truth.

A future adapter may expose a narrow projection such as:

```rust
pub struct RegenerativeWaterUseBinding {
    pub water_source_ref: String,
    pub water_state_ref: String,
    pub intended_use_profile_ref: String,
    pub evidence_snapshot_ref: String,
}
```

but the fields remain references to authoritative/prior evidence, not duplicated water-state values.

## 10. Water rights remain separate from physical availability

```text
water physically present
!= lawful/legitimate withdrawal right
```

and:

```text
withdrawal right
!= physical delivery capacity
```

REGEN does not adjudicate statutory, riparian, customary, indigenous, municipal, contractual, or other water rights.

It may preserve exact rights/allocation assessment references where required.

## 11. Stewardship is not title

Water stewardship/governance roles do not automatically imply ownership or unrestricted extraction authority.

Likewise, a governance approval does not manufacture physical water or acceptable quality.

Social authority and physical/environmental state remain separate.

## 12. Sustainable withdrawal is not source volume

A source may contain a large physical volume while only a smaller or zero additional withdrawal is ecologically acceptable.

```text
water present
!= sustainably withdrawable water
```

The ecological/depletion proposition may depend on watershed, groundwater, environmental-flow, seasonal, habitat, recharge, drought, legal, and other adopted constraints.

REGEN-023 defines no universal withdrawal fraction.

## 13. Storage is not replenishment

Stored water can improve short-term resilience, but:

```text
large storage
!= sustainable source
```

and:

```text
storage capacity
!= stored usable inventory
```

Inventory, capacity, quality, losses, currentness, and replenishment remain separately evidenced.

## 14. Groundwater stock is not renewable supply

Where groundwater applies:

```text
aquifer/storage estimate
!= sustainable recharge rate
!= sustainable withdrawal rate
```

A resilience model must not convert a large stock estimate into indefinite renewable supply.

## 15. Rainfall forecast is not captured inventory

```text
Forecast rainfall
!= captured water
!= stored inventory
```

Forecast/Scenario evidence remains its PEF class and can support planning only under exact policies.

## 16. Irrigation demand is not irrigation authority

A crop/soil model may estimate water demand or recommend irrigation.

That creates no valve/pump command or water-use authority.

```text
modelled demand
!= allocation right
!= available water
!= actuation command
```

Physical control remains outside REGEN's dependency-light evidence layer and would require a separately assured consequential-action boundary.

## 17. Application is not uptake

```text
water delivered/applied
!= soil retained water
!= plant-available water
!= plant uptake
!= yield benefit
```

Each step requires its own evidence/model proposition.

REGEN-015 remains the causal field-trial boundary for intervention/outcome claims.

## 18. Soil-water evidence stays in the soil/evidence model

Soil moisture, water retention, infiltration, drainage, and related measurements remain environmental/soil evidence rather than becoming fields owned by the Water bridge.

The bridge links water supply/use context to soil/field evidence without duplicating it.

## 19. Salinity/nutrient interactions remain contextual

A water source may carry salts or nutrients relevant to soil/crop systems.

```text
nutrient-bearing irrigation water
!= beneficial fertilizer input automatically
```

and:

```text
water available
!= salinity impact acceptable
```

REGEN-020 nutrient accounting and REGEN-017 contextual suitability may consume the relevant evidence without REGEN-023 inventing universal thresholds.

## 20. Reuse/recycled water requires explicit gates

Reused/recycled water can improve resilience, but source history and treatment context matter.

```text
reclaimed water exists
!= safe/suitable for intended use
```

The bridge preserves exact treatment/quality/profile/regulatory evidence references without providing treatment instructions.

## 21. Wastewater-derived water and nutrients stay separately qualified

Water reuse and nutrient recovery may share one physical stream, but:

```text
water-use suitability
!= nutrient-use suitability
```

REGEN-020 nutrient semantics and REGEN-023 water-use semantics remain separate even when they reference the same source stream.

## 22. Losses remain visible

A water accounting chain should be able to preserve, where evidenced:

```text
source withdrawal/capture
-> storage
-> conveyance
-> delivery
-> application/use
-> return/reuse
-> discharge/loss
```

Unmeasured loss is not zero.

## 23. Conservation claims require compatible basis

Water quantity may be expressed as volume, mass, rate, or integrated flow.

These cannot be silently added or compared without an explicit compatible basis/derivation.

A flow rate by itself is not a volume unless an exact time integration proposition exists.

## 24. Currentness is use-specific

REGEN-019B applies to water quantity, quality, rights/allocation, storage state, infrastructure availability, and ecological/depletion evidence separately.

A current quantity measurement with stale quality evidence does not become a current suitable-water theorem.

## 25. Spatial relation remains explicit

REGEN-019A owns exact spatial/geometry relations where needed.

```text
water-quality sample near source
!= source-wide representativeness
```

and:

```text
regional rainfall observation
!= exact field supply
```

## 26. Specimen/sample relation remains explicit

Where quality results rely on physical samples, REGEN-018 chain-of-custody/sample lineage remains separate.

```text
valid lab result
!= sample represents entire source/current state
```

## 27. Water infrastructure state is separate

A physically available source may still be unusable because critical infrastructure is unavailable.

Relevant dependencies can include, where applicable:

```text
pump
energy
storage
conveyance
filtration/treatment
controls
spares
maintenance
qualified operators
```

REGEN should model dependency state without taking over equipment control.

## 28. Energy-water coupling stays explicit

Water resilience may depend on energy, and energy systems may depend on water.

The system should expose cross-dependencies rather than calculate independent water and energy resilience scores and assume they can both be achieved simultaneously.

## 29. Black-start/local continuity matters

For essential water services, resilience analysis should distinguish normal service from degraded/local recovery modes.

A community may have water inventory but no ability to access/distribute it under grid/transport/equipment failure.

The later continuity layer should model that capability graph explicitly.

## 30. Ecological water is not leftover water

Environmental flows, habitat needs, wetland/watershed function, or other adopted ecological water constraints must not be treated merely as whatever remains after human demand.

Hard ecological constraints are not optimizer leftovers.

## 31. No weighted compensation of hard water constraints

```text
quality FAIL
+ high local resilience value
!= suitable
```

```text
ecological withdrawal constraint FAIL
+ high crop value
!= eligible withdrawal
```

```text
rights unresolved
+ abundant source
!= authorized use
```

## 32. No localism override

```text
local water source
!= preferable by definition
```

A local source can be contaminated, ecologically overdrawn, legally constrained, energy-intensive, or unreliable.

Locality is an optimization/preference dimension only after hard gates.

## 33. Water-service vector remains plural

Default resilience outputs should preserve dimensions such as:

```text
qualified source diversity
usable inventory
storage duration
replenishment dependency
quality-profile status
energy/infrastructure dependency
ecological withdrawal margin
rights/allocation state
transport/conveyance dependency
recovery options
```

Do not collapse them into one canonical `water resilience score` by default.

## 34. Regenerative-use binding is purpose-scoped

A regenerative application should bind exact intended purpose, for example a soil/crop/compost/processing context reference, rather than mint one universal `water_suitable=true` flag.

One positive binding cannot be reused automatically for every future purpose.

## 35. Existing Water bridge should remain the integration seam

The existing `mycelix-water/zomes/bridge` is the natural authoritative-domain integration seam to audit for later adapter work.

REGEN-023 itself does not modify that zome.

The first implementation should prefer a small adapter/projection layer over changes to the Water hApp unless a missing authoritative-domain primitive is demonstrated.

## 36. Symthaea boundary

Symthaea may later optimize qualified regenerative alternatives across water demand, storage, reuse, energy, cost, crop service, soil outcomes, ecological constraints, and disruption recovery.

It must consume hard water-quality, rights, ecological, and infrastructure eligibility as constraints rather than inventing authority or overriding them with model utility.

## 37. First implementation direction

Prefer a narrow bridge crate/module rather than another water store:

```text
crates/mycelix-regenerative-water-bridge
```

or an adapter adjacent to the existing REGEN domain crates.

It should depend on qualified shared evidence primitives and exact water-domain adapter interfaces, not on Holochain/runtime internals in the dependency-light decision model.

## 38. Initial regression campaign

A future implementation should test at least:

1. source identity does not imply quantity;
2. quantity does not imply quality;
3. quality for use A does not imply suitability for use B;
4. potable conformance does not automatically imply irrigation suitability;
5. irrigation-use conformance does not imply potable safety;
6. captured water does not become stored inventory automatically;
7. source flow does not prove destination delivery;
8. flow rate cannot masquerade as integrated volume;
9. forecast rainfall cannot mint observed inventory;
10. groundwater stock cannot become sustainable recharge/withdrawal automatically;
11. storage capacity cannot become stored inventory;
12. current quantity + stale quality cannot produce current suitability;
13. physical availability cannot produce withdrawal rights;
14. rights cannot produce physical availability;
15. ecological withdrawal failure cannot be overridden by economic/crop/resilience value;
16. local source cannot override safety/ecology gates;
17. reclaimed-water source cannot imply intended-use suitability;
18. water-use suitability cannot imply nutrient-use suitability;
19. valid sample does not imply source-wide representativeness;
20. delivered/applied water cannot imply uptake/yield effect;
21. infrastructure unavailable blocks service without changing source existence;
22. no bridge record carries pump/valve/process execution authority.

## 39. Qualification target

Executable bridge code should follow REGEN-008/008A:

```text
authored adapter/core
-> explicit pinned-toolchain preparation
-> machine-preserved source + lock
-> exact-byte ProductFrozen promotion
-> exact-head qualification
-> Q001 machine-readable receipt validation
```

Its qualification proposition must remain about reference/evidence composition, not real water safety or rights validity.

## 40. Deliberate non-claims

REGEN-023 establishes no water-right determination, withdrawal permit, potable certification, irrigation recommendation, treatment procedure, disinfection instruction, pumping schedule, irrigation amount, crop prescription, environmental-flow threshold, infrastructure safety certification, process authority, or physical actuation.

Its proposition is narrow:

> regenerative systems consume exact water-domain/evidence references for source, quantity, quality, timing, rights, ecological constraints, and service dependencies without duplicating the authoritative Water subsystem or collapsing those propositions into one availability bit.
