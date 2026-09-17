# REGEN-040 — Essential Regenerative Services v1

Status: preregistration only. This document freezes how REGEN represents essential regenerative services before Phase-E resilience modeling begins. It does not declare any place self-sufficient, prescribe a universal minimum service level, create emergency authority, or authorize physical action.

## 1. Purpose

Resilience must be evaluated against services people actually need, not against inventories or production totals in isolation.

Core theorem:

```text
resource exists
!= usable service exists

local production exists
!= essential service continuity
```

A regenerative system becomes meaningfully resilient only when evidence can connect resources, processes, infrastructure, skills and institutions to a declared service delivered to a declared population over a declared time horizon.

## 2. Essential-service identity

Each service definition should bind at minimum:

```text
service_id
service_revision
service_class
served_population_or_scope
service_location_or_region
service_unit_or_vector
minimum_service_requirement
assessment_horizon
maximum_tolerable_interruption where adopted
quality/safety profile refs where applicable
authoritative-domain refs
```

A service revision is immutable. Changing the required population, horizon, quality profile or minimum service proposition creates a new revision or assessment context rather than silently rewriting prior evidence.

## 3. Service classes

The initial ontology may represent categories such as:

- food and nutritional service;
- potable/process water service;
- sanitation and safe residual handling;
- seed/propagation service;
- soil-fertility / soil-function maintenance;
- essential energy service;
- cold/storage preservation service;
- repair/maintenance capability;
- critical material/feedstock availability;
- local information/coordination capability.

These are service classes, not claims that every community must satisfy the same threshold or satisfy every service locally.

## 4. Service requirement != supply strategy

A requirement states what service is needed.

A supply strategy states how that service may be provided.

Therefore:

```text
essential service
!= local production requirement
!= autarky requirement
!= one preferred technology
```

A service may legitimately be provided through local production, regional exchange, stored reserves, mutual aid, redundant suppliers, substitution, repair, demand adaptation, or combinations of these.

Locality may improve resilience in some failure modes while worsening efficiency, ecological impact or common-mode exposure in others.

## 5. Authoritative-domain ownership

REGEN-040 does not create new authoritative water, energy, health, property, rights, finance, marketplace or governance state.

It references existing domain evidence and adopted policy/profile identifiers.

Examples:

```text
Water domain -> water source/quality/allocation authority
Energy domain -> energy infrastructure/state authority
Commons/Property -> rights/stewardship authority
Supply Chain / Marketplace -> external sourcing observations
Manufacturing/Craft -> production/repair capability observations
Identity/Praxis/Governance -> actor/role/policy authority
```

REGEN owns only the cross-domain service-assessment proposition.

## 6. Service delivery observation

Observed service delivery should remain distinct from nominal production capacity.

A service observation may bind:

```text
service_revision_ref
observation_window
quantity/vector delivered
quality-profile result ref
population/scope served
interruptions
losses
unresolved components
evidence refs
```

For food, REGEN-021/026 semantics remain authoritative for the distinction between production, edible material, storage, access and consumption.

For water, REGEN-023 composes the existing Water hApp rather than duplicating it.

## 7. Capacity != delivery

The following are separate propositions:

```text
installed capacity
available capacity
qualified capacity
scheduled capacity
actual service delivered
service outcome
```

A machine, field, reservoir, battery, seed bank, compost pile or biochar unit may exist while contributing zero usable service during the assessment horizon.

## 8. Minimum requirement

The core does not invent universal minima.

A minimum service requirement must point to an adopted profile, explicit local planning assumption or other identified authority/evidence source.

```text
planner preference
!= adopted minimum requirement
```

Symthaea may compare scenarios against a supplied requirement. It may not create the social/legal authority that makes that requirement binding.

## 9. Time horizon

Resilience is horizon-dependent.

A system can be robust for 24 hours and fragile for 30 days.

Every continuity assessment therefore binds an explicit horizon and, where relevant, temporal resolution.

```text
service adequate at horizon H1
!= service adequate at horizon H2
```

## 10. Population/scope binding

Service sufficiency must bind the exact population or service scope represented by the assessment.

A result for one household, block, facility, farm, district or municipality cannot silently become a result for another.

Population/scope uncertainty remains explicit.

## 11. Quality and quantity remain separate

More output is not automatically more service.

Examples:

```text
water volume != potable water service
harvest mass != nutritional service
stored seed mass != propagation service
energy generation != essential-load service
organic residual mass != qualified soil amendment
```

Hard quality/safety constraints are evaluated before soft optimization of quantity, locality, cost or carbon preference.

## 12. Continuity state

A service assessment should use explicit states rather than a single scalar:

```text
Satisfied
Degraded
Failed
Unresolved
```

`Unresolved` is not treated as `Satisfied`.

The exact meanings and thresholds are profile-bound.

## 13. Degraded service

Degraded operation is first-class. A system may preserve part of an essential service while shedding lower-priority demand.

A degradation result should preserve at least:

- service fraction/vector still deliverable;
- affected scope;
- duration;
- violated/unresolved requirements;
- dependency or bottleneck evidence;
- substitution/fallback refs where relevant.

No universal moral or political priority ordering is embedded in the core.

## 14. Service dependency interface

REGEN-040 exposes the denominator used by REGEN-041.

Each service revision may identify typed dependency requirements such as:

```text
Resource
Energy
Water
LandOrSpace
Equipment
SparePart
Consumable
Skill
Labor
Information
Transport
Storage
ExternalService
InstitutionalPermission
```

The dependency itself is not assumed local or external at this layer.

## 15. Failure domains

A service can depend on multiple nominally independent resources that actually share one failure domain.

Examples include shared grid feed, shared road corridor, shared pump, shared upstream water source, shared supplier, shared software service, shared fuel stream or shared skilled operator.

REGEN-040 therefore permits explicit failure-domain references but leaves graph closure/adversarial analysis to REGEN-041/042/047.

## 16. Substitution

A substitute is contextual and profile-bound.

```text
substitute resource exists
!= substitute is qualified for this service
```

Substitution must preserve required safety, quality, rights, ecology and process compatibility.

A cheaper/local substitute cannot bypass a hard eligibility rule.

## 17. Stored reserves

Stored reserves are a supply strategy, not a service by themselves.

A reserve contribution should preserve:

```text
inventory identity
usable quantity/vector
quality/currentness
access/custody
withdrawal constraints
loss/degradation model
service conversion assumption
```

Inventory cannot be counted simultaneously as unrestricted near-term consumption, seed reserve, process feedstock and emergency reserve unless explicit partitions establish those quantities.

## 18. Locality firewall

REGEN-040 adopts the following rule:

```text
local
!= safe
!= sustainable
!= sufficient
!= resilient
```

and also:

```text
external
!= fragile
```

The resilience value of locality is demonstrated through dependency/failure analysis, not assumed axiomatically.

## 19. Ecological firewall

Essential-service pressure does not automatically erase ecological obligations.

REGEN-025 hard ecological constraints remain distinct from continuity objectives.

Emergency policy may define exceptional authority elsewhere, but this service core does not create it.

## 20. Rights and legitimacy firewall

Resource/service necessity does not itself create rights to harvest, withdraw, process, occupy, transfer or operate.

Rights/custody/authority remain separate inputs.

## 21. Economics firewall

Price, affordability and financing may affect practical service availability, but:

```text
cheap != resilient
expensive != resilient
profitable != essential
essential != free entitlement encoded by this core
```

Economic policy remains outside the service-definition theorem.

## 22. Outcome vector, not master score

REGEN-040 MUST NOT define `resilience = N/100`.

A service portfolio should preserve plural results, for example:

```text
food_service_state
water_service_state
sanitation_service_state
energy_service_state
seed_service_state
soil_function_state
repair_capability_state
unresolved_dependencies
```

Aggregation for dashboards may be performed as presentation, but the underlying plural outcomes remain inspectable.

## 23. Symthaea boundary

Symthaea may:

- model service demand and capacity;
- identify bottlenecks;
- compare scenarios;
- estimate uncertainty;
- propose experiments or substitutions;
- rank qualified options against supplied objectives.

Symthaea may not convert model output into adopted service requirements, rights, emergency powers or physical execution authority.

## 24. Initial adversarial cases

A future executable/service-model campaign should include at least:

1. high production but inadequate usable service;
2. enough quantity but failed quality profile;
3. adequate one-day service but failed thirty-day service;
4. stored reserve double-counted for two purposes;
5. substitute exists but is not qualified;
6. local source fails ecological eligibility;
7. external source is redundant and survives the local shock;
8. multiple local sources share one failure domain;
9. one unresolved dependency prevents a false `Satisfied` result;
10. Symthaea recommendation cannot alter the adopted requirement;
11. service result for one population cannot be reused for another;
12. capacity without actual delivery cannot be reported as delivered service.

## 25. Deliberate non-claims

REGEN-040 creates no claim of current food, water, energy, soil, sanitation, seed or infrastructure sufficiency; no universal minimum standard; no declaration of self-sufficiency; no ecological exemption; no property/right; no emergency authority; no economic entitlement; no physical-action authority; and no guarantee that local production is preferable.

Its proposition is narrow:

> resilience analysis should start from explicit, evidence-bound essential services and preserve the distinction between service requirements, supply strategies, capacities, actual delivery and outcomes.
