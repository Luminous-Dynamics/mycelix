# REGEN-041 — Local Dependency Closure v1

Status: preregistration only. This document freezes how REGEN represents local dependency closure for essential-service resilience. It does not declare autarky desirable, define a universal local-content target, create procurement authority, or authorize physical action.

## 1. Purpose

Local production can improve resilience when it removes fragile dependencies, shortens recovery paths, increases substitution options, or diversifies failure domains.

It can also create new fragility when local nodes share the same power, water, land, equipment, skill, supplier, transport, climate or governance dependency.

Therefore:

```text
more local production
!= more resilience by definition
```

The correct question is:

> for one declared essential service, over one declared horizon and shock set, which dependency chains remain satisfiable, substitutable or recoverable within the declared locality boundary?

## 2. Governing theorem

```text
essential-service requirement
+ explicit dependency graph
+ locality boundary
+ declared shock/failure set
+ time horizon
+ qualified substitutions
= inspectable dependency-closure result
```

not:

```text
local share of spend/production
= resilience
= sustainability
= self-sufficiency
```

## 3. Upstream denominator

REGEN-041 consumes REGEN-040 service revisions.

Dependency closure has no meaning without an exact service requirement.

```text
dependency graph exists
!= service continuity established
```

Every closure assessment binds:

- exact service revision;
- served population/scope;
- assessment horizon;
- locality boundary;
- shock/failure scenario;
- dependency graph revision;
- substitution profile revision;
- evidence snapshot.

## 4. Locality is explicit and plural

The core MUST NOT encode one universal definition of `local`.

A locality profile may distinguish scopes such as:

```text
OnSite
Neighborhood
Municipal
Regional
National
External
```

or another adopted jurisdiction/geographic/network topology.

The labels themselves create no preference or authority.

A dependency can be geographically local while operationally dependent on remote software, finance, fuel, spare parts, knowledge, network services or upstream materials.

## 5. Dependency-node ontology

Initial dependency classes may include:

```text
Resource
Feedstock
Water
Energy
LandOrSpace
Equipment
SparePart
Consumable
Storage
Transport
Skill
Labor
Information
SoftwareService
Communications
ExternalService
Supplier
InstitutionalPermission
FinanceOrPaymentRail
WasteOrResidualSink
```

Domain-specific systems remain authoritative for the underlying state.

REGEN-041 owns only dependency composition and closure analysis.

## 6. Dependency edge semantics

A dependency edge should bind at minimum:

```text
consumer_node
provider_node
dependency_kind
required_quantity_or_capability
quality/profile requirement
lead_time_or_latency where relevant
maximum interruption where relevant
substitutability class
failure_domain refs
evidence refs
```

An edge is not assumed satisfied merely because a provider exists.

## 7. Direct vs transitive closure

A local final producer may still depend on non-local upstream inputs.

Example pattern:

```text
local greenhouse
-> local irrigation pump
-> imported replacement controller
-> external firmware service
-> remote payment rail
```

Therefore:

```text
final production location
!= dependency closure location
```

REGEN-041 traces transitive dependencies until it reaches qualified terminal resources/capabilities, an explicit external dependency, a qualified substitute path, or an unresolved boundary.

## 8. Closure result is not one percentage

The core MUST NOT define a universal `local_closure_score = N/100`.

A closure result should preserve a vector/graph summary such as:

```text
service_state
locally_satisfied_critical_dependencies
externally_satisfied_critical_dependencies
qualified_substitutions
unresolved_dependencies
single_points_of_failure
shared_failure_domains
recovery_time_constraints
inventory/runway constraints
```

Presentation layers may summarize, but the graph and plural outcomes remain inspectable.

## 9. Hard vs soft dependencies

Dependencies should distinguish at least:

```text
HardRequired
Degradable
Optional
```

A failed optional dependency cannot be treated the same as a failed hard dependency.

A degradable dependency must bind the exact degraded service state it permits.

## 10. Failure-domain identity

Two suppliers are not necessarily redundant.

```text
two providers
!= two independent failure domains
```

Potential shared domains include:

- grid feeder;
- watershed/source;
- road/rail/port corridor;
- fuel supply;
- cloud/service provider;
- software package or update authority;
- upstream manufacturer;
- financial/payment infrastructure;
- single skilled operator/team;
- climate/weather exposure;
- legal/institutional permission;
- storage facility;
- common raw material.

Failure-domain references are first-class graph data.

## 11. Local redundancy firewall

Multiple local facilities sharing the same critical upstream dependency are not counted as independent resilience.

Examples:

```text
three local mills + one shared grid feeder
!= three independent milling paths

multiple local water tanks + one contaminated source
!= source redundancy
```

## 12. External redundancy firewall

External supply is not automatically fragile.

A diversified set of independent external providers with qualified inventories/substitution paths may survive a local shock better than one concentrated local source.

Therefore:

```text
external
!= single point of failure
```

## 13. Stock vs flow

Stored inventory and continuing production are different dependency strategies.

An assessment should distinguish:

```text
stock runway
replenishment flow
production capacity
recovery capacity
```

A finite reserve can close a dependency for horizon H1 while failing H2.

## 14. Runway

For stock-backed dependencies, closure must bind an explicit usable quantity and consumption/service-conversion assumption.

```text
inventory exists
!= sufficient runway
```

Unknown degradation, inaccessible stock or incompatible quality remains unresolved.

## 15. Lead time and recovery

Resilience is affected by how quickly a failed dependency can be replaced or repaired.

A substitution that arrives after the maximum tolerable interruption does not close the service dependency for that scenario.

```text
substitute exists eventually
!= continuity preserved
```

## 16. Substitution frontier

REGEN-041 records qualified alternative paths but does not decide every substitution tradeoff.

REGEN-043 owns deeper substitution-frontier analysis.

A substitute path must preserve required:

- safety/quality;
- ecological eligibility;
- rights/authority;
- process compatibility;
- timing/horizon constraints.

Locality or low cost cannot waive these.

## 17. Skills as dependencies

Equipment availability does not imply operational capability.

```text
equipment present
!= skill available
!= maintenance capability available
```

Critical skills may include operation, repair, inspection, calibration, agronomy, fabrication, logistics or software/system administration.

REGEN-044 may later model repair/skill depth in more detail.

## 18. Spare parts and tooling

A locally manufactured or operated system can still have remote closure if critical tooling, parts or consumables are externally unique.

Dependency analysis should distinguish:

```text
routine consumable
wear part
repairable component
specialized replacement
calibration/tooling dependency
```

No assumption is made that every component should be produced locally.

## 19. Energy dependency

Local food/water/material systems may depend on energy for pumping, processing, storage, communications or repair.

Energy dependency should bind the relevant essential-load service, not gross generation capacity.

REGEN composes the authoritative Energy domain rather than duplicating it.

## 20. Water dependency

Local agriculture or processing may remain dependent on external or fragile water sources.

REGEN-023/Water-domain quality, quantity, timing and allocation state remains authoritative.

```text
local land
!= locally closed water dependency
```

## 21. Ecological closure firewall

A dependency path is not closed if satisfying it requires violating adopted hard ecological obligations.

REGEN-025 remains upstream:

```text
physically obtainable locally
!= ecologically eligible locally
```

## 22. Rights/authority closure firewall

A resource can be local and technically usable while unavailable due to legitimate rights, custody, permit or governance constraints.

Need does not create authority.

An unresolved authority dependency remains unresolved rather than silently treated as accessible.

## 23. Economic/access dependency

Physical availability and practical access are distinct.

A critical dependency may fail because of:

- affordability;
- payment rail failure;
- contractual restriction;
- credit/finance dependence;
- supplier prioritization;
- market closure.

REGEN-041 records these as dependencies without inventing economic policy.

## 24. Information/software dependency

Modern local production may depend on remote digital systems.

Examples include:

- cloud control/monitoring;
- license servers;
- update/signing services;
- remote identity/authentication;
- proprietary diagnostics;
- remote documentation or knowledge services.

An offline/manual/local fallback may close some of these dependencies if separately qualified.

## 25. Communications dependency

Coordination, logistics and repair may depend on communications even when physical production is local.

A closure assessment should identify whether the declared service can continue during network partition and which functions degrade.

## 26. Institutional dependency

Some essential services depend on trusted organizations, shared rules, inspection, allocation, custody, conflict resolution or public infrastructure.

Institutional capacity is therefore a legitimate dependency class.

REGEN-041 does not reduce resilience to material self-sufficiency.

## 27. Circular-loop firewall

A circular material loop is not closed merely because outputs are reused locally.

The graph must still account for external:

- energy;
- replacement equipment;
- additives/consumables;
- testing;
- skills;
- transport;
- safe residual sinks;
- digital/institutional services.

For Terra-Preta-inspired loops:

```text
local biomass -> local biochar -> local soil
```

is only one visible segment of the dependency graph.

## 28. Dependency closure state

Each critical dependency path should resolve to an explicit state such as:

```text
ClosedWithinBoundary
ClosedByQualifiedSubstitution
ExternallyDependent
FailedForScenario
Unresolved
```

`Unresolved` never upgrades to `ClosedWithinBoundary`.

## 29. Service-level closure state

A service-level result is derived from its critical dependency paths and degradation rules.

Possible explicit states:

```text
ContinuitySatisfied
ContinuityDegraded
ContinuityFailed
ContinuityUnresolved
```

No numeric master resilience score is implied.

## 30. Shock-relative closure

Closure is scenario-dependent.

A local system may be closed against transport disruption but open against drought, grid failure or equipment loss.

Therefore every result binds a declared shock set.

```text
closed under scenario A
!= universally self-sufficient
```

REGEN-042 later owns compound-shock campaigns.

## 31. Horizon-relative closure

Closure is also horizon-dependent.

A reserve or local workaround may sustain one week but not one season.

All closure results bind an explicit horizon inherited from the service assessment.

## 32. Recovery vs continuity

A system that fails briefly and recovers is different from one that never loses service.

REGEN-041 should preserve both:

```text
continuity during shock
recovery path/time after failure
```

Later continuity campaigns may analyze these separately.

## 33. Common-mode adversarial cases

Future execution/model testing should include at least:

1. many local producers sharing one power source;
2. many local producers sharing one water source;
3. local production dependent on one imported spare;
4. local production dependent on one remote software service;
5. two external suppliers in independent failure domains;
6. nominal substitute failing quality eligibility;
7. substitute arriving too late for the service interruption limit;
8. inventory sufficient for H1 but not H2;
9. resource physically present but rights unresolved;
10. resource local but ecological floor violated;
11. equipment present but operator/repair skill absent;
12. circular material flow with an external energy bottleneck;
13. local system surviving transport shock but failing drought shock;
14. unresolved transitive dependency preventing a false closure claim.

## 34. Symthaea interface

Symthaea may consume the dependency graph to:

- identify critical cuts/bottlenecks;
- compare locality strategies;
- search qualified substitution paths;
- identify common-mode risk;
- compare horizons/shock scenarios;
- propose resilience experiments or investments.

Symthaea output remains analysis/recommendation only.

It cannot make an unqualified dependency available or create authority to use it.

## 35. Investment/procurement boundary

Closure analysis may reveal high-value local investments such as storage, repair capability, redundant water/energy, tools, skills or qualified local production.

But:

```text
resilience benefit identified
!= procurement authorized
!= investment approved
```

REGEN-062–065 later own infrastructure/economic coordination profiles.

## 36. Deliberate non-claims

REGEN-041 creates no claim that any community is self-sufficient, that local production is universally superior, that external trade is undesirable, that a particular service threshold is correct, that any resource may legally/ecologically be used, that any procurement should occur, or that any physical action is authorized.

Its proposition is narrow:

> local resilience should be evaluated as explicit, transitive, failure-domain-aware dependency closure for declared essential services, horizons and shocks—not as a slogan, locality percentage or universal self-sufficiency objective.
