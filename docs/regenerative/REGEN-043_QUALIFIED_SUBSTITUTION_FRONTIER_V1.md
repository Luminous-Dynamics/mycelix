# REGEN-043 — Qualified Substitution Frontier v1

Status: preregistration only.

This contract defines how Mycelix REGEN may reason about substitutions inside one declared essential-service and shock context without turning “an alternative exists” into “the service is resilient.”

It consumes the service denominator from REGEN-040, dependency closure from REGEN-041, and compound-shock state from REGEN-042.

## 1. Core theorem

```text
candidate alternative
+ exact service requirement
+ exact shock state
+ exact dependency context
+ quality/safety/ecology/rights gates
+ timing and duration feasibility
+ capacity and compatibility
= substitution candidate outcome
```

not:

```text
alternative exists
= qualified substitute
= activated substitute
= service restored
= resilient system
```

## 2. Substitution is service-relative

A thing does not become a substitute in the abstract.

A substitution statement must bind:

- the exact essential-service requirement;
- population/scope;
- horizon;
- shock/campaign state;
- source dependency being replaced or bypassed;
- expected service contribution;
- prerequisite dependencies;
- hard eligibility gates;
- timing and duration assumptions.

The same candidate may be suitable for one service requirement and unsuitable for another.

## 3. Candidate, qualified, activated and observed are distinct

The initial model must preserve at least four separate states:

```text
SubstitutionCandidate
QualifiedSubstitution
ActivatedSubstitution
ObservedSubstitutionOutcome
```

with the invariant:

```text
candidate
!= qualified
!= activated
!= successful outcome
```

No earlier state may be used as evidence for a later state.

## 4. No locality shortcut

Locality is one dependency property, not a hard-coded quality signal.

```text
local
!= available
!= independent
!= safe
!= sustainable
!= sufficient
!= timely
```

and:

```text
external
!= fragile
!= unavailable
!= undesirable
```

A local substitute that shares the failed grid, watershed, supplier, cloud service, road, repair technician, finance rail or authority dependency with the failed primary path may provide little or no additional shock resilience.

## 5. No nominal-capacity shortcut

Installed or nameplate capacity is not service-delivery capacity.

A substitution must distinguish:

```text
nominal capacity
available capacity under current state
usable capacity after hard gates
service-deliverable capacity over horizon
```

Constraints may include resource inventory, conversion efficiency, operating window, lead time, storage, maintenance state, operator availability and current competing commitments.

## 6. Hard-gate intersection

A candidate substitution must never gain eligibility because soft benefits outweigh a failed hard gate.

Conceptually:

```text
QualifiedSubstitution =
    Capability
    ∩ Quality
    ∩ Safety
    ∩ Ecology
    ∩ Rights/Authority
    ∩ Timing
    ∩ Compatibility
    ∩ CurrentState
```

Never union.

Economic benefit, locality, carbon benefit, convenience, model confidence, political preference or scarcity pressure cannot manufacture a missing hard gate.

## 7. Timing is first-class

A substitute that becomes available after the service failure window may still be useful for recovery, but it is not an immediate continuity substitute.

The model must keep distinct:

- detection time;
- decision/authorization time;
- mobilization time;
- installation/configuration time;
- warm-up/maturation time where relevant;
- delivery lead time;
- usable start time;
- usable duration;
- replenishment/recovery time.

Therefore:

```text
exists eventually
!= available in time
```

## 8. Stock, flow and renewal are distinct

Substitutions may depend on finite stocks, continuing flows, regenerating resources or combinations of these.

A finite reserve may bridge a disruption without closing the dependency permanently.

The model must preserve:

```text
reserve stock
continuing production/flow
renewal rate
replenishment dependency
```

rather than calling all four “capacity.”

## 9. No reserve double-spending

One reserve or alternative resource must not be simultaneously credited to incompatible service plans.

If the same stored biomass, water, fuel, food stock, battery reserve, vehicle, worker, laboratory, spare part or financial reserve appears in multiple substitution plans, the campaign must expose the competing claims.

```text
resource listed in two plans
!= resource available twice
```

## 10. Common-mode dependency firewall

A substitute must expose its transitive prerequisites so apparent diversity does not hide common-mode dependence.

Examples of dependency classes include:

- grid feeder or fuel supply;
- watershed/source;
- upstream material supplier;
- road/rail/port corridor;
- shared machine/controller;
- software/cloud/API dependency;
- communications network;
- payment/finance rail;
- laboratory/testing service;
- specialist operator or repair skill;
- legal/permit/authority dependency;
- storage or cold-chain facility.

Two substitutes sharing the same failed prerequisite do not constitute independent redundancy for that shock.

## 11. Substitution may create new dependencies

The evaluator must record dependencies introduced by the substitute itself.

For example, replacing one process with another may require different tooling, skills, testing, energy quality, storage, residual handling or maintenance.

The model must not compare only the removed dependency.

```text
primary dependency removed
+ larger hidden replacement dependency
!= closure improvement by definition
```

## 12. Compatibility is explicit

A substitution must identify compatibility with the consuming system or service interface.

Compatibility may include exact profile, connection/interface, material class, quality range, process expectations, storage/handling constraints, timing and recipient capability.

“Same category” is not enough.

## 13. Quantity and quality are separate

A substitute may meet quantity while failing quality, or vice versa.

Therefore:

```text
quantity sufficient
!= service sufficient
```

Quality may include domain-owned propositions such as water quality, feedstock contamination profile, food/nutritional suitability, amendment quality, energy characteristics, material grade or storage state.

REGEN-043 does not define those sciences; it references their authoritative assessments.

## 14. Ecology remains a hard boundary

Scarcity or emergency modeling must not silently turn ecologically unavailable resources into substitutes.

Where a resource is ecologically constrained, the applicable REGEN-025 obligation outcome remains a hard gate unless a separately legitimate emergency authority explicitly changes the applicable policy outside this model.

The model itself creates no such authority.

## 15. Rights and authority remain external propositions

Physical possession or technical feasibility does not imply a right to withdraw, harvest, transfer, operate, consume or redirect a resource.

Substitution candidates must preserve exact rights/authority references where the service path requires them.

```text
can technically use
!= may legitimately use
```

## 16. Safety and suitability do not inherit across substitutes

A candidate alternative does not inherit the safety or suitability profile of the primary resource or process merely because it serves a similar role.

Each substitute must satisfy the exact hard gates appropriate to its own identity and use context.

## 17. Partial substitution is first-class

A substitute need not replace 100% of the failed dependency to be useful.

The evaluator should represent contribution explicitly rather than collapsing to yes/no where the service model supports partial delivery.

This permits results such as:

```text
primary failed
substitute A covers bounded share for 48h
substitute B covers another non-overlapping share
residual service deficit remains explicit
```

without calling the service fully restored.

## 18. Layered substitution

A substitution may itself rely on another substitution.

The graph may therefore include chains, but cycles must be detected and must not manufacture closure.

```text
A substitutes for B
B substitutes for A
```

is not a complete dependency solution.

## 19. Substitution graph cycles

The deterministic model must reject or explicitly mark unresolved cyclic substitution plans where the cycle has no independently grounded resource/service contribution.

## 20. Sequencing matters

In compound shocks, substitute availability may depend on earlier decisions or stock consumption.

A substitution plan evaluated at campaign start may no longer be valid after another service consumes the required reserve.

Therefore every qualification is state-bound.

```text
qualified at t0
!= qualified at tN automatically
```

## 21. Evidence snapshot binding

Every qualified substitution result must bind the exact evidence/dependency snapshot it was evaluated against.

Changed inventory, quality, authority, equipment state, ecological state or shock state requires re-evaluation rather than ambient reuse of `qualified=true`.

## 22. Uncertainty remains visible

Unknown capacity, unknown lead time, unresolved rights, unresolved quality, uncertain compatibility or unresolved ecological state must remain unresolved.

The model must never replace unknown values with optimistic defaults solely to complete a substitution path.

## 23. Failure reasons are plural

A candidate may fail for more than one reason.

The first model should preserve a bounded set of machine-readable reason codes so reviewers can distinguish at least:

- capacity insufficient;
- quality gate failed;
- safety gate failed;
- ecology gate failed;
- rights/authority unresolved or failed;
- lead time too long;
- duration too short;
- incompatible interface/profile;
- dependency unavailable;
- shared failure domain;
- stock already committed;
- required skill unavailable;
- required repair/tooling unavailable;
- evidence stale/unresolved;
- cyclic substitution dependency.

## 24. Economics are separate from eligibility

Cost, price, financing need and economic loss may be recorded for planning, but they do not rewrite hard eligibility.

After hard qualification, economics may help compare feasible alternatives.

Before hard qualification, a lower-cost alternative remains merely lower-cost and unqualified.

## 25. Carbon/climate claims remain separate

A lower-carbon substitute is not automatically more resilient, safe, suitable or locally beneficial.

A resilient substitution outcome does not establish a carbon credit or climate claim.

Climate authority remains outside this contract.

## 26. Terra-Preta / regenerative-loop example boundary

A local biochar/compost/soil loop may provide useful substitution paths only where its prerequisites and service role are explicit.

For example, a locally produced soil amendment may reduce dependence on some imported inputs while still depending on:

- qualified biomass/feedstock;
- safe transformation/maturation;
- energy or process heat;
- operator skill;
- testing/quality verification;
- equipment/spares;
- suitable soil/crop/site context;
- water;
- ecological retention;
- residual handling.

REGEN-043 therefore prevents “locally produced amendment” from being counted as a closed resilience loop when those dependencies remain open.

## 27. External trade can itself be a resilience asset

The model must permit diversified external suppliers, reciprocal trade, regional mutual aid and geographically independent sources to improve resilience.

Local dependency closure is not autarky.

A robust service may intentionally combine:

```text
local baseline capability
+ local reserves
+ regional substitution
+ diversified external supply
+ repair/recovery capacity
```

## 28. Frontier, not universal winner

The output should be a plural substitution frontier rather than one universally best alternative.

Useful dimensions can include:

- service contribution;
- usable start time;
- duration;
- qualified capacity;
- recovery time;
- dependency independence;
- ecological status;
- rights status;
- safety/quality status;
- required skills/tooling;
- resource competition;
- cost/economic burden;
- evidence completeness.

The core does not assign universal weights.

## 29. Symthaea boundary

Symthaea may:

- discover candidate substitutions;
- traverse dependency graphs;
- compare scenarios;
- identify bottlenecks;
- estimate consequences with uncertainty;
- propose experiments or preparedness investments.

Symthaea may not:

- turn an unresolved candidate into a qualified substitute;
- waive a hard ecology/rights/safety/quality gate;
- create emergency authority;
- reserve or consume a real resource;
- actuate infrastructure.

## 30. Mycelix boundary

Mycelix may coordinate evidence, identity, policy references, rights/authority references, resource commitments, service requirements and reviewable decisions through the authoritative domains that own them.

REGEN-043 does not replace those domains with a new substitution authority.

## 31. Initial deterministic data model direction

A later dependency-light implementation may define structures conceptually equivalent to:

```text
SubstitutionCandidate
SubstitutionRequirement
SubstitutionDependency
HardGateRef
CandidateEvaluation
QualifiedSubstitution
SubstitutionActivationRef
ObservedSubstitutionOutcome
```

with opaque exact references into authoritative domain evidence.

It should not import Holochain, networking, marketplaces, Symthaea, device control or domain-specific scientific engines into the pure evaluator.

## 32. Initial deterministic evaluation order

The first executable evaluator should follow a stable order such as:

```text
identity/scope
-> service requirement binding
-> state/evidence snapshot binding
-> hard gate completeness
-> compatibility
-> capacity/basis
-> timing/duration
-> dependency availability
-> common-mode check
-> resource-commitment conflict
-> cycle check
-> outcome
```

Evaluation order must not allow a later soft benefit to undo an earlier hard failure.

## 33. Minimum synthetic campaign

A first executable campaign should include at least:

1. fully qualified independent substitute passes;
2. candidate with insufficient quantity fails;
3. quantity-sufficient but quality-failed candidate fails;
4. unresolved quality remains unresolved;
5. ecology failure remains ineligible despite severe service deficit;
6. rights failure remains ineligible despite locality;
7. local substitute sharing the failed grid is not independent;
8. two geographically distinct external suppliers remain distinguishable;
9. substitute available after deadline cannot satisfy immediate continuity;
10. short-duration reserve covers only part of horizon;
11. one reserve cannot be double-spent across two service plans;
12. replacement dependency introduces a new bottleneck;
13. incompatible process/interface fails;
14. stale evidence forces re-evaluation;
15. changed inventory invalidates prior capacity claim;
16. cyclic substitution path does not manufacture closure;
17. partial substitution preserves explicit residual deficit;
18. multiple partial substitutes can compose only without double-counting;
19. common operator/repair skill creates shared failure domain;
20. common storage facility creates shared failure domain;
21. qualified candidate is not treated as activated;
22. activated candidate is not treated as successful outcome;
23. observed positive outcome does not generalize to another service/context automatically;
24. lower cost cannot override a failed hard gate;
25. lower carbon cannot override a failed hard gate;
26. model confidence cannot create qualification;
27. local production alone cannot create qualification;
28. external origin alone cannot disqualify a candidate.

## 34. Relationship to REGEN-044

REGEN-043 exposes tooling, spare, operator and repair dependencies but does not model repair capability deeply.

REGEN-044 owns repair/skill dependency projection and should feed exact capability state back into substitution evaluation.

## 35. Relationship to REGEN-045/046

REGEN-043 determines which substitution paths are qualified candidates for one service/shock state.

Later continuity/outcome work determines what service was actually preserved.

```text
qualified substitution plan
!= continuity outcome
```

## 36. Relationship to REGEN-047

REGEN-047 should adversarially attack the substitution graph for hidden common modes, fake independence, duplicated reserves, stale evidence and correlated failures.

## 37. Deliberate non-claims

REGEN-043 creates no current claim of:

- community self-sufficiency;
- disaster readiness;
- service continuity;
- sustainability superiority;
- ecological sufficiency;
- property/right validity;
- safety or quality of any real material;
- adequacy of any real emergency reserve;
- best procurement choice;
- carbon benefit;
- economic optimality;
- emergency authority;
- physical execution.

Its proposition is deliberately narrow:

> a substitution should count toward regenerative resilience only when its exact service contribution, timing, capacity, compatibility, transitive dependencies and hard eligibility gates remain valid for the declared state and shock context.
