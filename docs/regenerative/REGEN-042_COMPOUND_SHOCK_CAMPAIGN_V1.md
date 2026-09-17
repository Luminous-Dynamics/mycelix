# REGEN-042 — Compound Shock Campaign v1

Status: preregistration only. This document freezes the first Phase-E compound-shock campaign contract for regenerative-service resilience. It defines simulation/evidence semantics only. It does not predict real disaster probability, authorize emergency action, or create physical control authority.

## 1. Purpose

Single-failure resilience can hide systemic fragility.

A system that survives drought alone, grid loss alone, or transport disruption alone may still fail when stresses overlap, occur in sequence, share a common cause, or delay recovery.

Core theorem:

```text
explicit service requirements
+ exact dependency graph
+ preregistered shock timeline
+ deterministic transition rules
+ explicit degraded/failure outcomes
= reviewable compound-shock experiment
```

not:

```text
synthetic scenario survives
= real community is resilient
= future disaster outcome predicted
```

## 2. Upstream contracts

REGEN-042 consumes:

- REGEN-040 exact essential-service revisions;
- REGEN-041 exact dependency-closure graph/revision;
- relevant authoritative domain snapshots/evidence;
- explicit substitution/recovery assumptions available at the campaign start.

It does not silently invent missing service minima, dependencies, ecological permissions, rights or substitutions.

## 3. Campaign identity

Every campaign should bind at minimum:

```text
campaign_id
campaign_revision
service_revision_refs
dependency_graph_revision
baseline_snapshot_ref
shock_scenario_refs
simulation/model revision
randomness policy / seed set if used
assessment horizon
time resolution
endpoint set
acceptance/non-claim statement
```

Changing any material element creates a new campaign revision rather than rewriting historical results.

## 4. Baseline first

The campaign begins with a no-shock or declared-reference baseline using the same service/dependency semantics.

```text
baseline failure
=> shock interpretation blocked or explicitly limited
```

A broken baseline cannot be used to infer shock sensitivity.

## 5. Shock event model

A shock event should identify:

```text
shock_id
shock_class
target dependency/failure-domain refs
start time
duration or recovery rule
severity/profile ref
state transition semantics
source/evidence or synthetic-fixture marker
```

The core does not infer disaster probability from the existence of a fixture.

## 6. Initial shock classes

The first synthetic campaign should be capable of representing at least:

- drought / water-availability stress;
- grid or essential-energy interruption;
- transport/logistics interruption;
- critical equipment loss;
- crop/biological production stress;
- storage/cold-chain loss;
- communications/network partition;
- critical labor/skill unavailability;
- upstream supplier interruption;
- payment/market-access interruption where relevant.

These are scenario classes, not claims about local hazard frequency.

## 7. Compound means temporal composition

The campaign must preserve event order and overlap.

```text
A then B
!= B then A
!= A and B simultaneously
```

because inventories, recovery state, crop stage, maintenance backlog and substitution availability may differ.

## 8. Common-cause vs independent shocks

Two failed dependencies are not automatically two independent shocks.

A campaign should identify whether failures arise from:

```text
IndependentEvent
SharedFailureDomain
CascadingFailure
DemandShock
RecoveryFailure
UnresolvedCause
```

This prevents counting one upstream event as multiple independent stressors.

## 9. Demand shocks

Resilience can fail through demand increase even when supply infrastructure remains intact.

Campaigns may represent bounded demand changes separately from supply loss.

```text
service deficit
!= supply failure necessarily
```

## 10. State carried across events

Shock events operate on evolving state rather than independent static snapshots.

State may include:

- usable inventory/runway;
- current service delivery state;
- degraded equipment state;
- recovery/repair progress;
- seasonal or biological state;
- qualified substitution availability;
- unresolved dependencies;
- storage losses;
- resource depletion.

A later event sees the state left by earlier events.

## 11. Service outcomes

Each declared REGEN-040 service should preserve an explicit trajectory such as:

```text
Satisfied
Degraded
Failed
Unresolved
```

with time intervals, affected scope and reasons.

No master resilience score replaces the service trajectories.

## 12. Dependency outcomes

REGEN-041 dependency states remain inspectable throughout the campaign:

```text
ClosedWithinBoundary
ClosedByQualifiedSubstitution
ExternallyDependent
FailedForScenario
Unresolved
```

A service failure should be traceable to the dependency cuts that produced it where the model permits.

## 13. Degradation before failure

The campaign must not force binary up/down semantics where partial service is meaningful.

Examples include reduced irrigation coverage, reduced cold-storage capacity, lower processing throughput, reduced nutritional-service coverage or lower essential-energy service.

Degradation rules must come from the adopted service/dependency profiles rather than arbitrary post hoc thresholds.

## 14. Recovery is first-class

For each failed/degraded dependency, the model should distinguish:

```text
continuity during shock
recovery initiation
recovery prerequisites
recovery duration
post-recovery service state
```

Fast recovery is different from uninterrupted continuity.

## 15. Recovery resources are dependencies

Repair/recovery may itself require:

- spare parts;
- tools;
- energy;
- transport;
- skilled labor;
- diagnostics/information;
- permissions;
- communications;
- external suppliers.

The campaign must not assume recovery capacity appears for free.

REGEN-044 later deepens the skill/repair projection.

## 16. Inventory depletion

Stocks are consumed over simulated time according to explicit service-conversion assumptions.

A campaign must not reset inventory between overlapping shocks unless the scenario explicitly models replenishment.

```text
reserve survives shock A
!= same full reserve available for shock B
```

## 17. Seasonal/biological state

Agricultural shocks may depend strongly on timing.

A drought or equipment failure during one crop stage may differ from the same duration at another.

Where the model lacks adequate biological fidelity, the result remains a scenario sensitivity test rather than an agronomic prediction.

## 18. Environmental constraints remain hard

Shock conditions do not automatically relax REGEN-025 ecological obligations.

A scenario may separately test an explicitly adopted emergency-policy profile, but the base resilience model does not manufacture environmental exemptions.

## 19. Rights/authority remain hard

A shock does not create ownership, withdrawal, processing or operating authority.

If a fallback requires unresolved authority, that fallback remains unresolved in the base campaign.

## 20. Quality/safety remain hard

A substitute source that cannot satisfy required safety/quality profiles is not counted as successful continuity.

```text
physically available substitute
!= qualified substitute
```

## 21. Deterministic baseline before adaptive policy

The first campaign uses deterministic transition/dispatch rules wherever practical.

Symthaea or learned/HDC policy should initially run in shadow/recommendation mode against the same frozen fixtures.

```text
adaptive policy improves synthetic result
!= authority to operate real infrastructure
```

## 22. Shock matrix

The first campaign family should include at least:

1. drought only;
2. grid interruption only;
3. transport interruption only;
4. critical equipment loss only;
5. crop stress only;
6. drought + grid overlap;
7. drought + transport overlap;
8. grid + equipment overlap;
9. transport + spare-part dependence;
10. crop stress followed by storage loss;
11. grid loss followed by delayed transport recovery;
12. shared water-source failure affecting multiple local producers;
13. common-grid failure affecting multiple local processors;
14. local shock while independent external supplier remains available;
15. external transport shock while local qualified reserve remains available;
16. multi-shock case with one unresolved dependency preventing a false PASS.

## 23. Ordering campaign

At least one scenario family should permute the order of the same two or three events to expose path dependence.

For example:

```text
transport -> equipment
vs
equipment -> transport
```

may differ because spare parts can be prepositioned in one sequence and inaccessible in the other.

## 24. Duration campaign

At least one scenario family should hold shock class constant while varying duration/horizon.

This reveals threshold behavior in stocks, crop stages, storage and recovery capacity without turning the threshold into a universal prediction.

## 25. Common-mode campaign handoff

REGEN-042 identifies shared failure domains, but REGEN-047 later owns the dedicated adversarial common-mode program.

REGEN-042 should preserve enough provenance to replay any discovered common-mode weakness there.

## 26. Substitution handoff

REGEN-043 later explores the substitution frontier in more depth.

REGEN-042 only consumes substitutions that are explicitly available/qualified in the frozen scenario state.

It does not invent a substitute after observing failure merely to rescue the result.

## 27. No post hoc rescue

Once a campaign is frozen, new resources, substitutions, rules or thresholds cannot be added to make a failed scenario pass.

A changed assumption creates a new campaign revision.

## 28. Null/adverse outcomes

A campaign in which local production performs worse, a fallback fails, or external diversity outperforms local closure is valid evidence.

Negative results are preserved rather than reframed as implementation noise without evidence.

## 29. Comparative claims

The campaign may compare named architectures/scenarios under the exact same frozen fixtures and endpoints.

It must not generalize:

```text
architecture A outperformed B in fixture set X
```

into:

```text
A is universally more resilient than B
```

## 30. Metrics remain plural

Potential reported quantities include:

- service-hours satisfied/degraded/failed;
- population/scope affected;
- unmet service quantity/vector;
- inventory runway;
- recovery time;
- number/identity of critical dependency cuts;
- substitution activations;
- unresolved dependency count/identity;
- ecological/right/quality gates encountered;
- external vs local dependency usage by exact graph path.

No weighted master score is normative.

## 31. Probability firewall

Synthetic campaign frequency is not hazard probability.

Running 100 drought fixtures and observing 40 failures does not establish a 40% real-world drought failure probability unless a separate statistically justified hazard/sampling model supports that interpretation.

## 32. Scenario severity firewall

A severity label such as `mild`, `severe` or `extreme` must resolve to an explicit profile/fixture definition.

Labels alone have no quantitative meaning.

## 33. Model-fidelity declaration

Each campaign receipt/report should declare what dynamics are represented and omitted.

Examples:

```text
mass accounting represented
network topology represented
repair lead time represented
crop physiology simplified
market behavior omitted
human adaptation simplified
```

Missing fidelity becomes a non-claim, not hidden certainty.

## 34. Independent oracle target

Where practical, mass/runway/dependency-state transitions should have an implementation-independent oracle or independently specified expected vectors.

The simulation should not be considered validated merely because it agrees with itself.

## 35. Reproducibility

A campaign receipt should bind:

- exact code/model revision;
- exact fixture digests;
- exact service/dependency profile revisions;
- exact seeds where randomness exists;
- toolchain/runtime identity;
- endpoint definitions;
- output/evidence digests.

A changed model or fixture starts a new evidence lineage.

## 36. Symthaea role

Symthaea may use frozen campaign results to:

- discover bottlenecks;
- compare strategies;
- propose substitutions;
- propose storage/repair/infrastructure investments;
- select informative future experiments;
- explain failure chains.

Symthaea cannot change the frozen campaign while evaluating it and cannot convert a recommendation into authority or execution.

## 37. Human/institutional adaptation

Human adaptation can be represented when explicit rules/evidence exist, but it must not be an unlimited catch-all rescue variable.

Unmodeled improvisation remains outside the demonstrated theorem.

## 38. First exit criterion

REGEN-042 v1 is ready to feed later Phase-E work only when a frozen synthetic campaign can:

1. reproduce the baseline;
2. execute the declared single and compound shocks deterministically at the stated fidelity;
3. preserve path/order dependence;
4. trace service degradation/failure to dependency state;
5. preserve hard ecology/rights/quality constraints;
6. preserve unresolved states;
7. preserve adverse/null outcomes;
8. emit replayable evidence without claiming real-world hazard probability.

## 39. Deliberate non-claims

REGEN-042 establishes no real disaster forecast, no hazard probability, no guaranteed service continuity, no universal shock severity mapping, no emergency policy, no ecological/right override, no procurement priority, no proof of real community resilience, and no physical-action authority.

Its proposition is narrow:

> regenerative resilience should be challenged with preregistered, temporally composed, failure-domain-aware shock campaigns whose service/dependency outcomes remain explicit, replayable and bounded by the model's declared fidelity.
