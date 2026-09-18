# REGEN-042A — Shock Transition Algebra and Counterfactual Comparability v1

Status: preregistration hardening only

Parent: REGEN-042 compound-shock campaign v1

Program: Luminous-Dynamics/mycelix#940

## Purpose

REGEN-042 correctly freezes temporally composed compound-shock campaigns, but an executable campaign still needs a precise transition algebra. Without one, two implementations can consume the same event timeline and disagree about simultaneous events, no-op events, capacity reductions, recovery contention, horizon truncation, or comparison validity while each still claims to implement the same campaign.

REGEN-042A closes that ambiguity without changing REGEN-042's authority or real-world non-claims.

Its proposition is narrow:

```text
frozen baseline state
+ typed shock effects
+ canonical event ordering / conflict semantics
+ explicit transition invariants
+ explicit recovery contention
+ exact comparison identity
= replayable compound-shock state transition experiment
```

not:

```text
replayable experiment
= real disaster forecast
= real hazard probability
= operational emergency plan
= authority
```

## 1. State transition is the normative primitive

A campaign advances one immutable prior state into one successor state.

Conceptually:

```text
S[t+1] = ApplyInterval(
    prior_state = S[t],
    events = E[t],
    recovery_actions = R[t],
    demand = D[t],
    exogenous_inputs = X[t],
)
```

The transition emits a receipt binding the exact prior state, exact event set, exact ruleset, and exact successor state.

A result record must never rewrite the prior state in place as if history had always contained the later shock.

## 2. Append-only causal history

Every state transition should bind:

```text
prior_state_ref
transition_ref
event_refs
ruleset_ref
successor_state_ref
```

Normative:

```text
successor state may extend history
successor state may not rewrite history
```

Correcting an input creates a new lineage rather than mutating an already-used campaign receipt.

## 3. No-op identity

The executable campaign needs an identity law.

For a shock event whose declared effect is exactly zero under the frozen effect profile:

```text
Apply(S, NoOp) = S
```

apart from explicitly declared bookkeeping such as recording that the no-op fixture was evaluated.

Likewise, inserting a zero-duration event with no impulse semantics must not change physical/service state.

This fixture catches implementations that accidentally trigger fallback, recovery, inventory consumption, timers, or service degradation merely because an event object exists.

## 4. Typed effect algebra

A shock label is not an effect.

The first executable campaign should reduce named scenarios into typed effects such as:

```text
DependencyUnavailable { dependency_id }
CapacityScaled { dependency_id, factor }
CapacityCapped { dependency_id, maximum }
DemandScaled { service_id, factor }
LeadTimeExtended { dependency_id, delta }
InventoryLoss { stock_id, quantity }
QualityStateChanged { source_id, new_state_ref }
FailureDomainUnavailable { failure_domain_ref }
RecoveryResourceUnavailable { resource_id }
```

The exact enum may evolve, but effects with materially different semantics must not be encoded through one overloaded scalar `severity` field.

## 5. Severity labels are aliases only

A label such as `mild`, `severe`, or `extreme` may identify a frozen profile, but the transition kernel consumes the resolved typed effects, not the adjective.

```text
severity label
-> exact profile reference
-> exact typed effects
```

A label without a resolvable profile is not executable campaign input.

## 6. Dimensional compatibility

An effect may target only a compatible state dimension.

Examples:

- capacity scaling targets a capacity-bearing dependency or service;
- inventory loss targets an identified stock on the same commodity basis;
- lead-time extension targets a provision/recovery path with time semantics;
- demand scaling targets a service-demand state;
- failure-domain outage targets dependencies explicitly bound to that domain.

A drought fixture cannot directly mutate electrical capacity merely because the scenario author wants correlated stress. The causal edge must be represented explicitly.

## 7. Exogenous shock != derived consequence

The campaign must distinguish what was injected from what the model derived.

```text
ExogenousEvent
StateTransition
DerivedDependencyFailure
DerivedServiceDeficit
DerivedRecoveryDelay
```

A service deficit caused by an injected grid loss must not be logged as a second independent shock.

This prevents causal double counting and misleading statements such as “two shocks occurred” when one was only the consequence of another.

## 8. Simultaneous-event semantics

Events with the same effective timestamp need deterministic semantics.

The campaign may not rely on container iteration order, hash-map order, thread scheduling, or source-file ordering.

A same-time event set is first canonicalized by stable event identity.

Then one of two rules applies:

1. effects are proven commutative over the targeted state and may be reduced as a set; or
2. overlapping effects use an explicit deterministic reducer / conflict rule.

If neither condition is established, the event set is ambiguous and execution must fail unresolved rather than pick an arbitrary order.

## 9. Commutativity is earned, not assumed

For disjoint state targets it may be possible to prove:

```text
Apply(Apply(S, A), B) = Apply(Apply(S, B), A)
```

That does not imply all shocks commute.

Inventory loss and replenishment, capacity loss and recovery, or demand spike and stock allocation may be order-sensitive.

REGEN-042A requires tests for both commuting and intentionally non-commuting pairs.

## 10. Sequence identity

For ordered events:

```text
A ; B
!= B ; A
```

unless the implementation proves equivalence for that exact pair and state.

Campaign receipts therefore bind the ordered transition sequence, not merely an unordered set of scenario labels.

## 11. Overlap identity

A sequence and an overlap are distinct experiments:

```text
A followed by B
!= A overlapping B
```

Overlap may create resource contention or service deficits that neither isolated interval exhibits.

## 12. Capacity is not binary availability

A dependency may remain available while losing capacity.

The executable state should preserve at least:

```text
availability state
nominal capacity
currently usable capacity
committed capacity
remaining capacity
```

where applicable.

A capacity reduction must not be coerced into complete failure merely because the implementation supports only booleans.

## 13. Capacity floor and service floor are separate

A dependency can retain positive capacity while the service falls below its adopted minimum floor.

```text
dependency capacity > 0
!= service requirement satisfied
```

Conversely a degraded dependency may still deliver the adopted service floor through reserve/substitution.

## 14. Non-negative stock invariant

A shock cannot consume more modeled stock than exists and then continue with negative inventory.

An attempted loss larger than current modeled inventory should resolve according to an explicit rule such as clamping loss to the available stock while recording unmet requested loss, or rejecting an invalid fixture.

The chosen rule must be frozen; silent negative stock is prohibited.

## 15. Conservation under shocks

Where a shock represents physical loss or transfer, the affected accounting boundary must state where the quantity went or mark it as an explicit loss sink / unresolved residual.

A shock event may change availability without destroying physical matter—for example access loss or rights unavailability. Those semantics must not silently reduce a material stock.

```text
unavailable != physically destroyed
```

## 16. Demand shock is its own state transition

A demand increase must not directly reduce supply capacity.

It changes the required service trajectory; later allocation/service evaluation determines the resulting deficit.

This keeps:

```text
supply loss
!= demand increase
```

## 17. Switching is not free

Fallback or substitution activation may have explicit:

- activation delay;
- switching loss;
- startup energy/resource requirement;
- minimum dwell/cooldown;
- transfer capacity;
- authority/evidence prerequisites.

Absent such a profile, the campaign must not assume instantaneous costless switching when that assumption is material to the claimed continuity result.

## 18. Fallback exhaustion

A fallback backed by a finite reserve remains stateful across the campaign.

```text
fallback succeeded earlier
!= fallback remains fully available later
```

Its reserve, wear, cooldown, replenishment, or capacity state carries forward.

## 19. Recovery candidate != recovery execution

The campaign distinguishes:

```text
RecoveryProposed
RecoveryFeasible
RecoveryQualifiedAvailable
RecoveryScheduled
RecoveryExecutedInModel
RecoveryVerifiedInModel
```

The exact implementation vocabulary may be smaller, but a plan must not become successful recovery merely because it exists.

Real-world recovery execution and authority remain outside the model.

## 20. Recovery resources are reservable finite resources

Compound failures can compete for the same:

- repair crew;
- specialist;
- tool;
- spare;
- transport path;
- energy reserve;
- diagnostic capability;
- communication channel.

A campaign must not allow one repair resource to repair two simultaneous failures at full capacity unless an explicit capacity/scheduling model supports it.

## 21. Recovery queue semantics

Where multiple recovery candidates contend for a finite resource, the campaign needs an explicit scheduling rule or must report the ordering as unresolved.

It must not silently use data-structure iteration order.

The scheduling rule is a scenario/model assumption, not emergency authority or a normative priority recommendation.

## 22. Recovery contention can create secondary deficits

A service may fail later not because its own component was shocked, but because its repair resource was consumed by another recovery path.

That causal chain should remain inspectable:

```text
shock A
-> repair resource occupied
-> repair B delayed
-> service B deficit
```

This is a derived consequence, not another exogenous shock.

## 23. Recovery completion != restored service

A repaired dependency may require:

- verification;
- calibration;
- restart;
- warm-up;
- replenishment;
- reauthorization;
- downstream dependency recovery.

REGEN-044 deepens these semantics, but REGEN-042 execution must not equate “repair timer expired” with service restored unless the frozen profile explicitly permits it.

## 24. Horizon censoring

Campaign end is an observation boundary, not a claim about infinite future state.

If a failed dependency has not recovered before the assessment horizon, distinguish:

```text
NotRecoveredWithinHorizon
```

from:

```text
Unrecoverable
```

Likewise a service still healthy at horizon end is not proven indefinitely resilient.

## 25. Right censoring in recovery metrics

A recovery-time statistic must not assign the campaign horizon as if it were the actual recovery time for unrecovered cases.

Unrecovered-at-horizon cases remain censored / unresolved according to the declared analysis method.

## 26. Freshness of baseline evidence

If the baseline contains observation-backed state, each required observation should carry an evidence revision/currentness state compatible with the campaign start.

A stale observation must not be silently refreshed by simulation.

Synthetic fixtures remain explicitly synthetic.

## 27. Frozen initial-state identity

Counterfactual comparisons are valid only when competing architecture/policy runs begin from the same frozen initial-state identity unless the comparison explicitly studies differing baselines.

For ordinary architecture comparison:

```text
initial state A == initial state B
service profile A == service profile B
event schedule A == event schedule B
endpoint definitions A == endpoint definitions B
```

before outcome differences are interpreted as architecture/policy differences.

## 28. Randomness pairing

When randomness exists, paired counterfactual runs should use the same declared seed/sample identity where the model permits common random numbers.

Different random draws must not be presented as if they isolate an architecture effect.

A seed is a reproducibility identity, not evidence that a synthetic distribution matches reality.

## 29. Exact comparison key

A comparative receipt should bind:

```text
ComparisonKey {
    baseline_state_ref,
    service_profile_ref,
    dependency_graph_ref,
    event_schedule_ref,
    shock_effect_profile_refs,
    model_revision,
    endpoint_revision,
    horizon,
    time_resolution,
    seed_or_sample_ref,
}
```

A difference in any material key field makes the runs non-paired unless the study explicitly declares that field as the experimental variable.

## 30. One declared experimental variable

Where practical, an A/B resilience comparison should state the exact dimensions allowed to differ.

Everything else remains frozen.

This prevents model drift, fixture drift, or endpoint drift from being attributed to a changed resilience strategy.

## 31. No post-hoc comparison repair

If architecture B fails because a required substitute was not frozen in its initial profile, the evaluator may not add the substitute after seeing the result and still call the rerun the same campaign revision.

Changed assumptions create a new comparison key and new evidence lineage.

## 32. Endpoint identity stability

Service-hours, deficit quantity, recovery time, degraded intervals, dependency cuts, and unresolved states retain exact definitions throughout a campaign family.

Changing the endpoint implementation or interpretation starts a new endpoint revision.

## 33. State-machine totality

For every valid combination of prior dependency state and typed effect, the transition kernel should either:

- produce a valid successor state; or
- return a typed unresolved/unsupported/error result.

It must not panic or silently default to success for an unhandled combination.

## 34. Invalid fixture != adverse outcome

An invalid scenario input is different from a valid scenario that produces failure.

```text
InvalidFixture
!= ServiceFailed
```

The evidence report must preserve that distinction so malformed campaigns do not inflate failure statistics.

## 35. Deterministic replay invariant

For a deterministic campaign:

```text
same initial state
+ same canonical event stream
+ same ruleset/model revision
= byte-equivalent canonical transition receipts
```

or another explicitly defined canonical equivalence if byte identity is not feasible.

Any nondeterministic field must be excluded from the canonical scientific commitment or normalized explicitly.

## 36. Metamorphic invariants

The qualification corpus should include transformations whose expected result is known without a second implementation.

Examples:

- inserting a no-op event preserves substantive state;
- permuting proven-disjoint commuting shocks preserves final substantive state;
- splitting one constant-capacity outage into adjacent equivalent intervals preserves the same trajectory at declared observation boundaries;
- adding unused optional metadata does not change the physical/service result;
- increasing an unavailable period cannot improve service solely because the outage is longer, absent an explicit state-dependent mechanism;
- removing an available recovery resource cannot make a recovery path earlier unless another explicit scheduling interaction explains it.

Metamorphic tests complement, not replace, independent oracles.

## 37. Monotonicity is conditional

REGEN-042A deliberately does not assert universal “more severe shock => worse outcome” monotonicity.

Complex systems can contain thresholding, scheduling, or protective shutdown effects.

Any monotonic property must name the exact state dimensions and preconditions under which it is expected.

## 38. Counterexample retention

A surprising result that violates an expected metamorphic or analytic invariant is retained as a diagnostic counterexample until explained.

The fixture is not silently deleted because it is inconvenient.

## 39. Common-mode witness handoff

When a compound campaign discovers that apparently separate paths fail through one shared domain, it should emit a compact witness containing:

```text
campaign_ref
prior_state_ref
shock/event refs
shared_failure_domain_ref
affected dependency refs
affected service refs
transition receipt refs
```

REGEN-047 can consume that exact witness in its adversarial campaign rather than reconstructing the failure from prose.

## 40. Substitution witness handoff

When continuity depends on substitution, the campaign should preserve the exact candidate/qualification/activation references consumed.

REGEN-043 remains the owner of the substitution frontier semantics.

REGEN-042A does not invent substitutes dynamically.

## 41. Repair witness handoff

Recovery contention discovered here should preserve exact resource, queue, failure, and service references for REGEN-044.

This prevents later repair analysis from changing the original campaign state while explaining it.

## 42. Campaign result classes

A campaign execution should separate at least:

```text
ValidCompleted
ValidCompletedWithUnresolvedState
InvalidFixture
UnsupportedModelState
ExecutionFailure
```

A CI/runtime failure is not a resilience failure, and an unresolved model state is not a PASS.

## 43. Probability firewall remains absolute

Even with deterministic replay, exhaustive permutations of a synthetic fixture set do not establish event probabilities.

Parameter sweeps are sensitivity evidence unless a separately qualified statistical model gives the samples probabilistic meaning.

## 44. Authority firewall remains absolute

Nothing in the transition algebra may emit:

- emergency authority;
- procurement authority;
- resource seizure authority;
- ecological exemption;
- rights override;
- infrastructure control;
- physical actuation.

A high-confidence transition prediction remains a model result.

## 45. First executable kernel

The first executable REGEN-042A kernel should be intentionally small:

1. immutable canonical state ID;
2. typed dependency/service/stock IDs;
3. typed shock effects;
4. canonical same-time event reducer;
5. non-negative stock and finite-capacity invariants;
6. deterministic interval advancement;
7. finite recovery-resource reservation;
8. horizon-censoring semantics;
9. transition receipts;
10. comparison-key validation.

It should run entirely on synthetic fixtures first.

## 46. Minimum qualification fixtures

The first qualification campaign should include at least:

1. no-op identity;
2. zero-duration non-impulse identity;
3. two disjoint commuting effects in both orders;
4. two intentionally non-commuting effects in both orders;
5. ambiguous simultaneous same-target effects rejected;
6. binary dependency outage;
7. partial capacity reduction;
8. demand increase without supply mutation;
9. inventory loss bounded by stock;
10. availability loss without material destruction;
11. fallback activation with switching delay;
12. fallback reserve exhaustion across sequential shocks;
13. two repairs contending for one worker/tool/spare;
14. delayed recovery causing a secondary service deficit;
15. unrecovered-at-horizon censoring;
16. paired A/B comparison with exact shared key;
17. comparison rejected for mismatched baseline;
18. comparison rejected for mismatched event schedule;
19. deterministic replay;
20. common-mode witness export.

## 47. Promotion gate

An executable REGEN-042A subject should not be described as a qualified compound-shock transition kernel until its exact frozen head demonstrates:

- pinned toolchain and dependency graph;
- deterministic replay;
- no-op identity;
- explicit simultaneous-event semantics;
- typed effect/domain validation;
- non-negative stock / capacity invariants;
- recovery-resource contention;
- horizon censoring;
- exact comparison-key validation;
- retained invalid/adverse/unresolved distinctions;
- independent or metamorphic oracles;
- immutable postflight evidence;
- no authority-bearing output.

## 48. Relationship to REGEN-042

REGEN-042 remains the campaign-level scientific constitution.

REGEN-042A supplies the missing executable transition semantics.

It does not retroactively rewrite REGEN-042's frozen ProductHead. If adopted, downstream executable work should bind both exact revisions.

## 49. Relationship to REGEN-043..047

REGEN-043 owns substitute qualification/frontiers.

REGEN-044 owns repair/skill qualification and recovery detail.

REGEN-045 owns cross-scope continuity composition.

REGEN-046 owns plural result reporting.

REGEN-047 owns adversarial/common-mode falsification.

REGEN-042A provides the transition/witness substrate those layers consume; it does not absorb their authority or duplicate their models.

## 50. Deliberate non-claims

REGEN-042A establishes no real hazard rate, no disaster forecast, no real emergency plan, no guaranteed service continuity, no universal shock severity scale, no repair instruction, no resource-allocation policy, no ecological/right override, no procurement mandate, and no physical-action authority.

Its narrow claim is that a frozen compound-shock campaign needs an explicit deterministic state-transition algebra so temporal composition, contention, censoring, and counterfactual comparison are testable rather than implicit.
