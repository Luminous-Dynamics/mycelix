# REGEN-047 — Adversarial Common-Mode Campaign v1

Status: preregistration only.

This contract defines an adversarial campaign for Phase-E regenerative-resilience logic. Its purpose is to try to break the claims created by REGEN-040 through REGEN-046 by exposing hidden common modes, stale evidence, double counting, scope leakage, optimistic unknown handling, repair bottlenecks and score leakage.

It is a falsification campaign, not a real-world disaster forecast and not an attack playbook for physical infrastructure.

## 1. Core theorem

```text
frozen Phase-E subject
+ preregistered adversarial fixtures
+ independent invariants/oracles
+ deterministic replay
+ retained failures
= reviewable falsification evidence
```

not:

```text
campaign passes
= real world is safe
= all shocks covered
= disaster probability known
```

## 2. Campaign target

The campaign attacks the composed semantics of:

- REGEN-040 essential services;
- REGEN-041 local dependency closure;
- REGEN-042 compound shocks;
- REGEN-043 qualified substitutions;
- REGEN-044 repair/skill recovery;
- REGEN-045 continuity composition;
- REGEN-046 plural outcome reporting.

Each layer remains independently inspectable.

## 3. No probability laundering

Synthetic adversarial-case frequency is not a calibrated probability of real disruption, collapse, service loss or recovery.

The campaign is structural assurance evidence only.

## 4. Hidden common-mode attacks

Fixtures should inject dependencies that appear independent locally but share one upstream failure domain, including abstract classes such as:

- common energy source;
- common water source;
- common transport corridor;
- common supplier;
- common storage facility;
- common communications/cloud provider;
- common finance/payment rail;
- common laboratory/testing service;
- common repair specialist;
- common spare inventory;
- common authorization/credential service.

The expected property is that discovered common modes narrow or preserve resilience claims; they never increase independence by bookkeeping accident.

## 5. Fake redundancy attacks

Duplicating references to one physical/logical resource must not increase effective capacity or redundancy.

Examples include duplicate:

- generators/assets;
- inventory lots;
- transport vehicles;
- repair crews;
- spares;
- stored food/water;
- communication links backed by one carrier;
- supplier records pointing at one upstream source.

## 6. Reserve double-spend attacks

One reserve must not be credited simultaneously to mutually exclusive service, substitution, repair or mutual-aid plans.

The campaign must include conflicts across layers, not only inside one ledger.

## 7. Mutual-aid overcommitment attacks

A donor scope may not promise the same constrained capacity to multiple recipients and have every recipient count it as independently available in the same realized scenario.

## 8. Scope-leak attacks

Fixtures should attempt to reuse:

- one service outcome for another service;
- one geographic scope for another;
- one population denominator for another;
- one time horizon for another;
- one evidence snapshot for another;
- one adopted threshold/profile revision for another.

Exact scope mismatch must fail or remain unresolved.

## 9. Stale-evidence attacks

The campaign should mutate inventory, authority, ecological state, service requirement, equipment state, skill availability, shock state or population after qualification.

Old results must not remain ambient `true` without re-evaluation.

## 10. Unknown-to-success attacks

Fixtures should remove or corrupt facts required for qualification.

Expected behavior:

```text
required fact unknown
-> unresolved / ineligible as specified
```

never:

```text
required fact unknown
-> optimistic default PASS
```

## 11. Null/adverse-evidence deletion attacks

The campaign should ensure omitted negative observations cannot improve an outcome merely because the reporting path only retains positive evidence.

Historical adverse intervals/outcomes remain first-class.

## 12. Recovery-history erasure attacks

A recovered endpoint must not erase prior deficit duration, degraded operation, reserve depletion or failed substitutions.

## 13. Time-shift attacks

Adversarial fixtures should shift:

- substitute activation later;
- repair completion later;
- delivery lead time later;
- reserve replenishment later;
- service demand earlier;
- shock onset/overlap.

A path that misses the relevant service window must not be treated as continuity-preserving simply because it succeeds eventually.

## 14. Horizon substitution attacks

Short-horizon success must not be reused as long-horizon closure.

A 72-hour reserve result cannot silently become a 90-day resilience claim.

## 15. Capacity inflation attacks

The campaign should attempt to increase apparent capacity via:

- duplicate resource references;
- mixed accounting bases;
- nominal instead of usable capacity;
- ignoring competing commitments;
- ignoring losses/residuals;
- treating unavailable stock as available;
- treating future replenishment as current inventory.

## 16. Basis-mismatch attacks

Where exact accounting bases matter, mixed bases must not add or compare without an explicit valid conversion/derivation.

This carries forward the REGEN biomass mass-basis firewall.

## 17. Quality/safety bypass attacks

A quantity-sufficient resource with failed or unresolved quality/safety state must not become qualified because service demand is severe.

## 18. Ecology bypass attacks

A resource failing adopted hard ecological obligations must not become eligible because:

- it is local;
- it is cheap;
- it lowers modeled carbon;
- it increases service coverage;
- alternatives are scarce;
- model confidence is high.

## 19. Rights/authority bypass attacks

Technical feasibility and physical possession must not create missing withdrawal, transfer, operation, professional or emergency authority.

## 20. Substitution-cycle attacks

Fixtures should introduce cycles such as:

```text
A substitutes for B
B substitutes for A
```

without an independent grounded capacity source.

Cycles must not manufacture closure.

## 21. Dependency-cycle attacks

Cross-service cycles must remain explicit.

If water requires energy and energy requires water, the model must not mark both closed merely because each points to the other.

## 22. Repair-queue saturation attacks

Multiple failures should compete for the same:

- technician;
- workshop;
- diagnostic tool;
- calibration capability;
- spare part;
- transport resource.

The evaluator must preserve queueing/competition rather than repairing everything in parallel by default.

## 23. Skill-concentration attacks

The campaign should remove the only person, instructor, credential holder or specialist supporting multiple service paths.

Apparent asset redundancy should collapse appropriately where the skill dependency is common-mode.

## 24. Documentation/software dependency attacks

Fixtures may remove access to a required manual, configuration backup, firmware artifact, software tool or vendor portal.

The result should reflect the exact dependency rather than assume knowledge/software is always globally available.

## 25. Communication-loss attacks

The campaign should distinguish:

- loss of central/federated coordination;
- loss of local service operation;

and verify that one does not imply the other unless the declared dependency graph actually requires it.

## 26. Central-coordinator failure attacks

Locally autonomous service paths must remain locally available where designed, even when a federation/dashboard/coordination layer is unavailable.

The campaign must also expose cases that genuinely require central/shared infrastructure.

## 27. Demand-transfer attacks

Fixtures should transfer service demand across scopes and verify:

- denominators update;
- receiving constraints apply;
- donor/recipient reserves are not double counted;
- aggregate success does not hide localized deficit.

## 28. Aggregation-masking attacks

Construct cases where average or aggregate metrics improve while one known subgroup/scope materially degrades.

REGEN-046 reporting must preserve the degradation rather than report only the aggregate.

## 29. Score-leakage attacks

The campaign should statically/dynamically reject introduction of opaque universal fields or outputs equivalent to:

```text
resilience_score
self_sufficiency_score
overall_grade
universal_tier
```

unless a later explicitly adopted method defines a narrow scoped metric with non-universal claims.

## 30. Hidden-weighting attacks

If a comparison layer ranks alternatives, the exact weights/priorities must be explicit adopted inputs.

The core must not smuggle an undocumented value system into a supposedly objective result.

## 31. Model-confidence attacks

Increase AI/model confidence while keeping a hard evidence/authority/ecology/safety failure unchanged.

The hard result must not improve.

## 32. Locality-label attacks

Flip a dependency label between local/regional/external without changing its actual failure-domain structure.

The resilience result must not improve merely because the string says `local`.

## 33. Carbon-label attacks

Add a favorable carbon/climate claim to an otherwise unqualified resource/path.

It must not bypass service, ecology, rights, safety or quality requirements.

## 34. Economic-label attacks

Make an unqualified option cheaper or more profitable.

Hard eligibility must remain unchanged.

## 35. Circularity-label attacks

Mark a loop as circular while retaining a hidden external bottleneck.

Material recirculation alone must not create dependency closure.

## 36. Common-mode discovery monotonicity

When analysis discovers that two previously separate dependencies actually share a failure domain, the independence claim may narrow or stay unchanged but must not widen.

## 37. Stock monotonicity

All else equal, reducing an available finite reserve must not increase computed reserve runway or directly supported service capacity.

## 38. Delay monotonicity

All else equal, delaying a substitute or repair beyond a service deadline must not improve immediate continuity.

## 39. Deficit monotonicity

All else equal, extending a known below-floor interval must not reduce reported deficit duration.

## 40. Duplicate-reference invariance

Duplicating an identical exact resource/dependency reference in an input set must not create additional capacity.

## 41. Input-order invariance

Where semantics are set-based and no declared priority exists, reordering equivalent inputs must not change the result.

## 42. Identifier-renaming invariance

Renaming opaque identifiers consistently must not change substantive outcomes.

## 43. Explicit-priority sensitivity

Where a declared repair/restoration priority is an input, changing that input may change trajectory.

The campaign must ensure the change is attributable to the explicit policy input, not nondeterministic iteration order.

## 44. Deterministic replay

Given identical frozen inputs, model version and seed/config where applicable, deterministic baseline evaluators must replay byte/semantic-equivalent outcomes as specified.

## 45. Fuzzed malformed inputs

Later executable work should fuzz bounded schemas for:

- malformed IDs;
- oversized references;
- missing mandatory bindings;
- contradictory state;
- duplicate IDs;
- invalid cycles;
- impossible time intervals;
- arithmetic overflow/underflow;
- unsupported schema versions.

Malformed input must fail structurally rather than silently coerce to plausible state.

## 46. Arithmetic boundaries

All exact counters/masses/durations/capacities must test boundary conditions and overflow behavior appropriate to their declared types.

No wraparound may create capacity or erase deficit.

## 47. State-machine invalid-transition attacks

Attempt impossible transitions such as:

```text
candidate -> observed success
failed -> recovered without recovery evidence
unresolved -> qualified without new evidence
reserved -> consumed without consumption event
```

The executable types/evaluators should reject them where the architecture makes them impossible by construction.

## 48. Negative controls

The campaign must contain controls that should *not* change outcomes, so the test suite does not merely reward conservatism.

Examples:

- irrelevant metadata change;
- reordering set-like inputs;
- consistent opaque-ID rename;
- adding an independent unused resource;
- adding evidence that is valid but irrelevant to the evaluated proposition.

## 49. Positive controls

The campaign must also contain legitimate improvements, such as:

- adding a genuinely independent qualified substitute;
- adding a compatible spare;
- restoring a required skilled capability;
- extending reserve runway with non-overcommitted stock;
- restoring a failed communications dependency where actually required.

A model that never allows improvement is not correct.

## 50. Adversarial corpus immutability

A qualification campaign must bind the exact fixture corpus identity.

Failures cannot be removed post hoc to obtain PASS without creating a new campaign lineage/version.

## 51. Failure retention

Historical red runs remain evidence.

A later fixed PASS must not rewrite a prior demonstrated failure as if it never occurred.

## 52. Oracle independence

Where practical, critical invariants should be checked by an implementation-independent oracle or simple alternative evaluator, not only self-tests inside the product implementation.

## 53. Mutation testing

Later qualified implementations should consider mutation testing for high-value predicates such as:

- hard-gate checks;
- duplicate/double-spend checks;
- scope/evidence-snapshot binding;
- time-window checks;
- cycle detection;
- outcome-state separation.

Surviving relevant mutations are evidence gaps, not automatic product failures unless the qualification contract declares them so.

## 54. Property testing

Property-based tests should target algebraic and monotonic properties that are actually valid, avoiding invented monotonicity where demand transfer, threshold changes or path-dependent dynamics make it false.

## 55. Common-mode graph mutation

The campaign should systematically merge previously independent dependency nodes into shared failure domains and verify the result does not accidentally claim more independence.

## 56. Shock-order permutations

For preregistered compound events, selected order permutations should be tested because:

```text
A then B
!= B then A
!= simultaneous A+B
```

where state depletion/recovery is path-dependent.

## 57. No post-hoc rescue assumptions

After observing a failed fixture, the campaign may not invent a new reserve, substitute, technician, authority or replenishment path inside that same frozen evidence run.

A new assumption requires a new scenario/campaign subject.

## 58. No post-hoc threshold weakening

A failed service floor cannot be turned into PASS by changing the adopted threshold after results are known within the same frozen campaign lineage.

## 59. No selective horizon shortening

A long-horizon failure cannot be hidden by reporting only the earlier successful interval unless the report explicitly identifies the shortened horizon as a different proposition.

## 60. No selective population shrinking

A deficit cannot be hidden by silently removing affected recipients from the denominator.

Population-scope changes create a new outcome context.

## 61. Cross-domain authority retention

Water, Energy, Climate, Finance, Commons/Property, identity/credential and other authoritative domains retain ownership of their state.

REGEN-047 attacks the integration assumptions; it does not replace those authorities.

## 62. Symthaea boundary

Symthaea may generate adversarial candidates, search graph mutations, identify bottlenecks and explain failures.

It may not:

- alter hard gates to obtain PASS;
- create authority;
- rewrite failed evidence;
- convert synthetic failure frequency into real hazard probability;
- execute physical attacks or interventions.

## 63. Safe research boundary

The campaign operates on synthetic/digital dependency models and evidence contracts.

It intentionally avoids actionable instructions for sabotaging, disabling or exploiting real critical infrastructure, equipment, networks, water systems, energy systems or other physical services.

## 64. Initial executable campaign groups

A first implementation should group fixtures roughly as:

```text
A. identity/scope binding
B. stock/capacity accounting
C. common-mode/fake redundancy
D. substitution qualification
E. repair/skill bottlenecks
F. temporal/horizon attacks
G. cross-scope/mutual-aid composition
H. hard-gate bypass attempts
I. outcome aggregation/score leakage
J. malformed/schema/property tests
```

## 65. Minimum named fixtures

At minimum include explicit cases for:

1. duplicate reserve reference;
2. overcommitted mutual aid;
3. shared upstream supplier hidden behind two local vendors;
4. shared grid behind two nominally independent assets;
5. shared technician across multiple repairs;
6. shared spare inventory across repair plans;
7. substitute available after deadline;
8. stale evidence snapshot;
9. stale authority/credential state;
10. unresolved ecology presented as eligible;
11. unresolved quality presented as usable;
12. local-label-only improvement attempt;
13. carbon-label-only improvement attempt;
14. cheap-option hard-gate bypass attempt;
15. cyclic substitution path;
16. cyclic service dependency;
17. average masks subgroup outage;
18. recovery erases historical outage attempt;
19. horizon-shortening attempt;
20. population-denominator shrinking attempt;
21. central coordination failure with independent local service preserved;
22. true central dependency failure where service should degrade;
23. repair queue saturation;
24. missing documentation/software dependency;
25. duplicate exact IDs do not create capacity;
26. input-order invariance;
27. consistent identifier rename invariance;
28. legitimate independent substitute improves result;
29. legitimate spare/skill restoration improves recovery;
30. no universal resilience-score field emitted.

## 66. PASS meaning

A campaign PASS means only:

> the exact frozen Phase-E implementation satisfied the exact preregistered adversarial corpus and declared invariants under the recorded environment.

It does not prove complete real-world resilience.

## 67. Promotion discipline

If executable code is later built, qualification should follow the established REGEN discipline:

```text
authored subject
-> exact-head preparation
-> ProductFrozen dependency graph
-> independent/adversarial campaign
-> machine-readable receipt
-> retained evidence capsule
```

No queued or stale run is a PASS.

## 68. Deliberate non-claims

REGEN-047 creates no current claim of:

- disaster readiness;
- critical-infrastructure security;
- national/civilizational resilience;
- societal-collapse probability;
- completeness of modeled shocks;
- safety of any real physical intervention;
- emergency authority;
- permission to disrupt any real service;
- physical execution.

Its proposition is deliberately narrow:

> resilience claims should survive deliberate attempts to expose hidden common modes, double counting, stale evidence, scope leakage, temporal mistakes, hard-gate bypasses and reporting distortions before they are trusted even as bounded synthetic evidence.
