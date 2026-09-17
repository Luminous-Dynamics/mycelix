# REGEN-046 — Plural Resilience Outcome Vector v1

Status: preregistration only.

This contract defines how REGEN should report resilience results without collapsing service continuity, recovery, dependency closure, ecology, rights, uncertainty and distribution into one universal score.

It consumes REGEN-040 through REGEN-045.

## 1. Core theorem

```text
explicit service/scope/horizon
+ observed or simulated service trajectory
+ dependency/substitution/recovery state
+ hard-gate outcomes
+ evidence/uncertainty state
= plural resilience outcome vector
```

not:

```text
many dimensions
-> hidden weighting
-> resilience = N/100
```

## 2. No universal scalar

The core MUST NOT emit a universal resilience score, grade, star rating, tier or winner.

Different services and communities may adopt different priorities, but those preferences remain explicit policy/configuration rather than hidden model constants.

## 3. Outcome identity

Each outcome must bind:

- exact scenario/campaign identity;
- exact service profile;
- exact geographic/population scope;
- exact horizon;
- exact dependency/evidence snapshot;
- exact model/evaluator version;
- exact assumptions and unresolved fields.

An outcome from one context is not ambient truth for another.

## 4. Required plural dimensions

A first outcome representation should preserve at least the following categories where applicable:

1. service requirement and delivered trajectory;
2. time below declared service floor;
3. deficit magnitude and duration;
4. population/service-recipient coverage;
5. reserve/runway state;
6. dependency closure state;
7. qualified substitution state;
8. repair/recovery state;
9. shared-failure/common-mode exposure;
10. ecological hard-gate state;
11. rights/authority hard-gate state;
12. quality/safety hard-gate state;
13. evidence completeness/currentness;
14. uncertainty/unresolved state.

No universal weights are assigned.

## 5. Service continuity is a trajectory

The outcome should preserve time-indexed or interval-indexed service state where the campaign provides it.

```text
service at final time
!= service history
```

A recovered service that suffered a long deficit is distinguishable from uninterrupted service.

## 6. Minimum floor and abundance are distinct

Meeting a declared floor should not erase how far above or below the floor the delivered service was.

The representation should distinguish:

- requirement floor;
- delivered amount/state;
- margin above/below floor.

## 7. Deficit duration remains visible

A short disruption and long disruption with the same final state are not equivalent.

The outcome should preserve deficit intervals rather than only endpoint state.

## 8. Population coverage remains visible

The same service quantity can imply different outcomes depending on the declared population/service-recipient denominator.

The outcome should preserve served and unserved coverage rather than a single average where possible.

## 9. Averages must not erase concentrated failure

If an average service metric meets a floor while a subset of the declared scope experiences severe deficit, that distributional fact must remain visible.

REGEN-046 does not define a universal fairness policy; it prevents aggregation from hiding known heterogeneity.

## 10. Unknown is not zero

Missing or unresolved data must not be encoded as zero deficit, zero risk, zero dependency or successful closure.

Use explicit unknown/unresolved state.

## 11. Negative and adverse outcomes are first-class

A failed campaign or degraded service is valid evidence.

Outcome schemas must not be designed only around successful resilience narratives.

## 12. Dependency closure is not service outcome

A service can temporarily remain above floor despite an open future dependency because of reserves.

Likewise a highly closed dependency graph may still deliver poor service due to insufficient capacity.

Therefore:

```text
dependency closure
!= delivered service
```

Both dimensions remain separate.

## 13. Reserve runway is not continuity proof

A reserve runway estimate is one state variable.

It does not prove replenishment, recovery or service continuity beyond the modeled horizon.

## 14. Qualified substitution is not observed success

The vector should distinguish:

- substitute candidate state;
- qualified substitute state;
- activated substitution state;
- observed/simulated substitution contribution.

## 15. Repairability is not recovery

Repair candidate, repair-in-progress, verified recovery and restored service remain separate outcomes from REGEN-044.

## 16. Common-mode exposure remains structural

The vector may report explicit common-mode dependencies or counts/categories, but must not compress them into a magic redundancy percentage unless a later adopted method defines exactly what that number means.

## 17. Ecology is not a soft resilience dimension

A hard ecological violation must remain a hard violation.

It must not be averaged away by strong service continuity, low cost, locality or fast recovery.

## 18. Rights/authority are not soft penalties

An unauthorized or rights-invalid path cannot be converted into a good resilience outcome by adding utility elsewhere.

The outcome keeps these states explicit.

## 19. Safety/quality remain hard state

A service quantity delivered through a failed quality/safety gate does not become equivalent to a qualified service delivery.

## 20. Evidence completeness is explicit

The vector should distinguish results based on:

- complete current evidence;
- partial evidence;
- stale evidence;
- inferred/model-derived state;
- unresolved evidence.

No confidence value can manufacture a hard-gate PASS.

## 21. Observation and simulation remain distinct

Every outcome must identify whether each material result is:

- observed;
- deterministic synthetic;
- model-projected;
- inferred;
- recommended/planned.

```text
simulated resilience
!= observed resilience
```

## 22. Scenario frequency is not hazard probability

Repeated synthetic campaigns may reveal structural weaknesses.

Their failure frequency is not automatically a calibrated real-world probability.

## 23. Incomparability is allowed

Two systems may be better on different dimensions.

The reporting layer should permit:

```text
A better on recovery speed
B better on ecological closure
A and B incomparable overall
```

without forcing an overall winner.

## 24. Pareto analysis is optional, not authority

A later analysis may identify Pareto-dominated or non-dominated alternatives under declared dimensions.

That mathematical relationship does not create a normative ranking or policy decision.

## 25. Threshold profiles are explicit

Where a community/organization adopts thresholds, the exact threshold profile must be versioned and referenced.

A PASS under one profile is not automatically PASS under another.

## 26. Thresholds do not create scalar equivalence

Passing all declared thresholds may establish bounded conformance to that profile, not a universal resilience level.

## 27. No hidden lexicographic ordering

The core must not secretly treat one dimension as always primary unless the adopted profile explicitly declares that priority.

## 28. Locality is descriptive

The vector may report local/regional/external dependency composition, but locality itself is not a positive or negative score.

## 29. Trade dependence is descriptive

External dependency may be diversified and robust or concentrated and fragile.

The output records the structure rather than assigning ideological value.

## 30. Circularity is descriptive and bounded

The outcome may report closed material/nutrient loops where demonstrated.

Circularity must not imply safety, ecology, sufficiency or continuity by itself.

## 31. Resource efficiency is separate from resilience

A more efficient system may use fewer resources but have less redundancy.

A more redundant system may consume more reserve capacity.

The vector keeps efficiency and continuity-related outcomes distinct.

## 32. Economic outcomes are separate

Cost, capital requirement, operating cost, lost output and economic burden may be recorded, but do not rewrite hard resilience dimensions.

No universal cost-resilience exchange rate is defined.

## 33. Carbon/climate outcomes are separate

Climate/carbon evidence may be linked from the Climate authority, but must not be folded into an opaque resilience score.

## 34. Uncertainty is dimension-specific

Uncertainty about water supply is not necessarily uncertainty about repair skill.

The model should permit uncertainty/unresolved state per dimension rather than one global confidence number.

## 35. Model disagreement is visible

If multiple models or assumptions produce materially different trajectories, the reporting layer should preserve that disagreement rather than average it into false precision.

## 36. Sensitivity is not robustness proof

A sensitivity analysis may identify influential assumptions.

It does not establish that all omitted uncertainty has been bounded.

## 37. Worst-case and expected-case remain distinct

Scenario summaries must label whether an outcome is baseline, expected under declared assumptions, worst examined fixture, or another defined statistic.

The core does not infer real-world probabilities from those labels.

## 38. Cross-service coupling remains visible

If water failure drives food failure, the reporting structure should preserve the causal/dependency link rather than present two unrelated red cells.

## 39. Recovery dependence remains visible

A service recovered because another service came back first should preserve that recovery dependency.

## 40. Time horizons remain separate

A system may perform well for 72 hours and poorly for 90 days.

The vector must not combine horizons without an explicit aggregation method.

## 41. Scope aggregation is explicit

Household, neighborhood, settlement and regional outcomes may differ.

A higher-level vector must identify how lower-level outcomes were composed.

## 42. Simpson-style aggregation hazards are guarded

Where subgroup or sub-scope outcomes are available, an aggregate improvement must not erase a known severe subgroup degradation.

The core should preserve the underlying partition references necessary for review.

## 43. Outcome provenance is immutable

Once emitted, an outcome should retain the exact scenario/evidence/model identities used.

New evidence produces a new outcome rather than silently rewriting historical evidence.

## 44. Comparison provenance is explicit

A comparison should bind the exact outcome identities it compares.

Changing one subject requires a new comparison result.

## 45. Recommendation is separate from outcome

Symthaea may use plural outcomes to recommend investments or experiments.

The recommendation is a new proposition, not part of the observed/simulated outcome itself.

## 46. Symthaea boundary

Symthaea may:

- generate plural reports;
- detect tradeoffs;
- identify bottlenecks;
- perform sensitivity/Pareto analysis;
- propose experiments or investments;
- explain why outcomes differ.

Symthaea may not:

- invent hidden weights and present them as objective resilience;
- convert model confidence into hard-gate PASS;
- change adopted thresholds/authority;
- execute physical interventions.

## 47. Mycelix boundary

Mycelix may store/coordinate exact identities, evidence, adopted threshold/service profiles and reviewable outcomes through the appropriate domains.

REGEN-046 is a reporting/composition contract, not a governance authority.

## 48. Initial deterministic model direction

A later dependency-light implementation may define structures conceptually equivalent to:

```text
ResilienceOutcomeId
OutcomeContext
ServiceOutcome
DeficitInterval
CoverageOutcome
DependencyClosureOutcome
SubstitutionOutcomeRef
RecoveryOutcomeRef
HardGateOutcomes
EvidenceState
UncertaintyState
PluralResilienceOutcome
```

No field named `overall_score`, `resilience_score`, `grade`, `tier` or equivalent belongs in v1.

## 49. Initial comparison model direction

A pure comparison layer may emit:

```text
DimensionImproved
DimensionDegraded
DimensionEquivalent
DimensionIncomparable
DimensionUnresolved
```

for declared dimensions without producing an overall winner.

## 50. Minimum synthetic campaign

A first executable campaign should include at least:

1. final recovery does not erase earlier deficit interval;
2. same endpoint with different deficit duration produces distinguishable outcomes;
3. same total supply with different population denominator produces distinguishable outcomes;
4. average service above floor does not erase a known unserved subgroup;
5. unknown evidence remains unresolved rather than zero;
6. ecological failure remains explicit despite strong continuity;
7. rights failure remains explicit despite strong continuity;
8. quality failure remains explicit despite quantity sufficiency;
9. high dependency closure with insufficient capacity remains distinguishable from adequate service;
10. low closure with reserve-supported short-horizon service remains distinguishable;
11. qualified substitution does not equal observed successful substitution;
12. repair candidate does not equal recovered service;
13. local share does not determine outcome score;
14. external diversified supply can coexist with strong continuity;
15. circularity can coexist with poor continuity;
16. efficiency can improve while redundancy degrades;
17. two alternatives can be explicitly incomparable;
18. Pareto relation does not create policy winner;
19. profile PASS binds exact profile version;
20. changing profile yields a new conformance result;
21. model uncertainty is dimension-specific;
22. model disagreement remains visible;
23. 72-hour and 90-day outcomes remain separate;
24. aggregate outcome preserves severe known sub-scope deficit;
25. observed and simulated results cannot be conflated;
26. synthetic failure frequency is not emitted as hazard probability;
27. recommendation output is distinct from outcome evidence;
28. no universal scalar field is present.

## 51. Relationship to REGEN-047

REGEN-047 should adversarially attack the entire Phase-E reporting chain for score leakage, hidden weighting, common-mode omission, aggregation masking, stale evidence and fake independence.

## 52. Deliberate non-claims

REGEN-046 creates no current claim of:

- universal resilience;
- civilizational resilience;
- disaster readiness;
- best community or system;
- objective policy ranking;
- collapse probability;
- ecological sufficiency;
- economic optimality;
- emergency authority;
- physical action.

Its proposition is deliberately narrow:

> resilience outcomes should remain plural, scoped, temporal, evidence-bearing and inspectable so tradeoffs, deficits, uncertainty and hard constraints cannot be hidden inside a universal score.
