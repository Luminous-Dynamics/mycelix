# REGEN-052 — Pareto Treatment Analysis v1

Status: preregistration only. This contract defines recommendation-only multi-dimensional comparison of already-eligible trial candidates. It creates no winner, policy mandate, authorization, or execution authority.

## 1. Purpose

REGEN-050/051 deliberately keep candidate eligibility and expected information separate from agronomic benefit, burden, cost, ecology, resilience and uncertainty. REGEN-052 freezes how those plural dimensions may be compared without collapsing them into one hidden utility score.

Core theorem:

```text
eligible candidates
+ exact dimension definitions
+ exact evidence/model snapshots
+ hard-gate outcomes
+ declared directionality
= inspectable Pareto relations/frontier
```

not:

```text
Pareto frontier
= ranked winner
= socially preferred treatment
= authorized treatment
```

## 2. Hard gates precede Pareto analysis

A candidate that fails a hard safety, ecology, rights, consent, quality, contamination, suitability or protocol gate is not rehabilitated by favorable soft dimensions.

Hard-gate state is never converted into a numeric penalty.

## 3. Dimension identity

Each compared dimension must bind:

- exact dimension ID;
- definition and units/scale;
- higher/lower/is-target directionality;
- exact evidence/model source;
- time horizon;
- population/site scope;
- uncertainty representation;
- revision/version.

`yield`, `cost`, `soil carbon`, `water demand`, `expected information`, and `resilience` are not interchangeable scalar coordinates without those identities.

## 4. No hidden normalization

Normalization, transformation, clipping, winsorization, standardization or sign reversal must be explicit and versioned.

A normalized value may not replace the retained original dimension evidence.

## 5. Dominance semantics

Candidate A may dominate B only under the exact declared dimension set and comparison rules.

Changing the dimension set or horizon creates a different proposition.

```text
A dominates B under D1
!= A dominates B under D2
```

## 6. Uncertainty-aware dominance

Where dimension estimates carry uncertainty, the system must declare whether dominance uses point estimates, intervals, posterior probabilities or another exact rule.

Overlapping/insufficient evidence may yield `UnresolvedRelation` rather than forced dominance.

## 7. Missing is not neutral

A missing or not-evaluable dimension cannot silently become zero, average, or best/worst.

Candidate comparability may become unresolved when required dimensions are missing.

## 8. Incomparability is valid

Two candidates may be mutually non-dominating.

The output may legitimately contain a frontier of several incomparable candidates.

The system must not manufacture a ranking merely for UI convenience.

## 9. No universal weights

REGEN-052 v1 contains no universal weighted-sum utility function.

If a later adopted decision profile supplies weights/preferences, that is a separate explicit social/governance proposition and must preserve the unweighted dimension vector.

## 10. Cost and affordability

Cost is a descriptive/planning dimension, not a universal objective.

Low cost cannot override hard safety/ecology/rights gates.

Affordability may differ by community, funding source and time horizon and therefore needs explicit context.

## 11. Locality and circularity

Locality, circularity and recovered-material share may be reported as dimensions but carry no automatic positive sign.

A local option with shared grid/tooling/lab dependencies or ecological harm does not receive a resilience/sustainability bonus by label.

## 12. Carbon firewall

Carbon-related dimensions remain descriptive/model outputs unless qualified by the Climate authority path.

Agronomic Pareto analysis cannot mint carbon-removal or credit authority.

## 13. Resilience dimensions

REGEN-040–047 outputs may contribute plural resilience dimensions, but there is no single resilience coordinate unless an exact adopted profile defines a bounded derived statistic.

Underlying service/dependency outcomes remain accessible.

## 14. Expected information dimension

REGEN-051 expected-information estimates may be included, but information gain is not treatment efficacy.

A highly informative treatment can remain unattractive on other dimensions and still be scientifically useful.

## 15. Control/null candidate

The control/null candidate remains in the comparison when it is part of the trial design.

The analysis must not assume intervention dominates non-intervention merely because the intervention has more modeled attributes.

## 16. Baseline/reference choice

Any delta or percentage-improvement dimension must bind its exact baseline/reference candidate and baseline evidence snapshot.

Changing baseline changes the proposition.

## 17. Time horizon

Short-term and long-term effects are separate dimensions/propositions unless an explicit temporal aggregation rule exists.

A short-term yield gain cannot silently offset a long-term soil/water/ecology loss.

## 18. Distributional outcomes

Population/subgroup/site heterogeneity must not disappear into one mean when material to the decision.

REGEN-053 owns heterogeneous-response analysis; REGEN-052 must preserve subgroup dimensions or unresolved heterogeneity rather than claiming universal benefit.

## 19. Negative/null evidence

Null and adverse outcomes remain visible dimensions/evidence.

Candidate removal because it performed poorly is retained as analysis lineage rather than erased from the frontier history.

## 20. Frontier identity

A Pareto result should bind:

- candidate-set identity;
- dimension-set identity;
- hard-gate profile identity;
- evidence/model snapshot;
- comparison rule revision;
- uncertainty rule revision;
- resulting frontier/member identities.

## 21. Sensitivity analysis

The system may compute multiple frontiers under alternate declared dimension sets, horizons or uncertainty rules.

Differences between frontiers are evidence about sensitivity, not a reason to secretly choose whichever frontier contains a favored candidate.

## 22. Presentation firewall

UI sorting, color, default ordering or highlighting must not be represented as scientific dominance or policy preference.

A deterministic display order may exist for reproducibility but is not an evaluative ranking.

## 23. Recommendation boundary

REGEN-052 emits comparison evidence only.

```text
ParetoMember
!= RecommendedTreatment
!= AdoptedProtocol
!= AuthorizedIntervention
```

REGEN-054 owns the recommendation-only bridge.

## 24. Qualification targets

At minimum:

1. hard-gate failure cannot enter eligible frontier;
2. known dominance is detected deterministically;
3. incomparable candidates remain incomparable;
4. missing required dimension yields unresolved relation;
5. dimension-set change changes frontier identity;
6. baseline change changes delta proposition;
7. no hidden weighted score is emitted;
8. uncertainty overlap may remain unresolved;
9. locality/circularity labels do not alter dominance unless explicitly dimensions;
10. null/control candidate is preserved;
11. carbon dimension creates no carbon authority;
12. expected information creates no efficacy claim;
13. subgroup adverse outcome cannot be erased by favorable aggregate without declared aggregation;
14. UI ordering is separate from Pareto relation.

## 25. Deliberate non-claims

REGEN-052 establishes no overall best treatment, social preference, ethical acceptability, agronomic efficacy, universal sustainability, carbon authority, funding/procurement decision, trial authorization, or physical action.

Its proposition is deliberately narrow: preserve transparent multi-dimensional tradeoffs among already-eligible candidates without manufacturing a single objective or winner.
