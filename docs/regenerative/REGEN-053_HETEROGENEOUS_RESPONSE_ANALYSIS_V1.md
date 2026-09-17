# REGEN-053 — Heterogeneous Response Analysis v1

Status: preregistration only. This contract defines how regenerative trial outcomes may vary across sites, soils, seasons, populations, materials, process batches, and other declared contexts without converting aggregate effects into universal claims.

## Core theorem

```text
frozen trial/outcome evidence
+ exact subgroup/context definitions
+ exact interaction model/rule
+ multiplicity/uncertainty treatment
= bounded heterogeneous-response evidence
```

not:

```text
aggregate mean positive
= everyone/site/context benefits
```

## Preregistered vs exploratory

Subgroups, effect modifiers, stratification variables, interaction tests, and decision thresholds must be labeled as preregistered or exploratory. Post-hoc slices cannot be silently promoted into confirmatory findings.

## Context identity

Every heterogeneous-response result binds exact:
- trial/protocol identity;
- candidate/treatment identity;
- endpoint identity;
- evidence snapshot;
- site/soil/material/batch/season/population context;
- subgroup/effect-modifier definition and revision;
- analysis method/model revision;
- missingness/exclusion policy.

## Aggregate firewall

The system preserves both aggregate and disaggregated outcomes where material. A favorable aggregate cannot erase a harmful or unresolved subgroup outcome, and a favorable subgroup cannot imply global benefit.

## Small groups and sparse contexts

Low-sample contexts remain low-information. The system must not transform high variance, zero events, or model shrinkage into false certainty. Partial pooling/shrinkage, if used, must be explicit and preserve the distinction between observed local evidence and model-informed estimates.

## Multiplicity

Searching many subgroups/effect modifiers creates multiplicity. The analysis records the search space and exact correction/decision rule where inferential claims depend on it.

`one significant subgroup among many searches != robust heterogeneity`

## Continuous modifiers

Continuous soil, climate, dose, age, resource, or process variables should not be arbitrarily dichotomized without retaining the cut rule and original variable semantics.

Changing a cut point creates a different analysis proposition.

## Missingness and representation

Missing subgroup/context labels, sparse sampling, withdrawal, censoring, inaccessible sites, and measurement failure remain explicit. Unknown membership is not assigned to the reference group by default.

## Site/generalization firewall

```text
worked at site A
!= works at site B
```

Transfer requires explicit similarity/transportability evidence or remains unresolved. Geographic proximity alone does not establish comparable soil, climate, management, rights, water, or social context.

## Material/batch heterogeneity

A result for one biochar/compost/feedstock batch does not automatically transfer to another batch merely because the product category name is the same. Exact lineage/profile identity remains bound.

## Time heterogeneity

Responses may differ by season, crop stage, duration, lag, and repeated application history. Short-term response cannot silently become persistent long-term response.

## Distributional harm

When some contexts benefit while others are harmed, the result remains plural. REGEN-052 Pareto analysis must not hide adverse subgroups inside a favorable average dimension.

## Model boundary

Learned/HDC/causal models may estimate heterogeneous effects, but model outputs remain `Inferred` unless grounded as otherwise qualified evidence. Model confidence does not establish subgroup truth.

## Privacy boundary

Subgroup analysis should use the minimum identifying detail necessary. Small-cell privacy protections may make some estimates unavailable; privacy redaction is not evidence of zero harm or zero benefit.

## Fairness boundary

REGEN-053 may expose uneven service/outcome distributions but does not encode one universal fairness objective or allocate rights/resources. Normative decisions remain in adopted governance/policy layers.

## Replication

A discovered heterogeneous response should be eligible for targeted replication. Replication candidate generation remains bounded by REGEN-050 eligibility; a subgroup finding cannot itself create intervention authority.

## Negative/null outcomes

Null, adverse, reversed, or unstable subgroup effects remain first-class evidence and are not removed to preserve a clean overall narrative.

## Qualification targets

At minimum:
1. favorable aggregate cannot erase adverse subgroup;
2. favorable subgroup cannot imply universal benefit;
3. exploratory subgroup is never labeled preregistered;
4. subgroup definition revision changes analysis identity;
5. missing subgroup membership remains unknown;
6. sparse subgroup uncertainty stays visible;
7. batch identity prevents category-level transfer;
8. site-A evidence does not automatically transfer to site B;
9. time-horizon change creates a distinct proposition;
10. model-inferred heterogeneity remains distinct from observed evidence;
11. multiplicity/search-space metadata retained;
12. privacy suppression does not become zero effect;
13. null/adverse subgroup outcomes retained;
14. no subgroup result creates authority or execution.

## Deliberate non-claims

REGEN-053 establishes no universal treatment effect, demographic/biological determinism, ethical allocation rule, agronomic recommendation, causal truth beyond the declared analysis, consent, authority, process execution, or physical action.
