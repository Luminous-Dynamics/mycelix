# REGEN-054 — Recommendation-Only Experiment Bridge v1

Status: preregistration only. This contract defines how bounded experiment intelligence may emit reviewable recommendations without creating protocol adoption, authorization, scheduling, procurement, execution, or physical actuation.

## Core theorem

```text
qualified evidence/model inputs
+ bounded candidate analysis
+ exact recommendation rationale
+ uncertainty/non-claims
= reviewable recommendation
```

not:

```text
recommendation
= adopted protocol
= authorized intervention
= command
= execution
```

## Upstream inputs

A recommendation may reference exact outputs from REGEN-050–053:

- bounded candidate identity;
- expected-information evidence;
- Pareto/tradeoff evidence;
- heterogeneous-response evidence;
- exact evidence/model snapshots;
- control/null option;
- unresolved/adverse findings.

It does not own or rewrite those outputs.

## Recommendation identity

Each recommendation binds:

- recommendation ID/digest;
- exact candidate(s) referenced;
- exact evidence snapshot;
- exact model/analysis revisions;
- exact recommendation policy revision;
- rationale/evidence references;
- uncertainty and unresolved conditions;
- hard-gate status;
- issued-at/currentness metadata where applicable;
- explicit non-claims.

Free-form explanation text is not the normative identity.

## Recommendation states

At minimum:

```text
RecommendConsideration
RecommendNoIntervention
RecommendFurtherEvidence
RecommendReplication
UnableToRecommend { reasons }
```

The system must be able to recommend no intervention or further evidence rather than always selecting an action.

## Hard-gate inheritance

A recommendation cannot resurrect a candidate rejected by safety, ecology, rights, consent, contamination, quality, suitability, protocol, or other hard eligibility gates.

Hard-gate failure is not a negative weight.

## No hidden winner

If REGEN-052 yields incomparable candidates, REGEN-054 may preserve that plurality.

A recommendation policy may state what additional evidence or adopted preference would resolve the choice, but must not manufacture a scientific winner.

## Uncertainty inheritance

Unresolved/missing evidence from upstream remains visible. Recommendation generation cannot map `Unresolved` to PASS merely because another dimension is favorable.

## Currentness

Recommendations are bound to exact evidence/model/policy snapshots. Later evidence does not retroactively update an old recommendation; it creates a new recommendation lineage.

Stale recommendation handling must be explicit.

## Rationale boundary

Structured rationale references may explain why a candidate is recommended for consideration. Rationale is not authority.

An LLM-generated narrative may accompany the structured result but cannot alter candidate identity, hard-gate state, or normative recommendation fields.

## Adoption boundary

A separate human/governance/domain authority may adopt, modify, reject, defer, or request more evidence.

```text
Recommendation
-> optional review/adoption process
-> adopted protocol/policy
```

The bridge does not skip the adoption step.

## Authorization boundary

Even an adopted trial protocol is not automatically authorized for a particular site, material, person, organization, date, device, or process execution.

Authorization remains scoped and external to REGEN-054.

## Scheduling/procurement boundary

Recommendation output cannot reserve inventory, spend funds, schedule workers, book equipment, create purchase orders, reserve land/water, or consume materials.

Those are separate authorized actions.

## Execution boundary

No REGEN-054 type contains:

- actuator command;
- process-control setpoint;
- device command;
- reservation acceptance token;
- process-execution capability;
- credential that grants physical authority.

## Symthaea role

Symthaea may generate analyses, explanations and recommendations. Its intelligence, confidence, consciousness estimate, HDC state, model accuracy, reputation or capability does not create authority.

## Conflicting recommendations

Multiple models or analyses may disagree. Conflict is preserved explicitly rather than resolved by arbitrary confidence averaging unless an exact adopted meta-policy exists.

## Human override language

Human reviewers may reject a recommendation, but “human approved” must not become a generic bypass of hard ecological/safety/legal constraints. The relevant authority must be explicit and scoped.

## Recommendation expiry/revalidation

Where currentness matters, a recommendation may carry revalidation conditions or expiry policy references. Expiry does not imply the opposite recommendation is true; it means the old recommendation is no longer current enough for its intended use.

## Negative/adverse evidence

Recommendations retain adverse/null evidence that materially affects the rationale. Positive model predictions cannot erase observed adverse outcomes.

## Audit lineage

A recommendation lineage should retain:

```text
inputs
-> analyses
-> candidate comparison
-> recommendation policy
-> structured recommendation
-> later review/adoption outcome
```

Review/adoption events are downstream evidence, not edits to the original recommendation.

## Qualification targets

At least:
1. hard-gate failure cannot produce positive intervention recommendation;
2. no eligible candidate may yield `RecommendNoIntervention` or `UnableToRecommend`;
3. unresolved evidence remains visible;
4. recommendation identity changes with evidence/model/policy revision;
5. stale recommendation cannot silently refresh itself;
6. incomparable candidates may remain plural;
7. recommendation contains no execution capability;
8. LLM text cannot alter normative structured fields;
9. model confidence cannot create authority;
10. recommendation cannot reserve/spend/schedule/execute;
11. adoption event does not mutate original recommendation bytes;
12. adverse evidence cannot be silently removed;
13. conflicting models can remain unresolved;
14. downstream authority must be explicitly separate.

## Deliberate non-claims

REGEN-054 establishes no optimal treatment, governance mandate, consent, legal right, ethics approval, procurement authority, scheduling authority, process authority, device authority, physical execution, or actuation.

Its proposition is deliberately narrow: carry bounded experiment intelligence across a review boundary as evidence-bearing recommendation, never as authority.