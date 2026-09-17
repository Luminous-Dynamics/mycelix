# REGEN-050 — Bounded Trial Candidate Generator v1

Status: preregistration only. This contract defines a recommendation-only experiment-candidate surface. It does not authorize, schedule, finance, execute, or physically apply any intervention.

## 1. Purpose

Symthaea/Mycelix should be able to generate useful local regenerative trial candidates without allowing an optimizer to create new permissions, silently widen hard constraints, erase a control/null option, or turn model confidence into physical authority.

Core theorem:

```text
frozen design space
+ exact evidence snapshot
+ exact eligible materials/sites
+ explicit endpoint set
+ hard constraint profile
+ deterministic candidate rules
= bounded trial candidates
```

not:

```text
model can imagine candidate
= candidate is eligible
= candidate should be run
= candidate is authorized
= candidate may be executed
```

## 2. Upstream contracts

REGEN-050 composes, where applicable:

- REGEN-010/019 shared evidence admission;
- REGEN-011–017 material, contamination, suitability and field-trial contracts;
- REGEN-018/019A/019B specimen, spatial and currentness semantics;
- REGEN-020–027 nutrient/food/water/ecology/recipe evidence;
- REGEN-040–047 resilience and common-mode analysis.

It does not replace any of them.

## 3. Candidate input capsule

One generator invocation should bind an exact input capsule containing at least:

- site/plot/context subject identities;
- exact evidence snapshot;
- exact admissible material/input identities;
- exact adopted safety/quality/ecology/rights profiles;
- exact allowed factor definitions and ranges;
- exact endpoint set;
- exact control/null-arm policy;
- exact trial-resource budget/profile;
- exact generator policy revision;
- deterministic seed when stochastic enumeration is intentionally used.

Missing required inputs fail closed or produce an explicitly unresolved candidate set.

## 4. Hard design-space intersection

Candidate generation is an intersection operation:

```text
CandidateSpace =
    DeclaredFactors
  ∩ QualifiedInputEligibility
  ∩ SiteSuitability
  ∩ EcologicalConstraints
  ∩ Rights/Consent/AuthorityConstraints
  ∩ SafetyConstraints
  ∩ TrialResourceConstraints
  ∩ ProtocolConstraints
```

No soft objective may enlarge that intersection.

## 5. Null/control candidate

The generator must preserve an explicit null/control option whenever the adopted trial protocol requires one.

It must also be able to return:

```text
NoEligibleInterventionCandidate
```

when evidence or eligibility is insufficient.

Therefore:

```text
candidate generator invoked
!= intervention required
```

## 6. No optimizer-created authority

The generator receives authority/eligibility boundaries as inputs. It does not issue them.

It cannot:

- create land/access/removal rights;
- create consent;
- create material qualification;
- create ecological eligibility;
- create process authority;
- create physical actuation authority;
- waive contamination or safety gates;
- upgrade unresolved evidence into PASS.

## 7. Candidate identity

Each candidate should have a stable identity/digest derived from its canonical structured content and generator/input capsule identity.

Candidate identity must not depend on free-form explanation text.

Two semantically different factor assignments must not share one identity.

## 8. Factor semantics

Each factor must bind:

- factor identity;
- value/type/unit or categorical domain;
- allowed range/set;
- exact provenance/profile that established the allowed domain;
- whether it is controlled, observed-only, stratification-only, or prohibited from intervention.

Observed covariates must never become intervention knobs merely because a model can vary them numerically.

## 9. Candidate feasibility states

At minimum:

```text
EligibleCandidate
IneligibleCandidate { reasons }
UnresolvedCandidate { reasons }
ControlCandidate
```

Ineligible and unresolved candidates remain visible for audit and must not be silently dropped from retained generation evidence.

## 10. Endpoint boundary

Candidate generation may bind a preregistered endpoint set but may not select new favorable endpoints after seeing outcomes.

```text
candidate generation
!= endpoint shopping
```

REGEN-015 trial evidence remains the owner of outcome/deviation/missingness semantics.

## 11. Information and utility separation

REGEN-050 may enumerate candidates and structural metadata needed by later selectors.

It should not itself collapse:

- expected information;
- predicted agronomic benefit;
- cost;
- resilience value;
- carbon implications;
- ecological burden;
- operational effort

into one hidden utility score.

REGEN-051 may evaluate expected information under a separately frozen theorem.

## 12. Learned-model firewall

Learned/HDC/LLM models may suggest or score candidate ideas only inside the already-frozen admissible domain.

A model-proposed value outside the domain is rejected, not clipped into eligibility without explicit canonical rules.

Model confidence does not widen candidate bounds.

## 13. Determinism and replay

For deterministic modes, identical canonical inputs must yield the identical ordered candidate set.

For stochastic enumeration, the exact seed/algorithm/version must make the generation replayable.

Nondeterministic external calls do not belong in the normative candidate kernel.

## 14. Candidate-count limits

The generator requires explicit bounded limits to prevent combinatorial explosion from becoming an accidental denial-of-service or hidden search heuristic.

Truncation must be observable and reproducible.

```text
truncated set
!= complete design space
```

## 15. Diversity without privilege

Candidate diversity may be tracked across factor-space coverage, but diversity itself is not evidence of quality, safety, or expected benefit.

A candidate must never be retained merely to satisfy a diversity quota if it violates a hard gate.

## 16. Existing recipe lineage

REGEN-027 recipes may seed candidate definitions, but:

```text
recipe exists
!= candidate eligible here
```

The candidate must still satisfy current site/material/evidence/profile constraints.

Local adaptations become explicit candidate revisions, not silent mutations of the parent recipe.

## 17. Resilience experiment boundary

REGEN-040–047 outcomes may motivate trial questions, but synthetic resilience results cannot manufacture real-world treatment eligibility.

A candidate targeting resilience must name the exact service and outcome dimensions it is intended to observe.

## 18. Negative and null candidate evidence

The system retains:

- no-eligible-candidate results;
- candidates rejected by hard gates;
- candidates unresolved because evidence is missing;
- generator truncation;
- control/null candidates.

These are first-class evidence, not failures to hide.

## 19. Human/adopted policy boundary

Candidate generation is recommendation infrastructure.

A later adoption/authorization layer decides whether any candidate becomes a protocol.

```text
GeneratedCandidate
!= AdoptedTrialProtocol
!= AuthorizedIntervention
!= ExecutedIntervention
```

## 20. Initial executable direction

A first dependency-light kernel should contain only:

- canonical factor/domain structures;
- exact input capsule references;
- hard intersection validation;
- deterministic candidate enumeration;
- bounded candidate-count handling;
- stable candidate identity;
- null/control preservation;
- fail-closed result states.

No Holochain runtime, LLM runtime, optimizer service, database/network client, marketplace, actuator, or physical-control dependency belongs in the normative core.

## 21. Initial qualification targets

At least:

1. deterministic replay;
2. no candidate outside any hard factor bound;
3. unresolved input cannot become eligible candidate;
4. null/control preservation;
5. zero eligible candidates is valid;
6. model confidence cannot widen bounds;
7. observed-only variable cannot become intervention factor;
8. exact candidate identity changes when semantics change;
9. truncation is explicit/replayable;
10. ineligible/unresolved candidate reasons retained;
11. no authority/execution field in candidate core;
12. recipe input does not bypass current eligibility;
13. soft objective cannot rescue hard-gate failure;
14. duplicate semantic candidates deduplicate deterministically or fail explicitly according to policy;
15. candidate order remains deterministic under canonical input ordering.

## 22. Deliberate non-claims

REGEN-050 establishes no agronomic efficacy, material safety, ecological benefit, causal effect, optimal treatment, trial ethics approval, consent, funding approval, land/process authority, process execution, carbon claim, market recommendation, or physical actuation.

Its proposition is deliberately narrow: generate reviewable bounded experiment candidates without widening the authority or eligibility supplied to it.
