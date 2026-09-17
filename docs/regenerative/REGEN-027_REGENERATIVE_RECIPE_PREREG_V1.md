# REGEN-027 — Regenerative Recipe Artifact Preregistration v1

Status: preregistration only. This document freezes the semantics of a versioned regenerative procedure/recipe artifact so local experimental knowledge can be shared without turning one successful context into universal advice, safety authority, or automatic execution.

## 1. Purpose

REGEN already freezes evidence, material, soil, biomass, water, nutrient, ecological, suitability, and trial semantics. REGEN-027 defines how a reusable human-readable/machine-readable procedure lineage may reference those propositions without replacing them.

Core theorem:

```text
versioned recipe artifact
+ exact applicability context
+ exact input requirements
+ explicit evidence lineage
+ explicit non-claims
= reproducible knowledge candidate
```

not:

```text
recipe exists
= safe
= suitable everywhere
= validated
= authorized
= automatically executable
```

## 2. Identity

REGEN-027 reuses the already-qualified `RegenerativeRecipeId` from REGEN-002.

A stable recipe identity does not imply one immutable body forever. A recipe lineage should also bind an exact revision/version identity and content digest.

Conceptually:

```text
recipe_id
+ revision
+ content_digest
```

Changing consequential procedure semantics produces a new revision. Silent mutation of an old revision is not allowed.

## 3. Recipe vs evidence

A recipe may reference evidence, but is not itself observational truth.

```text
recipe claim
!= observation
!= trial result
!= scientific conclusion
```

Any expected effect should point to supporting evidence/trials and preserve whether that support is Observed, Derived, Inferred, Forecast, or Scenario.

## 4. Context is mandatory

A recipe must not be represented as context-free.

Applicability may reference exact contextual propositions such as:

- site/plot or site-class context;
- soil evidence/profile;
- climate/weather context;
- water source/use-suitability context;
- biomass/feedstock profile;
- biochar/compost/amendment profile;
- contamination profile;
- ecological-obligation profile;
- crop/biological target;
- infrastructure/facility class;
- adopted quality profile;
- regulatory/rights prerequisites where relevant.

Missing context remains missing.

```text
successful elsewhere
!= applicable here
```

## 5. Input requirements are predicates, not inventory

A recipe may define required input properties or exact profile references.

It must not turn those requirements into claims that the inputs are actually present, safe, rights-cleared, or qualified.

```text
recipe requires qualified feedstock
!= qualified feedstock exists
```

Actual inputs remain bound through REGEN-011/012/013/014/016/017 and authoritative external systems.

## 6. Procedure description vs machine command

A recipe may describe ordered conceptual steps, dependencies, hold points, observation requirements, and verification checkpoints.

It must not contain a privileged machine-control object or create physical execution authority.

```text
procedure step
!= actuator command
```

Any later automation belongs behind the separate consequential-physical-action safety architecture and device/process-specific authority.

## 7. No hidden universal operating values

REGEN-027 should not use one global recipe to hard-code universal pyrolysis settings, composting durations, amendment rates, irrigation quantities, nutrient doses, or other context-sensitive operational values.

Where a recipe contains bounded parameters, their applicability must be scoped to an exact revision/context/profile and remain distinct from authorization.

A field-trial recipe may deliberately use abstract treatment identifiers or externally adopted parameter profiles.

## 8. Preconditions

A recipe may declare explicit preconditions such as:

```text
required quality profile PASS
required contamination assessment PASS
required ecological eligibility PASS
required water-use suitability PASS
required rights/authority reference present
required equipment/facility capability PASS
```

The recipe evaluator may report an unmet prerequisite. It must not manufacture the prerequisite.

## 9. Contraindications and exclusions

A recipe may carry explicit exclusions/contraindication references.

Absence of a known exclusion is not proof of universal safety.

```text
no listed contraindication
!= no possible hazard
```

Unknown or unresolved required exclusion checks fail closed for any consequential downstream recommendation that depends on them.

## 10. Expected outcomes

Expected outcomes should be plural and evidence-bearing rather than one scalar success score.

Examples may include:

- crop/biomass service;
- soil state;
- water service;
- nutrient state;
- ecological indicators;
- energy/material coproduct service;
- labor/infrastructure requirements;
- economic observations;
- carbon/climate observations.

No dimension automatically substitutes for another.

## 11. Prediction vs result

A recipe may contain expected or modeled outcomes, but those remain forecasts/scenarios.

```text
expected yield improvement
!= observed yield improvement
```

Actual results belong in REGEN-015 field-trial/outcome evidence and related observations.

## 12. Replication lineage

Using an existing recipe in a new site/trial should create a replication/use record rather than rewriting the original recipe as successful there.

Conceptually:

```text
recipe revision
-> application/trial protocol reference
-> observed outcome bundle
```

A later recipe revision may cite that new evidence.

## 13. Forks and local adaptation

Local adaptation is expected.

Changing material assumptions, procedure semantics, context bounds, parameter profiles, or decision gates should create a new revision/fork lineage with provenance to the parent.

```text
local adaptation
!= corruption of canonical parent history
```

## 14. Negative and null evidence

Recipes must be able to accumulate evidence of null, adverse, contradictory, or context-specific outcomes.

A recommendation/ranking layer must not consider only positive replications.

```text
successful trials only
!= evidence base
```

## 15. Evidence snapshot

A recipe assessment should identify the exact evidence snapshot used to decide applicability/recommendation.

REGEN-019B applies:

```text
same recipe revision
+ different evidence snapshot
= potentially different applicability result
```

Historical assessments are not silently rewritten by newer evidence.

## 16. Qualification state

Recipe syntax/structural qualification is distinct from scientific validation.

Potential states should preserve distinctions such as:

```text
authored
structurally_validated
replicated_in_context
qualified_for_exact_profile
superseded
withdrawn
```

The protocol must not use one boolean `validated=true` to conflate these propositions.

## 17. Recommendation boundary

Symthaea may compare candidate recipes only after hard prerequisites are satisfied or explicitly represented as unresolved.

Model confidence cannot turn an unresolved prerequisite into PASS.

```text
model prefers recipe A
!= recipe A is authorized
```

## 18. Carbon/climate firewall

Agronomic/ecological recipe success does not imply carbon-removal qualification.

```text
successful regenerative recipe
!= carbon removal
!= carbon credit
```

Climate authority remains with its separate methodology/MRV chain.

## 19. Economic firewall

Low cost, high return, local sourcing, or strong market demand cannot override safety, contamination, ecological, rights, or suitability gates.

Economic performance is a downstream objective among already-eligible alternatives.

## 20. Privacy and sensitive knowledge

Not all recipes must be globally public.

The artifact model should allow visibility/access references without treating secrecy as invalidity or disclosure as proof.

This may matter for private farm process knowledge, indigenous/traditional knowledge, commercial methods, or sensitive ecological locations.

Rights/licensing/attribution remain separate explicit propositions.

## 21. Authorship and attribution

Recipe provenance should preserve exact authorship/contributor references, parent revisions, source evidence, licensing/usage terms where applicable, and revision history.

Authorship does not imply scientific validity or authority.

## 22. Deterministic structural core

A later dependency-light `mycelix-regenerative-recipes` crate may own only deterministic structure/validation such as:

```text
RegenerativeRecipe
RecipeRevision
ApplicabilityRequirement
InputRequirement
ProcedureStepRef
ExpectedOutcomeRef
ContraindicationRef
RecipeLineage
```

It should not own databases, Holochain, marketplaces, Symthaea, physical device control, or domain-specific scientific engines.

## 23. First regression campaign

A first executable campaign should prove at least:

1. recipe ID and exact revision/content identity remain separate;
2. empty applicability context cannot masquerade as universal applicability unless an exact profile explicitly permits it;
3. input requirement cannot instantiate inventory;
4. required quality profile unresolved prevents positive applicability;
5. required contamination gate failure prevents positive applicability;
6. required ecological gate failure prevents positive applicability;
7. rights prerequisite unresolved prevents positive applicability where required;
8. procedure description contains no actuator authority;
9. expected outcome remains distinct from observed outcome;
10. forecast/scenario evidence class remains preserved;
11. one positive trial cannot mark recipe universally validated;
12. null/adverse replication remains representable;
13. fork/revision preserves parent provenance;
14. materially changed procedure cannot retain old revision digest;
15. new evidence snapshot does not mutate old applicability result;
16. local adaptation does not rewrite parent history;
17. agronomic success does not create carbon authority;
18. economic benefit cannot override hard gates;
19. model preference cannot create authorization;
20. withdrawal/supersession remains explicit rather than deleting history;
21. attribution/licensing metadata does not create scientific truth;
22. no record carries physical-execution authority.

## 24. Terra-Preta-specific significance

Terra-Preta-inspired systems are a primary motivating use case for this artifact because outcomes depend strongly on feedstock, process history, soil, climate, crop, water, contamination profile, amendment combination, and local management.

REGEN-027 therefore deliberately encourages:

```text
local recipe lineage
+ small trial
+ measured outcome
+ replication/adaptation
```

rather than:

```text
one universal biochar/compost recipe
```

## 25. Qualification discipline

Any executable recipe core should follow REGEN-008/008A:

```text
authored source
-> pinned-toolchain preparation
-> machine-preserved source + lock
-> ProductFrozen promotion
-> exact-head qualification
-> Q001 receipt
```

Scientific efficacy remains a separate evidence proposition even after software qualification.

## 26. Deliberate non-claims

REGEN-027 establishes no universal agricultural recommendation, soil-treatment rate, pyrolysis recipe, composting recipe, irrigation instruction, nutrient dose, food-safety procedure, environmental permit, rights determination, agronomic efficacy, carbon removal, market value, governance authority, process-execution authority, or physical actuation.

Its proposition is deliberately narrow: regenerative procedural knowledge should be versioned, contextual, evidence-linked, forkable, reviewable, and impossible to mistake for universal truth or automatic authority.
