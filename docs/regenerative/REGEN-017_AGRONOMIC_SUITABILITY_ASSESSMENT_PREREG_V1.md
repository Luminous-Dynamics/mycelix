# REGEN-017 — Agronomic Suitability Assessment Preregistration v1

Status: architecture preregistration only. No universal agronomic recommendation, application instruction, governance authority, climate authority, or physical-action authority.

## 1. Purpose

Freeze the evidence and claim boundary for determining whether a particular regenerative material may be suitable for a particular agronomic/restoration context.

The central theorem is:

```text
material quality conformance
!= agronomic suitability
```

and:

```text
suitability in context A
!= suitability in context B
```

## 2. Suitability is relational

Agronomic suitability is not an intrinsic scalar stored on a material batch.

A bounded assessment relates at least:

```text
exact material subject
x exact site/plot context
x intended use/purpose
x crop/plant/restoration target when applicable
x evidence snapshot
x adopted decision profile
-> bounded assessment result
```

Changing any material factor may require a new assessment.

## 3. Subject identity

The assessment MUST bind the exact material subject under evaluation, such as a `BiocharBatchId`, `CompostBatchId`, or `CoCompostedAmendmentBatchId`.

```text
material class
!= exact batch identity
```

A later split, blend, reprocessing event, or replacement batch does not inherit an earlier suitability result automatically.

## 4. Site/plot context

The assessment SHOULD bind exact `RegenerativeSiteId` and `SoilPlotId` context where the proposition is plot-specific.

REGEN-010 evidence supplies soil observations; the suitability layer does not duplicate pH, carbon, nutrient, water, contamination, spatial, temporal, uncertainty, or provenance fields.

```text
site identity
!= site state
```

Currentness of site evidence remains an explicit consuming-policy question.

## 5. Intended-use context

An assessment SHOULD declare its intended purpose/use, because the evidence needed for one purpose may differ from another.

Examples may include soil conditioning, nutrient-management support, water-management support, restoration research, trial eligibility, or another declared use.

The core does not define a universal list of beneficial uses and does not assert efficacy from the label.

## 6. Biological target context

Where relevant, the assessment SHOULD bind the crop, plant community, restoration target, or other biological context.

```text
suitable for target X
!= suitable for all crops/species
```

Unknown target context remains unknown rather than being replaced by a generic default.

## 7. Evidence snapshot

A suitability result SHOULD bind an exact evidence snapshot/bundle identity or equivalent immutable reference sufficient to reconstruct which observations and material evidence were evaluated.

```text
same material + later evidence
!= same assessment proposition
```

A reassessment after new evidence creates a new revision/result rather than rewriting history.

## 8. Quality-profile relation

REGEN-003 may bind an exact adopted material-quality or suitability profile.

A profile conformance result can be a prerequisite, but:

```text
material conforms to profile P
!= material suitable for site S
```

The suitability profile/rule set itself should be versioned/digest-bound and its adoption/authority remains separate.

## 9. Contamination boundary

Where an adopted suitability profile requires contaminant/hazard evidence, REGEN-016 provides that evidence interpretation.

```text
missing required contamination evidence
-> insufficient evidence / fail-closed under profile
```

not:

```text
missing evidence -> clean
```

Passing one contamination profile still does not establish broader agronomic efficacy.

## 10. Material-property evidence

Material composition/properties remain evidence about the exact material subject through PEF and the appropriate batch lineage.

Input material properties do not automatically substitute for transformed-output properties.

## 11. Outcome model

The generic assessment should preserve a non-totalizing result family conceptually equivalent to:

```text
SuitableUnderProfile
ConditionallySuitableUnderProfile
UnsuitableUnderProfile
InsufficientEvidence
NotEvaluated
```

Exact names may change in implementation.

Every favorable/unfavorable result is scoped to the exact adopted profile, context, and evidence snapshot.

```text
SuitableUnderProfile(P)
!= universally suitable
```

## 12. Conditional suitability

A conditional result MUST preserve its exact conditions/restrictions as reviewable data or references.

Conditions do not create execution authority.

```text
condition satisfied in assessment
!= field operation authorized
```

## 13. Hard constraints vs preferences

The assessment architecture SHOULD separate hard eligibility/safety constraints from soft preferences or optimization objectives.

Examples of hard gates may be defined by an adopted profile; the generic core does not hard-code universal thresholds.

A high predicted yield, local sourcing advantage, low cost, or climate benefit MUST NOT compensate for a failed hard eligibility constraint.

## 14. No master suitability score

REGEN-017 MUST NOT define a canonical scalar such as:

```text
suitability = 0.83
```

as the universal decision primitive.

Plural evidence, constraints, conditions, uncertainty, and reasons remain inspectable. A later profile may calculate an explicit derived score if needed, but the score retains lineage and does not erase hard-gate outcomes.

## 15. Trial evidence relation

REGEN-015 field-trial evidence may inform suitability assessment.

```text
one successful trial
!= universal suitability
```

Trial context, design, material batch, site, crop, season, endpoints, deviations, and uncertainty remain visible.

A null/adverse trial is first-class evidence and must not be filtered out merely because other trials were favorable.

## 16. Model/inference boundary

Symthaea or another model may generate a suitability hypothesis/recommendation using qualified evidence.

Such a result is `Inferred`/model-derived where appropriate and requires provenance lineage.

```text
model confidence
!= agronomic authority
!= application authority
```

A human/institutional decision layer remains separate.

## 17. Freshness/currentness

A structurally valid assessment is not automatically current forever.

Freshness/expiry requirements belong to the adopted profile/use context and may depend on the underlying evidence types.

```text
valid historical evidence
!= current-enough evidence
```

Unknown temporal support remains unknown.

## 18. Spatial/representativeness boundary

A soil observation bound to a plot does not itself prove that the sample represents the entire plot. Likewise, one plot does not automatically represent an entire farm, region, or soil class.

Suitability claims MUST remain scoped accordingly.

## 19. Legal/rights boundary

A favorable agronomic suitability assessment does not establish:

- ownership or stewardship rights;
- permission to access land;
- permission to apply material;
- regulatory compliance;
- food/feed approval;
- water rights;
- community consent.

Those remain with their owning domains/jurisdictions.

## 20. Climate/carbon boundary

```text
agronomically suitable
!= climate-beneficial
!= durable carbon removal
!= carbon-credit eligible
```

Climate/MRV remains downstream and independent.

## 21. Economic/resilience boundary

```text
agronomically suitable
!= economically optimal
!= resilience-optimal
```

Cost, labor, logistics, local production, ecological opportunity cost, and resilience remain plural downstream considerations.

## 22. Proposed executable layering

A future dependency-light suitability crate/profile SHOULD compose:

```text
qualified REGEN-010 soil/evidence waist
+ exact qualified material/batch lineage
+ REGEN-016 contamination evidence where required
+ optional qualified REGEN-003 adopted quality/suitability profile
+ REGEN-015 trial evidence where available
```

It SHOULD NOT directly depend on physical-control runtimes. Symthaea integration should be a recommendation-only adapter.

## 23. Qualification target

Future implementation should demonstrate at least:

1. exact material/site/plot identity binding;
2. exact evidence-snapshot binding;
3. adopted-profile revision binding;
4. missing required evidence -> insufficient/fail-closed result;
5. hard constraints cannot be offset by soft scores/preferences;
6. context changes require distinct/revised assessment;
7. trial evidence preserves null/adverse outcomes and context;
8. PEF raw-vs-lineaged evidence preservation;
9. no automatic action/authority field;
10. serde/top-level revalidation where enabled;
11. ProductFrozen dependency qualification per REGEN-008;
12. exact ProductHead and clean checkout evidence.

## 24. Deliberate non-claims

REGEN-017 preregistration establishes no universal suitability rule, application rate, crop response, treatment efficacy, food/feed safety, legal permission, climate benefit, carbon removal, economic superiority, resilience superiority, governance authority, field-operation authority, or physical actuation.
