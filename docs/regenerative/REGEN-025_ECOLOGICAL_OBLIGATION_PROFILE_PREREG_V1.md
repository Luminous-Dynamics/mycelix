# REGEN-025 — Ecological Obligation Profile Preregistration v1

Status: preregistration only. This document defines a cross-domain ecological-constraint/obligation boundary for regenerative systems. It does not define universal ecological thresholds, land-management prescriptions, treatment instructions, harvest limits, water-withdrawal limits, or physical-control authority.

## 1. Purpose

REGEN already has domain-specific ecological firewalls: biomass removal must not become sustainable because material exists; Water must not treat environmental water as a leftover; nutrient recovery must not ignore ecological loss pathways; residue routing must not convert local circularity into ecological benefit automatically.

REGEN-025 generalizes the shared theorem:

```text
resource/material/service opportunity
+ exact ecological obligation profile
+ exact evidence snapshot
+ explicit constraint evaluation
= reviewable ecological eligibility state
```

not:

```text
high sustainability/resilience/carbon/economic value
= ecological eligibility
```

## 2. Ecological obligations are not one score

The protocol MUST NOT reduce ecology to one canonical `ecology_score`, `nature_positive`, `biodiversity_score`, or weighted utility number by default.

An adopted ecological profile may contain multiple independent hard constraints and informational/preference dimensions.

Where one hard constraint fails or remains unresolved, surplus performance elsewhere cannot silently compensate unless the adopted profile explicitly defines that relation and the relation itself is appropriate to the claim.

## 3. Cross-domain obligation classes

An adopted profile may define obligations concerning, for example:

```text
soil cover / erosion protection
soil-organic-matter retention
nutrient-export limits
habitat/deadwood/refuge retention
protected/sensitive species
riparian/wetland buffers
environmental water/flow
withdrawal/depletion/recharge constraints
landscape connectivity
pollinator habitat
seasonal/phenological restrictions
biosecurity/disease constraints
fire/fuel ecological constraints
material-removal limits
water-quality discharge constraints
```

This list is illustrative, not a universal ecology standard.

## 4. Exact profile identity is mandatory

A positive or negative ecological conclusion must bind an exact profile/rule revision rather than an ambient claim that something is `sustainable`.

Conceptually:

```rust
pub struct EcologicalProfileRef(String);
```

The first implementation may use a bounded opaque exact reference rather than introduce a global canonical ecology ontology.

```text
profile referenced
!= issuer authenticated
!= profile adopted legitimately
!= profile scientifically adequate
```

Those remain separate propositions.

## 5. Obligation identity and result are separate

A profile may carry several obligations. Each obligation should have a stable exact key/ref and one separately reviewable result.

Conceptually:

```rust
pub enum EcologicalConstraintResult {
    Satisfied(...),
    Violated(...),
    Unresolved(...),
}
```

Avoid free-standing booleans that can coexist with contradictory evidence/reasons.

## 6. Unresolved is not pass

```text
no evidence of harm
!= constraint satisfied
```

and:

```text
constraint not measured
!= zero impact
```

An `Unresolved` obligation cannot become a positive ecological eligibility theorem merely because no explicit violation was recorded.

## 7. Hard versus informational dimensions are explicit

The adopted profile should distinguish at least:

```text
HardConstraint
InformationalIndicator
PreferenceObjective
```

or an equivalent exact policy structure.

Only the adopted profile determines which constraints block positive eligibility.

A model or UI must not infer that distinction from wording, severity labels, or confidence scores.

## 8. Hard constraints are non-compensable by default

For a hard obligation:

```text
FAIL or UNRESOLVED
+ high carbon benefit
+ high local resilience
+ high revenue
+ high yield
!= ecological PASS
```

This is the cross-domain generalization of REGEN-011B5.

## 9. Evidence admission is not ecological evaluation

REGEN-019D shared admission may validate the provenance/structure of PEF evidence used by an ecological assessment.

```text
all evidence validly admitted
!= ecological obligation satisfied
```

The ecological evaluator still binds exact profile logic, exact evidence snapshot, exact domain subject, and any explicit derivation/model lineage.

## 10. Currentness is obligation-specific

REGEN-019B currentness applies independently to relevant evidence inputs.

One current observation does not make every ecological dimension current.

```text
current water quantity
+ stale habitat evidence
!= current whole-profile ecological eligibility
```

The assessment records the exact evidence snapshot used.

## 11. Spatial scope is explicit

REGEN-019A owns exact geometry/containment/sampling-frame relations.

Ecological obligations may apply to different spatial supports:

```text
lot/source area
plot/site
riparian buffer
watershed
habitat patch
landscape/corridor
region
```

A regional indicator cannot automatically satisfy a site-scale obligation or vice versa.

## 12. Representativeness remains separate

A valid sample/observation does not necessarily represent the subject or spatial extent required by an ecological obligation.

REGEN-018 and REGEN-019A preserve specimen/sampling/spatial relations where applicable.

## 13. Temporal scope remains explicit

An obligation may concern an instant, interval, season, trend, or cumulative condition.

```text
one-time PASS
!= long-term ecological sufficiency
```

Likewise a historical violation does not automatically prove current violation after conditions change.

## 14. Baseline and counterfactual are not interchangeable

Where a claim compares an intervention to a baseline/counterfactual, the baseline and method must be explicit.

```text
current state acceptable
!= intervention caused improvement
```

and:

```text
post-intervention improvement
!= no ecological tradeoff elsewhere
```

REGEN-015 remains the field/causal evidence boundary for interventions.

## 15. Domain-specific ownership remains local

REGEN-025 provides the shared ecological-obligation grammar, not domain-specific science.

Examples:

- REGEN-011B5 consumes it for biomass-removal ecology;
- REGEN-023/Water consumes it for withdrawal/environmental-flow constraints;
- REGEN-020 may consume it for nutrient-export/loss constraints;
- REGEN-024 may consume it for residual diversion/removal constraints;
- soil/food/seed domains may consume it where explicit ecological profiles apply.

No domain loses ownership of its detailed evidence/context.

## 16. Material mass accounting remains separate

A material partition or conservation balance cannot establish ecological sufficiency by itself.

```text
mass retained
!= ecological function retained
```

Likewise ecological eligibility does not create or increase physical material quantity.

## 17. Water accounting remains separate

```text
environmental flow obligation satisfied
!= water physically available for use
```

and:

```text
water available
!= environmental flow obligation satisfied
```

REGEN-023/Mycelix Water remain authoritative for water-domain state.

## 18. Nutrient accounting remains separate

```text
nutrient recovered
!= nutrient export/ecological loss constraint satisfied
```

A recovered material may still involve excessive upstream extraction, runoff, emissions, or other ecological costs.

REGEN-020 keeps nutrient stocks/flows explicit.

## 19. Habitat service cannot be inferred from material category

```text
deadwood / crop residue / vegetation mass present
!= habitat obligation satisfied
```

Ecological function can depend on size, structure, location, continuity, timing, species, decomposition state, landscape context, and other factors.

No universal biomass-retention fraction belongs in this core.

## 20. Biodiversity evidence remains plural

Species richness, abundance, occupancy, functional diversity, genetic diversity, habitat condition, connectivity, and other biodiversity dimensions are not interchangeable.

The profile identifies which proposition matters rather than collapsing them into one implicit biodiversity number.

## 21. Protected/legal ecology remains a separate authority layer

An ecological profile may reference protected-area, species, habitat, permit, stewardship, or other authoritative records.

REGEN-025 does not adjudicate law, title, land rights, treaty/customary rights, or regulatory status.

```text
legal constraint reference
!= scientific ecology result
```

and vice versa.

## 22. Rights and ecology can both block action

An ecologically eligible option may still be rights-ineligible; a rights-cleared option may still be ecologically ineligible.

```text
rights PASS ∩ ecology PASS
```

may both be required by a downstream policy.

Neither creates the other.

## 23. Climate/carbon remains separate

```text
carbon benefit
!= ecological benefit
```

A carbon-positive intervention may harm habitat, water, nutrients, or other ecological functions.

Conversely an ecological intervention may have uncertain carbon effects.

REGEN-070+ climate/carbon claims remain separate.

## 24. Locality and circularity remain non-authoritative

```text
local
!= ecologically eligible

circular
!= ecologically eligible
```

Local/circular properties can be preference/objective dimensions only after hard obligations are satisfied.

## 25. No optimizer authority over hard constraints

Symthaea or any other optimizer may receive:

```text
eligible candidate set
+ informational indicators
+ preference objectives
```

It may rank/explore among eligible candidates.

It MUST NOT alter an ecological obligation result from `Violated/Unresolved` to `Satisfied` because of modeled utility.

## 26. Policy/profile update does not rewrite history

A newer ecological profile revision creates a new assessment.

Old assessment records remain bound to the exact profile/evidence snapshot used at the time.

```text
new profile
!= retroactive mutation of old ecological result
```

## 27. Evidence update does not rewrite history

Likewise later environmental observations create a new evaluation state, not silent replacement of earlier evidence.

This supports audit/replay and prevents live queries from changing old decisions invisibly.

## 28. Assessment outcome is scoped

Conceptually:

```rust
pub struct EcologicalAssessment {
    pub assessment_ref: String,
    pub subject_ref: String,
    pub profile_ref: String,
    pub evidence_snapshot_ref: String,
    pub constraint_results: Vec<EcologicalConstraintResult>,
    pub overall: EcologicalEligibilityOutcome,
}
```

The exact implementation may differ, but a positive overall outcome must be mechanically consistent with every hard constraint result.

## 29. Overall outcome is fail-closed

Conceptually:

```rust
pub enum EcologicalEligibilityOutcome {
    Eligible,
    Ineligible,
    Unresolved,
}
```

Rules:

```text
any hard Violated -> Ineligible
else any hard Unresolved -> Unresolved
else all hard Satisfied -> Eligible
```

Informational/preference dimensions do not change that hard-gate reduction.

This is deterministic policy evaluation, not ecological science itself.

## 30. Empty hard-constraint set needs explicit policy

The core must not assume that a profile with zero hard constraints means universal ecological eligibility.

A profile must explicitly declare whether an empty hard-constraint set is valid for its purpose. Default first implementation should fail closed unless the profile contract explicitly permits it.

## 31. Duplicate/contradictory constraints fail

Within one exact profile/evaluation, duplicate obligation IDs or contradictory duplicate results must be rejected rather than deduplicated by dropping one.

No “last write wins” ecological theorem.

## 32. Reason taxonomy remains stable

`Violated` and `Unresolved` states should preserve stable reason codes plus evidence references.

Free-form explanation may supplement but does not control eligibility logic.

## 33. Unknown evidence is not zero impact

The system explicitly preserves missing/unmeasured/unresolved ecological dimensions instead of coercing them to favorable defaults.

This includes absent emission, loss, species, habitat, soil, or water observations.

## 34. Cross-domain common-mode effects remain visible

One intervention may satisfy a local constraint while shifting impact elsewhere.

Examples conceptually include:

```text
biomass removal -> soil/habitat/nutrient effects
water diversion -> downstream/ecosystem effects
residue diversion -> soil nutrient/cover effects
local production -> upstream imported material/energy effects
```

REGEN-025 supports multiple scoped obligations rather than declaring success from one local indicator.

## 35. Cumulative obligations need explicit lineage

Where cumulative extraction/loss/use matters, a one-event assessment cannot silently reset the cumulative ledger.

Cumulative or rolling-window evaluations need an exact state/history/evidence theorem defined by the adopted profile.

## 36. Reversibility/recovery is informational unless made hard

Recovery time, reversibility, restoration options, and resilience may be important ecology dimensions, but they do not automatically override direct hard constraints.

A profile may explicitly make them hard requirements where appropriate.

## 37. No physical action authority

An `EcologicalEligibilityOutcome::Eligible` is not a harvest permit, water-withdrawal command, land-use authorization, process start token, or actuator capability.

```text
ecological eligibility
!= social/legal authority
!= physical feasibility
!= execution authority
```

## 38. First executable direction

Prefer a tiny dependency-light policy/evidence composition crate/module, for example:

```text
crates/mycelix-regenerative-ecology
```

Likely foundation:

```text
mycelix-regenerative-core
mycelix-regenerative-admission
```

with domain evidence supplied through adapters/references. Avoid Holochain/runtime/control dependencies inside the deterministic evaluator.

## 39. First regression campaign

A future implementation should test at least:

1. one hard violated constraint -> overall Ineligible;
2. one hard unresolved with none violated -> overall Unresolved;
3. all hard satisfied -> overall Eligible;
4. informational violation cannot alter hard-gate result unless profile says it is hard;
5. preference objective cannot alter hard-gate result;
6. carbon benefit cannot override hard ecological violation;
7. resilience value cannot override unresolved hard ecology;
8. local/circular flags cannot override hard ecology;
9. material partition alone cannot produce ecological eligibility;
10. exact profile revision remains bound;
11. exact evidence snapshot remains bound;
12. changed profile creates a new assessment rather than rewriting old result;
13. changed evidence creates a new assessment;
14. duplicate constraint IDs fail;
15. contradictory duplicates fail;
16. zero hard constraints fail closed unless explicitly profile-permitted;
17. spatially mismatched evidence cannot be silently treated as satisfying a scoped obligation;
18. stale evidence cannot be silently treated as current enough;
19. valid PEF admission alone cannot satisfy an ecological constraint;
20. rights PASS does not create ecology PASS;
21. ecology PASS does not create rights PASS;
22. eligible ecology outcome carries no execution/actuation authority.

## 40. ProductFrozen qualification target

Executable ecology-policy code should follow REGEN-008/008A:

```text
authored source
-> explicit pinned-toolchain preparation
-> machine-preserved source + lock
-> exact-byte ProductFrozen promotion
-> exact-head qualification
-> Q001 machine-readable receipt validation
```

Its qualification claim remains policy/evidence consistency only, not real ecological truth.

## 41. Deliberate non-claims

REGEN-025 establishes no universal biodiversity standard, habitat threshold, soil-retention percentage, water-withdrawal fraction, species assessment, land-management instruction, harvest prescription, environmental permit, legal right, carbon claim, process authority, or physical actuation.

Its proposition is narrow:

> regenerative domains can bind exact, plural, fail-closed ecological obligations and evidence snapshots so hard ecological constraints remain explicit and non-compensable rather than being hidden inside mass balances or aggregate sustainability scores.
