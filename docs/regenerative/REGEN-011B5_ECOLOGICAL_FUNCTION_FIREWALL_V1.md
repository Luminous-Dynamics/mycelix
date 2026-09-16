# REGEN-011B5 — Ecological Function Firewall v1

Status: preregistration refinement only. This document prevents biomass mass-accounting constraints from being mistaken for ecological sustainability.

## 1. Core correction

A mass partition such as:

```text
accessible
= retained
+ allocable
+ unresolved_residual
```

is useful material accounting, but it does not prove that ecological functions are preserved.

Therefore:

```text
retained mass floor
!= ecological sufficiency
```

and:

```text
material allocation resolved
!= ecologically eligible removal
```

## 2. Separate material partition from ecology

The executable biomass core should separate two propositions:

```rust
pub struct BiomassMaterialPartition { ... }

pub enum EcologicalConstraintOutcome {
    Eligible(EcologicalEligibility),
    Ineligible(EcologicalIneligibility),
    Unresolved(EcologicalUnresolved),
}
```

The material partition owns exact checked quantity arithmetic.

The ecological constraint assessment owns the claim that the proposed removal is or is not compatible with an exact adopted ecological profile and evidence snapshot.

Do not use one type to imply both.

## 3. Material partition remains exact accounting

Conceptually:

```rust
pub struct BiomassMaterialPartition {
    pub partition_ref: String,
    pub subject_ref: String,
    pub accessible_mass: BiomassMass,
    pub retained_mass: BiomassMass,
    pub allocable_mass: BiomassMass,
    pub unresolved_residual_mass: BiomassMass,
}
```

All masses share one basis and satisfy exact checked arithmetic.

This record answers only:

> how is the declared accessible mass partitioned for planning/accounting?

It does not answer:

> does the retained material preserve the required ecological functions?

## 4. Ecological profile identity is explicit

A positive ecological eligibility theorem must bind an exact policy/profile/method revision:

```rust
pub struct EcologicalEligibility {
    pub assessment_ref: String,
    pub partition_ref: String,
    pub ecological_profile_ref: String,
    pub evidence_snapshot_ref: String,
    pub constraint_evidence_refs: Vec<String>,
}
```

`ecological_profile_ref` is opaque/bounded in v1 and must identify the exact adopted rule set or assessment procedure used.

No universal ecological threshold is hard-coded into the biomass core.

## 5. Non-mass ecological constraints are first-class

An ecological profile may require evidence/constraints concerning, for example:

- soil-cover continuity;
- erosion protection;
- standing/deadwood habitat;
- nesting/refuge structures;
- protected or sensitive species;
- spatial exclusion zones;
- riparian buffers;
- seasonal/phenological restrictions;
- nutrient export limits;
- soil-organic-matter retention;
- moisture/water-cycle impacts;
- disease/pest containment;
- fire/fuel-management constraints;
- grazing/feed competition;
- decomposer/fungal habitat;
- landscape connectivity.

This list is illustrative, not a universal ecological standard.

The key theorem is that any adopted non-mass constraint remains visible rather than being converted into one retained-mass percentage.

## 6. No weighted compensation

Ecological hard constraints cannot be compensated by unrelated benefits.

```text
habitat constraint FAIL
+ high carbon benefit
+ high revenue
+ high local resilience
!= ecological eligibility
```

Likewise:

```text
erosion constraint unresolved
+ abundant biomass mass
!= ecological eligibility
```

Economic, carbon, resilience, or model-confidence scores may rank options only after hard eligibility constraints are satisfied under the adopted profile.

## 7. Eligible feedstock requires both propositions

The positive feedstock constructor from REGEN-011B4 should consume both:

1. an exact `BiomassMaterialPartition` that bounds allocable mass; and
2. `EcologicalConstraintOutcome::Eligible` bound to that exact partition/profile/evidence snapshot.

Conceptually:

```rust
pub fn new_eligible(
    ...,
    partition: &BiomassMaterialPartition,
    ecology: &EcologicalEligibility,
    assessed_mass: BiomassMass,
    ...
) -> Result<FeedstockAssessment, BiomassError>;
```

It requires:

- `ecology.partition_ref == partition.partition_ref`;
- positive assessed mass;
- matching mass basis;
- `assessed_mass <= partition.allocable_mass`;
- exact ecological profile/evidence refs preserved.

An exact mass partition without ecological eligibility cannot mint `EligibleFeedstock`.

## 8. Ineligible ecology is explicit

Conceptually:

```rust
pub struct EcologicalIneligibility {
    pub assessment_ref: String,
    pub partition_ref: String,
    pub ecological_profile_ref: String,
    pub evidence_snapshot_ref: String,
    pub reason_codes: Vec<EcologicalConstraintReasonCode>,
}
```

Reason codes are non-empty and stable.

Examples of reason classes may include:

```rust
ProtectedAreaConstraint,
HabitatRetentionConstraint,
ErosionConstraint,
NutrientExportConstraint,
SeasonalConstraint,
WaterConstraint,
ProfileLimitExceeded,
```

Exact vocabulary belongs to implementation review and adopted profile semantics.

## 9. Unresolved ecology is distinct

`EcologicalUnresolved` is neither eligible nor ineligible.

It must preserve why the current evidence/profile evaluation cannot decide, such as:

```text
missing evidence
stale evidence
spatial relation unresolved
profile revision unavailable
sampling representativeness unresolved
method execution unresolved
```

Unresolved ecology cannot mint reservation capacity.

## 10. Evidence admission remains separate

REGEN-019D may validate the provenance/admission of ecological evidence inputs.

That does not prove the ecological assessment conclusion.

```text
all input observations validly admitted
!= ecological conclusion valid
```

The ecological assessment keeps its own exact method/profile and evidence snapshot.

## 11. Spatial semantics remain explicit

Many ecological constraints are spatial.

REGEN-019A remains the boundary for exact geometry/sampling-frame relationships.

A point/bounding box attached to PEF evidence is not automatically proof that the constraint applies to or is satisfied within the biomass source area.

## 12. Currentness remains explicit

REGEN-019B remains the boundary for purpose-specific currentness.

An ecological assessment must preserve the exact evidence snapshot it used so later updates do not silently rewrite an older eligibility decision.

## 13. Symthaea optimization boundary

Symthaea may later compare eligible biomass allocations for resilience, carbon, cost, logistics, energy, or other objectives.

It must receive hard ecological eligibility as a constraint, not invent or relax it as an optimizer weight.

Conceptually:

```text
candidate options
-> hard ecological eligibility filter
-> only then Pareto / optimization analysis
```

not:

```text
one weighted utility score
-> trade habitat loss for enough other benefit
```

## 14. Terra-Preta/biochar relevance

This firewall is especially important for local biochar systems.

```text
biomass residue exists
!= residue is ecologically surplus
```

Material that appears economically unused may still provide soil cover, nutrient return, moisture retention, habitat, fungal/decomposer substrate, or erosion protection.

REGEN must therefore never encourage residue removal solely because a pyrolysis facility can consume it.

## 15. No universal localism rule

Likewise:

```text
locally sourced
!= ecologically preferable
```

A local feedstock that violates ecological constraints can be worse than importing a qualified residual stream from elsewhere.

Locality may be an optimization/preference dimension only after hard ecological and safety gates.

## 16. Added error distinctions

The executable biomass error surface should distinguish at least:

```rust
EcologicalPartitionMismatch,
EcologicalProfileMissing,
EcologicalEvidenceSnapshotMismatch,
EcologicalConstraintIneligible,
EcologicalConstraintUnresolved,
```

These must not collapse into generic `FeedstockInvalid`.

## 17. Added regression requirements

The executable biomass campaign gains at least:

1. exact material partition alone cannot mint eligible feedstock;
2. eligible ecology bound to the exact partition can proceed;
3. ecology bound to a different partition fails;
4. ecology bound to a different evidence snapshot fails where exact snapshot match is required;
5. ecology ineligible cannot mint feedstock eligibility;
6. ecology unresolved cannot mint feedstock eligibility;
7. positive assessed mass above allocable partition mass still fails even with ecology eligible;
8. ecological eligibility does not increase allocable mass;
9. carbon/revenue/resilience metadata cannot override ecological ineligibility;
10. local-source metadata cannot override ecological ineligibility;
11. valid PEF inputs alone do not create ecological eligibility;
12. changed ecological profile revision requires a new assessment;
13. later evidence does not mutate an older eligibility record;
14. ecological eligibility creates no process execution authority.

## 18. Deliberate non-claims

This refinement defines no universal ecology standard and proves no real habitat, erosion, soil, nutrient, biodiversity, water, carbon, land-right, or sustainability outcome.

Its proposition is narrow:

> exact biomass mass allocation and ecological-function eligibility are separate propositions, and a positive feedstock path requires both under an exact adopted ecological profile/evidence snapshot.
