# REGEN-011B4 — Fail-Closed Biomass Assessment Outcomes v1

Status: preregistration refinement only. This document makes contradictory ecological-allocation and feedstock-assessment states harder to represent before executable biomass code exists.

## 1. Problem

Earlier sketches used independent fields such as:

```rust
resolution: AllocationResolution
```

and:

```rust
disposition: AssessmentDisposition
reason_codes: Vec<FeedstockReasonCode>
```

Those shapes permit contradictory combinations unless every constructor and deserializer remembers to reject them, for example:

```text
Eligible + failure reasons
Resolved + inconsistent mass partition
Unresolved + apparently authoritative positive capacity
```

The v1 executable design should make these states structurally difficult or impossible to construct.

## 2. Ecological allocation is an outcome enum

Replace the independent resolution flag with a variant-owned payload:

```rust
pub enum EcologicalAllocationOutcome {
    Resolved(ResolvedEcologicalAllocation),
    Unresolved(UnresolvedEcologicalAllocation),
}
```

The two variants do not share fields that would accidentally make an unresolved record look resolved.

## 3. Exact resolved mass partition

A resolved ecological allocation carries an exact same-basis partition:

```rust
pub struct ResolvedEcologicalAllocation {
    pub assessment_ref: String,
    pub subject_ref: String,
    pub accessible_mass: BiomassMass,
    pub retained_mass: BiomassMass,
    pub allocable_mass: BiomassMass,
    pub unresolved_residual_mass: BiomassMass,
    pub method_ref: String,
    pub evidence_refs: Vec<String>,
}
```

All four masses must use the same `BiomassMassBasis` and satisfy exact checked arithmetic:

```text
accessible_mass
=
retained_mass
+ allocable_mass
+ unresolved_residual_mass
```

This is stronger and more transparent than only requiring:

```text
retained + allocable <= accessible
```

because every unallocated gram remains visible instead of disappearing into an implicit remainder.

`unresolved_residual_mass > 0` is permitted. It means that mass is not available for allocation merely because the rest of the partition is resolved.

## 4. Unresolved allocation carries no allocable-capacity claim

Conceptually:

```rust
pub struct UnresolvedEcologicalAllocation {
    pub assessment_ref: String,
    pub subject_ref: String,
    pub method_ref: String,
    pub evidence_refs: Vec<String>,
    pub reason_codes: Vec<EcologicalAllocationReasonCode>,
}
```

The unresolved variant MUST NOT carry `allocable_mass`.

If an accessible quantity is useful context, it should be referenced through evidence or a separately typed optional contextual field that cannot be mistaken for allocable capacity.

An unresolved assessment cannot mint feedstock eligibility or reservation capacity.

## 5. Ecological reason taxonomy

The first core should define stable machine-readable unresolved/failure reasons, for example:

```rust
pub enum EcologicalAllocationReasonCode {
    EvidenceIncomplete,
    CurrentnessUnresolved,
    SpatialRelationUnresolved,
    RetentionRequirementUnresolved,
    QuantityBasisMismatch,
    MethodUnsupported,
}
```

The exact list may be refined during implementation, but an `Unresolved` allocation must carry at least one reason code.

## 6. Feedstock assessment uses an outcome enum

Replace an independent disposition + free-standing reason vector with:

```rust
pub enum FeedstockAssessmentOutcome {
    Eligible(EligibleFeedstock),
    Ineligible(IneligibleFeedstock),
    Unresolved(UnresolvedFeedstock),
}
```

A common outer record may preserve stable identity/scope:

```rust
pub struct FeedstockAssessment {
    pub assessment_ref: String,
    pub lot_id: BiomassLotId,
    pub state_snapshot_ref: String,
    pub process_profile_ref: String,
    pub outcome: FeedstockAssessmentOutcome,
}
```

## 7. Eligible outcome carries positive scope, not failure reasons

Conceptually:

```rust
pub struct EligibleFeedstock {
    pub assessed_mass: BiomassMass,
    pub ecological_allocation_ref: String,
    pub rights_resolution_ref: String,
    pub custody_resolution_ref: String,
    pub evidence_snapshot_ref: String,
    pub additional_prerequisite_refs: Vec<String>,
}
```

An eligible outcome does not carry failure reason codes.

It must be constructed only through a validating constructor that receives the exact prerequisite records or verified prerequisite references required by the v1 theorem.

## 8. Eligible mass is bounded by ecological allocation

The preferred constructor should take the resolved ecological allocation by reference:

```rust
pub fn new_eligible(
    ...,
    ecological: &ResolvedEcologicalAllocation,
    assessed_mass: BiomassMass,
    ...
) -> Result<FeedstockAssessment, BiomassError>;
```

It requires:

- exact matching mass basis;
- `assessed_mass.amount > 0`;
- `assessed_mass <= ecological.allocable_mass`;
- matching subject/lot scope where the domain model provides that relation;
- valid bounded prerequisite references.

This prevents an eligible feedstock assessment from claiming more process-eligible mass than the exact ecological allocation makes allocable.

## 9. Ineligible outcome requires reasons

Conceptually:

```rust
pub struct IneligibleFeedstock {
    pub reason_codes: Vec<FeedstockReasonCode>,
}
```

The reason list must be non-empty, bounded, canonical, and duplicate-free.

An ineligible outcome carries no reservation capacity.

## 10. Unresolved outcome requires reasons

Conceptually:

```rust
pub struct UnresolvedFeedstock {
    pub reason_codes: Vec<FeedstockReasonCode>,
}
```

Again, the list is non-empty, bounded, canonical, and duplicate-free.

`Unresolved` means the available evidence/state does not support either a positive eligibility theorem or a definitive ineligibility theorem.

It is not an alias for `Ineligible` and not a weak form of `Eligible`.

## 11. Rights/custody basis remains explicit

The eligible variant carries distinct `rights_resolution_ref` and `custody_resolution_ref` rather than inferring those propositions from raw reference presence.

The biomass core still does not decide real-world law or authenticity.

The refs mean only:

```text
this exact external/prior assessment was part of the eligibility basis
```

not:

```text
the core independently proved legal validity
```

## 12. Evidence snapshot is part of positive scope

The eligible outcome binds an exact `evidence_snapshot_ref`.

This prevents a later live query or changed evidence set from silently becoming the basis for an older positive assessment.

Currentness remains REGEN-019B policy, but the assessment records which snapshot was actually used.

## 13. Additional prerequisites are explicit

Process-specific constraints such as composition or contamination profiles may vary by process.

Do not hard-code every possible future prerequisite into the core outcome enum.

Instead, a bounded canonical `additional_prerequisite_refs` list may preserve exact process-specific assessment references.

Their presence creates no automatic truth; the process-profile assessment constructor determines which are required.

## 14. Reservation capacity derives only from the eligible variant

REGEN-011B2 remains in force, but its constructor becomes mechanically simpler:

```rust
ReservationCapacity::from_assessment(&assessment)
```

matches only:

```rust
FeedstockAssessmentOutcome::Eligible(...)
```

and fails closed for `Ineligible` and `Unresolved`.

The maximum reservation mass is exactly the eligible `assessed_mass` or a separately justified narrower amount; it can never exceed the resolved ecological `allocable_mass` used to construct that outcome.

## 15. Serde must preserve variant invariants

If serde is enabled, tagged enum deserialization must not bypass constructor validation.

Tests must attempt malformed wire states such as:

- `eligible` with zero assessed mass;
- `eligible` with missing prerequisite refs;
- `ineligible` with empty reasons;
- `unresolved` with empty reasons;
- resolved ecological allocation with mixed mass bases;
- resolved ecological allocation whose partition does not sum exactly;
- unresolved ecological allocation carrying an allocable-capacity field if the wire format can express unknown fields.

## 16. Exact partition arithmetic

With REGEN-011B1 `u64` milligrams, partition validation uses checked arithmetic only.

No overflow can be converted into a valid partition.

Prefer computing:

```text
retained + allocable
then + unresolved_residual
```

with checked additions and exact equality to accessible mass.

## 17. No optimization override

An ecological-retention violation or unresolved allocation is not a soft penalty that can be outweighed by price, local sourcing, carbon, predicted yield, resilience, or model confidence.

```text
hard prerequisite failure
+ high utility elsewhere
!= eligible
```

## 18. Added regression requirements

The executable campaign gains at least:

1. exact ecological partition succeeds;
2. implicit missing residual is not accepted where equality is required;
3. mixed mass bases fail;
4. checked partition overflow fails;
5. unresolved allocation cannot expose allocable capacity;
6. unresolved allocation requires non-empty reasons;
7. eligible feedstock cannot be built from unresolved allocation;
8. eligible feedstock mass above allocable mass fails;
9. eligible feedstock mass equal to allocable mass passes;
10. eligible feedstock mass below allocable mass passes;
11. zero eligible mass fails;
12. eligible outcome cannot carry failure reasons;
13. ineligible outcome requires reasons;
14. unresolved feedstock outcome requires reasons;
15. exact evidence snapshot/prerequisite refs are preserved;
16. reservation capacity can only derive from eligible outcome;
17. ineligible and unresolved outcomes cannot mint reservation capacity;
18. utility/price/carbon metadata cannot compensate for ecological prerequisite failure.

## 19. Deliberate non-claims

This refinement proves no ecological truth, legal validity, evidence currentness, contamination safety, process suitability, process execution, or physical actuation.

Its proposition is narrow:

> contradictory positive/negative/unresolved biomass assessment states are structurally separated, and positive eligible mass is bounded by one exact resolved ecological allocation rather than an ambient boolean or optimizer score.
