# REGEN-011B — Executable Biomass Core Contract v1

Status: preregistration only. This document freezes the first dependency-light executable contract for biomass/feedstock evidence, allocation, assessment, and reservation. It authorizes no process execution or physical actuation.

## 1. Purpose

REGEN-011 freezes the biomass availability ladder. REGEN-011A freezes consumption of the shared PEF admission theorem. REGEN-011B now narrows those ideas into an implementation-exact Rust design so the first executable biomass crate can be reviewed against a finite contract rather than invented during coding.

Core theorem:

```text
canonical lot identity
+ exact quantity basis
+ exact evidence bindings
+ ecological allocation state
+ rights/custody references
+ feedstock assessment
+ bounded reservation arithmetic
= reviewable biomass planning state
```

not:

```text
reviewable biomass planning state
= legal title
= ecological truth
= clean feedstock
= process execution
= qualified output
```

## 2. Hard upstream gate

Executable REGEN-011B implementation begins only after REGEN-019D shared admission earns exact-head ProductFrozen PASS.

The intended dependency direction is:

```text
mycelix-core-types
mycelix-regenerative-core
mycelix-regenerative-admission
        -> mycelix-regenerative-biomass
```

`mycelix-regenerative-quality` may be added later as an optional feature/profile dependency. The first biomass core should not require it merely to represent material state.

No Holochain, Marketplace, Manufacturing, Symthaea, Climate, Finance, networking, storage, database, or actuator runtime belongs in the initial core.

## 3. Existing identities reused exactly

The first implementation reuses qualified REGEN-002 identities:

```rust
BiomassLotId
RegenerativeSiteId
RegenerativeFacilityId // only where a facility reference is materially needed
```

No `MaterialStateSnapshotId`, `ReservationId`, `AllocationId`, `RightsRecordId`, or `ProcessRunId` is added implicitly to the already-qualified identity grammar.

Where the first version requires one of these references, it uses a bounded opaque exact reference string. A future typed identity requires its own reviewed REGEN identity revision.

## 4. Numeric discipline

The initial core MUST NOT use `f32` or `f64` for mass conservation, reservation, or allocation arithmetic.

The canonical mass primitive should be exact non-negative integer milligrams:

```rust
pub struct MassMg(u128);
```

Properties:

- zero is representable where semantically valid;
- checked addition/subtraction only;
- no saturating arithmetic;
- no implicit unit conversion;
- no negative material quantities;
- overflow is an explicit error.

`u128` milligrams gives ample range for community/industrial material accounting while retaining exact arithmetic.

A later need for finer-than-milligram analytical values belongs in PEF measurements, not in the physical reservation ledger.

## 5. Quantity basis is part of the type

The first core freezes:

```rust
pub enum BiomassMassBasis {
    AsReceived,
    DryMatterEquivalent,
}

pub struct BiomassMass {
    pub amount: MassMg,
    pub basis: BiomassMassBasis,
}
```

Arithmetic across different bases is rejected.

```text
100 kg AsReceived
!= 100 kg DryMatterEquivalent
```

No constructor or arithmetic helper performs moisture conversion.

Moisture/basis conversion remains a separately evidenced Derived PEF proposition admitted through REGEN-019D.

## 6. Semantic lot profile

A biomass lot is a semantic physical-lot subject, not a timeless snapshot of its current material state.

Initial shape:

```rust
pub struct BiomassLotProfile {
    pub lot_id: BiomassLotId,
    pub source_site: Option<RegenerativeSiteId>,
    pub formation_ref: String,
    pub parent_lot_refs: Vec<BiomassLotId>,
}
```

Validation freezes:

- bounded `formation_ref`;
- canonical sorted/deduplicated parent list;
- no self-parent;
- bounded parent count;
- zero parents permitted for an originating lot;
- the profile contains no mutable mass, moisture, quality, custody, or suitability truth.

`parent_lot_refs` records direct material ancestry only. It does not by itself prove conservation.

## 7. Material-state snapshot

Mutable material facts are represented in immutable snapshots:

```rust
pub struct BiomassStateSnapshot {
    pub lot_id: BiomassLotId,
    pub snapshot_ref: String,
    pub quantity: BiomassMass,
    pub evidence: Vec<BiomassEvidenceBinding>,
}
```

The snapshot is not an event log and not a database row. It is one validated evidence-bound material-state proposition.

The initial core does not invent a universal timestamp/currentness rule. REGEN-019B currentness remains a separate evaluation.

## 8. Biomass evidence bindings

Biomass evidence uses the qualified shared admission kernel rather than copying PEF fields.

A biomass binding conceptually contains:

```rust
pub struct BiomassEvidenceBinding {
    pub role: BiomassEvidenceRole,
    pub expectation: OwnedEvidenceExpectation,
}
```

The implementation may use an owned biomass wrapper around REGEN-019's borrowed `EvidenceExpectation` fields so records can be stored/serialized without giving the shared crate storage responsibilities.

The role vocabulary must be explicit and bounded.

Initial role enum should remain narrow, for example:

```rust
pub enum BiomassEvidenceRole {
    AsReceivedMass,
    DryMatterMass,
    MoistureFraction,
    Composition,
    ContaminationIndicator,
    SourceOccurrence,
    EcologicalAllocationInput,
}
```

Roles do not encode values. Values/units/uncertainty/time/space/provenance remain PEF-owned.

A future role addition is an API revision and should be tested for claim widening.

## 9. Shared admission call boundary

A biomass resolver supplies either raw or lineaged PEF evidence to the shared admission crate.

The biomass core performs:

```text
BiomassEvidenceBinding
-> EvidenceExpectation
-> admit_pef_evidence()
-> biomass role/context validation
```

A shared-admission failure is preserved as a distinct error cause.

The biomass crate MUST NOT flatten it into a generic `InvalidFeedstock` error.

## 10. Physical accessibility assessment

Physical accessibility is a separate proposition from occurrence.

Conceptual shape:

```rust
pub struct PhysicalAccessibilityAssessment {
    pub lot_or_source_ref: String,
    pub accessible_mass: BiomassMass,
    pub evidence_refs: Vec<String>,
    pub assessment_ref: String,
}
```

The initial core treats this as caller-supplied assessed state with structural invariants only. It does not calculate accessibility from geography/logistics automatically.

## 11. Ecological allocation assessment

Ecological retention is a hard eligibility boundary, not an optimizer weight.

Conceptual shape:

```rust
pub enum AllocationResolution {
    Resolved,
    Unresolved,
}

pub struct EcologicalAllocationAssessment {
    pub subject_ref: String,
    pub accessible_mass: BiomassMass,
    pub retention_mass: BiomassMass,
    pub allocable_mass: BiomassMass,
    pub resolution: AllocationResolution,
    pub method_ref: String,
    pub evidence_refs: Vec<String>,
}
```

For a `Resolved` assessment, all three masses must share the same basis and satisfy exactly:

```text
retention_mass + allocable_mass <= accessible_mass
```

The relation is `<=`, not forced equality, because unresolved/non-allocable residual may exist.

`Unresolved` cannot carry a positive eligibility theorem merely because numbers are present.

No weighted optimization may compensate for violation of ecological retention.

## 12. Rights and custody references

The core does not decide law.

It may structurally preserve distinct references:

```rust
pub struct RightsAndCustodyReferences {
    pub ownership_refs: Vec<String>,
    pub custody_refs: Vec<String>,
    pub access_right_refs: Vec<String>,
    pub removal_right_refs: Vec<String>,
    pub transfer_right_refs: Vec<String>,
    pub processing_authority_refs: Vec<String>,
}
```

These vectors are separately bounded, sorted, and deduplicated.

Their presence means only that references were supplied.

```text
reference present
!= authentic
!= current
!= sufficient
!= legally valid
```

No single `owner_ref` or `authorized=true` field belongs in v1.

## 13. Custody is state, not ownership

Where the first implementation needs a custody state, use an explicit state enum rather than infer it from ownership references:

```rust
pub enum CustodyResolution {
    Resolved,
    Unresolved,
}
```

An unresolved custody state blocks process eligibility if the adopted process policy requires controlled custody.

The core itself does not invent that policy.

## 14. Feedstock assessment is tri-state

Avoid a boolean `suitable`/`approved` field.

Freeze:

```rust
pub enum AssessmentDisposition {
    Eligible,
    Ineligible,
    Unresolved,
}
```

Conceptual feedstock assessment:

```rust
pub struct FeedstockAssessment {
    pub lot_id: BiomassLotId,
    pub state_snapshot_ref: String,
    pub process_profile_ref: String,
    pub assessed_mass: BiomassMass,
    pub disposition: AssessmentDisposition,
    pub evidence_refs: Vec<String>,
    pub reason_codes: Vec<FeedstockReasonCode>,
}
```

`Eligible` means only eligible under the exact declared process profile/evidence state represented by that record.

It does not imply universal feedstock quality or execution authority.

## 15. Reason taxonomy is stable and non-freeform

The first executable implementation should provide stable reason codes for machine tests and adapters.

Likely minimum set:

```rust
pub enum FeedstockReasonCode {
    EvidenceIncomplete,
    EcologicalAllocationUnresolved,
    EcologicalLimitExceeded,
    RightsUnresolved,
    CustodyUnresolved,
    QuantityBasisMismatch,
    CompositionUnresolved,
    ContaminationUnresolved,
    ProcessProfileMismatch,
}
```

Free-form explanatory text may exist in adapters/evidence bundles but does not control the core decision.

## 16. Assessment authority is intentionally absent

`FeedstockAssessment` is evidence/planning state. It does not carry:

```text
operator identity
command
actuator target
start time
machine control
execution token
```

A process-control system cannot treat possession of an `Eligible` record as an executable command.

## 17. Reservation identity remains opaque in v1

Because REGEN-002 did not freeze `ReservationId`, the initial core uses a bounded opaque exact `reservation_ref`.

Conceptual reservation:

```rust
pub struct ProcessInputReservation {
    pub reservation_ref: String,
    pub lot_id: BiomassLotId,
    pub feedstock_assessment_ref: String,
    pub reserved_mass: BiomassMass,
    pub process_profile_ref: String,
}
```

Reservation is a planning/accounting claim only.

```text
reservation
!= custody
!= title
!= machine command
!= execution
```

## 18. Reservation ledger arithmetic

The dependency-light core may provide a pure ledger evaluator:

```rust
pub fn evaluate_reservations(
    available: BiomassMass,
    reservations: &[ProcessInputReservation],
) -> Result<ReservationBalance, BiomassError>;
```

Every reservation must:

- reference the same lot;
- use the same mass basis as `available`;
- have non-zero mass;
- have a unique reservation reference.

Checked arithmetic enforces:

```text
sum(reserved) <= available
```

No overbooking, saturation, or silent truncation.

Output:

```rust
pub struct ReservationBalance {
    pub available: BiomassMass,
    pub reserved: BiomassMass,
    pub unreserved: BiomassMass,
}
```

with exact same-basis arithmetic.

## 19. Reservations do not consume material

The core must not silently decrement lot state when a reservation exists.

A later explicit transformation/consumption event is required to change physical material state.

Thus:

```text
reserved_mass
!= consumed_mass
```

This distinction is critical for retries, cancellation, process failure, and multi-system planning.

## 20. Reservation cancellation

Cancellation/release is represented by evaluating the current reservation set or by an explicit higher-level event. The v1 value object should remain immutable.

Do not mutate a reservation in-place from `active` to `cancelled` and then treat both states as the same evidence object.

If lifecycle tracking is required, adapters/event stores own the event sequence.

## 21. Split / merge / blend boundary

The biomass core may validate lineage structure, but it must not fabricate conservation evidence.

A future transformation event should bind:

```text
input lot(s)
+ exact input quantities/bases
+ output lot(s)
+ exact output quantities/bases
+ explicit measured loss
+ explicit unresolved residual
```

REGEN-011B does not need to implement generic transformation events in the first tranche if doing so expands the theorem unnecessarily.

The first implementation should prefer lot profile + snapshot + assessment + reservation.

## 22. Contamination remains separate

`ContaminationIndicator` evidence can be bound structurally, but contamination conformance is owned by REGEN-016.

The first biomass core does not implement universal thresholds.

```text
contamination evidence present
!= clean
```

and

```text
no contamination evidence
!= clean
```

## 23. Quality-profile integration is optional and downstream

REGEN-003 provides exact profile/adoption identity, not material conformance.

Therefore v1 biomass need not depend on `mycelix-regenerative-quality` at all.

A later feature may add exact quality/adoption references to an assessment, but:

```text
quality profile reference
!= conformance
```

and the optional dependency must not change core lot/reservation arithmetic.

## 24. Currentness remains external

The biomass core must not add a universal TTL or freshness duration.

REGEN-019B evaluates whether exact evidence snapshots are current enough for a declared purpose/profile.

The biomass record may preserve the exact evidence/snapshot references used by an assessment so currentness can be evaluated reproducibly outside the core.

## 25. Spatial relation remains external

The core does not infer containment from `RegenerativeSiteId`, source text, or PEF spatial support.

REGEN-019A owns the later geometry/sampling-frame relation.

## 26. Specimen relation remains external

Laboratory evidence can be shared-admitted while specimen-to-lot identity remains unresolved.

REGEN-018 owns specimen chain-of-custody.

Biomass must not collapse:

```text
valid analytical observation
= tested specimen belongs to lot
= specimen represents lot
```

## 27. Serde boundary

If serde is included, it should be optional exactly as in the existing regenerative core crates.

Deserialization must re-run all structural invariants.

No serde path may construct invalid:

- mass basis/value;
- lot profile;
- allocation arithmetic;
- duplicate rights refs;
- duplicate reservations;
- over-reservation state.

## 28. Core error taxonomy

The initial error enum should preserve failure layer and machine-stable semantics. Minimum conceptual classes:

```rust
pub enum BiomassError {
    EmptyReference,
    ReferenceTooLong,
    DuplicateReference,
    SelfParent,
    TooManyParents,
    ArithmeticOverflow,
    QuantityBasisMismatch,
    EcologicalAllocationInvalid,
    SharedEvidenceAdmission(EvidenceAdmissionError),
    DuplicateEvidenceRole,
    DuplicateReservation,
    ReservationLotMismatch,
    ZeroReservation,
    OverReserved,
    InvalidAssessment,
}
```

Exact variants may be refined during implementation, but shared admission failures must remain recognizable rather than being flattened.

## 29. No clocks in the dependency-light core

Do not call wall clock or monotonic clock APIs inside the core.

If an adapter needs timestamped events, timestamps are caller-supplied evidence/state. This keeps qualification deterministic and avoids pretending clock availability implies currentness.

## 30. No random IDs in the core

The core does not generate IDs or reservation references. Callers supply already-valid canonical/opaque references.

This keeps the crate deterministic and removes RNG/runtime dependencies.

## 31. No hidden normalization

The core must not silently:

- lowercase arbitrary external references;
- trim evidence IDs before matching;
- change mass basis;
- convert units;
- deduplicate by dropping conflicting entries;
- coerce unknown into zero;
- choose one of multiple observations automatically.

Fail or require an explicit upstream derivation instead.

## 32. First implementation file shape

Preferred initial crate:

```text
crates/mycelix-regenerative-biomass/
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── mass.rs
│   ├── lot.rs
│   ├── evidence.rs
│   ├── allocation.rs
│   ├── assessment.rs
│   └── reservation.rs
└── tests/
    └── biomass_contract.rs
```

A single-file implementation is acceptable if smaller; module count is not itself a quality metric.

## 33. Direct dependency budget

Initial `Cargo.toml` should contain only:

```text
mycelix-regenerative-core
mycelix-regenerative-admission
```

plus optional serde if/when wire-format testing is included.

`mycelix-core-types` need not be direct if the biomass crate only receives PEF candidates through types already re-exported/required by the shared admission API; if direct PEF role validation is necessary, the dependency must be explicit rather than transitively relied upon.

No broad workspace dependency should be added for convenience.

## 34. First executable campaign

The first authored implementation should exercise at least:

### Mass/basis

1. exact milligram round trip;
2. checked addition/subtraction;
3. overflow rejection;
4. basis mismatch rejection;
5. zero/positive semantics where applicable.

### Lot/profile

6. canonical `BiomassLotId` preservation;
7. parent deduplication rejection;
8. self-parent rejection;
9. no quality/authority fields in identity/profile.

### Shared evidence

10. Reported/Observed success where role permits;
11. bare Derived rejection;
12. bare Inferred rejection;
13. bare Forecast rejection;
14. bare Scenario rejection;
15. lineaged computed success;
16. ID substitution rejection;
17. phenomenon substitution rejection;
18. class substitution rejection;
19. invalid nested PEF rejection before biomass semantics.

### Ecological allocation

20. compatible-basis resolved allocation passes;
21. retention + allocable > accessible fails;
22. basis mismatch fails;
23. unresolved allocation does not become eligible.

### Rights/custody

24. categories remain separate;
25. duplicate references fail;
26. reference presence creates no eligibility automatically.

### Assessment

27. `Eligible`, `Ineligible`, and `Unresolved` remain distinct;
28. eligible assessment remains process-profile scoped;
29. eligible assessment cannot be treated as reservation/execution.

### Reservation

30. same-lot same-basis reservation accounting passes;
31. duplicate reservation ref fails;
32. lot mismatch fails;
33. basis mismatch fails;
34. zero reservation fails;
35. over-reservation fails;
36. exact unreserved remainder is preserved;
37. reservations do not mutate/consume lot state.

### Serde if enabled

38. every invariant above is revalidated after deserialization.

## 35. Adversarial consumer theorem

At least one test should deliberately provide evidence that is perfectly valid under REGEN-019D but insufficient for biomass eligibility.

Example structure:

```text
valid Observed occurrence evidence
+ no ecological allocation resolution
+ no rights/custody resolution
=> shared admission PASS
=> biomass process eligibility NOT ESTABLISHED
```

This is the central consumer firewall.

## 36. Preparation before ProductFrozen

Follow the successful REGEN-010/019 pattern:

```text
authored biomass source
-> explicit Rust 1.96 preparation
-> machine-preserved formatted source + Cargo.lock capsule
-> verify exact capsule path/digests
-> separate ProductFrozen ProductHead
-> exact-head qualification
```

Do not check in a hand-generated lock and call it equivalent to a recorded preparation lineage.

## 37. Toolchain theorem

REGEN-008A applies from the first biomass preparation workflow.

Every material formatter/resolver/test/lint command explicitly selects `+1.96.0` or proves the active toolchain immediately at that stage.

A green run under a different recorded toolchain is not promotable into the intended lineage.

## 38. ProductFrozen qualification target

The biomass ProductFrozen campaign should bind at least:

- exact biomass ProductHead;
- exact parent;
- exact path scope;
- exact REGEN-019D shared-admission dependency identity;
- exact REGEN-002 identity source identity;
- exact checked-in biomass Cargo.lock;
- Rust/Cargo/rustfmt 1.96.0;
- full biomass contract tests;
- strict Clippy;
- source/lock immutability;
- clean checkout;
- Q001 machine-readable receipt validation;
- `dependency_state=ProductFrozen`;
- explicit `system_closure=unfrozen` unless separately strengthened.

## 39. Qualification proposition

A future PASS proposition should remain narrow, approximately:

> The exact REGEN-011B biomass-core ProductHead preserves canonical lot identity, explicit mass basis, shared PEF admission, ecological allocation invariants, separated rights/custody references, tri-state feedstock assessment, and bounded non-overbooking reservation arithmetic under the recorded ProductFrozen Rust 1.96 Cargo graph.

It must not claim real biomass sustainability or process readiness.

## 40. Deliberate non-claims

Even a successful implementation/qualification establishes no:

- actual biomass existence;
- ecological sustainability of any real removal;
- land/title/access/harvest/transfer/processing rights;
- authenticity of custody records;
- specimen representativeness;
- contamination safety;
- process safety;
- pyrolysis suitability of a real feedstock;
- reactor operating parameters;
- output biochar quality;
- agronomic suitability;
- carbon removal or carbon-credit eligibility;
- market value;
- economic optimality;
- resilience superiority;
- governance authority;
- machine-control authority;
- physical actuation authority.

REGEN-011B is a deterministic accounting/evidence boundary for later qualified systems, not an autonomous resource-management authority.
