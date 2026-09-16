# REGEN-011B2 — Reservation Capacity Witness v1

Status: refinement of REGEN-011B/011B1 only. This document fixes the reservation API boundary before executable biomass code exists.

## 1. Defect being prevented

The initial REGEN-011B sketch allowed:

```rust
pub fn evaluate_reservations(
    available: BiomassMass,
    reservations: &[ProcessInputReservation],
) -> Result<ReservationBalance, BiomassError>;
```

A naked `BiomassMass` is too weak a reservation input because it does not prove which lot, state snapshot, feedstock assessment, or process profile produced that capacity.

That would permit accidental bypass of the intended planning chain:

```text
lot state
-> ecological/rights/evidence assessment
-> process-scoped feedstock assessment
-> reservation
```

## 2. Assessment identity must be explicit

The executable `FeedstockAssessment` shape MUST include its own bounded exact reference:

```rust
pub struct FeedstockAssessment {
    pub assessment_ref: String,
    pub lot_id: BiomassLotId,
    pub state_snapshot_ref: String,
    pub process_profile_ref: String,
    pub assessed_mass: BiomassMass,
    pub disposition: AssessmentDisposition,
    pub evidence_refs: Vec<String>,
    pub reason_codes: Vec<FeedstockReasonCode>,
}
```

This corrects the earlier mismatch where a reservation carried `feedstock_assessment_ref` but the assessment itself had no corresponding identity/reference field.

`assessment_ref` is opaque/bounded in v1 because REGEN-002 has not frozen a typed `FeedstockAssessmentId`.

## 3. Positive reservation-capacity witness

Reservations should consume a scoped witness, not a naked mass value.

Conceptual shape:

```rust
pub struct ReservationCapacity {
    lot_id: BiomassLotId,
    assessment_ref: String,
    state_snapshot_ref: String,
    process_profile_ref: String,
    maximum_mass: BiomassMass,
}
```

Fields SHOULD be private with read-only accessors so callers cannot construct an internally inconsistent witness with a struct literal.

## 4. Witness construction

The core exposes a constructor approximately equivalent to:

```rust
impl ReservationCapacity {
    pub fn from_eligible_assessment(
        assessment: &FeedstockAssessment,
    ) -> Result<Self, BiomassError>;
}
```

It succeeds only when:

- `assessment.disposition == AssessmentDisposition::Eligible`;
- `assessment.assessment_ref` is structurally valid;
- `assessment.state_snapshot_ref` is structurally valid;
- `assessment.process_profile_ref` is structurally valid;
- `assessment.assessed_mass.amount > 0`;
- the assessment itself satisfies every REGEN-011B structural invariant.

`Ineligible` and `Unresolved` assessments cannot mint reservation capacity.

## 5. Scope binding

The witness binds all reservation-relevant scope:

```text
lot
+ exact feedstock assessment
+ exact material-state snapshot
+ exact process profile
+ exact mass basis
+ maximum reservable mass
```

A reservation cannot substitute any of those fields without failing validation.

## 6. Reservation shape

Conceptually:

```rust
pub struct ProcessInputReservation {
    pub reservation_ref: String,
    pub lot_id: BiomassLotId,
    pub feedstock_assessment_ref: String,
    pub state_snapshot_ref: String,
    pub process_profile_ref: String,
    pub reserved_mass: BiomassMass,
}
```

Adding `state_snapshot_ref` prevents a reservation made against one material state from silently floating onto a later state snapshot merely because the lot ID is unchanged.

## 7. Reservation evaluator

Replace the naked-mass API with:

```rust
pub fn evaluate_reservations(
    capacity: &ReservationCapacity,
    reservations: &[ProcessInputReservation],
) -> Result<ReservationBalance, BiomassError>;
```

Each reservation must exactly match the capacity witness for:

- `lot_id`;
- `feedstock_assessment_ref`;
- `state_snapshot_ref`;
- `process_profile_ref`;
- `BiomassMassBasis`.

It must also carry a unique reservation reference and non-zero mass.

## 8. Arithmetic theorem

Using checked `u64` milligram arithmetic from REGEN-011B1:

```text
sum(reserved_mass) <= capacity.maximum_mass
```

The evaluator returns exact:

```rust
pub struct ReservationBalance {
    pub capacity: BiomassMass,
    pub reserved: BiomassMass,
    pub unreserved: BiomassMass,
}
```

No saturation, floating arithmetic, hidden conversion, or negative balance is permitted.

## 9. Witness is not truth amplification

A `ReservationCapacity` proves only that the **core structurally derived a reservation scope from an `Eligible` assessment record**.

It does not independently prove that the assessment's ecological, legal, contamination, specimen, currentness, spatial, or process assumptions are true.

Therefore:

```text
ReservationCapacity
!= real-world sustainability proof
!= legal authority
!= safe process input
```

The witness is a software-state firewall, not a scientific/legal oracle.

## 10. No execution authority

The positive witness must not be named `Authorization`, `Permit`, `ExecutionCapability`, or similar.

It carries no:

- operator identity;
- machine/device identity;
- actuator command;
- execution nonce;
- start instruction;
- physical-control capability.

```text
ReservationCapacity
!= ProcessExecutionAuthority
```

## 11. Snapshot drift

A new `BiomassStateSnapshot` does not automatically update an existing witness.

If the material state changes and that change matters to eligibility, a new assessment and therefore a new reservation-capacity witness is required.

The v1 core must not silently rebind an old reservation to a new snapshot.

## 12. Assessment supersession

Likewise, a later feedstock assessment does not mutate earlier reservation-capacity objects.

Adapters/event stores may represent supersession/revocation, but immutable core records remain tied to the exact assessment reference they were derived from.

## 13. Cancellation remains separate

Cancelling a reservation removes it from the active reservation set used for a new balance evaluation. It does not alter the historical reservation object or capacity witness.

## 14. New error distinctions

The executable error taxonomy should distinguish at least:

```rust
AssessmentNotEligible,
ReservationAssessmentMismatch,
ReservationSnapshotMismatch,
ReservationProcessProfileMismatch,
ReservationLotMismatch,
ReservationBasisMismatch,
DuplicateReservation,
ZeroReservation,
OverReserved,
```

These must not collapse into one generic reservation failure because each indicates a different invariant breach.

## 15. Added regression requirements

The first biomass campaign gains at least these tests:

1. eligible assessment can mint reservation capacity;
2. ineligible assessment cannot mint capacity;
3. unresolved assessment cannot mint capacity;
4. zero assessed mass cannot mint positive capacity;
5. reservation with wrong lot fails;
6. reservation with wrong assessment ref fails;
7. reservation with wrong snapshot ref fails;
8. reservation with wrong process profile fails;
9. reservation with wrong mass basis fails;
10. duplicate reservation ref fails;
11. sum exactly equal to capacity passes with zero remainder;
12. sum below capacity passes with exact remainder;
13. sum above capacity fails;
14. changed later snapshot does not mutate/rebind existing capacity;
15. later assessment does not mutate/rebind existing capacity;
16. capacity object exposes no execution-authority field.

## 16. Deliberate non-claims

This refinement establishes no real-world eligibility, sustainability, legal right, custody authenticity, contamination safety, process safety, machine authorization, or physical actuation.

Its proposition is narrow:

> reservation arithmetic cannot be entered through an arbitrary mass value; it must be scoped to one exact structurally eligible feedstock assessment and material-state snapshot.
