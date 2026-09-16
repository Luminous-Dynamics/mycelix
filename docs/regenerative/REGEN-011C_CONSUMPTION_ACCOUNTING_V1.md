# REGEN-011C — Biomass Consumption Accounting v1

Status: preregistration only. This document defines the accounting boundary between a process-input reservation and an evidence-bound claim that material was actually consumed from a biomass lot. It creates no machine-control or reactor authority.

## 1. Core distinction

```text
reservation
!= consumption
!= process execution
!= output qualification
```

A reservation is planning state. A consumption record is material-accounting evidence. Neither is a machine command.

## 2. Upstream dependencies

REGEN-011C assumes the REGEN-011B lineage:

- canonical `BiomassLotId`;
- exact `BiomassMass` / basis semantics;
- `BiomassMassAssertion` evidence firewall;
- `FeedstockAssessment` fail-closed outcome model;
- `ReservationCapacity` scoped to one exact eligible assessment;
- `ProcessInputReservation` bound to exact lot / assessment / snapshot / process profile.

Executable consumption code remains downstream of qualified REGEN-019D and qualified executable biomass core.

## 3. Consumption record

Conceptual shape:

```rust
pub struct BiomassConsumptionRecord {
    pub consumption_ref: String,
    pub reservation_ref: String,
    pub lot_id: BiomassLotId,
    pub feedstock_assessment_ref: String,
    pub state_snapshot_ref: String,
    pub process_profile_ref: String,
    pub process_run_ref: String,
    pub consumed_mass: BiomassMass,
    pub evidence_refs: Vec<String>,
}
```

All opaque refs are bounded and exact. `process_run_ref` remains opaque because REGEN-002 does not freeze a typed process-run ID.

## 4. Reservation match theorem

A consumption record is valid against a reservation only if all scope dimensions match exactly:

```text
reservation_ref
lot_id
feedstock_assessment_ref
state_snapshot_ref
process_profile_ref
mass basis
```

and:

```text
consumed_mass.amount > 0
```

A process run cannot consume from a reservation that belongs to a different lot, assessment, snapshot, profile, or basis.

## 5. Partial consumption is first-class

A reservation may be consumed partially.

For one reservation:

```text
sum(consumed) <= reserved_mass
```

The remainder is:

```text
remaining_reserved = reserved_mass - sum(consumed)
```

with checked same-basis arithmetic.

No implicit assumption says a reservation was fully consumed merely because one process event occurred.

## 6. Anti-double-consumption

The first pure evaluator should reject:

- duplicate `consumption_ref`;
- duplicate exact accounting events;
- consumption beyond reserved mass;
- basis mismatch;
- reservation/snapshot/profile mismatch.

Conceptually:

```rust
pub fn evaluate_consumption(
    reservation: &ProcessInputReservation,
    records: &[BiomassConsumptionRecord],
) -> Result<ConsumptionBalance, BiomassError>;
```

with:

```rust
pub struct ConsumptionBalance {
    pub reserved: BiomassMass,
    pub consumed: BiomassMass,
    pub remaining_reserved: BiomassMass,
}
```

## 7. Evidence requirement

A consumption record MUST carry a non-empty bounded evidence-reference set.

```text
record exists
!= physical consumption proven
```

The refs preserve what evidence supports the accounting event. The dependency-light biomass core validates structure and accounting scope, not real-world sensor truth.

If a later adapter uses PEF/process telemetry, that evidence remains subject to its own provenance/currentness/specimen/device rules.

## 8. No command/ack conflation

The following do not prove consumption by themselves:

```text
machine command sent
machine command acknowledged
operator clicked start
reservation allocated
```

A consumption record must remain separate from command/ack evidence.

This follows the broader REGEN/Aegis discipline:

```text
requested action
!= executed action
!= observed physical outcome
```

## 9. Snapshot consequence is separate

A validated consumption record does not mutate the original `BiomassStateSnapshot` in place.

A later/new snapshot may assert a lower remaining lot mass, but that is a distinct evidence-bound state proposition.

The core may offer a pure accounting helper for expected remaining mass, but it must not claim that computed remainder is a new observed physical snapshot.

```text
computed remainder
!= observed remaining inventory
```

## 10. Reservation release

Unused reservation capacity is not consumed material.

A caller may later release/cancel the remaining reservation through a separate planning event/state transition. Consumption records remain immutable historical accounting evidence.

## 11. Process-run scope

Multiple reservations may feed one process run; one reservation may be partially consumed by one or more process-run accounting events only if the adopted higher-level process semantics permit it.

The v1 evaluator operates per reservation to avoid hidden cross-lot aggregation.

Cross-reservation/process-run reconciliation belongs to a later explicit material-flow theorem.

## 12. Biochar handoff

REGEN-012 should consume exact biomass input accounting from REGEN-011C rather than interpreting reservation mass as consumed process input.

Intended boundary:

```text
qualified biomass reservation
-> evidence-bound consumption accounting
-> REGEN-012 process input record
-> biochar transformation/output lineage
```

not:

```text
reservation created
-> assume full feedstock consumed
-> mint biochar batch lineage
```

## 13. No output inference

A consumption record establishes no char yield, gas yield, condensate yield, useful heat, emissions, carbon persistence, or output qualification.

Those remain REGEN-012 process/output propositions.

## 14. No execution authority

`BiomassConsumptionRecord` carries no actuator command, machine target, operator capability, safety override, or execution authorization.

The core cannot start a reactor, feeder, conveyor, valve, pump, or other physical equipment.

## 15. No automatic conservation closure

Input consumption accounting alone does not close a transformation mass balance.

REGEN-012 must still represent:

```text
consumed input
=
identified outputs
+ measured/estimated loss
+ unresolved residual
```

with compatible quantity bases.

## 16. Error taxonomy additions

The executable biomass error surface should distinguish at least:

```rust
DuplicateConsumption,
ConsumptionReservationMismatch,
ConsumptionAssessmentMismatch,
ConsumptionSnapshotMismatch,
ConsumptionProcessProfileMismatch,
ConsumptionBasisMismatch,
ZeroConsumption,
OverConsumed,
MissingConsumptionEvidence,
```

## 17. Regression requirements

The first executable consumption campaign gains at least:

1. exact full consumption passes;
2. exact partial consumption passes;
3. multiple partial records sum correctly;
4. consumed sum above reservation fails;
5. zero consumed mass fails;
6. wrong lot fails;
7. wrong assessment fails;
8. wrong snapshot fails;
9. wrong process profile fails;
10. wrong basis fails;
11. duplicate consumption ref fails;
12. missing evidence refs fails;
13. remaining reserved amount is exact;
14. consumption does not mutate original reservation;
15. consumption does not mutate original state snapshot;
16. command/ack-only evidence cannot be interpreted as consumption by the dependency-light core;
17. consumption record cannot mint output/biochar qualification;
18. consumption record exposes no execution-authority field.

## 18. Qualification boundary

A future executable REGEN-011C implementation must earn its own exact ProductHead qualification.

```text
qualified reservation arithmetic
!= qualified consumption accounting
```

and:

```text
qualified consumption accounting
!= qualified REGEN-012 biochar transformation
```

## 19. Deliberate non-claims

REGEN-011C establishes no physical consumption truth, device telemetry truth, process safety, reactor operation, output quantity, biochar quality, agronomic suitability, carbon removal, economic value, process authority, or physical actuation.

Its proposition is narrow:

> material-accounting claims of biomass consumption are explicitly scoped to exact reservations and cannot silently exceed, substitute, or mutate the reservation they consume against.
