# MYC-INT-018O — H1 Pre-Energization Installation and Run Package

Status: design/evidence fixture only. Tracks #3256. Child of MYC-INT-018N / PR #3254.

## Purpose

Define the exact evidence package that must exist before H1 may progress from hardware candidates and interface assumptions into integrated physical bring-up.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_018O_H1_INSTALL_RUN_PACKAGE.json`

## Core rule

```text
candidate admitted
+ interface matrix complete
!= safe installation
!= approved wiring
!= safe first power
!= qualified run
```

018O is a review package, not an electrical certification or actuator authorization.

## Package generations

Every change to delivered hardware revision, wiring, protection, geometry, firmware/protocol semantics, calibration procedure, or stop condition creates a new package generation.

A package used for a run must remain immutable afterward; deviations are appended as run evidence rather than retroactively editing history.

## Required package sections

018O binds:

- exact delivered hardware/install inventory;
- exact manuals/datasheets/pinout evidence;
- power rails and current budget;
- fuse/protection and switching elements;
- ground/isolation/level-shifting assumptions;
- connector/pin map;
- wet/dry physical layout;
- hydraulic geometry and fittings;
- sensor installation profiles;
- calibration/check plan;
- manual stop/override;
- first-power checklist;
- H1b clean-water checklist;
- H1c chemistry-sensing checklist;
- stop/abort conditions;
- run-evidence obligations.

## Local safety invariant

Stopping H1 must never depend on Mycelix, Holochain, Symthaea, Fleet, or network availability.

The package may be approved for a bounded bring-up step, but that approval never becomes autonomous-control authority.

## H1 staging

- **H1a** — dry instrumentation/power bring-up;
- **H1b** — clean-water circulation and leak/hydraulic observation;
- **H1c** — nutrient-solution chemistry sensing after H1b evidence.

No automatic nutrient or pH dosing is included.

## Observation ceiling

A sensor producing a number is not proof of calibration or suitability.

```text
reading produced
!= calibration established
!= process suitability
```

Calibration/check evidence is retained separately from provider/device assertions.

## Run evidence

Every H1 run should retain the exact package generation, installed subjects, pre-run checklist result, calibration/check refs, Edge provider versions, observation export, manual interventions, injected faults/actions, stop/abort events, deviations/unknowns, and post-run result.

## Nonclaims

018O is not electrical/EMC/waterproofing certification, food-safety approval, agronomic qualification, purchase approval, installation approval, or autonomous-control authorization.
