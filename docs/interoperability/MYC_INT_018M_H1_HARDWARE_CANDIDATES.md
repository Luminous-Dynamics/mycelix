# MYC-INT-018M — H1 Concrete Hardware Candidate Census

Status: research / candidate evidence only. Tracks #3245. Child of MYC-INT-018L / PR #3244.

Observed research date: 2026-09-27.

## Purpose

Apply the frozen 018L hardware-admission rubric to concrete currently documented products without turning research into a purchase recommendation or hardware qualification claim.

Canonical machine-readable candidate records:

`docs/interoperability/fixtures/MYC_INT_018M_H1_HARDWARE_CANDIDATES.json`

## Candidate composition under review

```text
Raspberry Pi 4
+ Atlas Scientific isolated EZO pH
+ Atlas Scientific isolated EZO EC
+ Atlas Scientific PT-1000 / EZO RTD temperature
+ DFRobot SEN0217 pulse flow
+ DFRobot FIT0200 low-voltage pump
+ DFRobot A02YYUW distance-to-surface sensor
+ independent float threshold switch
+ DFRobot SEN0454 leak detector
+ Sensirion SHT45 ambient temperature/RH
+ INA260 low-voltage DC power observation
```

Dissolved oxygen and PPFD remain deferred unless the selected H1/H2 profile makes them load-bearing.

This composition is a research hypothesis, not an order list.

## Why this shape

The first H1 bench should favor instruments that are easy to challenge and qualify rather than devices that merely present polished dashboards.

The strongest current candidates tend to provide:

- local digital readout;
- documented units/scaling;
- explicit calibration/check methods;
- source measurements separate from convenience assessments;
- low-voltage integration;
- replaceable/serviceable probes;
- explicit protocol behavior;
- no required cloud dependency.

## Key candidate observations

### Raspberry Pi 4

Retained because it already fits the Luminous Edge hardware path and exposes Linux, Ethernet, USB and GPIO/I2C/UART interfaces. Official Raspberry Pi material states production commitment through at least January 2034.

018L state: `MoreEvidenceRequired`.

Still required: exact H1 NixOS/Luminous Edge image execution, adapter fixtures, restart/power-loss campaign, storage durability profile, enclosure and wet/dry integration.

### Atlas Scientific EZO pH

The current pH kit combines a complete pH measurement front end, calibration solutions, probe/storage materials and electrical isolation. The EZO circuit exposes local UART/I2C and explicit calibration/temperature-compensation semantics.

018L state: `MoreEvidenceRequired`.

This is preferable to treating a raw electrochemical probe as a general-purpose SBC voltage sensor.

### Atlas Scientific EZO EC

The K 1.0 kit combines probe, calibration materials, isolated carrier and a local UART/I2C conductivity front end.

018L state: `MoreEvidenceRequired`.

Physical work must still bind temperature compensation, cleaning/contamination behavior and the actual H1c concentration range.

### Atlas Scientific dissolved oxygen

Technically attractive local instrument with explicit calibration and environmental compensation, but 018K marks DO as conditional.

018L state: `Deferred` for generic H1; `MoreEvidenceRequired` only if the selected physical process requires DO.

### Atlas Scientific PT-1000 / EZO RTD

Explicit physical temperature channel with local UART/I2C front end.

018L state: `MoreEvidenceRequired`.

It remains separate from ambient temperature/RH.

### DFRobot SEN0217 flow

Low-cost pulse-output flow sensor with documented pulse conversion and range overlapping the candidate H1 circulation pump.

018L state: `MoreEvidenceRequired`.

Physical qualification must characterize low-end repeatability, orientation, pulse-counter behavior and GPIO-safe electrical interfacing.

### DFRobot FIT0200 pump

4.5–12 V low-cost submersible circulation candidate.

018L state: `MoreEvidenceRequired`.

Its nominal flow specification is not used as measured flow evidence.

### A02YYUW ultrasonic distance sensor

Local UART distance-to-surface candidate with IP67 enclosure.

018L state: `MoreEvidenceRequired`.

```text
distance-to-surface
!= liquid level
!= reservoir volume
```

Those projections require an exact mounting and reservoir-geometry profile.

### Independent float threshold switch

A simple float/reed threshold channel is useful as an independent low/high witness alongside continuous distance measurement.

018L state: `MoreEvidenceRequired`.

It does not become continuous level.

### DFRobot leak detector

Candidate independent spill-tray water-presence witness.

018L state: `MoreEvidenceRequired`.

Its supply/output still requires an appropriate dry-side isolated/GPIO-safe interface.

### Sensirion SHT45 breakout

Strong ambient reference candidate: local I2C, explicit temperature/RH channels, CRC and factory calibration material.

018L state: `MoreEvidenceRequired`.

Placement, airflow and self-heating must be profile-bound; it is not solution temperature.

### INA260

Useful low-voltage DC voltage/current/power observer with I2C and integrated shunt.

018L state: `MoreEvidenceRequired`.

Use only within its documented DC envelope. Inductive-pump transient suppression remains a separate electrical design obligation.

### PPFD sensor

A USB Apogee SQ-520 is a plausible explicit-profile PPFD reference, but plant-light performance is not required for H1 instrumentation qualification.

018L state: `Deferred` to H2 unless the H1 profile explicitly needs PPFD.

## Candidate-selection theorem

```text
manufacturer specification
!= delivered hardware revision
!= installed behavior
!= calibrated instrument
!= H1-qualified channel
```

No candidate advances beyond `MoreEvidenceRequired` from desk research alone.

## Physical qualification obligations

Before any candidate can become `AdmittedForH1Profile`:

1. record exact delivered model/revision/firmware;
2. capture and version exact manufacturer documentation;
3. review electrical interfaces and wet/dry topology;
4. execute the exact Luminous Edge acquisition path;
5. characterize nominal/missing/timeout/decode/reconnect behavior;
6. execute calibration/check procedures where applicable;
7. cross-check critical measurements independently;
8. exercise pump/flow/restriction relationships;
9. power-cycle and partition the acquisition system;
10. map FI-01 through FI-18 to physical evidence;
11. retain synthetic/replay provenance separately from physical observations;
12. preserve observation/assessment/analysis/authority/effect boundaries.

## Procurement boundary

Price, stock and lead-time observations are volatile external facts.

018M records them only to inform later procurement research. They do not affect protocol identity and do not authorize purchase.

Local South African availability, landed cost, warranty/support and delivery time remain a separate procurement pass.

## Nonclaims

018M establishes no purchase recommendation, local availability, compatibility, calibration, sensor accuracy in our bench, electrical/water safety, agronomic performance, food-safety status, or autonomous-control qualification.
