# MYC-INT-018W — H2 PAR/PPFD and Dissolved-Oxygen Hardware Candidates

Status: hardware research fixture only. Tracks #3312. Parent: MYC-INT-018U / PR #3310.

## Purpose

Extend the H1 hardware-admission program with the two measurement classes the H2 DWC lettuce profile needs materially:

- quantum PAR/PPFD measurement for DLI evidence;
- dissolved oxygen measurement for the DWC root-zone profile.

Canonical fixture:

`docs/interoperability/fixtures/MYC_INT_018W_H2_SENSOR_CANDIDATES.json`

## Current candidates

### PAR / PPFD

- Apogee SQ-520 USB;
- Apogee SQ-522-SS Modbus;
- LI-COR LI-190R.

### Dissolved oxygen

- Atlas Scientific EZO Dissolved Oxygen kit;
- DFRobot SEN0237-A Gravity analog D.O. kit.

All candidates remain `MoreEvidenceRequired`.

## Why the PAR line is separate from H1 light

H1's generic `light` channel is intentionally too weak for crop-performance evidence.

H2 requires:

```text
quantum PAR/PPFD observation
+ exact sampling and integration coverage
-> candidate DLI evidence
```

Not:

```text
lux
fixture watts
fixture command state
manufacturer nominal output
```

The later run profile must bind sensor placement, sampling cadence, coverage gaps and spatial scope.

## Candidate integration implications

### SQ-520 USB

Low hardware-integration burden because it is a digital USB sensor with internal calibration and storage, but Linux/Raspberry Pi acquisition is not assumed merely because the connector is USB. The exact local protocol/driver path still needs evidence.

### SQ-522-SS Modbus

Good fit for a generic read-only Modbus provider: explicit RS-232/RS-485 Modbus RTU, digital PPFD and sensor-specific calibration. It still requires a reviewed serial interface, power domain, address/register profile and exact device identity.

### LI-190R

Useful reference/cross-check candidate with strong published calibration/cosine-response characteristics, but it is fundamentally a sensor that needs an appropriate logger/amplifier/signal-conditioning chain. It is not a direct general-purpose SBC input.

### Atlas EZO D.O.

Fits the existing Atlas chemistry line well: UART/I2C, isolated carrier in the kit, explicit calibration and compensation semantics, plus published Raspberry Pi examples. Probe maintenance, electrolyte/calibration evidence and compensation sources remain part of the evidence subject.

### DFRobot SEN0237-A

Useful independent/lower-cost analog candidate, but admission burden is higher: an ADC path is needed, analog zero cannot collapse acquisition faults, temperature compensation must be explicit, and service handling includes a corrosive NaOH filling solution.

## Important boundaries

```text
PPFD spot reading
!= canopy DLI

DO reading
!= oxygenation system health
!= root health

sensor calibrated flag
!= independent calibration proof

hardware admitted
!= agronomic qualified
```

## Nonclaims

018W does not select a purchase, establish Raspberry Pi/Linux compatibility, calibration, DLI accuracy, DO adequacy, electrical safety, crop performance, food safety or N2 maturity.
