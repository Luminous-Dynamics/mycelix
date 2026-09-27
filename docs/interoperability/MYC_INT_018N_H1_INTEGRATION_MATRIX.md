# MYC-INT-018N — H1 Electrical / Interface / Evidence Integration Matrix

Status: design/evidence fixture only. Tracks #3252. Child of MYC-INT-018M / PR #3246.

Canonical machine-readable matrix:

`docs/interoperability/fixtures/MYC_INT_018N_H1_INTEGRATION_MATRIX.json`

## Purpose

Turn the H1 hardware candidate census into a concrete integration boundary before wiring or energizing anything.

The matrix records what is known from current candidate evidence and, more importantly, what still requires physical/electrical review.

```text
candidate appears compatible
!= logic levels proven compatible
!= grounding/isolation proven safe
!= installation approved
!= channel qualified
```

## Power domains

The first H1 layout must keep at least these conceptual domains explicit:

- dry logic / Edge host;
- sensor/transmitter supply;
- low-voltage load/pump supply;
- wet/process-facing probe domain.

A shared supply or ground is an engineering decision to be justified per installed configuration, not inferred merely because two devices operate at low voltage.

## Measurement-chain rule

Chemistry probes remain behind their intended measurement front ends.

For the current Atlas candidates:

```text
pH / EC / DO probe
-> EZO measurement front end
-> isolated/digital carrier/profile where selected
-> Edge read-only provider
```

Do not bypass this chain and reinterpret raw electrochemical probe signals as generic SBC voltages.

## Pump boundary

The selected low-voltage pump is only a physical-load candidate in 018N.

```text
pump requested/commanded
!= pump electrically energized
!= water flowing
```

The pump requires a later actuator/power package covering switching, protection, manual override, authorization, and postcondition evidence.

SENSOR-001 V1 remains read-only and may not acquire write authority merely because the pump is on the same Edge host.

## Flow interface

The pulse-flow candidate requires an explicit counter/input circuit or qualified controller input.

The matrix deliberately does not assume that the pulse output is safe for direct Pi GPIO connection until its exact electrical-output profile is captured.

A manufacturer pulse-to-volume factor is a conversion-profile input, not H1 calibration evidence by itself.

## Reservoir state

Two independent candidate classes remain intentionally different:

```text
distance-to-surface sensor
!= threshold float switch
!= liquid level
!= reservoir volume
```

Geometry and mounting convert distance to level/volume only through an explicit profile.

The float switch remains a threshold witness and is useful as an independent cross-check rather than a lower-resolution version of the same continuous measurement.

## Leak detector

The leak candidate uses its own supply and relay/contact-style output.

Treat it through a protected/isolated dry-side input profile. Do not merge its detector supply/interface with Pi GPIO assumptions.

## Ambient sensing

SHT45 remains an ambient air temperature/RH channel only.

It must not become the nutrient-solution temperature source merely because it reports temperature.

Placement, airflow and self-heating belong to the installed profile.

## DC power evidence

INA260 is retained only for low-voltage DC H1 branches.

It can help distinguish:

```text
pump command
from
pump electrical behavior
```

but neither power nor current establishes hydraulic flow.

Inductive-load transients and exact current-path topology remain physical design obligations.

## Deferred channels

DO remains deferred unless the selected H1c method makes dissolved oxygen load-bearing.

PPFD remains deferred to H2 unless H1 is intentionally expanded into crop/light validation.

Deferred does not mean unsupported or undesirable; it means unnecessary for the current experiment generation.

## Failure observability

018N makes the multi-channel nature of several H1 fault cases explicit. For example:

- `pump-command-without-flow` requires both command/electrical evidence and independent flow evidence;
- low-reservoir evidence requires level/distance or threshold evidence;
- an energy spike requires a separate power/current observer;
- a leak should be directly observed by a leak/contact source unless a separate inference profile is explicitly qualified;
- stale data requires acquisition/currentness evidence, not merely an unusual numeric value.

## Pre-energization package

018N intentionally stops before wiring approval.

A future H1 installation/run package must contain at minimum:

1. exact delivered models and revisions;
2. captured manufacturer manuals/datasheets;
3. reviewed wiring diagram;
4. connector/pinout map;
5. fuse/protection values;
6. full power budget;
7. wet/dry physical layout;
8. enclosure/IP assumptions;
9. manual stop/override;
10. calibration/check sequence;
11. first-power procedure;
12. clean-water leak test.

No physical energization should be inferred from the existence of the candidate census or this matrix.

## Edge handoff

Read-side acquisition remains owned by Luminous Edge #3354/#3357.

```text
native device
-> Edge provider evidence
-> later qualified Mycelix observation projection
```

The integration matrix records candidate adapter needs but does not mint EPI identities or authorize downstream semantic promotion.

## Nonclaims

018N establishes no electrical-safety certification, EMC compliance, waterproofing, product compatibility, installed reliability, sensor calibration, actuator authority, procurement recommendation, crop suitability, or hardware qualification.
