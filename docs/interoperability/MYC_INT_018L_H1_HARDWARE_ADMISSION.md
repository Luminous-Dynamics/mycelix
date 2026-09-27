# MYC-INT-018L — H1 Hardware Capability Admission Rubric

Status: planning/profile only. Tracks #3243. Child of MYC-INT-018K / PR #3242.

## Purpose

Select future H1 instruments by evidence-bearing capabilities rather than brand familiarity or a single weighted score.

Canonical questionnaire/profile:

`docs/interoperability/fixtures/MYC_INT_018L_H1_HARDWARE_ADMISSION.json`

## Decision form

Allowed outcomes:

- `AdmittedForH1Profile`;
- `AdmittedWithDeclaredLimitations`;
- `MoreEvidenceRequired`;
- `IncompatibleWithH1Profile`;
- `Deferred`.

There is deliberately no weighted overall score or protocol-level winner.

```text
one candidate has more features
!= better H1 evidence subject
```

## What the rubric favors

The profile favors hardware whose behavior can be reconstructed and challenged:

- named physical quantity and unit/profile;
- documented local read interface;
- explicit missing/error states;
- stable device/model/profile identity;
- documented calibration/check procedure;
- raw measurements separate from convenience statuses;
- serviceable probes/components;
- local/offline operation;
- low-voltage wet-side integration where practical;
- ability to cross-check critical measurements.

## What the rubric penalizes or refuses

Examples:

```text
cloud-only read path
undocumented scaling
opaque quality score only
timeout encoded as zero
runtime port/path treated as durable identity
calibrated=true treated as independent proof
write/control permission required merely to read basic telemetry
firmware changes wire semantics with no profile generation
```

## Chemistry instrument boundary

For pH, conductivity and optional dissolved oxygen, H1 should prefer a complete measurement/transmitter front-end whose conditioning, calibration and digital output semantics are documented.

A raw electrochemical probe must not be treated as a generic SBC voltage sensor.

```text
probe electrical response
!= qualified pH/EC/DO observation
```

## Hydraulic boundary

Pump command, electrical state and water flow stay separate.

For flow measurement, require either an actual flow reading or an explicit pulse/count conversion profile.

For reservoir state:

```text
level
!= volume
```

unless geometry/conversion is explicitly bound.

## Power boundary

Prefer low-voltage subsystem power/energy observation for H1 where practical.

Keep:

```text
instantaneous power
!= accumulated energy
```

and do not introduce mains measurement merely to make the demo look more complete.

## Edge/provider compatibility

Candidate hardware should be assessed against Luminous Edge #3354/#3357 rather than requiring a Mycelix-specific driver.

Useful local interfaces may include:

- serial/USB;
- I2C/SPI where appropriate;
- read-only Modbus;
- OPC UA where equipment supports it;
- MQTT Sparkplug / bounded MQTT;
- Home Assistant / Matter bridge for suitable prosumer/building devices.

The protocol itself is never hardware identity.

## Evidence record

A later product-candidate record should bind:

- candidate identity/model/revision;
- exact source/manual/spec evidence;
- firmware/schema version where material;
- per-criterion finding;
- limitations/unknowns;
- final admission state under exact 018L profile generation.

A later data-sheet update or firmware/schema change may require a fresh candidate generation/review.

## Procurement boundary

018L may later support shopping research, but:

```text
AdmittedForH1Profile
!= recommended purchase
!= purchase authority
!= delivered hardware
!= installed hardware
!= qualified hardware
```

Price, availability, warranty, lead time, shipping and local sourcing remain separate procurement facts.

## Nonclaims

This profile does not certify product quality, sensor accuracy, calibration, electrical safety, water/process suitability, agronomic performance, commercial suitability or autonomous-control safety.
