# Passive Provenance v2.1 Hardening Notes

The next schema hardening step is to make completeness machine-enforceable rather than
documentary.

## Required constraints

The provenance validator should require exactly one claim for each of:

- `moving_solid_components`
- `mechanical_joints`
- `active_power_w`
- `commanded_actuators`
- `requires_external_control`

An array-length check alone is insufficient because five duplicate claims could otherwise
look complete.

JSON Schema Draft 2020-12 can express this with per-field `contains` constraints plus
`minContains: 1` and `maxContains: 1`. The runtime implementation should apply the same
rule so schema validation and application validation cannot disagree.

Field values should also be type-checked by field: component/joint/actuator counts are
non-negative integers, active power is a non-negative number, and control/state flags are
booleans.

## Integration invariant

Symthaea may refuse to score an incomplete candidate. Mycelix must never transform that
refusal into a positive passive claim merely because the artifact exists in the ledger.

Rejected and incomplete records should remain queryable as negative evidence.