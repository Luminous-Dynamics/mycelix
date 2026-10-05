# Passive Provenance v2.1 Hardening Notes

The schema hardening step is now implemented for the machine-readable v2 contract: completeness is enforced structurally rather than documentary.

## Required constraints

The provenance validator should require exactly one claim for each of:

- `moving_solid_components`
- `mechanical_joints`
- `active_power_w`
- `commanded_actuators`
- `requires_external_control`

An array-length check alone is insufficient because five duplicate claims could otherwise
look complete.

JSON Schema Draft 2020-12 expresses this with per-field `contains` constraints plus
`minContains: 1` and `maxContains: 1`; the v2 schema now carries those constraints.
The runtime implementation should apply the same rule so schema validation and
application validation cannot disagree.

Status/source/value compatibility is also fail-closed: `Unobserved` claims cannot carry a value; every other status requires a value; and each evidence status is restricted to the source kinds capable of establishing that status. A declaration may originate from a design or manufacturing declaration, simulation status from a simulation declaration, verification status from a verification record, and measured/validated status from a measurement record.
A `Validated` claim now has its own `ValidationRecord` source kind rather than being represented as merely measured evidence. Separately, when `eligible_for_passive_scoring` is true, the schema requires every mandatory passive field to have a non-`Unobserved` status; the decision cannot outrun the evidence.

Field values are also type-checked by field in the v2 schema: component/joint/actuator
counts are non-negative integers, active power is a non-negative number, and control/state
flags are booleans. The type constraints are conditional on the semantic `field` value.

## Runtime parity remains required

The schema is now fail-closed for duplicate required claims and incompatible value types.
A runtime validator must implement the same invariants before accepting or promoting a
record; schema validation alone is not evidence that every application path enforces them.

## Integration invariant

Symthaea may refuse to score an incomplete candidate. Mycelix must never transform that
refusal into a positive passive claim merely because the artifact exists in the ledger.

Rejected and incomplete records should remain queryable as negative evidence.