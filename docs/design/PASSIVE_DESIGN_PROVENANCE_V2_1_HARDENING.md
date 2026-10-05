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

## Numeric-type parity

The v2 schema's evidence `value` union must use `anyOf`, not `oneOf`: JSON Schema
defines `number` as including integers, so an integer instance can satisfy both the
`integer` and `number` branches. `oneOf` would therefore reject valid integer
count claims because two subschemas match. The schema now uses `anyOf`.

The runtime validator follows the same data-model semantics: both JSON integers and
integral JSON numbers such as `1.0` are accepted for count fields, while fractional,
negative, or non-finite values remain rejected.

A regression also locks the schema itself to the inclusive `anyOf` construction so the
numeric union cannot silently regress to an exclusive union.

## Runtime parity

The semantic runtime parity layer is now implemented in
`crates/mycelix-bridge-common/src/passive_provenance.rs` and re-exported by
`mycelix-bridge-common`.

It mirrors the schema's acceptance invariants for:

- exactly one claim for each mandatory passive field;
- allowed field/status/source discriminators;
- field-specific non-negative numeric and boolean value types;
- `Unobserved` forbidding a value;
- stronger statuses requiring a value;
- status/source compatibility, including `Validated -> ValidationRecord`;
- positive passive-score eligibility requiring every mandatory field to be observed.

The runtime validator is deliberately not a second JSON Schema engine. Structural schema
validation remains the schema authority; application code that accepts or promotes a
record must apply the runtime semantic validator as well.

The validator has adversarial unit tests for duplicates, missing claims, wrong types,
negative numbers, unobserved values, missing stronger-status values, source mismatches,
validated-source requirements, eligibility drift, optional repeated mechanism claims,
and unexpected claim properties, plus explicit negative-evidence preservation.
The validator accepts a well-formed record whose decision is non-eligible and whose
mandatory evidence includes `Unobserved` fields; it must not turn incompleteness into
an admission failure merely because the candidate is not positively scorable.

## Integration invariant

Symthaea may refuse to score an incomplete candidate. Mycelix must never transform that
refusal into a positive passive claim merely because the artifact exists in the ledger.

Rejected and incomplete records should remain queryable as negative evidence.