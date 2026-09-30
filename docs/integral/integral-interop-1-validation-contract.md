# Integral Interop 1 — Validation Contract

**Status:** ReferenceModelOnly  
**Profile:** `integral-interop-1`  
**Projection:** `integral-interop-1-design-semantic-v1`

## Purpose

The selected OAD semantic projection must be structurally valid before it can
receive a D6S semantic commitment.

Presence alone is insufficient. A field with the wrong JSON type, an explicit
null, or a non-integral numeric value is rejected before canonicalization.

## Selected shape

- `design_version.id`: string
- `design_version.spec_id`: string
- `design_version.materials`: array of strings
- `design_version.parameters.bill_of_materials_kg`: object whose values are integral JSON numbers
- `design_version.parameters.production_steps`: array of objects
- `production_steps[].name`: string
- `production_steps[].estimated_hours`: integral JSON number
- `production_steps[].skill_tier`: string
- `production_steps[].tools_required`: array of strings
- `production_steps[].sequence_index`: integral JSON number
- `production_steps[].safety_notes`: string

Missing and explicit-null selected fields are rejected.

## Numeric boundary

This profile deliberately accepts only integral JSON numbers for selected
numeric fields. This is a constraint of the current D6S-CANON-1 boundary, not
a claim that Integral's broader OAD model is intrinsically integer-valued.

Integral's public developer guide describes BOM quantities as including
quantities and units, while OAD documentation describes production designs as
capturing BOMs, production steps, skill requirements, and ecological
information. A future profile that selects fractional quantities or ecological
coefficients therefore needs an explicit decimal/fixed-point representation
rather than silently changing D6S-CANON-1.

## Machine-readable corpus

The adversarial acceptance/rejection cases are frozen in:

`mycelix-manufacturing/crates/cos_conformance/testdata/integral_interop_1_validation_vectors.json`

The corpus is intentionally mutation-oriented so another implementation can
reproduce the same validation boundary without depending on Rust-specific
control flow.

## Authority boundary

This contract is a Mycelix interoperability profile, not a claim that the
public Integral material is a ratified wire schema. The public OAD material
describes certified designs as version-locked inputs to COS, and the developer
guide describes data contracts as versioned independently of the systems they
connect. This profile therefore freezes only the selected semantic boundary
needed for the ReferenceModelOnly D6X demonstration.


## Declarative vector schema

The validation corpus is executable rather than prose-only. Each vector declares:

- `operation`: currently `none`, `set`, or `remove`;
- `path`: a dot-separated path with optional array indices;
- `value`: the replacement value for `set`;
- `expected`: `accept` or `reject`.

The conformance test applies every vector and checks both structural validation
and commitment production. A rejected vector must therefore produce no semantic
commitment, not merely an error from an auxiliary validator.
