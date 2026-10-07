# Integral Interop 1 — OAD field selection contract

This document freezes the **reference-model semantic boundary** for the current Integral interoperability fixture. It is not an assertion about a ratified Integral wire schema.

## Selected D6X semantic inputs

The current `integral-interop-1-design-semantic-v1` projection consumes:

- `design_version.id`
- `design_version.spec_id`
- `design_version.materials`
- `design_version.parameters.bill_of_materials_kg`
- `design_version.parameters.production_steps`

The complete `production_steps` objects are selected, including their:

- `name`
- `estimated_hours`
- `skill_tier`
- `tools_required`
- `sequence_index`
- `safety_notes`

## Intentionally unselected from D6X semantic identity

These remain part of the complete upstream record and may participate in D6S, audit, certification, provenance, or lifecycle evidence, but do not become D6X dependencies under this profile:

- `fixture_id`
- `fixture_version`
- `status`
- `source.*`
- `design_version.parent_version_id`
- `design_version.label`
- `design_version.created_at`
- `design_version.authors`
- `design_version.cad_files`
- `design_version.change_log`
- `design_version.status`
- `design_version.superseded_by_version_id`
- `certification.*`

## Classification rule

A field must not silently enter the D6X semantic commitment because it happens to be added to the upstream JSON object. New fields require an explicit selection decision and a corresponding conformance-test update.

This is a **dependency-selection contract**, not a claim that unselected data is unimportant. Integral's public architecture describes certified design data as feeding COS and separately describes certification/operational signals and feedback flows; the Mycelix boundary therefore records which particular semantics are dependencies for this D6X closure rather than treating every upstream field as equally dependency-bearing.
