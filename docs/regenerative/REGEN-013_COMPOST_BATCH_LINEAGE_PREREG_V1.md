# REGEN-013 — Compost Batch Lineage Preregistration v1

Status: architecture-only preregistration. No operating recipe, quality threshold, agronomic recommendation, certification, climate claim, market authority, or physical-action authority.

## Purpose

Define how identified input lots, declared process evidence, explicit losses/residuals, and an identified `CompostBatchId` compose without turning process participation into a quality or suitability claim.

## Core theorem

```text
identified inputs
+ documented transformation
+ identified output batch
!= qualified compost
!= agronomic suitability
!= nutrient availability
```

## Input semantics

Each input contribution should bind an exact source-lot reference, quantity, quantity basis, declared input role, and relevant evidence references. Listing an input does not imply the entire source lot was consumed.

At minimum, quantity accounting must distinguish `AsReceivedMass` from `DryMatterEquivalent`. Incompatible bases cannot be added or subtracted silently. Any basis conversion is a computed PEF product with lineage.

## Process evidence

Process observations remain PEF-owned. REGEN-013 stores role/context references rather than copying value, unit, uncertainty, spatial, temporal, or provenance fields.

Elapsed time, observation count, or process completion do not by themselves establish output maturity, stability, cleanliness, nutrient value, or suitability.

## Explicit material accounting

At the declared fidelity:

```text
sum(inputs)
=
identified compost output
+ identified removed/captured outputs
+ measured or estimated losses
+ unresolved residual
```

`unmeasured != zero`. The model must permit unresolved residual rather than inventing precision to force closure.

## Lot transformation

Split, merge, blend, screening, or reprocessing transitions require explicit lineage. A child batch does not automatically inherit every parent claim.

```text
qualified A + unresolved B != qualified blend
```

## PEF inheritance

REGEN-013 adopts the same resolved-evidence rule as REGEN-010:

- raw `Reported | Observed` evidence may be admitted after PEF validation;
- `Derived | Inferred | Forecast | Scenario` evidence requires validated PEF lineage;
- exact observation ID and expected phenomenon must match.

## Output qualification firewall

`CompostBatchId` is identity, not qualification.

```text
batch identity
!= quality-profile conformance
!= agronomic suitability
!= nutrient-release behavior
!= carbon-removal authority
```

Quality/admissibility belongs to separately adopted profiles and evidence.

## Proposed later executable crate

`crates/mycelix-regenerative-compost`

Likely direct dependencies: regenerative core, PEF/core types, the qualified REGEN-010 evidence waist, optional quality-profile types, and optional serde. Runtime networking, Holochain, Symthaea, Climate, Finance, Marketplace, and process-control dependencies stay outside the initial core.

## Qualification expectations

A later implementation should use exact ProductHead identity, REGEN-008 ProductFrozen dependency semantics where practical, explicit system-closure classification, strict tests/lints, incompatible-basis rejection, residual/loss mutation tests, computed-evidence lineage tests, and claim-inheritance regressions.

## Deliberate non-claims

This preregistration establishes no operating instruction, quality threshold, contaminant result, legal compliance, certification, crop response, application rate, climate benefit, carbon credit, economic superiority, process execution authority, or physical actuation.
