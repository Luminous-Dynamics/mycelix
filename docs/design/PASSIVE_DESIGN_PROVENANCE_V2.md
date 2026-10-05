# Passive Design Provenance v2

## Purpose

Passive engineering claims need a stronger epistemic boundary than a single aggregate
"passive" flag. A value of zero is meaningful only when the field was actually observed
or explicitly declared by an identified source.

v2 therefore records one evidence claim per field and preserves the status of that claim.

## Required safety fields

A passive candidate is incomplete until all five are explicitly represented:

- moving solid components
- mechanical joints
- active power
- commanded actuators
- external control requirement

Optional mechanism-state fields may record fluid motion, distributed deformation, and phase
change, but they do not substitute for the five safety-critical fields.

## Evidence state

`Unobserved` is distinct from `Declared`, `Simulated`, `Verified`, `Measured`, and
`Validated`. The provenance layer must never coerce an unobserved field into a safe zero.

The `source_kind` identifies where the assertion came from. `evidence_digest` binds the
claim to a reproducible record instead of trusting the text surrounding the claim.

## Decision boundary

`eligible_for_passive_scoring` is a derived decision, not a subjective label.

A candidate is eligible only when required evidence is complete, declarations are
non-contradictory, the passive contract is satisfied, and the downstream evaluator
accepts the candidate geometry.

A surrogate or reduced-order solver remains explicitly marked as such; it does not become
an exact physical verification merely because it converged.

## Digital thread

`intent -> candidate -> evidence -> simulation -> verification -> fabrication -> measurement -> validation`

The provenance record connects:

`intent -> candidate -> evidence -> simulation -> verification -> fabrication -> measurement -> validation`

Artifact and evidence digests permit Mycelix to retain lineage without duplicating the
physics implementation in the civic/provenance layer.

This aligns with the broader manufacturing digital-thread direction: CAD/PMI, simulation,
verification, manufacturing, and quality information should remain traceable across the
product lifecycle rather than being collapsed into an opaque final score.

## Negative results

Rejected candidates are evidence too. Implementations should preserve failed evaluations,
their reasons, solver configuration digests, and parent lineage so subsequent search can
avoid rediscovering the same dead ends.

## Boundary for Symthaea

Symthaea remains responsible for geometry generation, physics, solver execution, and the
truth status of physical results. Mycelix records the lineage, claims, authorization, and
traceability of those results.

The design question remains:

> What arrangement of matter causes the desired function to emerge?

The provenance question is:

> What evidence lets another party distinguish a promising design from a merely plausible story?

The schema is machine-readable in `passive-design-provenance-v2.schema.json`.