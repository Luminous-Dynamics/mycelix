# Passive Design Provenance

## Purpose

A generated engineering artifact is not trustworthy merely because an AI system produced it.

For passive-function engineering, Mycelix should preserve the chain:

    design intent
        -> candidate
        -> simulation
        -> verification
        -> fabrication
        -> measurement
        -> validation

Each transition is evidence-bearing and independently inspectable.

## Passive design claim

A passive-design claim should be represented as a claim with:

- design artifact digest
- passive-function contract digest
- evidence digest(s)
- solver / simulator identity and version
- manufacturing profile
- declared assumptions
- uncertainty statement
- verification state
- lineage to parent candidates
- timestamp / sequence information

The claim should never imply that simulation proves a physical result that has not been measured.

## Evidence states

The fabrication ecosystem should distinguish these states:

### Generated

The candidate geometry or material arrangement was produced by a generator.

This establishes authorship/provenance, not correctness.

### Simulated

A declared model evaluated the candidate.

The record must include boundary conditions, model version, numerical settings, and solver identity.

### Verified

A reproducible verification procedure independently confirmed a declared result or invariant.

Verification should identify exactly what was checked.

### Fabricated

A physical artifact was manufactured from a specific artifact digest.

### Measured

A measurement was taken from the physical artifact under declared conditions.

### Validated

The measured result satisfied the acceptance criteria defined by the design contract.

A system must not silently upgrade one state into another.

## Passive-specific invariants

For a strict no-moving-parts claim, the evidence surface should support at least:

- zero moving solid components
- zero mechanical joints
- zero commanded actuators
- zero active power above the contract limit
- no required external control channel

The system may still explicitly record:

- fluid motion
- distributed material deformation
- phase changes
- acoustic waves
- electromagnetic fields
- optical propagation
- chemical transport

These are physical dynamics, not automatically "moving parts."

## Trust model

A candidate should be discoverable even when it fails.

Failure is useful evidence.

Examples:

    PASSIVE_REJECTED_MOVING_COMPONENT
    PASSIVE_REJECTED_ACTIVE_POWER
    SIMULATION_FAILED_STABILITY
    STRUCTURAL_MARGIN_TOO_LOW
    MANUFACTURING_FEATURE_UNRESOLVED
    MEASUREMENT_OUTSIDE_ACCEPTANCE_BAND

This allows Mycelix to preserve negative knowledge instead of repeatedly rediscovering failed regions of the design space.

## Lineage

Generative exploration should form an explicit directed acyclic graph:

    parent
      |
      +-- mutation A -- simulation -- fail
      |
      +-- mutation B -- simulation -- pass
                         |
                         +-- independent verify
                         |
                         +-- fabricate
                         |
                         +-- measure
                              |
                              +-- validate

Each child design should reference its parent artifact digest and transformation descriptor.

This enables:

- reproducibility
- diversity analysis
- novelty analysis
- rollback
- attribution
- negative-result reuse

## Independence

A verification record should identify whether the verifier is:

- the same implementation as the generator
- a different implementation
- an independent solver
- a closed-form analytical check
- a physical measurement

The highest confidence should require stronger independence, not merely repeated execution of the same code path.

## Surrogate-model boundary

Surrogate and learned physics models are valuable for search acceleration.

Their results must be marked distinctly from direct solver results.

A recommended lineage is:

    surrogate prediction
          |
          v
    candidate ranking
          |
          v
    direct simulation
          |
          v
    independent verification

A surrogate prediction may select what to simulate. It should not silently become the simulation evidence itself.

## Governance and release

For designs that can affect physical safety, economic resources, or public infrastructure, the provenance chain should support governance policies such as:

- minimum evidence depth
- required independent verifier classes
- mandatory uncertainty reporting
- manufacturing qualification
- measured acceptance criteria
- revocation of superseded or contradicted claims

Mycelix can then govern the transition from "interesting candidate" to "eligible artifact" without asserting that governance itself makes the physics true.

## Candidate query model

A useful future query is not:

    "Show me good passive designs."

It is:

    "Show me passive designs for function F that satisfy contract C,
     have direct-solver evidence from backend versions B,
     have independent verification V,
     have no unresolved safety violations,
     and are not descendants of a known-failed lineage."

That is a much stronger epistemic primitive.

## Initial implementation recommendation

The first Mycelix integration should remain schema-first and conservative.

Add a passive-design provenance entry that references existing artifact, verification, attribution, and bridge mechanisms instead of introducing a parallel evidence system.

The entry should be able to point to Symthaea evidence by content digest while preserving Mycelix's own local validation rules.

The goal is composability:

    Symthaea discovers
        ->
    simulation stack evaluates
        ->
    Mycelix records / coordinates evidence
        ->
    manufacturing pipeline acts
        ->
    measurement returns to the evidence graph

## Research boundary

The system does not yet establish that Symthaea can discover a superior passive physical design.

The research question to test is narrower and stronger:

> Can HDC-guided, physics-constrained generative search improve the diversity and efficiency of candidate exploration under an explicit no-moving-parts contract, while preserving independent evidence boundaries?

That question is experimentally tractable.

It also gives us a clean route from the current fabrication kernel to a future autonomous engineering loop without pretending that generated geometry is already validated reality.


## Machine-readable schema

The initial machine-readable contract is maintained at `docs/design/passive-design-provenance-v1.schema.json`.
