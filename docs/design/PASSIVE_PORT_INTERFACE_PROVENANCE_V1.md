# Passive Port Interface Provenance v1

The passive port interface is a first-class provenance artifact between
functional design intent, realized geometry, and eventual solver boundary
conditions.

The artifact binds:

- stable port identity
- interface position
- aperture geometry
- outward normal
- interface plane
- solver-boundary identity
- deterministic artifact digest

The central invariant is lineage integrity: a geometry realization must not
silently substitute a different boundary location, aperture, orientation, or
solver target.

## Epistemic boundary

`semantic_status = BoundaryInterfaceIntent` means the artifact describes what
boundary should exist.

`solver_binding_verified = false` means the solver has not yet been shown to
consume this exact identity.

`physical_transport_unproven = true` means geometry and topology are not being
promoted into a claim about actual transport.

Solver convergence, boundary-condition application, mesh quality, physical
transport, manufacturing realization, and measurement remain independent
evidence layers.

## Symthaea linkage

The `artifact_digest` is the digest of the corresponding Symthaea
`PortInterface` value. This gives Mycelix provenance a stable handle for
cross-system verification without copying a potentially drifting description.

## Architectural consequence

The passive pipeline can now be represented as:

`function`
→ `void graph`
→ `typed interface`
→ `material candidate`
→ `boundary realization`
→ `solver binding`
→ `physics evidence`
→ `measurement`

This is intentionally compatible with current topology-optimization practice:
boundary segments are explicit solver inputs in TOFLUX, and recent generative
topology work increasingly places connectivity and manufacturability constraints
inside generation/optimization rather than repairing them afterward.
