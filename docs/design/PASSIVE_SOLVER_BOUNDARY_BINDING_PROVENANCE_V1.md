# Passive Solver Boundary Binding Provenance v1

The solver-binding artifact is the provenance bridge between a typed geometric
interface and a concrete solver boundary entity.

It records:

- the stable port id
- the exact typed-interface digest
- the solver-neutral boundary domain/id
- the adapter identity
- the opaque external solver boundary handle
- the candidate-geometry digest
- the digest of the realized boundary patch
- explicit solver-binding verification state
- explicit physical-transport state

## Invariant

A string such as inlet is not sufficient evidence. The same solver name can
refer to a different surface after remeshing or geometry modification.

Therefore a verified binding requires the exact interface identity and exact
realized boundary identity to travel together.

## Evidence boundary

solver_binding_verified = true means only that the concrete adapter established
the declared mapping.

physical_transport_unproven = true remains mandatory because boundary binding
does not prove numerical convergence, correct discretization, transport
performance, experimental agreement, or manufacturing fidelity.

## Cross-system lineage

Symthaea emits the typed interface identity and the binding artifact.
Mycelix preserves those identities as provenance.

This allows a later solver result to cite the same interface and realized-patch
digests without duplicating or silently rewriting the geometric definition.

## Architectural position

function
→ topology
→ typed interface
→ material candidate
→ realized boundary selection
→ solver binding
→ solver result
→ measurement
