# Passive Solver Boundary Binding Set Provenance v1

This schema represents a complete, already-validated solver-boundary binding set.

## Identity

The set artifact records:

- `binding_set_digest`: the deterministic order-independent digest returned by Symthaea;
- `binding_count`: number of bound typed interfaces;
- `candidate_geometry_digest`: common semantic/CAD/CSG candidate identity;
- `candidate_mesh_digest`: common exact TriangleMesh identity;
- `binding_digests`: the complete per-binding identities.

`SolverBoundaryBindingSet` is a coordination/lineage artifact. It does not add
physical evidence beyond the per-binding `AdapterAttested` boundary mapping.

## Invariants

The set is only valid after Symthaea's binding-set validation has established:

- one-to-one interface↔binding coverage;
- unique solver-boundary identities;
- one common candidate geometry identity;
- one common exact candidate mesh identity;
- valid per-binding interface/rim certificates.

Do not interpret the set digest as solver convergence, solver acceptance, or
physical transport evidence.
