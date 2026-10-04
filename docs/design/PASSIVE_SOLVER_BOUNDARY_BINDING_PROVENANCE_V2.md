# Passive Solver Boundary Binding Provenance v2

Version 2 adds the exact candidate-mesh identity to the solver-binding provenance contract.

The binding now distinguishes:

1. semantic/CAD/CSG candidate identity;
2. the exact TriangleMesh representation presented to the adapter;
3. the actual solver boundary patch selected by the adapter.

This closes a subtle provenance gap in which a caller could preserve the same semantic geometry digest while accidentally binding a regenerated or modified mesh.

## Verification boundary

A verified binding means the adapter established the declared mapping and the mesh-backed binding constructor recorded the exact candidate mesh identity.

It does not prove solver convergence, numerical correctness, physical transport, manufacturing fidelity, or measurement agreement.

## Compatibility

passive-solver-boundary-binding-v1.schema.json remains as a historical v1 schema. New binding artifacts should use v2 because candidate mesh identity is now mandatory.

## Pipeline

function
→ topology
→ typed interface
→ material candidate
→ exact candidate mesh
→ realized boundary patch
→ solver binding
→ solver result
→ measurement
