# Passive Solver Boundary Binding Provenance v3

Version 3 makes realized boundary selection independently verifiable rather than an adapter-supplied opaque digest.

## Required lineage

The provenance record links:

1. semantic/CAD/CSG candidate identity;
2. exact TriangleMesh identity presented to the adapter;
3. typed interface identity;
4. independently certified realized boundary-patch identity;
5. solver adapter and external boundary handle.

## Boundary-patch certificate

v3 records deterministic evidence for the selected patch:

- boundary edge count;
- boundary perimeter in micrometers;
- maximum interface-plane residual in micrometers;
- maximum aperture-radial residual in micrometers;
- boundary patch digest derived from interface identity, exact mesh identity, selection, perimeter, and residuals.

The certificate rejects selections that contain non-boundary edges, omit expected rim edges, fail the declared interface geometry, or do not form one connected closed loop.

## Binding-set integrity

The Symthaea binding registry enforces a one-to-one interface↔binding mapping,
unique solver-boundary identities, and one common semantic geometry digest and
exact mesh digest across the complete set. A deterministic binding-set digest
then commits to the whole validated collection.

The boundary matching tolerance is recorded as part of the realized-boundary
identity. The current connectivity default is 50 micrometers (0.05 mm), and
provenance consumers must not silently substitute a different tolerance.

The mesh-side boundary selection represents the typed interface rim on the
candidate surface. It is distinct from the solver's own face/patch topology;
the opaque external solver handle records that separate adapter mapping.

## Compatibility

`passive-solver-boundary-binding-v2.schema.json` remains historical. New binding artifacts should use v3.

## Verification boundary

`solver_binding_verified=true` means the checked binding constructor verified the typed interface, exact candidate mesh, and complete realized boundary selection.

`physical_transport_unproven=true` remains mandatory. The artifact does not establish solver convergence, numerical correctness, physical transport, manufacturing fidelity, or experimental agreement.

## Pipeline

function
→ topology
→ typed interface
→ material candidate
→ exact candidate mesh
→ certified boundary patch
→ solver binding
→ solver execution
→ numerical evidence
→ measurement
