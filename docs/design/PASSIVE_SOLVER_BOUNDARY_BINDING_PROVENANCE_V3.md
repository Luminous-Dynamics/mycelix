# Passive Solver Boundary Binding Provenance v3

Version 3 makes realized boundary selection independently verifiable rather than an adapter-supplied opaque digest.

## Required lineage

The provenance record links:

1. semantic/CAD/CSG candidate identity;
2. exact TriangleMesh identity presented to the adapter;
3. typed interface identity;
4. independently certified realized boundary-patch identity;
5. solver adapter and external boundary handle;
6. sealed evidence level indicating what was actually established.

## Boundary-patch certificate

v3 records deterministic evidence for the selected patch:

- boundary edge count;
- boundary perimeter in micrometers;
- maximum interface-plane residual in micrometers;
- maximum aperture-radial residual in micrometers;
- boundary patch digest derived from interface identity, exact mesh identity, selection, perimeter, and residuals;
- `evidence_level=AdapterAttested` for ordinary adapter binding;
- `solver_entity_fingerprint`, `solver_entity_mapping_digest`, and the canonical observation receipts when the binding is promoted to `SolverInputEntityAttested` or `SolverEntityAttested`.
- `solver_entity_observation_digest`, the digest of the adapter-owned canonical input/entity observation from which that fingerprint was derived.
- `solver_entity_observation_kind` and `solver_entity_observation_source_digest`, preserving the semantic kind and source-artifact lineage of that observation.
- `SolverInputEntityAttested` is intentionally intermediate: it records a concrete solver-input entity such as an OpenFOAM boundary patch, while remaining below live `SolverEntityAttested` evidence.
- the solver-entity mapping digest is adapter-specific, preventing an attestation receipt from being transplanted between adapter implementations.


The certificate rejects selections that contain non-boundary edges, omit expected rim edges, fail the declared interface geometry, or do not form one connected closed loop.

## Binding-set integrity

The Symthaea binding registry enforces a one-to-one interface↔binding mapping,
unique solver-boundary identities, and one common semantic geometry digest and
exact mesh digest across the complete set. A deterministic binding-set digest
then commits to the whole validated collection.

The boundary matching tolerance is recorded as part of the realized-boundary
identity. The current connectivity default is 50 micrometers (0.05 mm), and
provenance consumers must not silently substitute a different tolerance.

Boundary matching fails closed when the supplied tolerance is greater than or equal to the declared aperture radius, preventing a broad annulus from being mistaken for the typed aperture rim.

The recommended pre-dispatch gate validates the complete binding set against one
semantic candidate geometry digest and one exact candidate mesh before a solver
adapter is allowed to proceed. This makes candidate drift an explicit provenance
failure, not a convention left to individual callers. The resulting collection
is represented by `passive-solver-boundary-binding-set-v1.schema.json`.

The mesh-side boundary selection represents the typed interface rim on the
candidate surface. It is distinct from the solver's own face/patch topology;
the opaque external solver handle records that separate adapter mapping.

For OpenFOAM input provenance, the progressively stronger observation path may
bind the declared `boundary` patch to its exact `faces` range and, when the
serialized `points` artifact and explicit unit scale are supplied, require the
patch perimeter to equal the independently certified candidate rim. Distinct
serialized topology edges that collapse to the same quantized portable edge
identity fail closed; repeated point indices, degenerate faces, and non-manifold
patch edges are likewise rejected. Serialized boundary-face ordering must also
produce a normal aligned with the typed interface outward normal, so reversed
face orientation is not treated as equivalent geometry.

An optional `neighbour` artifact establishes the internal-face count and therefore
the complete non-overlapping, contiguous boundary-patch partition. A stronger
complete-topology input path can additionally require the `owner` artifact's
cardinality to equal the global face count. Inconsistent optional-artifact
combinations fail closed rather than silently degrading the evidence level.
These are input-artifact checks; they do not assert that a live solver loaded the
files or that numerical physics is correct.

A live-capable adapter may additionally introspect that concrete solver entity and
emit the sealed `SolverEntityAttested` receipt. The receipt is deliberately
portable: the core records only the fingerprint and binding digest, while the
adapter owns the vendor-specific entity extraction.

## Compatibility

`passive-solver-boundary-binding-v2.schema.json` remains historical. New binding artifacts should use v3.

## Verification boundary

`solver_binding_verified=true` means the sealed construction path established a checked binding. With `evidence_level=AdapterAttested`, the adapter's mapping draft was accepted after checking the typed interface, exact candidate mesh, and complete candidate-surface interface rim.

With `evidence_level=SolverInputEntityAttested`, the binding additionally carries a canonical entity fingerprint, observation receipts, and a mapping digest tied to the exact rendered input artifact. With `evidence_level=SolverEntityAttested`, the same receipt structure represents additional live-capable solver-side inspection. Neither is independent solver truth: the neutral core cannot inspect vendor-specific solver state itself.

The observation digest is an adapter-owned receipt for a canonical introspection record. The neutral core records and cryptographically binds it, but does not interpret vendor-specific contents.

Neither stronger evidence level means the solver accepted, loaded, executed, or numerically validated the boundary.

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

## Dispatch evidence requirement

A consumer may require `SolverEntityAttested` explicitly at the final binding-set gate. Ordinary `AdapterAttested` records remain valid provenance, but they do not satisfy a workflow that requires concrete solver-side entity evidence.
