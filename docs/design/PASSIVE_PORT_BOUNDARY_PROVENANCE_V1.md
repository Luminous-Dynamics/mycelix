# Passive Port Boundary Provenance v1

This record describes how boundary openings in a passive candidate were interpreted.

The provenance identity is separate from the evidence itself:

- candidate_geometry_digest identifies the candidate.
- boundary_policy identifies which ports were explicitly allowed to open.
- the boundary counts record what the mesh actually contained.
- status records whether those openings matched the declared policy.

physical_transport_unproven is required to remain true.

An ExpectedOpeningsOnly record means only that all observed boundary edges were
geometrically compatible with explicit port anchors. It is not a fluid transport
certificate.

An UnexpectedOpenings or AmbiguousOpening result is useful negative evidence and
must remain queryable rather than being rewritten into a successful passive record.

This separation is intentional: a topology constraint should be capable of failing
without corrupting the underlying design intent.

A required_open_ports entry is stronger than merely permitting an opening: the
record is non-admissible when a required port has no observed boundary edge associated
with its anchor. This prevents an internal anchor from being mistaken for an external
interface merely because it lies inside a connected mesh.
