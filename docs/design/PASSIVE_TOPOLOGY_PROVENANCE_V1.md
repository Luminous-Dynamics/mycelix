# Passive Topology Provenance v1

Symthaea can now emit a deterministic structural signature for explicit negative-space
intent. Mycelix should store that signature as provenance, not as a physical claim.

## Recommended record

`topology.digest` — canonical digest of the complete CSG tree.

`topology.node_count` — total CSG nodes.

`topology.primitive_count` — primitive nodes.

`topology.void_cut_count` — explicit subtraction operations.

`topology.void_node_count` — nodes contained inside subtraction tool subtrees.

`topology.intersection_count` — explicit intersection operations.

`topology.transform_count` — transforms in the complete tree.

`topology.max_void_depth` — deepest subtraction tool depth.

`topology.semantic_status = IntentOnly` — mandatory epistemic label.

## Invariant

The topology digest is an identity/lineage primitive. It must never be interpreted as
evidence that an internal cavity is enclosed, connected, printable, accessible, or
functionally effective.

Those properties require downstream geometric, manufacturing, and physical evidence.

## Why the separation matters

Internal channels and optimized void distributions can be important design variables,
but recent work shows that manufacturability and process planning can materially change
whether an optimized internal-channel design is actually realizable.

Therefore the provenance thread should preserve:

`geometry intent -> topology identity -> geometric validation -> manufacturing analysis -> physics -> measurement`

rather than collapsing these into a single 'optimized' assertion.

## Negative results

Different topology identities that fail the same physical objective should remain distinct
records. Their failure modes can become reusable search knowledge without claiming that
one failed geometry proves another geometry will fail.