# Passive Functional-Graph Realization Provenance v1

This schema records the comparison between a functional void graph and the topology
actually observed in a candidate geometry.

## Identity

The record carries:

- artifact_digest: stable provenance identity for the evidence record.
- graph_digest: digest of the intended FunctionalVoidGraph.
- candidate_geometry_digest: digest of the realized candidate geometry.

Identity is not evidence and is not verification.

## Realization evidence

The aggregate status distinguishes:

- InvalidGraph
- NoFlowPathsDeclared
- AllDeclaredPathsConnected
- PartialRealization
- NoDeclaredPathsConnected

Each declared flow edge also carries its port-level status and, when available,
the mesh component assigned to each anchor.

The record may additionally contain unexpected_connectivity entries when the realized
mesh places two represented ports in the same component even though the intent graph
declares no FlowPath in either direction. These are topology-divergence observations,
not claims that the accidental connection is physically useful.

## Epistemic invariant

physical_transport_unproven is required and must be true.

This schema therefore cannot accidentally turn geometric connectedness into a claim
that the device transports fluid or energy successfully.

Solver evidence and manufacturability evidence are optional links. When present they
should point to independent evidence records rather than replacing the topology
observation.

## Failure preservation

A PartialRealization record is valid provenance. It is not an error to store it;
it is important negative evidence showing that the realized candidate diverged from
its declared functional topology.

That distinction supports search systems that learn from rejected candidates without
silently rewriting the intended design graph to match what happened to be generated.
