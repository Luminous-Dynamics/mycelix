# Mycelix manufacturing process execution evidence links v1

This additive MFG-PROC-001B slice links manufacturing execution evidence to the existing Economic Reality Graph and to the typed routing reference from MFG-PROC-001A. It intentionally does not define another resource/event/claim graph.

## Contract

`ProcessExecutionEvidenceBundleV1` carries:
- one execution-event reference;
- one `TypedRoutingStepRefV1` that binds the process definition to an exact source-routing system, identifier, revision, digest, and step sequence;
- input and output resource references;
- supporting event references (for example, source-owned measurement or inspection events);
- conformity-claim references, if supplied by the source system.

Every graph reference contains a graph-system identifier, entity kind, opaque entity identifier, revision, and content digest. Repeated entity identity within a reference group is rejected even if a caller tries to disguise the duplicate with a changed revision or digest. Inputs and outputs are separate groups intentionally: transformation or reuse of the same resource identity is not categorically forbidden.

## Ownership and epistemic boundary

The Economic Reality Graph remains the owner of resource/event/claim records, event semantics, measurements, provenance, corrections, supersession and claim lifecycle. This bundle is only a typed cross-reference envelope.

A passing structural validator does **not** establish that:
- referenced objects exist or resolve;
- digests were recomputed from source bytes or the source was authenticated;
- an event physically happened;
- an output is conforming or a process/equipment is qualified;
- a conformity claim is current, accepted, independently inspected or authorized;
- the process is safe to run or that the work order may be dispatched.

Adapters must resolve each source reference, recompute digests against exact source bytes when available, and invoke the source owner's schema and authority checks. Claims remain separate from measurements and from independently verified findings. Missing links remain missing; the validator never fills them with inferred evidence.

## Compatibility and standards

The bundle is a sidecar. It does not change legacy Holochain operation/routing entry layouts and does not create a machine-command or dispatch path. Existing event/resource/claim formats are referenced through versioned adapters rather than copied into this schema. ISA-95 / IEC 62264, OPC UA, EPCIS, QIF, and other profiles may be mapped by adapters, but this contract does not itself claim standards conformance.

## Local verification

Run the focused workflow for the isolated manufacturing workspace. Its initial lockfile bootstrap currently produces a review artifact rather than pinning dependency resolution in Git; do not treat that bootstrap as reproducible dependency locking. The PR should remain draft until a generated lockfile is committed and the workflow tests against that committed lock with `--locked`.
