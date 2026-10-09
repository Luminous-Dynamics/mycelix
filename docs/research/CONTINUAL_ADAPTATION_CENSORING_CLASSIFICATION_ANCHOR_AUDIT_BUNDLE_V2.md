# Audit evidence bundle v2: verifier integrity and protocol-bound inputs

Status: research-only.

Version 1 pinned fixture blobs and cross-layer identity references. Version 2 closes two gaps exposed by attempting replay:

1. a verifier implementation can be substituted without changing fixture bytes;
2. fixtures can be syntactically valid yet structurally mis-nested or lack a declared cross-layer identity.

Version 2 therefore pins both artifact inputs and verifier source files by Git blob SHA.

## Exact inventory

The manifest pins:
- the deterministic execution-receipt builder and independent Python/Node receipt verifiers;
- the read-only report workflow and the separately permissioned main-branch attestation workflow;
- the full prior research fixture set;
- COSE receipt registry, signed receipt bytes, and adversarial corpus;
- partitioned-gossip event traces and expected outcomes;
- each Python and Node verifier used by the preceding layers;
- the earlier v1 audit bundle and its own campaign;
- exact PR/head topology for the stacked research sequence.

A verifier or fixture replacement at the same path is unresolved until the manifest is deliberately updated and the change is reviewed.

## Structural checks beyond blob identity

The bundle auditor parses every pinned JSON artifact and checks:
- witness registry / trust-root binding;
- the VDS identifier declared by both bundle and VDS fixture;
- tree-head fixture topology: seven sibling head variants, with four-observer quorums at sizes 4 and 7;
- reconstructed size-4 and size-7 Merkle roots;
- receipt transparency-service registry hashes;
- COSE receipt VDS binding and RFC profile labels;
- static and simulated gossip VDS/registry bindings;
- that all mandatory upstream verifiers are enabled;
- that hosted PASS, full SCITT interoperability, live-network convergence, organizational independence, and private-key custody remain unclaimed.

An item is not considered replayable because a filename exists or a verifier source is present; the exact contents, structure, and cross-layer bindings must agree.

## Campaign

The 34-case campaign covers:
- positive exact-input bundle;
- artifact substitutions for every major layer;
- verifier-source substitutions across the stack, including the two v2 audit verifiers and execution-receipt tools;
- source-workflow and attestation-workflow substitutions;
- VDS and topology identity mutations;
- verifier prerequisite weakening;
- false SCITT interoperability claim;
- synthetic hosted-PASS injection.

Python and Node perform the audit independently and must emit byte-identical reports.

## Meaning of the result

The only positive v2 verdict is `evidence-ready`, not `qualified`.

It means the stated inputs, verifier source identities, topology, and claim limits are internally consistent and are ready for independent replay. It does not mean the upstream proof suites passed, that GitHub-hosted execution completed, that the network converges, or that real-world witness governance is independent.

Hosted status remains a separate evidence channel. A queued workflow cannot satisfy it.

## Standards boundary

The COSE receipt layer uses RFC 9942 receipt envelopes and VDS proof labels, deterministic CBOR rules from RFC 8949, and RFC 9162 SHA-256 Merkle proofs. Complete external SCITT interoperability remains explicitly unclaimed until independently exercised against a separate standards implementation.

References:
- RFC 9942: https://www.rfc-editor.org/rfc/rfc9942.html
- RFC 8949: https://www.rfc-editor.org/rfc/rfc8949.html
- RFC 9162: https://www.rfc-editor.org/rfc/rfc9162.html
