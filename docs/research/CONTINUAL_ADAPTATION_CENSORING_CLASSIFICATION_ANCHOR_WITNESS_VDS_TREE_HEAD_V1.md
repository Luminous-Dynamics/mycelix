# Authenticated VDS tree heads research v1

Status: research-only.

The append-only VDS in the preceding layer proves that a root corresponds to an ordered statement history and that an older root can be shown to extend into a newer root. This layer closes a separate trust seam: the tree head itself must be authenticated by the observer that claims it.

The resulting pipeline is:

    witness-authenticated observation
        ->
    ordered VDS
        ->
    reconstructed Merkle root
        ->
    observer-signed tree head
        ->
    quorum / non-equivocation
        ->
    consistency proof
        ->
    cross-observer qualification

Each tree-head signature binds:

- tree-head schema and fixed domain;
- observer identity;
- registered key identifier;
- registered witness identity commitment;
- registry identifier/version;
- VDS identifier;
- manifest version;
- tree size;
- reconstructed Merkle root.

A signed head for a fork is therefore a meaningful cryptographic observation. It is not downgraded into a signature failure merely because it disagrees with other observers. The verifier first authenticates each head and then compares the authenticated claims.

## Key lifecycle

Tree-head authentication reuses the cryptographic witness registry from #4870. Key validity is checked against the manifest version carried by the signed head. A retired or revoked historical key may still possess a mathematically valid Ed25519 signature, but the verifier rejects it when the key is outside its admissible version interval.

## History binding

Before any quorum decision, the verifier reconstructs the Merkle Tree Hash from the ordered VDS entries and compares it with every signed root. The head is therefore not merely a signature over an arbitrary root string.

For two authenticated heads, the older and newer tree sizes are compared and the RFC 9162 consistency proof is checked. A later smaller tree is a rollback. A same-size different root is equivocation. A larger tree must demonstrate append-only extension.

## Adversarial campaign

The 21-case campaign covers:

- authenticated baseline and forward head quorums;
- quorum after one observer disappears;
- a cryptographically valid same-size fork;
- wrong-key signature;
- revoked-key replay;
- key-rotation rollback;
- non-canonical signing;
- domain and algorithm substitution;
- root, VDS, registry, and manifest mutation;
- duplicate observer identity;
- below-threshold observations;
- signature wrapping;
- authenticated 4 -> 7 consistency;
- authenticated rollback;
- authenticated consistency-proof tampering.

Python and Node implementations independently verify the heads, reconstruct roots, enforce key lifecycle, evaluate quorum/non-equivocation, and verify consistency proofs. Their generated reports are compared byte-for-byte by CI.

## Claim ceiling

This demonstrates authenticated observer statements and authenticated tree heads relative to the pinned research witness registry and deterministic VDS fixture.

It does not establish independent organizational custody of keys, production VDS availability, gossip completeness, inclusion-proof service behavior, SCITT receipt interoperability, or resistance to a compromise of the trusted witness quorum.

RFC 9162 specifies signed tree heads and Merkle consistency proofs as separate mechanisms; SCITT's VDS requirements likewise separate append-only history from non-equivocation and replayability. This research layer keeps those properties separate rather than treating a signed root as proof of history by itself.
