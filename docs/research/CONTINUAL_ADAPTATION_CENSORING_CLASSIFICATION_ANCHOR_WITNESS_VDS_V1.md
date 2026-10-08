# Anchor witness append-only VDS research v1

Status: research-only.

This layer adds a concrete append-only history proof above cryptographically authenticated witness observations.

The VDS profile follows the core Merkle-tree construction from RFC 9162: leaves use a 0x00 domain separator, internal nodes use 0x01, and consistency proofs demonstrate that an older tree is a prefix of a newer tree. RFC 9162 defines consistency proofs specifically to prove the append-only property. (RFC 9162, Sections 2.1.1 and 2.1.4)

The semantic pipeline is now:

    authenticated witness observations
      -> ordered statement sequence
      -> Merkle VDS
      -> signed/observed tree head
      -> consistency proof
      -> cross-observer consistency
      -> non-equivocation

## Research profile

The fixture uses SHA-256 and canonical JSON statement bytes. The tree root is reconstructed from the complete ordered entry sequence rather than trusting a stored root.

For a transition from tree size m to n:

    root(m) + root(n) + consistency_proof(m,n)
        -> append-only decision

A same-size pair with different roots is an explicit observer fork/equivocation. A smaller later tree is treated as rollback rather than silently accepted. A larger tree without a valid consistency proof is unresolved.

## Campaign

The 15-case corpus covers:

- reconstructing valid tree heads;
- valid 4 -> 7 append proof;
- same-size fork;
- rollback;
- mutated, empty, extended, and truncated proofs;
- wrong first and second roots;
- reordered history;
- mutated tail;
- forked prefix observed against a later head;
- identical same-size observers.

Python and Node implementations independently compute the tree and verify the RFC-style consistency proof, then emit byte-identical reports.

## Claim ceiling

This demonstrates a concrete append-only VDS and cross-observer consistency proof over research fixtures.

It does not yet prove:

- independently governed witness organizations or key custody;
- a production VDS service;
- network availability or gossip completeness;
- a signed tree-head/receipt wire format interoperable with SCITT;
- inclusion proofs or non-inclusion proofs;
- protection against an adversary controlling the trust root and all observers.

SCITT requires an applicable VDS to be append-only, non-equivocating, and replayable; SCITT also allows consistency proofs as an additional proof type. (RFC 9943, Section 5.1.3)

The next boundary is therefore not another hash check. It is making tree heads and consistency receipts independently observable and authenticated, then testing delayed observer convergence and split-view detection.

No hosted PASS is claimed until the exact-head workflow completes.
