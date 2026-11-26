# Mycelix Merkle Proof Verifier

A Rust crate containing two deliberately separate research components:

1. An RFC 9162 SHA-256 Merkle Tree Hash implementation for append-only log inclusion and consistency proof generation/verification.
2. An experimental 256-bit sparse Merkle map for arbitrary-key membership and non-membership. The sparse-map algorithm is a Mycelix research profile, **not RFC 9162 and not yet a registered RFC 9942 VDS/VDP profile**.

The crate uses the repository's RustCrypto sha2 = 0.10.9 dependency and does not implement cryptographic primitives itself.

## RFC 9162 append-only log profile

- Leaf hash: SHA256(0x00 || entry).
- Interior node hash: SHA256(0x01 || left || right).
- Empty-tree and arbitrary-size tree-head calculation.
- Inclusion proof generation and verification.
- Append-only consistency proof generation and verification.
- Strict tree-size, leaf-index, path-length, missing-node, extra-node and root checks.
- Published seven-leaf RFC example plus exhaustive bounded tests.

The verifier proves properties relative to supplied roots. It does not authenticate log identity, signed tree heads, signer keys, witness quorums or non-equivocation.

## Experimental sparse assertion index

- 256-bit key/value map with independent domain-separated SHA-256 hashing.
- Membership and non-membership proofs with exactly 256 sibling hashes.
- Query key derivation bound to verifier, policy epoch, subject, claim, dimension and exact assertion reference.
- Value commitment to receipt digest, owner and non-zero owner-local sequence.
- Independent Go reference and shared known-answer vectors.
- Exact scope: a non-membership proof establishes only that one assertion reference is absent from one supplied sparse-map root. It does not prove no other assertion exists for the claim, that any temporal query has no matching candidate, or that the map is complete.

See EXPERIMENTAL_SPARSE_MAP_V1.md for the profile, proof semantics, shared Rust/Go known-answer vectors, completeness requirements and VDS interoperability boundary.

## Run

    rustup toolchain install 1.96.0 --profile minimal
    cargo +1.96.0 fmt --all -- --check
    cargo +1.96.0 test --locked
    cargo +1.96.0 clippy --locked --all-targets -- -D warnings
    cargo +1.96.0 run --locked --example append_only_log
    cargo +1.96.0 run --locked --example benchmark_sparse_map

The cross-language Go conformance suite is in tools/economic/temporal-frontier-model/conformance/go/ and is run by the Economic OS exact-head workflow.

## Trust boundary

Neither RFC 9162 proofs nor sparse-map proofs authenticate supplied roots. Checkpoint signatures, log/profile identity, key lifecycle, witness independence, split-view detection, source-frontier binding and authenticated completeness are separate proof obligations. A valid sparse non-membership proof is not a global absence finding. A valid receipt/index entry is not proof that the underlying event is true or a mutation is authorized.

Existing Mycelix CIV and SCITT work owns signed checkpoints, witness governance and transparency-service composition; this crate must consume qualified outputs rather than create a duplicate log or witness service.
