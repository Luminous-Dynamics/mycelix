# RFC 9162 SHA-256 Proof Verifier

A small Rust implementation of the RFC 9162 Merkle Tree Hash primitives and the inclusion / append-only consistency proof verification algorithms.

## API

- `leaf_hash(entry)`: SHA-256 of the RFC 9162 0x00-prefixed leaf input.
- `node_hash(left, right)`: SHA-256 of the RFC 9162 0x01-prefixed node input.
- `merkle_tree_hash_from_leaf_hashes` and `merkle_tree_hash`: RFC largest-power-of-two tree shape.
- `verify_entry_inclusion` / `verify_leaf_inclusion`: inclusion relative to a supplied root, index and tree size.
- `verify_consistency`: append-only consistency between two supplied roots and tree sizes.

The exact algorithms are in RFC 9162 sections 2.1.1, 2.1.3.2 and 2.1.4.2:
https://www.rfc-editor.org/rfc/rfc9162.html

## Trust boundary

A successful inclusion proof means that the leaf hashes to the supplied root at the supplied position and tree size. A successful consistency proof means the old supplied root is a prefix of the new supplied root. Neither result authenticates the root, log identity, tree-head signer, witness set, split-view status, source-universe completeness, payload truth or authorization.

Use the typed result values:
- `InclusionValidRelativeToSuppliedRoot`
- `ConsistencyValidRelativeToSuppliedRoots`

The caller must independently authenticate the log/tree-head identity and signer, apply a witness and split-view policy, and prove any required source-universe completeness. Mycelix DESCI's governed checkpoint/witness and mirror-observation model is a separate trust layer; it is not silently imported by this primitive crate.

## Tests

The test suite includes fixed SHA-256 vectors, every leaf inclusion for trees of 1–64 leaves, every old/new append-only prefix pair for trees up to 64 leaves, and negative controls for tree-size errors, wrong leaves/roots, malformed/truncated/extra paths, path ordering and node mutation. It also consumes the shared cross-language fixture at `tools/economic/temporal-frontier-model/RFC9162_VECTORS_V1.txt`; the Go reference harness at `tools/economic/temporal-hindsight/reference/rfc9162vectors/` reads the exact same file, so leaf/MTH/inclusion bytes must agree between implementations. A separate `XENIA_CHECKPOINT_V1_KAT.txt` fixture is consumed by both Go and Rust tests to verify the RFC 9162 leaf hash over Xenia's canonical 297-byte journal-checkpoint payload. This is a byte-interop test only; it does not authenticate a tree head or claim effect-owner truth.

Run:

    cargo +1.96.0 fmt --manifest-path Cargo.toml -- --check
    cargo +1.96.0 test --manifest-path Cargo.toml --all-targets
    cargo +1.96.0 test --release --manifest-path Cargo.toml --all-targets
    cargo +1.96.0 clippy --manifest-path Cargo.toml --all-targets -- -D warnings

## Benchmark

`examples/benchmark.rs` measures full MTH, inclusion verification and consistency verification at 1k/10k/100k leaves. It performs warm-up and 30 timed samples per operation and emits raw CSV samples plus p50/p95/p99. It is a local microbenchmark, not a SOTA claim: it excludes signatures, witness quorum, network, checkpoint retrieval, source-roster completeness and end-to-end temporal query cost.
