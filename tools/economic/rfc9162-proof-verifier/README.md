# Mycelix RFC 9162 Proof Verifier

A dependency-free-at-the-application-layer Rust API for SHA-256 Merkle Tree Hashes, RFC 9162 inclusion proofs, and RFC 9162 append-only consistency proofs. The crate uses the repository's established RustCrypto `sha2 = 0.10.9` dependency; it does not implement cryptographic primitives itself.

## Implemented

- RFC 9162 SHA-256 leaf hash: `SHA256(0x00 || entry)`.
- RFC 9162 SHA-256 interior node hash: `SHA256(0x01 || left || right)`.
- Empty-tree and arbitrary-size tree-head calculation.
- Inclusion proof generation and verification.
- Append-only consistency proof generation and verification, including the zero-size and equal-size edge cases.
- Strict tree-size, leaf-index, path-length, missing-node, extra-node, and root checks.
- Fixed seven-entry test vector plus exhaustive inclusion and consistency checks over bounded trees.

## Run

```sh
rustup toolchain install 1.96.0 --profile minimal
cargo +1.96.0 fmt --all -- --check
cargo +1.96.0 test --locked
cargo +1.96.0 clippy --locked --all-targets -- -D warnings
cargo +1.96.0 run --locked --example append_only_log
```

## Trust boundary

A successful inclusion proof establishes that a specific leaf is included under the supplied root. A successful consistency proof establishes that the supplied later tree is an append-only extension of the supplied earlier tree.

Neither proof authenticates either tree head. The caller must independently verify checkpoint signatures, log identity, key lifecycle/revocation, witness policy, and cross-verifier gossip/equivocation detection. RFC 9162 itself notes that Merkle auditing alone does not prevent a log from presenting inconsistent views to different clients. This crate does not implement signed tree heads or the trust/witness layer.

The RFC defines an experimental transparency protocol, not a complete application authorization system. Mycelix frontier integration must additionally bind owner identity, verifier acceptance, policy epoch, sequence semantics, and causal dependencies.

Specification: https://www.rfc-editor.org/rfc/rfc9162.html