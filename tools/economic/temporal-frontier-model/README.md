# Mycelix Verifier-Relative Causal Frontier Model

This document defines the verifier-relative frontier model and its canonical proof contract over owner-scoped append-only receipt histories. In the current public tree, this directory contains the model README and canonical encoding specification; it is not itself a Cargo crate. The separate Rust proof primitive is `mycelix-workspace/crates/mycelix-rfc9162-proof-verifier`. The model builds on the timestamp-relative query kernel in `tools/economic/temporal-evidence` and moves the distributed-knowledge selector from a wall-clock cutoff K toward an accepted causal frontier F.

    (subject, valid-time V, verifier X, accepted causal frontier F)

A wall-clock K remains useful for reporting. It is not proof that a verifier's replicated state incorporated a receipt.

## Cryptographic proof path

The Rust primitive crate at `mycelix-workspace/crates/mycelix-rfc9162-proof-verifier` implements RFC 9162 SHA-256 leaf/node hashing, Merkle Tree Hash construction, inclusion paths and append-only consistency paths:

- Each canonical owner-cut entry is included under the aggregate checkpoint root.
- Each canonical verifier-acceptance receipt is included under its owner-history root.
- Changed owner-history roots are comparable only with a valid RFC 9162 consistency proof and matching tree sizes.
- Receipt digests are recomputed from canonical receipt bytes before their inclusion paths are accepted.

A successful result is named `InclusionValidRelativeToSuppliedRoot` or `ConsistencyValidRelativeToSuppliedRoots`: the paths recompute relative to the supplied root(s). They do **not** mean those roots, log identity or tree-head signers are trusted or authenticated. The primitive crate deliberately has no key, witness or authority dependency.

## Model

- `OwnerCutV1` binds owner identity, stable history identity, highest owner-local sequence, number of committed receipt leaves, owner-history root, declared gaps, and a checkpoint inclusion witness.
- `VerifierFrontierV1` binds verifier identity, policy epoch, aggregate checkpoint root, parent references, and one canonically ordered owner cut per covered owner.
- `KnowledgeReceiptV1` binds verifier, policy epoch, assertion ID/digest, receipt-acceptance owner, owner-local sequence, causal dependencies, and its canonical receipt digest.
- `OwnerConsistencyLinkV1` carries old/new tree sizes, roots and an RFC 9162 consistency path.
- `FrontierRelationV1` distinguishes Equal, Before, After, Concurrent, Unknown and Conflict.
- `ReceiptVisibilityV1` distinguishes inclusion valid relative to the supplied root, BeyondCoveredCut and Indeterminate.

## Critical semantics

1. **Partial order, not a global clock.** Owner-local sequence positions are compared per owner. Crossed owner positions remain Concurrent.
2. **Sequence is not a Merkle leaf index.** A cut stores highest sequence and committed tree size separately. A declared gap shifts the leaf index of later receipts without changing their owner sequence.
3. **Canonical payloads.** Versioned, domain-separated, length-prefixed encodings are specified in [CANONICAL_ENCODING_V1.md](CANONICAL_ENCODING_V1.md). Strings use UTF-8 and u32 big-endian lengths; sequences, tree sizes and range endpoints use u64 big-endian; dependency lists are sorted before encoding.
4. **Policy binding.** The checkpoint owner-cut entry commits verifier identity and policy epoch. Each knowledge receipt separately commits its verifier and policy epoch; a receipt cannot be replayed under a different policy epoch.
5. **Specific inclusion survives unrelated gaps.** A receipt after a declared gap can still have a valid inclusion proof at its mapped leaf index. A receipt whose sequence lies inside a declared gap is Indeterminate.
6. **No false absence.** BeyondCoveredCut means only that the receipt sequence lies beyond this gap-free represented cut. It is not a global claim that the verifier never learned the fact. Prefix gap accounting is structural and is not an authenticated completeness/absence proof.
7. **Causal closure.** Every declared causal dependency must fall within a represented owner prefix with no declared gaps through that dependency position.
8. **Verifier and policy isolation.** A frontier rejects receipts for another verifier or policy epoch.
9. **Fork and proof handling.** Different roots at the same owner sequence are Conflict. Changed roots require a mathematically valid RFC 9162 consistency proof. Unknown remains the result when roots or cut domains cannot be compared safely.

## Run

Run the RFC 9162 proof primitive from the workspace root:

```sh
cd mycelix-workspace
cargo +1.96.0 fmt --manifest-path crates/mycelix-rfc9162-proof-verifier/Cargo.toml -- --check
cargo +1.96.0 test --manifest-path crates/mycelix-rfc9162-proof-verifier/Cargo.toml --all-targets
cargo +1.96.0 clippy --manifest-path crates/mycelix-rfc9162-proof-verifier/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --manifest-path crates/mycelix-rfc9162-proof-verifier/Cargo.toml --example benchmark
```

The model scenarios for receipt inclusion after declared gaps and concurrent versus ordered frontiers remain design/contract work until a full temporal-frontier Rust crate is added.

## Remaining trust boundaries

Not implemented here:

- Signed checkpoint/tree-head verification and trusted log identity.
- DID key lifecycle, key rotation/revocation and authority verification.
- Gossip, witness quorum or cross-client equivocation detection for split views.
- Cryptographic completeness proofs or authenticated receipt-inventory reconciliation proving absence.
- Verified parent-frontier DAG ancestry, merge protocol and durable Holochain persistence.
- The authority/policy gate which decides whether a query certificate can influence a journal, payment or external effect.

RFC 9162 itself warns that a misbehaving log can show inconsistent views to different clients; Merkle proofs alone do not provide split-view detection. Treat the tree-head signer and witness policy as an independent trust boundary.

This crate verifies proof paths relative to supplied roots. It is not production-qualified and makes no legal, financial, authorization or performance claim.

Related tracking: https://github.com/Luminous-Dynamics/mycelix/issues/6680