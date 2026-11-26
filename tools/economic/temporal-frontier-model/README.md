# Mycelix Verifier-Relative Causal Frontier Model

This Rust crate models verifier-specific frontiers across owner-scoped append-only histories. It depends on the repository's RFC 9162 SHA-256 Merkle proof verifier.

It is a follow-on to the timestamp-relative kernel in tools/economic/temporal-evidence. The query target evolves from:

    (subject, valid-time V, verifier X, knowledge-time cutoff K)

to:

    (subject, valid-time V, verifier X, accepted causal frontier F)

A wall-clock K can still be useful for reporting, but it is not proof that a verifier's distributed state incorporated a receipt.

## Model

- OwnerCutV1 binds an owner, stable history identity, highest owner sequence, committed Merkle tree size, owner-history root, RFC 9162 checkpoint inclusion proof, and explicit gap ranges.
- VerifierFrontierV1 binds a verifier, policy epoch, aggregate checkpoint root, parent frontier references, and one cut per covered owner.
- KnowledgeReceiptV1 binds verifier and policy epoch, assertion ID/digest, the verifier's receipt-acceptance log owner and local sequence, causal dependencies, and a canonical receipt digest. Its RFC 9162 inclusion path is verified against the owner-history root.
- OwnerConsistencyLinkV1 carries an RFC 9162 append-only consistency path between two owner-history roots and tree sizes.
- FrontierRelationV1 distinguishes Equal, Before, After, Concurrent, Unknown, and Conflict.
- ReceiptVisibilityV1 distinguishes MerkleInclusionProofValid, BeyondCoveredCut, and Indeterminate.

## Critical semantics

1. **Partial order, not a global clock.** Frontiers are compared owner by owner. Crossed owner positions yield Concurrent, not an invented total order.
2. **Proof paths are verified.** Owner-cut inclusion, receipt inclusion, and owner-history consistency paths are verified with RFC 9162 SHA-256 Merkle Tree Hash rules against supplied roots.
3. **Proof validity is not trust.** The roots, log identity, signatures, verifier DID, key lifecycle, and witness policy are not authenticated by this crate.
4. **Gaps block absence, not unrelated inclusion.** A specific receipt after an earlier gap can still have a valid Merkle inclusion proof at its mapped leaf index. A receipt at a sequence inside a gap is Indeterminate. Prefix gap accounting is not an authenticated completeness/absence proof.
5. **BeyondCoveredCut is not absence.** It means only that the receipt sequence is beyond the highest sequence represented by this cut and no gaps are declared. It does not prove the verifier never knew the fact; full accepted-receipt inventory reconciliation and authenticated completeness are still required.
6. **Causal closure.** Every dependency referenced by a covered receipt must have an owner cut with complete coverage through that dependency position. Same-owner dependencies must precede the receipt sequence.
7. **Verifier isolation.** A frontier cannot consume a receipt that names another verifier.
8. **Fork detection.** Different roots at the same owner sequence are Conflict. Different sequence positions require an explicit consistency-link reference or compare as Unknown.

## Run

    rustup toolchain install 1.96.0 --profile minimal
    cargo +1.96.0 fmt --all -- --check
    cargo +1.96.0 test --locked
    cargo +1.96.0 clippy --locked --all-targets -- -D warnings
    cargo +1.96.0 run --locked --example frontier_scenarios

## Cross-language conformance

An independent Go implementation under `conformance/go/` checks the same canonical byte vectors and RFC 9162 proofs. `CROSS_LANGUAGE_VECTORS_V1.md` records complete known-answer payload bytes and digests; the Go suite also exhaustively checks tree sizes 1–64 and the published seven-leaf RFC example.

## Qualification boundary

RFC 9162 inclusion and consistency proof bytes are now verified against supplied roots. Canonical owner-cut and receipt payloads are bound to those proofs. However, tree-head/checkpoint signatures, log identity, DID/key lifecycle and revocation, witness gossip/split-view detection, authenticated completeness/absence proofs, parent-frontier DAG/merge verification, and durable Holochain persistence are not implemented.

Not implemented here:

- signed checkpoint/tree-head verification and trusted log identity;
- signature, DID, key-rotation, revocation, or authority verification;
- gossip, witness quorum, or cross-client equivocation detection;
- authentic completeness/absence proofs for owner prefixes;
- durable checkpoint/frontier persistence or Holochain DHT integration;
- a verified merge protocol for concurrent frontiers;
- formal TLA+ model or authenticated completeness/absence profile.

This work is an Economic OS query/evidence adapter, not a second transparency service. It is intended to compose with the existing CIV Merkle checkpoint smoke (Symthaea #7244), signed-tree-head/witness state-machine work (Symthaea #7246), and Mycelix split-view TLA+ model (#5944). The production integration must consume authenticated roots/receipts from those qualified layers, validate causal dependency closure and parent/merge ancestry, and only then expose a cryptographically qualified frontier to the query kernel. A query result remains evidence—not permission to mutate a journal, payment, or external effect.

Related tracking: https://github.com/Luminous-Dynamics/mycelix/issues/6680