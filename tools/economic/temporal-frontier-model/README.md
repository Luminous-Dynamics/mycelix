# Mycelix Verifier-Relative Causal Frontier Model

This dependency-free Rust crate models the **structural semantics** of verifier-specific frontiers across owner-scoped append-only histories.

It is a follow-on to the timestamp-relative kernel in tools/economic/temporal-evidence. The query target evolves from:

    (subject, valid-time V, verifier X, knowledge-time cutoff K)

to:

    (subject, valid-time V, verifier X, accepted causal frontier F)

A wall-clock K can still be useful for reporting, but it is not proof that a verifier's distributed state incorporated a receipt.

## Model

- OwnerCutV1 binds an owner, stable history identity, covered sequence, owner-history root, a declaration binding that owner root to the aggregate checkpoint root, and explicit gap ranges.
- VerifierFrontierV1 binds a verifier, policy epoch, aggregate checkpoint root, parent frontier references, and one cut per covered owner.
- KnowledgeReceiptV1 binds a verifier, assertion ID/digest, the verifier's receipt-acceptance log owner and local sequence, causal dependencies, receipt digest, and an optional inclusion declaration. The owner is not automatically the source owner of the underlying fact.
- OwnerConsistencyLinkV1 represents the declared relationship between two owner-history roots at different sequences.
- FrontierRelationV1 distinguishes Equal, Before, After, Concurrent, Unknown, and Conflict.
- ReceiptVisibilityV1 distinguishes DeclaredIncluded, NotKnown, and Indeterminate.

## Critical semantics

1. **Partial order, not a global clock.** Frontiers are compared owner by owner. Crossed owner positions yield Concurrent, not an invented total order.
2. **No proof, no inclusion claim.** A receipt under a covered owner prefix is DeclaredIncluded only when the inclusion declaration names the cut's owner root and receipt digest and has structurally valid index/tree-size fields.
3. **Declared is not verified.** This crate does not execute Merkle proof verification, signature verification, checkpoint authentication, or parent-frontier consistency verification. The word DeclaredIncluded deliberately avoids implying cryptographic validation.
4. **Gaps block absence.** If the owner prefix contains a known gap, a receipt within that prefix is Indeterminate. The frontier cannot be compared as a complete cut while any included owner cut contains unresolved gaps.
5. **Not known requires complete coverage.** A receipt sequence beyond a gap-free cut is NotKnown; if the covered prefix itself contains gaps, the answer is Indeterminate. Prefix coverage is necessary but not sufficient for a general absence claim: callers must also reconcile the complete accepted-receipt inventory for that verifier and owner.
6. **Causal closure.** Every dependency referenced by a covered receipt must have an owner cut with complete coverage through that dependency position. Same-owner dependencies must precede the receipt sequence.
7. **Verifier isolation.** A frontier cannot consume a receipt that names another verifier.
8. **Fork detection.** Different roots at the same owner sequence are Conflict. Different sequence positions require an explicit consistency-link reference or compare as Unknown.

## Run

    rustup toolchain install 1.96.0 --profile minimal
    cargo +1.96.0 fmt --all -- --check
    cargo +1.96.0 test --locked
    cargo +1.96.0 clippy --locked --all-targets -- -D warnings
    cargo +1.96.0 run --locked --example frontier_scenarios

## Qualification boundary

The tests exercise deterministic structural rules only. Digest shape is checked, not evidence bytes. Checkpoint inclusion declarations, receipt inclusion declarations, and owner consistency links are checked for field consistency, not cryptographic truth.

Not implemented here:

- Merkle inclusion or append-only consistency proof verification;
- signature, DID, key-rotation, revocation, or authority verification;
- authentic completeness/absence proofs for owner prefixes;
- durable checkpoint/frontier persistence or Holochain DHT integration;
- a verified merge protocol for concurrent frontiers;
- an independent Rust/Go implementation or formal TLA+ model.

A production integration must verify proof bytes against canonical roots, bind owner roots into the authenticated checkpoint, validate causal dependency closure, verify parent/merge ancestry, and only then expose a cryptographically qualified frontier to the query kernel. A query result remains evidence—not permission to mutate a journal, payment, or external effect.

Related tracking: https://github.com/Luminous-Dynamics/mycelix/issues/6680