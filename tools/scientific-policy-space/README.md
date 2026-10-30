# Scientific Policy Decision-Space Generator v0.1

This isolated Rust workspace materializes the bounded abstract input domain described in issue #5530 and the research plan in PR #5531.

## Run

    cargo test --locked
    cargo run --locked -- ./target/scientific-policy-space

The command writes a stable JSONL corpus and a manifest containing source, generator, domain and corpus digests.

## Domain

- 8 actions from the pinned Rust `ScientificAction` enum.
- All 128 subsets of the 7 current `ScientificRole` values.
- 1,024 absent-stream inputs: every action and role subset. Ownership, organization and target-attestation dimensions are not applicable because the source match returns before a current projection can be read.
- 61,440 existing-stream inputs: actor is/is not creator; creator organization absent/present; acting organization absent/equal/different where equality is meaningful; ordinary/independent attestation; target attestation absent/authored by actor/authored by another.
- Total: 62,464 stable abstract cases.

Dimensions not inspected by a given action are intentionally retained for existing streams. This makes them explicit noninterference controls instead of silently pruning them.

## Expected decision boundary

Every generated row has `expected_decision: null` and `expected_decision_status: pending_authentic_rust_oracle`. This tool deliberately does not reimplement the authorization result and must not become its own oracle.

The pinned source is `scientific_governance.rs` at revision `061e34f24d406f19f6de0e61867305b6989f455f`, blob `488c69fd4b5540371f2714e9d5668aecc12e294d`. The credential-registry source module required by the DeSci crate is absent, so the authentic Rust adapter remains blocked. Cedar/Rego/CEL may consume the same generated abstract inputs, but their agreement before Rust oracle execution is engine-to-engine agreement only.

## Claim ceiling

This is a finite abstract input corpus, not a proof of authorization equivalence, a complete encoding of arbitrary identity/resource strings, a credential-registry implementation, or authorization of cryptographic identity, membership, replay, receipts or commits.
