# Scientific Policy Decision-Space Generator v0.1

This isolated Rust workspace materializes the bounded abstract input domain described in issue #5530 and the research plan in PR #5531.

## Run

    cargo test --locked
    cargo run --locked -- ./target/scientific-policy-space

The command writes a stable JSONL abstract corpus, a paired JSONL projection of each case into native Cedar request/entity data and OPA/Rego input JSON, and a manifest containing source, generator, domain and both corpus digests.

## Domain

- 8 actions from the pinned Rust `ScientificAction` enum.
- All 128 subsets of the 7 current `ScientificRole` values.
- 1,024 absent-stream inputs: every action and role subset. Ownership, organization and target-attestation dimensions are not applicable because the source match returns before a current projection can be read.
- 61,440 existing-stream inputs: actor is/is not creator; creator organization absent/present; acting organization absent/equal/different when a creator organization exists, or present-without-creator-organization when no creator organization exists; ordinary/independent attestation; target attestation absent/authored by actor/authored by another.
- Total: 62,464 stable abstract cases.

Dimensions not inspected by a given action are intentionally retained for existing streams. This makes them explicit noninterference controls instead of silently pruning them.

## Expected decision boundary

Every generated row has `expected_decision: null` and `expected_decision_status: pending_authentic_rust_oracle`. This tool deliberately does not reimplement the authorization result and must not become its own oracle.

The pinned source is `scientific_governance.rs` at base revision `f89b4bcdcfb901961e2ca11a577775acbf53a74d`, blob `dd9ea9267cdae2e2c67244e8a90821defaacefb3`. The authorization function body was compared against the earlier `061e34f...` snapshot and found byte-identical, but the generator pins the exact source file present in its PR base. A test recomputes the Git blob ID from the checked-out source file and fails on drift. The credential-registry source module required by the DeSci crate is absent, so the authentic Rust adapter remains blocked. The same 62,464 abstract contexts are also emitted as `scientific-policy-engine-inputs-v0.1.jsonl` with both Cedar PARC (`request` + `entities`) and OPA/Rego (`input`) shapes. Cedar/Rego may consume these exact projections, but their agreement before Rust oracle execution is engine-to-engine agreement only.

## Claim ceiling

This is a finite abstract input corpus, not a proof of authorization equivalence, a complete encoding of arbitrary identity/resource strings, a credential-registry implementation, or authorization of cryptographic identity, membership, replay, receipts or commits.
