# DJCC V0.1 independent Go oracle

This module contains an independently authored Go expectation table for the initial synthetic Decision Justification Composition Certificate (DJCC) seed manifest.

Run from this directory:

```sh
go test ./...
go test -race ./...
go vet ./...
```

## What is checked

The oracle checks the registry against its own expected disposition table:
- all eight frozen initial seed IDs occur exactly once;
- the eight stage IDs and every per-stage expected disposition are present;
- composition dispositions and five-dimensional claim ceilings match independent expected outcomes;
- each case is synthetic-only and has distinct positive/negative control references;
- hard-safety mutations have explicit operator and target;
- positive controls do not carry a mutation.

The hard-coded Go expectation table is not generated from the JSON manifest. This guards against accidental edits to the manifest silently changing the expected outcome.

## Rust candidate verifier

A separate dependency-free Rust candidate is at `mycelix-workspace/crates/mycelix-djcc-verifier`.

Run from the repository root:

```sh
cargo +1.96.0 fmt --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml -- --check
cargo +1.96.0 test --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets
cargo +1.96.0 clippy --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets -- -D warnings
```

The Rust candidate currently checks:
- all eight stage IDs are present;
- selected composition profile reference and digest match;
- one exact root scope digest/schema binding is present per stage;
- stage producer identity and source-commit syntax;
- stage output digest/schema shape;
- the fixed profile dependency graph and its canonical order;
- every dependency consumes exactly one named input binding with matching digest/schema;
- producer output digest/schema equals consumer dependency digest/schema;
- no orphan stage input bindings remain on unevaluated/not-applicable stages;
- profile-authorized `NotApplicable` only;
- no positive root claim dimension is established if any stage's corresponding claim ceiling is not established;
- deterministic reason/input ordering.

The fixed dependency graph is explicit in the source. A future profile registry may version alternative graphs, but a candidate cannot invent its own edges.

## What is not checked

The Go program validates the public seed manifest against its own expected-result table. The Rust program verifies structural composition of already supplied stage results. Neither currently verifies certificate signatures, source-log identity, authenticated owner-universe closure, Merkle root authority, semantic payload truth, legal authority, policy execution, one-shot effect admission, IVC proofs, or physical outcome.

## Shared Go/Rust stage-edge vectors

`DJCC_STAGE_EDGE_V1_KAT.tsv` specifies the first shared byte-level contract:
- domain separator `MYCELIX-DJCC-STAGE-EDGE-V1`;
- four UTF-8 strings, each prefixed by a big-endian u32 **byte** length: producer stage, consumer stage, input name, output schema reference;
- the producer's 32-byte output digest appended as raw bytes.

The Go oracle and Rust candidate independently construct the canonical preimage and compare it with the same frozen known-answer bytes. One vector contains a non-ASCII UTF-8 input name to catch character-count versus byte-count errors.

This is only a canonical stage-edge encoding vector. It is not yet a full certificate canonicalization, canonical JSON profile, or signed certificate digest. Root certificate encoding remains a separate deliverable.

The Go manifest oracle and the Rust candidate still do not compare full candidate certificate outputs against one another. Shared stage-edge byte parity is the first narrow interoperability gate.

## Claim boundary

This is **seed-registry validation plus a structural candidate verifier and one shared stage-edge byte vector**, not an end-to-end trust verifier or production assurance result. The eight seed references are symbolic until exact synthetic fixture bytes are frozen and hashed. Go/Rust edge parity does not establish complete certificate parity, external source truth, legal effect or physical outcome.
