# Scientific Policy Metamorphic Corpus v0.1

A standalone Rust tool that creates deterministic transformations of the frozen
62,464-row Cedar/OPA input corpus. It does not calculate expected Allow/Deny
decisions and does not depend on the missing Rust authorization oracle.

## Run

```sh
cargo generate-lockfile --manifest-path tools/scientific-policy-metamorphic/Cargo.toml
cargo test --manifest-path tools/scientific-policy-metamorphic/Cargo.toml --locked
cargo clippy --manifest-path tools/scientific-policy-metamorphic/Cargo.toml --all-targets --locked -- -D warnings

cargo run --manifest-path tools/scientific-policy-metamorphic/Cargo.toml --locked -- \
  target/scientific-policy-space/manifest.json \
  target/scientific-policy-metamorphic
```

The input generator manifest and the two source JSONL corpora must come from
`tools/scientific-policy-space` at its frozen source-pinned baseline.

## Generated variants

Four folders are emitted. Each contains a copy of the original abstract corpus,
a transformed paired engine-input JSONL, a runner-compatible `manifest.json`
with the transformed-input SHA-256, and a separate `transform-manifest.json`.

| Transform | Intended relation | Meaning |
|---|---|---|
| `json-object-key-order-reversed` | Decision invariant | Reverse object-key serialization recursively |
| `json-whitespace-expanded` | Decision invariant | Re-serialize each JSONL record with insignificant whitespace |
| `cedar-entity-order-reversed` | Decision invariant | Reverse the explicit Cedar entity array without changing its members |
| `cedar-irrelevant-actor-added` | Decision invariant under stated precondition | Add a schema-shaped Actor with a unique, unreferenced UID and no parents |

All variants preserve the case IDs, action, actor, resource, context and Rego input.
The extra-entity transformation changes only the Cedar entity set and records
its non-reference precondition explicitly. Each transformed row is reparsed
before being written. The source input and output byte digests are independently
recorded.

## Engine evaluation boundary

Generation is not a metamorphic evaluation result. The intended next use is to
run each transformed folder through the exact pinned Cedar 4.14.0 and OPA 1.22.0
runners, compare each engine's per-case decisions with its own baseline, and
record any diagnostics, failures, or divergences. A transformation is not
qualified merely because its JSON parses.

The object-key and whitespace variants are pure serialization transformations.
Cedar entity order is treated as set-order-independent only under unique UID
preconditions. The added Actor is intended to be irrelevant because it is not
referenced by the request or policies; the actual Cedar evaluation must verify
that expectation rather than assuming it.

## Claim ceiling

This tool proves deterministic transformation and byte-level provenance of the
generated corpora. It does not prove decision invariance, Cedar/OPA agreement,
Rust policy parity, credential-registry correctness, or production authorization
safety.
