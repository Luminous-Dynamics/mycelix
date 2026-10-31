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
| `claim-proposed-contributor-removed` | Exactly one allow-to-deny transition | In the unique source case with an absent claim and contributor-only role, remove that role from Cedar and Rego while preserving the case ID and every other field |

All variants preserve the case IDs, action, actor, resource, context and Rego input.
The extra-entity transformation changes only the Cedar entity set and records
its non-reference precondition explicitly. Each transformed row is reparsed
before being written. The source input and output byte digests are independently
recorded.

## Engine evaluation boundary

Generation alone is not a metamorphic evaluation result. The dedicated
`.github/workflows/scientific-policy-metamorphic.yml` workflow runs both the
baseline and all four variants through Cedar 4.14.0 and OPA 1.22.0. It compares each engine against its own baseline using stable case IDs,
decision values, and determining-policy identifiers (Cedar
`Diagnostics::reason()`; OPA rule IDs). The four invariance transformations
must produce zero decision and policy-ID changes. The targeted negative control
must produce exactly one allow-to-deny transition, with exactly one
determining-policy change, and every other case must remain unchanged. Any
unexpected case-set change, decision drift, policy-ID drift, parser/schema
error, Cedar evaluation diagnostic, OPA failure, or incomplete output fails
qualification.

The Cedar runner requires the requested principal Actor and exactly one
resource Claim, unique UIDs, and schema-valid attributes. Additional unique
Actor entities are permitted for the explicitly documented irrelevant-entity
transformation; action entities remain supplied by the pinned schema. The
workflow preserves engine-specific diagnostics and provenance. The OPA batch
measurement and Cedar in-process measurement remain separately labelled and
must not be compared as apples-to-apples timing.

The object-key and whitespace variants are pure serialization transformations.
Cedar entity order is treated as set-order-independent only under unique UID
preconditions. The added Actor is intended to be irrelevant because it is not
referenced by the request or policies; the actual Cedar evaluation must verify
that expectation rather than assuming it.

## Claim ceiling

The generator proves deterministic transformation and byte-level provenance;
the dedicated workflow tests four preconditioned invariance transformations
plus one narrowly defined candidate-policy negative control against each
pinned engine's own baseline. The negative control is derived from the explicit
pure-policy rule for claim creation and is not an authentic Rust oracle result.
Neither establishes Cedar/OPA policy equivalence, authentic Rust policy parity,
credential-registry correctness, or production authorization safety.
