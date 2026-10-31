# Scientific Policy Metamorphic Corpus v0.1

A standalone Rust tool that creates deterministic transformations of the frozen
62,464-row Cedar/OPA input corpus. It does not calculate expected Allow/Deny
decisions and does not depend on the missing Rust authorization oracle.


## Deterministic counterexample shrinking

The crate now exports `shrink::minimize_json`, a generic deterministic reducer
that preserves a caller-supplied predicate. It explores a finite, documented
neighborhood of JSON reductions: object-member removal, array-element removal,
empty/shortened strings, numeric simplification and true-to-false changes.
Callers can protect JSON Pointer paths such as `/principal`, `/action` and
`/resource` so an authorization counterexample retains its request identity.

Each candidate records its sequence, JSON Pointer, operation, before/candidate
SHA-256, predicate outcome and any evaluation error. The trace also pins the
reduction profile, sorted-object JSON digest-encoding profile and protected
JSON Pointer set. It stores both the exact raw source-input SHA-256 and the
normalized parsed-value SHA-256; the supplied source bytes must parse to the
exact value being minimized. The reducer rejects duplicate object
member names (including escaped-equivalent names), invalid UTF-8, unpaired
surrogates and trailing JSON content. Callers remain responsible for
schema-specific and engine-specific authorization validation. Thus
whitespace/property-order changes alter the source digest but preserve the
normalized-value digest. This local encoding is not an RFC 8785/JCS claim.

Candidate descriptors are generated only up to the remaining evaluation
budget, and the full JSON value is cloned only when a candidate is evaluated.
The API caps source records at 4 MiB and candidate predicate evaluations at
100,000; exhaustion remains explicitly incomplete rather than claiming
1-minimality.

A complete trace claims only 1-minimality in that exact reduction
neighborhood. Candidate-budget exhaustion or an evaluation error returns an
incomplete trace and never claims minimality. The original input must satisfy
the predicate before shrinking begins. Object traversal and digest
serialization sort member names explicitly.

Initial tests cover deterministic output, protected identity fields,
nested resource protection, array-element reduction, exact source/value
identity separation, mismatched source rejection, duplicate/escaped duplicate
object keys, invalid UTF-8, unpaired surrogates, hard input/evaluation limits,
budget exhaustion, predicate errors and numeric convergence. This is a generic
JSON reducer: it is not yet wired to invoke Cedar/OPA or to minimize the
62,464-case engine corpus. It makes no Rust authorization parity or
policy-correctness claim.

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
