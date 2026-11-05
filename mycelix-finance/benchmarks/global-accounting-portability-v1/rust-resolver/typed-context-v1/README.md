# Mycelix Global Typed Context Resolver v1.0.0 (development draft)

This is the typed-context follow-on to [FIN-GLOBAL-RESOLVER-002 / #5846](https://github.com/Luminous-Dynamics/mycelix/issues/5846). It consumes the synthetic, typed, bitemporal context package introduced by [#5839 / PR #5840](https://github.com/Luminous-Dynamics/mycelix/pull/5840), and composes with the v1 profile-intersection resolver in [#5829](https://github.com/Luminous-Dynamics/mycelix/pull/5829).

## Scope

This binary answers only: **does the frozen context bundle contain exactly one evidence-supported fact satisfying each declared typed fact requirement at the requested valid-time and knowledge-time frontiers?**

It does not qualify a profile intersection, decide tax nexus or place of supply, execute a tax/accounting rule, validate an invoice, authorize a journal, or produce a filing conclusion.

## Contract

- SHA-256 pins the exact raw bytes of both the context bundle and the context schema. The resolver refuses either digest mismatch.
- The request selects one context by exact ID and binds the context bundle ID and schema `$id`.
- Each requirement matches the exact subject kind/ref, party role, actor bindings, jurisdiction fact role and full scope identity tuple (scope ID, namespace URI, code, level and parent ref).
- `evidence_supported` is the minimum accepted assertion state. Evidence refs are presence-checked only; this is not an authenticity, reliability, completeness or legal-sufficiency verdict.
- Only facts valid at `valid_at` and recorded by `knowledge_at` are visible. A request after the bundle's declared knowledge frontier is indeterminate because the snapshot cannot prove completeness at that later frontier.
- Duplicate matches and disputed / unverified facts require review. Unknown or temporally unavailable facts are indeterminate. A scope or typed-subject mismatch is unsupported.
- Requirements are conjunction-only. No wildcard, parent fallback, inferred place of supply, nearest jurisdiction or general rule-expression language exists.

## Files

- `typed_context_request_v1.0.0.schema.json`
- `typed_context_report_v1.0.0.schema.json`
- `typed_context_oracle_v1.0.0.schema.json`
- `fixtures/typed_context_requests_v1.0.0.json`
- `fixtures/typed_context_oracle_v1.0.0.json`
- `src/bin/resolve_typed_context.rs`

The public oracle is deliberately visible for deterministic reference tests. The Rust unit suite validates its Draft 2020-12 schema and checks all seven expected dispositions/reason codes against resolver outputs. It is not a hidden benchmark result. #5799 owns evaluator isolation, blinded candidate execution and canonical run manifests.

## Run

From the `rust-resolver` directory:

```sh
cargo test --locked --bin resolve_typed_context
cargo run --locked --bin resolve_typed_context -- --request typed-context-v1/fixtures/typed_context_requests_v1.0.0.json --request-index 0
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

`--request-index` selects an entry from the public request bundle. The standalone binary validates both Draft 2020-12 schemas, the context bundle, the selected request and the output report. The bundle and schema digests are checked over exact raw bytes.

## Benchmark controls

The public requests include:
- complete seller-establishment / ship-from / seller-registration / place-of-supply requirements;
- a same-flat-scope-set role swap;
- unknown place of supply;
- disputed seller tax registration;
- explicit second-establishment selection;
- country-versus-subnational mismatch;
- historical knowledge-frontier exclusion.

Passing the public fixture establishes only typed fact matching under the exact synthetic schema/data/request. It does not establish the correctness of the fact, law, policy or tax interpretation.

## Validation status

The schemas, fixtures and Rust binary are draft authoring artifacts. The current public candidate/oracle pairs have been fetched back and semantically audited; all seven synthetic expectations align with a separate logic simulation. This is not a Rust execution result. No compile, `cargo test`, rustfmt, Clippy or CI result is claimed until the exact head is executed and recorded.