# Typed Cross-Border Applicability Context (development draft)

This package is the first conformance artifact for [FIN-GLOBAL-CONTEXT-001 / #5839](https://github.com/Luminous-Dynamics/mycelix/issues/5839). It is designed to compose with the standalone resolver in [#5829](https://github.com/Luminous-Dynamics/mycelix/pull/5829).

## Why it exists

The v1 resolver's `jurisdiction_scopes: string[]` is an exact set, but it is not a transaction applicability model. Set equality loses the semantic role and subject attached to each scope. Two contexts can contain the same scope IDs while having different seller-establishment, ship-from, buyer-location, ship-to or place-of-supply assignments.

This draft keeps **who**, **which jurisdictional role**, **which namespace/code**, **when valid**, and **when known** as separate typed facts.

## Files

- `jurisdiction_context_bundle_v1.0.0.schema.json`: closed Draft 2020-12 schema for a context bundle.
- `fixtures/jurisdiction_context_candidates_v1.0.0.json`: public synthetic candidate contexts.
- `jurisdiction_context_oracle_v1.0.0.schema.json`: schema for the public reference oracle.
- `fixtures/jurisdiction_context_oracle_v1.0.0.json`: expected synthetic behavior, deliberately separate from candidate inputs.
- `../rust-resolver/src/bin/validate_applicability_context.rs`: standalone Rust CLI and unit tests for Draft 2020-12 validation, cross-reference integrity and eligibility filters.

The oracle is public for reproducible contract tests. It is **not** a hidden benchmark result. Hidden evaluation and the canonical run manifest remain owned by #5799.

## Semantics

1. A party's transaction role (for example seller or buyer) is separate from the fact's jurisdiction role (for example establishment or ship-from).
2. A jurisdiction scope is identified by the tuple `(namespace_uri, code, scope_level, parent_scope_ref)`, not by a country label or code string alone.
3. A fact binds one subject, one jurisdiction role and at most one scope. Conflicting claims are separate immutable facts linked by `conflicts_with_fact_refs`; they are not overwritten or resolved by array order.
4. `valid_from` is inclusive and `valid_to` is exclusive. `recorded_at` is the UTC knowledge/recording time. Historical evaluation must filter by both valid time and the request's `knowledge_at` frontier.
5. `evidence_supported` means an evidence reference is attached to the assertion. It does not claim that the source is authentic, complete, authoritative for the legal question or independently corroborated; those qualifications belong to #5798 and #5834.
6. `source_asserted`, `human_adjudicated`, `approved_rule_derived` and `unresolved` are different origins. A derived place-of-supply fact must bind an approved rule/evaluation reference. An unknown place of supply has no scope.
7. A party identity binding is always issuer-namespace scoped. Matching identifier strings under different namespaces do not merge parties.
8. Parent/child scope relationships are explicit. A country-level scope does not satisfy a subnational requirement by implication.

## Future resolver composition

A follow-on resolver request should carry a context digest and exact typed requirements, such as:

```text
subject_role = seller
jurisdiction_role = establishment_location
scope_ref = SYN-SCOPE-A
minimum_assertion_state = evidence_supported
valid_at = 2026-10-31
known_at = 2026-11-05T09:30:00Z
```

The v1.1 integration should begin with conjunction-only exact matching. No wildcard, nearest-parent fallback, address-to-place-of-supply inference or general-purpose policy expression language is needed. Legal applicability derivation remains with an approved policy profile under #5792; the resolver only checks whether the exact pre-qualified intersection matches.

## Local Rust validation

From `mycelix-finance/benchmarks/global-accounting-portability-v1/rust-resolver`:

```sh
cargo test --locked --bin validate_applicability_context
cargo run --locked --bin validate_applicability_context
cargo fmt -- --check
cargo clippy --locked --all-targets -- -D warnings
```

The validator uses the pinned `jsonschema` Draft 2020-12 implementation with format assertions enabled. It checks schema validity, candidate/oracle shape, unique identities, party-role and scope/fact references, reciprocal conflict links, valid-time intervals, and evidence/namespace boundaries. Its fact-eligibility helper requires a supported assertion, explicit scope, evidence reference, conflict-free state and visibility at both valid-time and knowledge-time frontiers. It is not a tax resolver and does not select a legally applicable jurisdiction.

## Synthetic claim ceiling

All party refs, scope namespaces, scope codes, evidence refs, profiles and derived-rule refs in the fixtures are synthetic. They encode no national law, tax rate, legal nexus, VAT rule or jurisdictional conclusion. Passing these fixtures demonstrates only typed-context preservation and deterministic conformance behavior.

## Validation status

These files and the Rust validator are authored draft artifacts. They have not yet been run through `cargo test`, rustfmt, Clippy or CI. The validator is designed to perform full Draft 2020-12 runtime validation when executed, but that execution has not yet been performed in this authoring environment. No pass is claimed.