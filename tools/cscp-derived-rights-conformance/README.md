# CSCP Derived-Asset Rights Conformance Oracle

A small Rust-native reference oracle for the frozen synthetic vectors in
`specs/creative-assets/examples/derived-rights-conformance-vectors.json`.

## Purpose and boundary

This executable makes the v0.1 research profile's stated decision examples
machine-checkable. It is a conformance oracle for the fixture vocabulary, not a
production authorization service, legal interpreter, credential verifier, or
substitute for Mycelix's existing CSCP authority.

The string permissions in early vectors are deliberately compact fixture
shorthand. A production CSCP adapter must evaluate fully scoped grants,
authority evidence, policy revisions, validity intervals and complete lineage
using the canonical CSCP policy infrastructure. This crate must not be wired
directly into provider dispatch as a new policy authority.

## Run

From the repository root:

```sh
cargo test --manifest-path tools/cscp-derived-rights-conformance/Cargo.toml
cargo run --manifest-path tools/cscp-derived-rights-conformance/Cargo.toml -- --check specs/creative-assets/examples/derived-rights-conformance-vectors.json
```

Exit codes:
- `0`: every vector's declared decision and reason code matched the oracle
- `1`: one or more vectors failed conformance
- `2`: usage, file, JSON, or corpus-structure error

## Evaluated invariants

- deny by default;
- indeterminate is never an enforcement permit;
- one prohibition in a multi-source derivation blocks the output;
- derivative-local output permissions cannot widen an ancestor restriction;
- missing policy state, incomplete lineage and unknown authority fail closed;
- stale cache epochs and cross-tenant access are denied;
- a Local Contexts / CARE governance signal is not itself an executable grant;
- community-scoped permissions require exact actor, operation, purpose and time;
- policy supersession is narrow and binds source, output, operation, grantee,
  purpose, jurisdiction, channel, interval, authority and approval evidence.

The CLI reports only conformance to these synthetic examples. A pass does not
qualify production CSCP behavior, downstream cache/index enforcement, community
approval, copyright/title, authenticity or legal compliance.
