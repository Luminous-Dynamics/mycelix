# Physical Artwork Passport Conformance Tool

A small Rust-native validator for the CSCP physical-artwork extension.

It checks two layers:

1. JSON Schema Draft 2020-12 structural validation, with format checking enabled.
2. CSCP-specific reference integrity: unique claim/evidence/event/right identifiers and resolvable references among claims, events, evidence, rights, condition observations, spatial assets and artwork claim links.

It performs no remote reference fetching and does not mutate the input. A valid record means only that the record conforms to this profile and its local references resolve. It does not prove that any assertion is true, that an issuer is authoritative, or that an artwork is authentic or legally owned.

## Run

From this directory:

```sh
cargo run -- \
  ../../specs/creative-assets/physical-artwork-extension.schema.json \
  ../../specs/creative-assets/examples/synthetic-sculpture.json
```

Exit codes:
- `0`: valid
- `1`: schema or reference-integrity failure
- `2`: usage, file, JSON, meta-schema, or validator-construction error

## Tests

```sh
cargo test
```

Tests cover the synthetic positive example and negative controls for missing required identity, unknown properties, malformed digests, dangling evidence references, duplicate assertion IDs, and rights claims without a source assertion.

## Dependency and status

Uses the Rust `jsonschema` crate pinned to 0.61.0 for Draft 2020-12 support. This source is a proposed implementation; compile/test status must be reported from an exact local or CI execution, not inferred from its presence in Git.
