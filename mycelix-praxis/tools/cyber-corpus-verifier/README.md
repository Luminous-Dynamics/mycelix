# Cyber corpus verifier

A small Rust CLI for fail-closed structural verification of the public Praxis cyber challenge seed. It is a corpus-integrity tool, not a cybersecurity solver, rubric scorer, or qualification authority.

From the Mycelix repository root:

```sh
cargo test --manifest-path mycelix-praxis/tools/cyber-corpus-verifier/Cargo.toml
cargo run --manifest-path mycelix-praxis/tools/cyber-corpus-verifier/Cargo.toml -- mycelix-praxis
```

A local run without a source-commit argument is marked `STRUCTURAL_PASS_UNPINNED_SOURCE_NOT_QUALIFICATION`. CI checks out the exact pull-request head commit and passes that SHA as the third CLI argument, producing a source-bound structural receipt. A caller-supplied SHA is provenance metadata, not proof that a dirty local working tree matches that commit.

The CLI hashes exact bytes before parsing challenge/oracle records, rejects unsafe manifest paths and all symlinked manifest/input components (preventing a path from escaping the supplied corpus root), rejects non-fragment `$ref` and `$dynamicRef` references so schemas cannot fetch arbitrary HTTP, file, or relative resources, checks the pinned file set and byte lengths, validates both schemas against the bundled JSON Schema Draft 2020-12 meta-schema, compiles local validators with format checks enabled, validates every challenge and oracle record against its schema, validates strict Rust data structures (unknown fields are rejected), checks duplicate identities and evidence references, and emits a deterministic machine-readable report. The validator is configured without HTTP or filesystem reference resolution; schemas must remain self-contained.

A successful disposition is explicitly `STRUCTURAL_PASS_NOT_CAPABILITY_PASS`. The verifier validates the two checked-in schemas against the bundled Draft 2020-12 meta-schema and validates every checked-in challenge/oracle instance with the pinned Rust implementation. This is not an independent standards audit; it does not score model answers, qualify a cyber range, establish learner efficacy, or prove real-world security. The public seed answer key is not suitable as a held-out evaluation set.

Dependencies are limited to Serde, serde_json, sha2, and the exact-pinned `jsonschema` crate (`=0.59.0`) with default features disabled. Remote HTTP and filesystem reference resolution are not enabled, and the verifier rejects external/relative schema references explicitly even if feature flags change later. The crate is isolated from the parent Holochain workspace so its native CLI dependencies are not compiled into WASM zomes.
