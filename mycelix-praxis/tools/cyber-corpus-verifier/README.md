# Cyber corpus verifier

A small Rust CLI for fail-closed structural verification of the public Praxis cyber challenge seed. It is a corpus-integrity tool, not a cybersecurity solver, rubric scorer, or qualification authority.

From the Mycelix repository root:

```sh
cargo test --manifest-path mycelix-praxis/tools/cyber-corpus-verifier/Cargo.toml
cargo run --manifest-path mycelix-praxis/tools/cyber-corpus-verifier/Cargo.toml -- mycelix-praxis
```

The CLI hashes exact bytes before parsing challenge/oracle records, rejects unsafe manifest paths, checks the pinned file set and byte lengths, validates strict Rust data structures (unknown fields are rejected), checks duplicate identities and evidence references, and emits a deterministic machine-readable report.

A successful disposition is explicitly `STRUCTURAL_PASS_NOT_CAPABILITY_PASS`. The verifier does not validate the full JSON Schema metaschema, score model answers, qualify a cyber range, establish learner efficacy, or prove real-world security. The public seed answer key is not suitable as a held-out evaluation set.

Dependencies are limited to Serde, serde_json, and sha2. The crate is isolated from the parent Holochain workspace so its native CLI dependencies are not compiled into WASM zomes.
