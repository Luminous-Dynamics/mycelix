# FIN-ASSURE-PROCEDURE verifier

Standalone, dependency-light Rust reference evaluator for the public synthetic risk/assertion/procedure/evidence corpus.

The default CLI runs **candidate corpus v1.2.0**, its versioned Draft 2020-12 input schema and public reference oracle. Each case result includes both the overall stand-back disposition and an explicit disposition for every risk/assertion pair. A case is not considered clear merely because one of several required pairs is supported.

The v1.2.0 evidence contract has no global reliability scalar. Each evidence item separately binds accuracy, completeness, authenticity, absence of bias and credibility. Each risk/assertion/purpose pair declares the attributes it requires; an attribute that is limited, unqualified or unknown blocks only the pair that needs it. The multi-pair FAP-023 fixture proves that the same artifact can support occurrence while being insufficient for completeness.

Run from this directory:

```sh
cargo test --locked
cargo fmt --all -- --check
cargo clippy --locked --all-targets -- -D warnings
cargo run --locked -- --out /tmp/fin-assure-procedure-report.json
```

The CLI supports `--input`, `--schema`, `--oracle`, `--oracle-schema` and `--out`. To replay v1.0.0 or v1.1.0, explicitly pass that release's candidate, input schema, public oracle and oracle schema. The report includes exact input/schema/oracle and verifier digests.

The public cases and oracle are regression references, not a blind model benchmark. Human judgment and authority remain external. This is not an audit, assurance opinion, ISA-compliance result or professional accounting conclusion.
