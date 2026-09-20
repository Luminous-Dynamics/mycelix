# PSI-002B2C r2Q — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
547eaf6e7ab55431647f510db3cec905a871c83e
```

This replaces superseded #2355.

## Exact checks

The qualifier binds the exact B2A dependency blobs and B2C r2 product blobs, then requires:

- explicit versioned `HeadEvidenceClass::wire_id()` values;
- exact `required_evidence_class` in `CurrentnessRequirementV1`;
- independence quorum counted by provider namespace;
- repeated provider namespace rejected even with another verifier key;
- evidence-profile and evidence-class mismatch split;
- fork/conflict, candidate-behind and candidate-ahead separation;
- max-age, exact clock profile, stale/future observation handling;
- strongest positive remains `StructurallyConsistentLatestClaim`;
- `registry_current`, `completeness_established` and `provider_evidence_verified` remain false;
- exact 11-case committed Rust corpus.

It rejects provider crypto/network dependencies and authority-bearing `CurrentUnderProfile` types in this structural crate.

## Offline gate

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No network fallback.

## Success ceiling

A real PASS may establish only exact structural evaluator execution. It does not establish provider evidence, trusted time, completeness, registry currentness, PSI security, composition qualification, production admission or application authority.
