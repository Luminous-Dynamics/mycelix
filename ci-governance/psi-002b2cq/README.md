# PSI-002B2CQ — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
cb2124800d98d1e51d21db37c44e0fe650611802
```

This qualifier materializes the exact B2A registry-subject dependency and B2C structural-currentness crate into a temporary sibling-crate layout and runs only offline Cargo commands from the B2C standalone workspace.

## Gate

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

## Static boundary checks

The qualifier requires:

- exact B2A dependency blobs;
- exact B2C product blobs;
- `StructurallyConsistentLatestClaim` vocabulary;
- explicit `registry_current() == false`;
- explicit `completeness_established() == false`;
- explicit `provider_evidence_verified() == false`;
- duplicate-provider rejection;
- conflict/fork handling;
- candidate-behind and candidate-ahead separation;
- exact provider-count requirement;
- max-age + clock-profile checks;
- future/stale observation checks;
- exact committed test count.

It rejects provider-crypto/network dependencies and rejects an authority-bearing `CurrentUnderProfile` type in this structural source.

## Success ceiling

A real PASS may establish only that this exact structural currentness-claim evaluator compiled and its registered tests passed.

It does not establish provider evidence, trusted time, completeness, registry currentness, PSI security, composition qualification, production admission, or application authority.
