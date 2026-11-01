# GCPN Compositional Assurance v0.1

Standalone Rust engine for deterministic, synthetic qualification of composed civil-protection evidence chains under interacting faults.

## Scope

The engine preserves claim-level dispositions and stage violations across six stages: forecast, observation, CAP lifecycle, population reach, decision and after-event review. It checks common-cause lineage, temporal visibility, source/version/subject alignment, message receipt identity, population accounting, paired comparison lineage, causal-claim support and post-processing leakage.

There is no single compensatory score. Missing or invalid evidence never silently becomes a pass. Different evaluator labels do not establish independence when evidence shares the same lineage root.

## Run

```sh
cargo run --manifest-path Cargo.toml
cargo test --manifest-path Cargo.toml --locked --offline
cargo clippy --manifest-path Cargo.toml --locked --offline --all-targets -- -D warnings
```

The CLI runs a clean deterministic synthetic scenario. Unit tests inject single and interacting faults and check typed dispositions and violation codes.

## Reproducibility

Reports use canonical sorted records and an FNV-1a 64-bit fingerprint. This is a non-cryptographic reproducibility diagnostic, not a digital signature, tamper-proof receipt or cryptographic attestation. Any future cryptographic receipt needs a separately reviewed hash/signature adapter and canonical serialization standard.

## Non-claims

This package is not an operational warning authority, forecast model, causal impact estimator or real-world system certification. It does not prove that supplied lineage metadata is truthful; it tests whether declared common-cause lineages are handled correctly.
