# Life proof-shape correlation benchmark

This dependency-free Rust reference harness follows the residual risks in the W3C Data Integrity Threat Model v1.1 Group Note Draft, 8 October 2026 (R9): even unlinkable derived proofs may leak through total statement count, the pattern of disclosed statement indexes, and globally unique `@id` values.

## Frozen synthetic corpus

- 120 synthetic subjects.
- 3 recipient domains × 2 sessions per subject.
- 720 presentations and 172,800 cross-recipient presentation pairs, including 1,440 same-subject positive pairs.
- Per-presentation proof references are fresh and generated from the presentation ordinal, not the hidden subject label.
- Statement count, revealed-index pattern, schema profile and a deliberately identifying synthetic global ID remain observable channels.
- Ground-truth subject labels are kept in the executable's in-memory fixture and are not exported in the feature corpus.

The proof tags are synthetic tokens, not cryptographic proofs. The attack is exact equality per feature channel, not a trained or adaptive attacker.

## What it measures

The report compares fresh proof-value equality, statement-count equality, disclosed-index pattern equality, count+index equality, schema-profile equality, global-ID equality, structure without the global ID, and all stable fields while deliberately ignoring fresh proof bytes.

Every channel reports TP/FP/FN/TN, precision, recall and false-positive rate. A global ID is an intentionally unsafe positive control. A fresh proof value is intentionally unique and should not match across presentations; it does not protect any other feature channel.

The feature export contains attacker-visible fields only and does not contain the hidden subject label. It does include the deliberate global-ID control so the leakage is explicit in this synthetic corpus.

## Standards context

- W3C VC Data Model 2.0 Recommendation (15 May 2025): https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/
- W3C Data Integrity Threat Model v1.1 Group Note Draft (8 October 2026): https://www.w3.org/TR/2026/DNOTE-vc-data-integrity-threat-model-1.1-20261008/
- W3C Data Integrity BBS Cryptosuites v1.0, Candidate Recommendation Draft: https://www.w3.org/TR/2026/CRD-vc-di-bbs-20260910/

The threat-model document is a Group Note Draft, not a normative security guarantee. The BBS cryptosuite is still a Candidate Recommendation Draft; this harness does not implement it or claim its conformance.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-proof-shape/Cargo.toml --all -- --check
cargo +1.96.0 test --locked --manifest-path research/life-proof-shape/Cargo.toml
cargo +1.96.0 clippy --locked --manifest-path research/life-proof-shape/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --locked --manifest-path research/life-proof-shape/Cargo.toml -- --write-corpus-dir target/life-proof-shape
sha256sum target/life-proof-shape/*
```

The printed FNV-1a fingerprint is non-cryptographic. The GitHub Actions workflow separately computes SHA-256 over the report and attacker-visible feature corpus and binds those hashes to the exact commit and workflow run.

## Next research steps

1. Add a second independently implemented evaluator.
2. Introduce normalized/padded presentation profiles only with a matched task-utility oracle and explicit protocol semantics. Padding should be tested as a design hypothesis, not assumed to fix index-pattern leakage.
3. Add skewed/correlated claim values and public auxiliary information under the separate LIFE-DISCLOSURE-COMPOSITION-001 workstream.
4. Only after the synthetic baseline qualifies, compare concrete version-pinned SD-JWT and BBS implementations, measuring proof generation/verification cost and all output-side leakage channels independently.

## Claim ceiling

A PASS establishes only the frozen synthetic feature-level evaluator and registered known-answer cases. It does not establish real BBS unlinkability, anonymity, differential privacy, post-quantum security, production security, legal compliance, standards conformance or superiority over industry systems.
